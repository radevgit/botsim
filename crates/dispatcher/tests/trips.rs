//! Trips to chargers and to parking places.

use botsim_dispatcher::{ChargePolicy, Dispatcher, Purpose, Robot, Work};
use botsim_model::time::Millis;

mod common;
use common::*;

const LIMITS: ChargePolicy = ChargePolicy {
    low: 25.0,
    high: 80.0,
    critical: 10.0,
};

/// A rig on the line with these stations, sending robots to charge.
fn charging(stations: &[(&str, usize)], robots: usize) -> Rig {
    let mut rig = Rig::new(line(stations), robots);
    let chargers = stations.iter().filter(|s| s.0 != "parking").count();
    assert_eq!(rig.dispatcher.set_charging(LIMITS), chargers);
    rig
}

#[test]
fn a_charger_goes_to_the_nearest_low_robot_first() {
    let mut rig = charging(&[("Charger", E)], 2);
    rig.dispatch(&[low(A, 20.0), low(D, 20.0)]);
    assert_eq!(rig.script.summary(&rig.graph), [(1, Purpose::Charge, E)]);
    let task = &rig.script.given[0].1;
    // the stand-in job: from the robot's node to the charger, numbered by trips
    assert_eq!((task.job.id, task.job.pickup, task.job.drop), (0, D, E));
    assert_eq!(task.job.created, Millis(1000));
    let counts = rig.dispatcher.counts();
    assert_eq!(counts.charge_trips, 1);
    assert_eq!(counts.charge_wait_seconds, 1.0, "robot 0 waited a second");

    // the charger has a robot on its way: no second one
    rig.script.clear();
    rig.dispatch(&[low(A, 20.0), busy(D)]);
    assert!(rig.script.given.is_empty());
    assert_eq!(rig.dispatcher.counts().charge_wait_seconds, 2.0);
}

#[test]
fn two_low_robots_do_not_get_the_same_charger() {
    let mut rig = charging(&[("charger", A), ("charger", E)], 2);
    rig.dispatch(&[low(D, 20.0), low(D, 20.0)]);
    assert_eq!(
        rig.script.summary(&rig.graph),
        [(0, Purpose::Charge, E), (1, Purpose::Charge, A)]
    );
}

#[test]
fn a_held_charger_is_not_free() {
    let mut rig = charging(&[("charger", E)], 1);
    rig.script.held.insert(E, 5);
    rig.dispatch(&[low(D, 20.0)]);
    assert!(rig.script.given.is_empty());
}

#[test]
fn a_robot_charges_from_its_arrival_to_the_upper_limit() {
    let mut rig = charging(&[("charger", E)], 1);
    rig.dispatch(&[low(D, 20.0)]);
    assert!(!rig.dispatcher.stop_charging(0, 90.0), "not there yet");
    rig.dispatcher.arrived(0, Purpose::Charge);

    // it gets no job while it charges
    rig.script.clear();
    rig.jobs(&[job(1, A, B)]);
    rig.dispatch(&[low(E, 50.0)]);
    assert!(rig.script.given.is_empty());
    assert!(!rig.dispatcher.stop_charging(0, 79.0));
    assert!(rig.dispatcher.stop_charging(0, 80.0));
    assert!(!rig.dispatcher.stop_charging(0, 80.0), "stopped once");
    rig.dispatch(&[low(E, 80.0)]);
    assert_eq!(rig.script.summary(&rig.graph), [(0, Purpose::Job, B)]);
}

#[test]
fn a_low_robot_on_a_charger_starts_charging_where_it_stands() {
    let mut rig = charging(&[("charger", E)], 2);
    // (the controller names the robot standing on a node as its holder)
    rig.script.held.insert(E, 1);
    rig.dispatch(&[low(D, 20.0), low(E, 20.0)]);
    assert_eq!(rig.script.charged_here, [(1, 0)]);
    assert!(rig.script.given.is_empty(), "no charger for robot 0");
    assert_eq!(rig.dispatcher.counts().charge_trips, 1);
    assert!(rig.dispatcher.stop_charging(1, 80.0));
}

#[test]
fn below_the_critical_limit_a_robot_takes_no_job() {
    let mut rig = charging(&[], 2);
    rig.jobs(&[job(1, A, B), job(2, A, B)]);
    // robot 0 is below the critical limit, robot 1 between the limits
    rig.dispatch(&[low(A, 9.0), low(A, 10.0)]);
    assert_eq!(rig.script.summary(&rig.graph), [(1, Purpose::Job, B)]);
    assert_eq!(rig.pending(), [2]);
    assert_eq!(rig.dispatcher.counts().charge_wait_seconds, 2.0);
}

#[test]
fn a_robot_below_the_low_limit_does_not_leave_its_parking_trip_for_a_job() {
    let mut rig = charging(&[], 1);
    let trip = Robot {
        work: Work::Parking {
            from: A,
            ahead: vec![],
            arrival: None,
        },
        ..low(A, 20.0)
    };
    rig.jobs(&[job(1, B, C)]);
    rig.dispatch(&[trip]);
    assert!(rig.script.extended.is_empty());
    assert_eq!(rig.pending(), [1]);
}

#[test]
fn a_refused_charging_trip_counts_and_frees_the_charger() {
    let mut rig = charging(&[("charger", E)], 1);
    rig.script.refuse.insert(0);
    rig.dispatch(&[low(D, 20.0)]);
    assert_eq!(rig.dispatcher.counts().charge_trips, 1);
    rig.dispatch(&[low(D, 20.0)]);
    assert_eq!(rig.script.given.len(), 2, "it is tried again");
    assert_eq!(rig.dispatcher.counts().charge_trips, 2);
}

#[test]
fn a_charging_trip_the_controller_cannot_give_leaves_the_charger_free() {
    let mut rig = charging(&[("charger", E)], 2);
    rig.script.invalid.insert(0);
    rig.dispatch(&[low(D, 20.0), low(A, 20.0)]);
    assert_eq!(
        rig.script.summary(&rig.graph),
        [(0, Purpose::Charge, E), (1, Purpose::Charge, E)]
    );
}

#[test]
fn idle_robots_park_the_nearest_pair_first() {
    let mut rig = Rig::new(line(&[("Parking", E)]), 2);
    assert_eq!(rig.dispatcher.counts().parking_places, 1);
    rig.dispatch(&[at(A), at(D)]);
    assert_eq!(rig.script.summary(&rig.graph), [(1, Purpose::Park, E)]);
    let task = &rig.script.given[0].1;
    assert_eq!((task.job.id, task.job.pickup, task.job.drop), (0, D, E));
    let counts = rig.dispatcher.counts();
    assert_eq!((counts.park_trips, counts.no_free_parking), (1, 1));
}

#[test]
fn a_robot_without_a_free_place_is_counted_once_until_it_gets_a_task() {
    let mut rig = Rig::new(line(&[("parking", E)]), 2);
    // a robot stands on the place: it is not free, and that robot is parked
    rig.dispatch(&[at(A), at(E)]);
    rig.dispatch(&[at(A), at(E)]);
    assert!(rig.script.given.is_empty());
    assert_eq!(rig.dispatcher.counts().no_free_parking, 1);
    // a job (robot 0 is the nearer one), after it counted again
    rig.jobs(&[job(1, A, B)]);
    rig.dispatch(&[at(A), at(E)]);
    assert_eq!(rig.script.summary(&rig.graph), [(0, Purpose::Job, B)]);
    rig.dispatch(&[at(A), at(E)]);
    assert_eq!(rig.dispatcher.counts().no_free_parking, 2);
}

#[test]
fn a_held_place_is_not_free() {
    let mut rig = Rig::new(line(&[("parking", E)]), 1);
    rig.script.held.insert(E, 3);
    rig.dispatch(&[at(A)]);
    assert!(rig.script.given.is_empty());
    assert_eq!(rig.dispatcher.counts().no_free_parking, 1);
}

#[test]
fn a_robot_that_refused_a_parking_trip_is_not_sent_again_until_it_had_a_job() {
    let mut rig = Rig::new(line(&[("parking", E)]), 1);
    rig.script.refuse.insert(0);
    rig.dispatch(&[at(A)]);
    assert_eq!(rig.dispatcher.counts().park_trips, 1, "refused ones count");
    rig.dispatch(&[at(A)]);
    assert_eq!(rig.script.given.len(), 1);

    rig.script.refuse.clear();
    rig.jobs(&[job(1, A, B)]);
    rig.dispatch(&[at(A)]);
    rig.dispatch(&[at(B)]);
    assert_eq!(
        rig.script.summary(&rig.graph)[1..],
        [(0, Purpose::Job, B), (0, Purpose::Park, E)]
    );
}

/// A robot at C on its way to a parking place, released up to D.
fn on_a_trip(rig: &Rig) -> Robot {
    Robot {
        work: Work::Parking {
            from: D,
            ahead: vec![edge(&rig.graph, C, D)],
            arrival: Some(edge(&rig.graph, C, D)),
        },
        ..at(C)
    }
}

#[test]
fn a_job_can_take_the_place_a_route_has_just_claimed() {
    // the job ends on the one parking place; the route claims it, so the robot
    // left over is not sent there in the same dispatch
    for claims in [true, false] {
        let mut rig = Rig::new(line(&[("parking", E)]), 2);
        rig.script.claim_goals = claims;
        rig.jobs(&[job(1, D, E)]);
        rig.dispatch(&[at(D), at(A)]);
        let parked = rig.script.given.iter().any(|(r, _)| *r == 1);
        assert_eq!(parked, !claims);
    }
}

#[test]
fn a_refused_extension_takes_the_job_and_frees_the_place_at_once() {
    let mut rig = Rig::new(line(&[("parking", E)]), 2);
    // robot 0 is sent to the place
    rig.dispatch(&[at(C), busy(A)]);
    assert_eq!(rig.script.summary(&rig.graph), [(0, Purpose::Park, E)]);
    // on the way it is given a job, which it refuses; robot 1 is not ready
    let trip = on_a_trip(&rig);
    let unready = Robot {
        ready: false,
        ..at(A)
    };
    rig.script.clear();
    rig.script.refuse.insert(0);
    rig.jobs(&[job(1, E, A)]);
    rig.dispatch(&[trip, unready]);
    assert_eq!(rig.script.extended.len(), 1);
    assert!(
        rig.pending().is_empty(),
        "a refused job is not queued again"
    );
    // the place is free for the other robot
    rig.script.clear();
    rig.dispatch(&[busy(D), at(A)]);
    assert_eq!(rig.script.summary(&rig.graph), [(1, Purpose::Park, E)]);
}

#[test]
fn an_extension_clears_the_memory_of_a_refused_parking_trip() {
    let mut rig = Rig::new(line(&[("parking", E)]), 1);
    rig.dispatcher.given_up(0, Purpose::Park);
    rig.dispatch(&[at(A)]);
    assert!(rig.script.given.is_empty(), "it refused: not sent again");
    let trip = on_a_trip(&rig);
    rig.jobs(&[job(1, E, A)]);
    rig.dispatch(&[trip]);
    assert_eq!(rig.script.extended.len(), 1);
    rig.dispatch(&[at(A)]);
    assert_eq!(rig.script.summary(&rig.graph), [(0, Purpose::Park, E)]);
}

#[test]
fn an_arrived_or_given_up_trip_frees_its_place() {
    for arrived in [true, false] {
        let mut rig = Rig::new(line(&[("parking", E)]), 2);
        rig.dispatch(&[at(C), busy(A)]);
        rig.script.clear();
        rig.dispatch(&[busy(C), at(A)]);
        assert!(rig.script.given.is_empty(), "robot 0 is on its way there");
        if arrived {
            rig.dispatcher.arrived(0, Purpose::Park);
        } else {
            rig.dispatcher.given_up(0, Purpose::Park);
        }
        rig.dispatch(&[busy(C), at(A)]);
        assert_eq!(rig.script.summary(&rig.graph), [(1, Purpose::Park, E)]);
    }
}

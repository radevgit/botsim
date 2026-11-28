//! Which robot gets which job.

use botsim_dispatcher::{Dispatcher, Purpose, Robot, Work};

mod common;
use common::*;

#[test]
fn a_job_goes_to_the_nearest_robot_ties_to_the_lower_number() {
    // A and E are as far from C: the robot with the lower number gets the job
    let mut rig = Rig::new(line(&[]), 2);
    rig.jobs(&[job(1, C, D)]);
    rig.dispatch(&[at(E), at(A)]);
    let (robot, task) = &rig.script.given[0];
    assert_eq!(*robot, 0);
    assert_eq!(task.job, job(1, C, D));
    assert_eq!(
        (task.purpose, task.from, task.pickup_at),
        (Purpose::Job, E, 2)
    );
    let g = &rig.graph;
    assert_eq!(
        task.edges,
        [edge(g, E, D), edge(g, D, C), edge(g, C, D)],
        "to the pickup, then to the drop"
    );
    assert_eq!(rig.script.given.len(), 1);
    assert!(rig.pending().is_empty());

    // the nearer robot, whatever its number
    let mut rig = Rig::new(line(&[]), 2);
    rig.jobs(&[job(1, C, D)]);
    rig.dispatch(&[at(A), at(D)]);
    assert_eq!(rig.script.summary(&rig.graph), [(1, Purpose::Job, D)]);
}

#[test]
fn the_oldest_job_goes_first_and_one_robot_gets_one_job() {
    let mut rig = Rig::new(line(&[]), 1);
    rig.jobs(&[job(1, E, D), job(2, A, B)]);
    // (the second job is nearer)
    rig.dispatch(&[at(A)]);
    assert_eq!(rig.script.given[0].1.job.id, 1);
    assert_eq!(rig.script.given.len(), 1);
    assert_eq!(rig.pending(), [2]);
}

#[test]
fn a_job_nobody_can_do_does_not_block_the_jobs_behind_it() {
    let mut rig = Rig::new(line(&[]), 2);
    // no route to X (job 1: its pickup; job 2: its drop)
    rig.jobs(&[job(1, X, A), job(2, A, X), job(3, B, C)]);
    rig.dispatch(&[at(A), at(E)]);
    assert_eq!(rig.script.given.len(), 1);
    assert_eq!(rig.script.given[0].1.job.id, 3);
    assert_eq!(rig.pending(), [1, 2], "they wait, in their order");
}

#[test]
fn a_job_whose_pickup_is_where_the_robot_stands_picks_at_once() {
    let mut rig = Rig::new(line(&[]), 1);
    rig.jobs(&[job(1, B, B)]);
    rig.dispatch(&[at(B)]);
    let task = &rig.script.given[0].1;
    assert!(task.edges.is_empty());
    assert_eq!(task.pickup_at, 0);
}

#[test]
fn busy_and_unready_robots_and_robots_on_no_node_get_nothing() {
    let mut rig = Rig::new(line(&[]), 3);
    rig.jobs(&[job(1, A, B)]);
    let unready = Robot {
        ready: false,
        ..at(A)
    };
    let nowhere = Robot {
        node: None,
        ..at(A)
    };
    rig.dispatch(&[busy(A), unready, nowhere]);
    assert!(rig.script.given.is_empty());
    assert_eq!(rig.pending(), [1]);
}

/// A robot at A, on its way to a parking place, released up to C.
fn on_a_trip(rig: &Rig) -> Robot {
    let g = &rig.graph;
    Robot {
        work: Work::Parking {
            from: C,
            ahead: vec![edge(g, A, B), edge(g, B, C)],
            arrival: Some(edge(g, B, C)),
        },
        ..at(A)
    }
}

#[test]
fn a_job_extends_a_parking_trip_from_its_last_released_node() {
    let mut rig = Rig::new(line(&[]), 1);
    let trip = on_a_trip(&rig);
    rig.jobs(&[job(1, D, E)]);
    rig.dispatch(&[trip]);
    assert!(rig.script.given.is_empty());
    let (robot, task) = &rig.script.extended[0];
    assert_eq!(*robot, 0);
    assert_eq!(
        (task.purpose, task.from, task.pickup_at),
        (Purpose::Job, C, 1)
    );
    let g = &rig.graph;
    assert_eq!(task.edges, [edge(g, C, D), edge(g, D, E)]);
    assert!(rig.pending().is_empty());
}

#[test]
fn the_way_still_to_drive_on_a_trip_counts_in_its_distance() {
    // C and E are both 10 m from the pickup D, but the robot on the trip has 20 m
    // to drive before it is at C: without them it would win the tie
    let mut rig = Rig::new(line(&[]), 2);
    let trip = on_a_trip(&rig);
    rig.jobs(&[job(1, D, E)]);
    rig.dispatch(&[trip.clone(), at(E)]);
    assert_eq!(rig.script.summary(&rig.graph), [(1, Purpose::Job, E)]);
    assert!(rig.script.extended.is_empty());
}

#[test]
fn a_job_at_the_end_of_the_released_part_does_not_extend_the_trip() {
    // an order update does not run the actions of its first node
    let mut rig = Rig::new(line(&[]), 1);
    let trip = on_a_trip(&rig);
    rig.jobs(&[job(1, C, D)]);
    rig.dispatch(&[trip]);
    assert!(rig.script.extended.is_empty() && rig.script.given.is_empty());
    assert_eq!(rig.pending(), [1]);
}

#[test]
fn a_refused_job_is_gone_and_the_robot_is_not_asked_again() {
    let mut rig = Rig::new(line(&[]), 1);
    rig.script.refuse.insert(0);
    rig.jobs(&[job(1, A, B), job(2, A, B)]);
    rig.dispatch(&[at(A)]);
    assert_eq!(rig.script.given.len(), 1, "one try per robot and dispatch");
    assert_eq!(rig.pending(), [2]);
}

#[test]
fn a_job_the_controller_cannot_give_stays_queued() {
    let mut rig = Rig::new(line(&[]), 1);
    rig.script.invalid.insert(0);
    rig.jobs(&[job(1, A, B), job(2, A, B)]);
    rig.dispatch(&[at(A)]);
    assert_eq!(rig.pending(), [1, 2]);

    // the same for an extension
    let trip = on_a_trip(&rig);
    rig.script.clear();
    rig.dispatch(&[trip]);
    assert_eq!(rig.script.extended.len(), 1);
    assert_eq!(rig.pending(), [1, 2]);
}

#[test]
fn the_route_is_found_with_the_cost_the_controller_names() {
    // two ways from A to D: straight on, or over the top
    let g = graph(
        &[
            ("A", 0.0, 0.0),
            ("B", 10.0, 0.0),
            ("C", 10.0, 10.0),
            ("D", 20.0, 0.0),
        ],
        &[("A", "B"), ("B", "D"), ("A", "C"), ("C", "D")],
    );
    let straight = [edge(&g, 0, 1), edge(&g, 1, 3)];
    let over = [edge(&g, 0, 2), edge(&g, 2, 3)];
    let mut rig = Rig::new(g, 1);
    rig.jobs(&[job(1, 0, 3)]);
    rig.dispatch(&[at(0)]);
    assert_eq!(rig.script.given[0].1.edges, straight);

    // the straight way is made dear
    let mut cost = vec![10.0; rig.graph.edges.len()];
    cost[straight[0]] = 1000.0;
    rig.script.costs.insert(0, cost);
    rig.script.clear();
    rig.jobs(&[job(2, 0, 3)]);
    rig.dispatch(&[at(0)]);
    assert_eq!(rig.script.given[0].1.edges, over);
}

#[test]
fn what_does_not_fit_the_layout_is_left_alone() {
    let mut rig = Rig::new(line(&[("parking", E)]), 3);
    let nodes = rig.graph.nodes.len();
    let edges = rig.graph.edges.len();
    // jobs outside the layout stay queued and block nothing
    rig.jobs(&[job(1, nodes, A), job(2, A, nodes + 7), job(3, A, B)]);
    let wrong_type = Robot {
        type_idx: 1,
        ..at(A)
    };
    let off_the_graph = at(nodes);
    let wrong_trip = Robot {
        work: Work::Parking {
            from: C,
            ahead: vec![edges],
            arrival: None,
        },
        ..at(A)
    };
    rig.dispatch(&[wrong_type, off_the_graph, wrong_trip]);
    assert!(rig.script.given.is_empty() && rig.script.extended.is_empty());
    assert_eq!(rig.pending(), [1, 2, 3]);

    // more robots than the dispatcher was started with: the extra one gets nothing
    let mut rig = Rig::new(line(&[]), 1);
    rig.jobs(&[job(1, A, B), job(2, A, B)]);
    rig.dispatch(&[at(A), at(A)]);
    assert_eq!(rig.script.summary(&rig.graph), [(0, Purpose::Job, B)]);
    rig.dispatcher.arrived(7, Purpose::Charge);
    rig.dispatcher.given_up(7, Purpose::Park);
    assert!(!rig.dispatcher.stop_charging(7, 100.0));
}

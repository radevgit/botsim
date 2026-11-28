//! Robots riding a lift (S46), on the official two-level layout: floor 1
//! (`Map_Z-Level_1`) has N1 and N2, floor 2 (`Map_Z-Level_2`) has N102 and N101;
//! the layout edges N2→N102 and N102→N2 are the rides.

use botsim_core::actions::ActionTimes;
use botsim_core::drive::Phase;
use botsim_core::lift::{DoorState, LiftSpec, LiftStop};
use botsim_core::observer::{Observer, RobotStatus, Snapshot};
use botsim_core::robot::Reception;
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::instant_actions::InstantActions;
use vda5050::order::Order;
use vdma_lif::Lif;

mod vda_port;
use vda_port::VdaWorld;

const DT: f64 = 0.05;
const LEVEL_1: &str = "Map_Z-Level_1";
const LEVEL_2: &str = "Map_Z-Level_2";

fn spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn id(serial: &str) -> RobotId {
    RobotId::new("ACME", serial)
}

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/two_levels.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

/// One lift "L1" with the stops N2 (floor 1) and N102 (floor 2, door `rear_at_top`
/// if true, else the front door); robots (serial, node).
fn world(doors: &[&str], rear_at_top: bool, robots: &[(&str, &str)]) -> World {
    let g = graph();
    let stop = |name: &str, floor, door| LiftStop {
        node: g.node_idx(name).unwrap(),
        floor,
        door,
        waiting: None,
    };
    let stops = vec![stop("N2", 0, 0), stop("N102", 1, rear_at_top as usize)];
    let floors = vec![g.map_idx(LEVEL_1).unwrap(), g.map_idx(LEVEL_2).unwrap()];
    let nodes: Vec<_> = robots.iter().map(|(_, n)| g.node_idx(n).unwrap()).collect();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_lift(LiftSpec {
        id: "L1".into(),
        floors,
        doors: doors.iter().map(|d| d.to_string()).collect(),
        stops,
        start_floor: 0,
        travel_seconds_per_floor: 4.0,
        door_open_seconds: 2.0,
        door_close_seconds: 2.0,
        door_hold_seconds: None,
    })
    .unwrap();
    for ((serial, _), n) in robots.iter().zip(nodes) {
        w.add_robot(id(serial), spec(), ActionTimes::new(1.0), n, 0.0)
            .unwrap();
    }
    w
}

/// An order along `nodes` (each pair joined by the edge `A-B` named after its
/// nodes); the first edge named `N2-N102` or `N102-N2` is a ride.
fn order(serial: &str, order_id: &str, nodes: &[&str]) -> Order {
    let ns: Vec<String> = nodes
        .iter()
        .enumerate()
        .map(|(i, n)| {
            format!(
                r#"{{"nodeId":"{n}","sequenceId":{},"released":true,"actions":[]}}"#,
                i * 2
            )
        })
        .collect();
    let es: Vec<String> = nodes
        .windows(2)
        .enumerate()
        .map(|(i, p)| {
            format!(
                r#"{{"edgeId":"{}-{}","sequenceId":{},"released":true,"actions":[]}}"#,
                p[0],
                p[1],
                i * 2 + 1
            )
        })
        .collect();
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"{serial}","orderId":"{order_id}","orderUpdateId":0,
        "nodes":[{}],"edges":[{}]}}"#,
        ns.join(","),
        es.join(",")
    );
    serde_json::from_str(&json).unwrap()
}

fn send(w: &mut World, o: &Order) {
    assert_eq!(w.send_order(o), Ok(Reception::Accepted));
}

fn phase(w: &World, serial: &str) -> Phase {
    w.phase(&id(serial)).unwrap()
}

/// Step until `done` holds; panics after 10 simulated minutes.
fn until(w: &mut World, mut done: impl FnMut(&World) -> bool) {
    for _ in 0..12_000 {
        if done(w) {
            return;
        }
        w.step();
    }
    panic!("never happened");
}

fn door(w: &World) -> DoorState {
    w.lift_status("L1").unwrap().door
}

fn map_of(w: &World, serial: &str) -> String {
    let p = w.pose(&id(serial)).unwrap();
    w.graph().maps[p.map].clone()
}

#[test]
fn a_robot_waits_at_the_lift_until_the_cabin_is_there_with_the_door_open() {
    let mut w = world(&["front"], false, &[("R-1", "N1")]);
    send(&mut w, &order("R-1", "o1", &["N1", "N2", "N102"]));
    until(&mut w, |w| phase(w, "R-1") == Phase::WaitingForLift);
    let p = w.pose(&id("R-1")).unwrap();
    assert_eq!((p.x, p.y), (11.0, 0.0));

    // the cabin is at floor 1 but nobody has called: the door is closed
    for _ in 0..1200 {
        w.step();
    }
    assert_eq!(phase(&w, "R-1"), Phase::WaitingForLift);
    assert_eq!(w.lift_rider("L1").unwrap(), None);

    // calling the lift opens the door; the robot gets in
    w.call_lift("L1", LEVEL_1).unwrap();
    until(&mut w, |w| phase(w, "R-1") == Phase::Riding);
    assert_eq!(door(&w), DoorState::Open);
    assert_eq!(w.lift_rider("L1").unwrap(), Some(&id("R-1")));
}

#[test]
fn the_robot_rides_to_the_other_floor_and_drives_on() {
    let mut w = world(&["front"], false, &[("R-1", "N2")]);
    send(&mut w, &order("R-1", "o1", &["N2", "N102", "N101"]));
    // at its stop already, the cabin is here: call it
    w.call_lift("L1", LEVEL_1).unwrap();
    until(&mut w, |w| phase(w, "R-1") == Phase::Riding);

    // while it rides it is still reported on its old floor, and the state does
    // not say it moves; the order is not done
    assert_eq!(map_of(&w, "R-1"), LEVEL_1);
    let s = w.state(&id("R-1")).unwrap();
    assert_eq!(s.last_node_id, "N2");
    assert!(!s.driving);
    assert!(!s.edge_states.is_empty());

    // the door stays open until released; then the cabin goes up when called
    w.call_lift("L1", LEVEL_2).unwrap();
    for _ in 0..400 {
        w.step();
    }
    assert_eq!(phase(&w, "R-1"), Phase::Riding, "the door is still open");
    w.release_lift("L1").unwrap();
    until(&mut w, |w| phase(w, "R-1") != Phase::Riding);

    // out at N102 on the upper floor, door open there
    let p = w.pose(&id("R-1")).unwrap();
    assert_eq!(map_of(&w, "R-1"), LEVEL_2);
    assert_eq!((p.x, p.y), (12.4, 3.4));
    let st = w.lift_status("L1").unwrap();
    assert_eq!((st.floor, st.door), (1, DoorState::Open));
    assert_eq!(w.lift_rider("L1").unwrap(), None);
    assert_eq!(w.state(&id("R-1")).unwrap().last_node_id, "N102");

    // and on to N101
    until(&mut w, |w| phase(w, "R-1") == Phase::Finished);
    let p = w.pose(&id("R-1")).unwrap();
    assert_eq!((p.x, p.y), (12.0, 3.4));
    assert_eq!(w.state(&id("R-1")).unwrap().last_node_id, "N101");
    assert!(w.collisions().is_empty());
}

#[test]
fn the_robot_gets_out_only_when_the_door_of_its_stop_is_open() {
    // the stop on the upper floor is at the rear door
    let mut w = world(&["front", "rear"], true, &[("R-1", "N2")]);
    send(&mut w, &order("R-1", "o1", &["N2", "N102"]));
    w.call_lift("L1", LEVEL_1).unwrap();
    until(&mut w, |w| phase(w, "R-1") == Phase::Riding);

    // the front door opens at the top: wrong door, the robot stays in
    w.release_lift("L1").unwrap();
    w.call_lift_door("L1", LEVEL_2, "front").unwrap();
    until(&mut w, |w| door(w) == DoorState::Open);
    for _ in 0..400 {
        w.step();
    }
    assert_eq!(phase(&w, "R-1"), Phase::Riding);

    // the rear door: out
    w.release_lift("L1").unwrap();
    w.call_lift_door("L1", LEVEL_2, "rear").unwrap();
    until(&mut w, |w| phase(w, "R-1") != Phase::Riding);
    assert_eq!(map_of(&w, "R-1"), LEVEL_2);
}

#[test]
fn only_one_robot_rides_at_a_time_and_the_one_in_the_cabin_touches_nobody() {
    let mut w = world(&["front"], false, &[("R-1", "N2"), ("R-2", "N1")]);
    send(&mut w, &order("R-1", "o1", &["N2", "N102"]));
    send(&mut w, &order("R-2", "o2", &["N1", "N2", "N102"]));
    w.call_lift("L1", LEVEL_1).unwrap();
    until(&mut w, |w| phase(w, "R-1") == Phase::Riding);

    // R-2 arrives at N2, where R-1 got in; the door is open but the cabin is taken
    until(&mut w, |w| phase(w, "R-2") == Phase::WaitingForLift);
    for _ in 0..400 {
        w.step();
    }
    assert_eq!(door(&w), DoorState::Open);
    assert_eq!(phase(&w, "R-1"), Phase::Riding);
    assert_eq!(phase(&w, "R-2"), Phase::WaitingForLift);
    assert!(w.collisions().is_empty(), "the rider is not on the floor");

    // up with R-1; then the cabin comes back for R-2
    w.release_lift("L1").unwrap();
    w.call_lift("L1", LEVEL_2).unwrap();
    until(&mut w, |w| phase(w, "R-1") == Phase::Finished);
    assert_eq!(phase(&w, "R-2"), Phase::WaitingForLift);
    w.release_lift("L1").unwrap();
    w.call_lift("L1", LEVEL_1).unwrap();
    until(&mut w, |w| phase(w, "R-2") == Phase::Riding);
    assert_eq!(w.lift_rider("L1").unwrap(), Some(&id("R-2")));
}

#[test]
fn a_ride_edge_without_a_lift_stop_is_refused_as_before() {
    // a lift without stops: nobody can ride
    let g = graph();
    let floors = vec![g.map_idx(LEVEL_1).unwrap(), g.map_idx(LEVEL_2).unwrap()];
    let n2 = g.node_idx("N2").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_lift(LiftSpec {
        id: "L1".into(),
        floors,
        doors: vec!["front".into()],
        stops: vec![],
        start_floor: 0,
        travel_seconds_per_floor: 4.0,
        door_open_seconds: 2.0,
        door_close_seconds: 2.0,
        door_hold_seconds: None,
    })
    .unwrap();
    w.add_robot(id("R-1"), spec(), ActionTimes::new(1.0), n2, 0.0)
        .unwrap();
    let r = w.send_order(&order("R-1", "o1", &["N2", "N102"])).unwrap();
    assert!(matches!(r, Reception::Rejected(_)), "{r:?}");
}

#[test]
fn wrong_stops_are_refused() {
    let g = graph();
    let floors = vec![g.map_idx(LEVEL_1).unwrap(), g.map_idx(LEVEL_2).unwrap()];
    let (n1, n102) = (g.node_idx("N1").unwrap(), g.node_idx("N102").unwrap());
    let spec_with = |stops: Vec<LiftStop>, id: &str| LiftSpec {
        id: id.into(),
        floors: floors.clone(),
        doors: vec!["front".into()],
        stops,
        start_floor: 0,
        travel_seconds_per_floor: 4.0,
        door_open_seconds: 2.0,
        door_close_seconds: 2.0,
        door_hold_seconds: None,
    };
    let mut w = World::new(g, DT, 0).unwrap();
    let stop = |node, floor, door| LiftStop {
        node,
        floor,
        door,
        waiting: None,
    };
    // N102 is not on floor 1, a door that does not exist, a floor that does not exist
    assert!(w.add_lift(spec_with(vec![stop(n102, 0, 0)], "A")).is_err());
    assert!(w.add_lift(spec_with(vec![stop(n1, 0, 1)], "A")).is_err());
    assert!(w.add_lift(spec_with(vec![stop(n1, 2, 0)], "A")).is_err());
    assert!(w.add_lift(spec_with(vec![stop(999, 0, 0)], "A")).is_err());
    // the same node twice, and a node that belongs to another lift
    assert!(
        w.add_lift(spec_with(vec![stop(n1, 0, 0), stop(n1, 0, 0)], "A"))
            .is_err()
    );
    // a waiting node: not the stop itself, not on another floor, not a missing node
    let waiting = |w: usize| LiftStop {
        waiting: Some(w),
        ..stop(n1, 0, 0)
    };
    assert!(w.add_lift(spec_with(vec![waiting(n1)], "A")).is_err());
    assert!(w.add_lift(spec_with(vec![waiting(n102)], "A")).is_err());
    assert!(w.add_lift(spec_with(vec![waiting(999)], "A")).is_err());
    assert!(w.add_lift(spec_with(vec![stop(n1, 0, 0)], "A")).is_ok());
    assert!(w.add_lift(spec_with(vec![stop(n1, 0, 0)], "B")).is_err());
}

#[test]
fn cancelling_in_the_cabin_finishes_the_ride_and_cancelling_before_it_stays() {
    let cancel = |serial: &str| -> InstantActions {
        serde_json::from_str(&format!(
            r#"{{"headerId":2,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
            "manufacturer":"ACME","serialNumber":"{serial}",
            "actions":[{{"actionType":"cancelOrder","actionId":"c1","blockingType":"NONE"}}]}}"#
        ))
        .unwrap()
    };

    // waiting at the entrance: the order is over at once, the robot stays on floor 1
    let mut w = world(&["front"], false, &[("R-1", "N2")]);
    send(&mut w, &order("R-1", "o1", &["N2", "N102", "N101"]));
    until(&mut w, |w| phase(w, "R-1") == Phase::WaitingForLift);
    w.send_instant_actions(&cancel("R-1")).unwrap();
    assert_eq!(phase(&w, "R-1"), Phase::Finished);
    assert_eq!(map_of(&w, "R-1"), LEVEL_1);

    // in the cabin: it cannot get out in between, so it rides to N102 and stops there
    let mut w = world(&["front"], false, &[("R-1", "N2")]);
    send(&mut w, &order("R-1", "o1", &["N2", "N102", "N101"]));
    w.call_lift("L1", LEVEL_1).unwrap();
    until(&mut w, |w| phase(w, "R-1") == Phase::Riding);
    w.send_instant_actions(&cancel("R-1")).unwrap();
    assert_eq!(phase(&w, "R-1"), Phase::Riding);
    w.release_lift("L1").unwrap();
    w.call_lift("L1", LEVEL_2).unwrap();
    until(&mut w, |w| phase(w, "R-1") == Phase::Finished);
    assert_eq!(map_of(&w, "R-1"), LEVEL_2);
    assert_eq!(w.state(&id("R-1")).unwrap().last_node_id, "N102");
}

#[test]
fn the_observer_sees_waiting_for_the_lift_and_riding() {
    use std::cell::RefCell;
    use std::rc::Rc;

    #[derive(Debug)]
    struct Recorder(Rc<RefCell<Vec<Snapshot>>>);
    impl Observer for Recorder {
        fn on_layout(&mut self, _: &Graph) {}
        fn on_step(&mut self, s: &Snapshot) {
            self.0.borrow_mut().push(s.clone());
        }
    }

    let mut w = world(&["front"], false, &[("R-1", "N2")]);
    let log = Rc::new(RefCell::new(Vec::new()));
    w.set_observer(Box::new(Recorder(log.clone())));
    send(&mut w, &order("R-1", "o1", &["N2", "N102"]));
    for _ in 0..100 {
        w.step();
    }
    w.call_lift("L1", LEVEL_1).unwrap();
    until(&mut w, |w| phase(w, "R-1") == Phase::Riding);
    w.step();
    let l = log.borrow();
    let status = |i: usize| l[i].robots[0].status;
    assert_eq!(status(50), RobotStatus::WaitingForLift);
    assert_eq!(status(l.len() - 1), RobotStatus::RidingLift);
}

#[test]
fn the_same_ride_gives_the_same_timeline() {
    let run = || {
        let mut w = world(&["front"], false, &[("R-1", "N1")]);
        send(&mut w, &order("R-1", "o1", &["N1", "N2", "N102", "N101"]));
        let mut log = Vec::new();
        for step in 0..4000 {
            if step == 400 {
                w.call_lift("L1", LEVEL_1).unwrap();
            }
            if step == 800 {
                w.release_lift("L1").unwrap();
                w.call_lift("L1", LEVEL_2).unwrap();
            }
            w.step();
            let p = w.pose(&id("R-1")).unwrap();
            log.push((p.map, p.x.to_bits(), p.y.to_bits(), phase(&w, "R-1")));
        }
        log
    };
    assert_eq!(run(), run());
}

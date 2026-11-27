use botsim_core::actions::ActionTimes;
use botsim_core::door::{Door, DoorError, DoorSpec};
use botsim_core::drive::Phase;
use botsim_core::errors::ErrorType;
use botsim_core::lift::DoorState;
use botsim_core::robot::Reception;
use botsim_core::world::{RobotId, World, WorldError};
use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

const DT: f64 = 0.05;

// ---- the door alone ----

fn spec(edges: Vec<usize>) -> DoorSpec {
    DoorSpec {
        id: "D1".into(),
        edges,
        passage_width: None,
        open_seconds: 2.0,
        close_seconds: 2.0,
        hold_seconds: None,
    }
}

fn door() -> Door {
    Door::new(spec(vec![0])).unwrap()
}

fn run(d: &mut Door, seconds: f64, occupied: bool) {
    for _ in 0..(seconds / DT).round() as u32 {
        d.step(DT, occupied);
    }
}

/// Steps until the state is `want`; returns the seconds it took.
fn until(d: &mut Door, want: DoorState, occupied: bool) -> f64 {
    for i in 0..100_000 {
        if d.state() == want {
            return i as f64 * DT;
        }
        d.step(DT, occupied);
    }
    panic!("never {want:?}");
}

#[test]
fn a_door_needs_edges_and_sensible_numbers() {
    let bad = |f: &dyn Fn(&mut DoorSpec)| {
        let mut s = spec(vec![0]);
        f(&mut s);
        Door::new(s).unwrap_err()
    };
    assert_eq!(bad(&|s| s.edges = vec![]), DoorError::BadEdges);
    assert_eq!(bad(&|s| s.edges = vec![1, 1]), DoorError::BadEdges);
    assert_eq!(
        bad(&|s| s.passage_width = Some(0.0)),
        DoorError::BadValue("passage_width")
    );
    assert_eq!(
        bad(&|s| s.open_seconds = -1.0),
        DoorError::BadValue("open_seconds")
    );
    assert_eq!(
        bad(&|s| s.close_seconds = f64::NAN),
        DoorError::BadValue("close_seconds")
    );
    assert_eq!(
        bad(&|s| s.hold_seconds = Some(0.0)),
        DoorError::BadValue("hold_seconds")
    );
}

#[test]
fn a_closed_door_stays_closed_until_it_is_asked_to_open() {
    let mut d = door();
    run(&mut d, 30.0, false);
    assert_eq!(d.state(), DoorState::Closed);
    d.open();
    d.step(DT, false);
    assert_eq!(d.state(), DoorState::Opening);
    let t = until(&mut d, DoorState::Open, false);
    assert!((t - 2.0).abs() < 0.15, "opening took {t} s");
    assert!(d.is_open());
    // and it stays open while the controller wants it
    run(&mut d, 60.0, false);
    assert!(d.is_open());
}

#[test]
fn release_closes_the_door_but_not_while_a_robot_is_in_it() {
    let mut d = door();
    d.open();
    until(&mut d, DoorState::Open, false);
    d.release();
    // a robot is on the edge: the door waits, however long
    run(&mut d, 30.0, true);
    assert_eq!(d.state(), DoorState::Open);
    // the robot has left: it closes
    d.step(DT, false);
    assert_eq!(d.state(), DoorState::Closing);
    let t = until(&mut d, DoorState::Closed, false);
    assert!((t - 2.0).abs() < 0.15, "closing took {t} s");
}

#[test]
fn opening_a_closing_door_waits_for_it_to_close_and_then_opens_again() {
    let mut d = door();
    d.open();
    until(&mut d, DoorState::Open, false);
    d.release();
    d.step(DT, false);
    assert_eq!(d.state(), DoorState::Closing);
    d.open();
    until(&mut d, DoorState::Closed, false);
    d.step(DT, false);
    assert_eq!(d.state(), DoorState::Opening);
}

#[test]
fn a_hold_time_releases_the_door_by_itself() {
    let mut s = spec(vec![0]);
    s.hold_seconds = Some(10.0);
    let mut d = Door::new(s).unwrap();
    d.open();
    until(&mut d, DoorState::Open, false);
    let t = until(&mut d, DoorState::Closing, false);
    assert!((t - 10.0).abs() < 0.2, "open for {t} s");
    assert!(!d.status().wanted_open);
}

#[test]
fn zero_times_make_an_instant_door() {
    let mut s = spec(vec![0]);
    s.open_seconds = 0.0;
    s.close_seconds = 0.0;
    let mut d = Door::new(s).unwrap();
    d.open();
    d.step(DT, false);
    assert_eq!(d.state(), DoorState::Open);
    d.release();
    d.step(DT, false);
    assert_eq!(d.state(), DoorState::Closed);
}

// ---- a door in the world ----

fn robot_spec() -> RobotSpec {
    // 1.2 m long, 0.8 m wide
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn id() -> RobotId {
    RobotId::new("ACME", "R-1")
}

/// Layout A(0,0) – B(10,0) – C(10,5); robot R-1 on A; a door on the edge A-B.
fn world(passage_width: Option<f64>) -> World {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    let g = Graph::from_lif(&lif).unwrap();
    let (a, ab) = (g.node_idx("A").unwrap(), g.edge_idx("A-B").unwrap());
    let mut w = World::new(g, DT, 0).unwrap();
    let mut s = spec(vec![ab]);
    s.passage_width = passage_width;
    w.add_door(s).unwrap();
    // added after the door: robots added later know the door too
    w.add_robot(id(), robot_spec(), ActionTimes::new(1.0), a, 0.0)
        .unwrap();
    w
}

fn order(nodes: &[(&str, &str)]) -> Order {
    // (node, edge to it); the first edge is ""
    let ns: Vec<String> = nodes
        .iter()
        .enumerate()
        .map(|(i, (n, _))| {
            format!(
                r#"{{"nodeId":"{n}","sequenceId":{},"released":true,"actions":[]}}"#,
                i * 2
            )
        })
        .collect();
    let es: Vec<String> = nodes
        .iter()
        .enumerate()
        .skip(1)
        .map(|(i, (_, e))| {
            format!(
                r#"{{"edgeId":"{e}","sequenceId":{},"released":true,"actions":[]}}"#,
                i * 2 - 1
            )
        })
        .collect();
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{}],"edges":[{}]}}"#,
        ns.join(","),
        es.join(",")
    );
    serde_json::from_str(&json).unwrap()
}

fn a_to_b() -> Order {
    order(&[("A", ""), ("B", "A-B")])
}

fn phase(w: &World) -> Phase {
    w.phase(&id()).unwrap()
}

#[test]
fn a_robot_waits_in_front_of_a_closed_door_and_goes_when_it_is_open() {
    let mut w = world(None);
    assert_eq!(w.send_order(&a_to_b()), Ok(Reception::Accepted));
    for _ in 0..400 {
        w.step();
    }
    // closed door: the robot stands on A (after turning), it has not moved
    assert_eq!(phase(&w), Phase::WaitingForClearance);
    assert_eq!(w.pose(&id()).unwrap().x, 0.0);
    assert_eq!(w.door_status("D1").unwrap().state, DoorState::Closed);

    // a door that is opening is not open yet
    w.open_door("D1").unwrap();
    for _ in 0..20 {
        w.step();
    }
    assert_eq!(w.door_status("D1").unwrap().state, DoorState::Opening);
    assert_eq!(w.pose(&id()).unwrap().x, 0.0);

    // open: the robot drives through to B
    for _ in 0..600 {
        w.step();
    }
    assert_eq!(phase(&w), Phase::Finished);
    assert!((w.pose(&id()).unwrap().x - 10.0).abs() < 1e-9);
}

#[test]
fn the_door_does_not_close_on_a_robot_that_is_passing() {
    let mut w = world(None);
    w.open_door("D1").unwrap();
    assert_eq!(w.send_order(&a_to_b()), Ok(Reception::Accepted));
    // wait until the robot is on the edge, then release the door
    for _ in 0..2000 {
        w.step();
        if w.pose(&id()).unwrap().x > 2.0 {
            break;
        }
    }
    assert!(w.pose(&id()).unwrap().x > 2.0);
    w.release_door("D1").unwrap();
    for _ in 0..40 {
        w.step();
        assert_eq!(
            w.door_status("D1").unwrap().state,
            DoorState::Open,
            "closed on a robot at x = {}",
            w.pose(&id()).unwrap().x
        );
    }
    // the robot arrives; then the door closes
    for _ in 0..1000 {
        w.step();
    }
    assert_eq!(phase(&w), Phase::Finished);
    assert!((w.pose(&id()).unwrap().x - 10.0).abs() < 1e-9);
    assert_eq!(w.door_status("D1").unwrap().state, DoorState::Closed);
}

#[test]
fn a_door_that_is_released_before_the_robot_starts_closes_in_its_face() {
    // the robot is waiting for the door to open, the controller opens and
    // releases it at once: the robot has not started, so the door closes again
    let mut w = world(None);
    assert_eq!(w.send_order(&a_to_b()), Ok(Reception::Accepted));
    for _ in 0..100 {
        w.step();
    }
    w.open_door("D1").unwrap();
    w.release_door("D1").unwrap();
    for _ in 0..600 {
        w.step();
    }
    assert_eq!(w.pose(&id()).unwrap().x, 0.0);
    assert_eq!(w.door_status("D1").unwrap().state, DoorState::Closed);
}

#[test]
fn a_robot_wider_than_the_door_cannot_pass() {
    // the robot is 0.8 m wide
    let mut w = world(Some(0.7));
    let r = w.send_order(&a_to_b()).unwrap();
    assert_eq!(r, Reception::Rejected(ErrorType::NoRouteToTarget));
    // it is not a rejection of the door but of this robot: a wider door takes it
    let mut w = world(Some(0.9));
    assert_eq!(w.send_order(&a_to_b()), Ok(Reception::Accepted));
}

#[test]
fn edges_without_a_door_are_not_affected() {
    let mut w = world(None);
    // robot on A: A-B has the door; B-C has none; send the robot from A to B
    // with the door open, then on to C without any further door command
    w.open_door("D1").unwrap();
    let o = order(&[("A", ""), ("B", "A-B"), ("C", "B-C")]);
    assert_eq!(w.send_order(&o), Ok(Reception::Accepted));
    for _ in 0..2500 {
        w.step();
    }
    assert_eq!(phase(&w), Phase::Finished);
    let p = w.pose(&id()).unwrap();
    assert!((p.x - 10.0).abs() < 1e-9 && (p.y - 5.0).abs() < 1e-9);
}

#[test]
fn wrong_doors_and_commands_are_errors() {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    let g = Graph::from_lif(&lif).unwrap();
    let (ab, bc) = (g.edge_idx("A-B").unwrap(), g.edge_idx("B-C").unwrap());
    let mut w = World::new(g, DT, 0).unwrap();
    // an edge that does not exist
    assert!(matches!(
        w.add_door(spec(vec![99])),
        Err(WorldError::BadDoorEdge(_))
    ));
    assert!(matches!(
        w.add_door(spec(vec![])),
        Err(WorldError::Door(DoorError::BadEdges))
    ));
    w.add_door(spec(vec![ab])).unwrap();
    // the same id, and an edge another door has
    assert!(matches!(
        w.add_door(spec(vec![bc])),
        Err(WorldError::DuplicateDoor(_))
    ));
    let mut other = spec(vec![ab]);
    other.id = "D2".into();
    assert!(matches!(w.add_door(other), Err(WorldError::BadDoorEdge(_))));
    // commands for a door that does not exist
    assert!(w.open_door("D9").is_err());
    assert!(w.release_door("D9").is_err());
    assert!(w.door_status("D9").is_err());
}

#[test]
fn doors_appear_in_the_observer_snapshots() {
    let mut w = world(None);
    assert!(w.snapshot().doors.iter().all(|d| d.id == "D1"));
    w.open_door("D1").unwrap();
    for _ in 0..80 {
        w.step();
    }
    let s = w.snapshot();
    assert_eq!(s.doors.len(), 1);
    assert_eq!(s.doors[0].state, DoorState::Open);
    assert_eq!(s.doors[0].edges, vec![w.graph().edge_idx("A-B").unwrap()]);
}

#[test]
fn the_same_commands_give_the_same_timeline() {
    let timeline = || {
        let mut w = world(None);
        w.send_order(&a_to_b()).unwrap();
        let mut out = Vec::new();
        for step in 0..1200 {
            if step == 100 {
                w.open_door("D1").unwrap();
            }
            if step == 300 {
                w.release_door("D1").unwrap();
            }
            w.step();
            out.push((w.pose(&id()).unwrap().x.to_bits(), w.doors()));
        }
        out
    };
    assert_eq!(timeline(), timeline());
}

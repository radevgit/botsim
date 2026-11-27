//! The onboard safety stop (S54): a robot brakes when another robot is in its
//! protective field.

use botsim_core::actions::ActionTimes;
use botsim_core::collision::{overlap, place};
use botsim_core::drive::Phase;
use botsim_core::observer::RobotStatus;
use botsim_core::robot::Reception;
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

const DT: f64 = 0.05;

fn spec() -> RobotSpec {
    // 1.2 m long, 0.8 m wide, up to 1.5 m/s, braking 0.8 m/s²
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn id(serial: &str) -> RobotId {
    RobotId::new("ACME", serial)
}

/// Layout A(0,0) – B(10,0) – C(10,5); robots (serial, node).
fn world(robots: &[(&str, &str)], safety: bool) -> World {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    let g = Graph::from_lif(&lif).unwrap();
    let nodes: Vec<_> = robots.iter().map(|(_, n)| g.node_idx(n).unwrap()).collect();
    let mut w = World::new(g, DT, 0).unwrap();
    for ((serial, _), n) in robots.iter().zip(nodes) {
        w.add_robot(id(serial), spec(), ActionTimes::new(1.0), n, 0.0)
            .unwrap();
    }
    w.set_safety_stop(safety);
    w
}

/// An order for `serial` along `from` → `to` by `edge`.
fn go(w: &mut World, serial: &str, from: &str, edge: &str, to: &str, order_id: &str) {
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"{serial}","orderId":"{order_id}","orderUpdateId":0,
        "nodes":[{{"nodeId":"{from}","sequenceId":0,"released":true,"actions":[]}},
                 {{"nodeId":"{to}","sequenceId":2,"released":true,"actions":[]}}],
        "edges":[{{"edgeId":"{edge}","sequenceId":1,"released":true,"actions":[]}}]}}"#
    );
    let o: Order = serde_json::from_str(&json).unwrap();
    assert_eq!(w.send_order(&o), Ok(Reception::Accepted));
}

fn x(w: &World, serial: &str) -> f64 {
    w.pose(&id(serial)).unwrap().x
}

#[test]
fn a_robot_stops_before_a_robot_in_its_way_and_does_not_collide() {
    // R-2 stands on B (x = 10); R-1 drives from A towards it
    let mut w = world(&[("R-1", "A"), ("R-2", "B")], true);
    go(&mut w, "R-1", "A", "A-B", "B", "o1");
    let mut saw_obstructed = false;
    for _ in 0..1600 {
        w.step();
        saw_obstructed |= w.snapshot().robots[0].status == RobotStatus::Obstructed;
    }
    assert!(w.collisions().is_empty(), "{:?}", w.collisions());
    assert!(saw_obstructed);
    // one safety stop, although it lasts to the end (and the field may let go and
    // catch again while the robot brakes)
    assert_eq!(w.safety_stops(), 1);

    // both are 1.2 m long: R-1's front touches R-2's rear at x = 8.8; it stops before
    let p = x(&w, "R-1");
    assert!(p < 8.8 && p > 8.0, "stopped at x = {p}");
    // it stands still, reports a protective field violation and is not driving
    let s = w.state(&id("R-1")).unwrap();
    assert!(s.safety_state.field_violation);
    assert!(!s.driving);
    assert_eq!(w.phase(&id("R-1")), Ok(Phase::Driving));
    // and it still has its order (it did not finish)
    assert_ne!(s.last_node_id, "B");
}

#[test]
fn without_the_safety_stop_the_same_run_collides() {
    let mut w = world(&[("R-1", "A"), ("R-2", "B")], false);
    go(&mut w, "R-1", "A", "A-B", "B", "o1");
    for _ in 0..1600 {
        w.step();
    }
    assert_eq!(w.collisions().len(), 1);
    assert_eq!(w.safety_stops(), 0);
}

#[test]
fn the_robot_goes_on_when_the_way_is_clear_again() {
    // R-2 drives off from B to C after a while; R-1 follows and arrives on B
    let mut w = world(&[("R-1", "A"), ("R-2", "B")], true);
    go(&mut w, "R-1", "A", "A-B", "B", "o1");
    for _ in 0..800 {
        w.step();
    }
    assert!(x(&w, "R-1") < 8.8);
    go(&mut w, "R-2", "B", "B-C", "C", "o2");
    for _ in 0..1600 {
        w.step();
    }
    assert!((x(&w, "R-1") - 10.0).abs() < 1e-9, "x = {}", x(&w, "R-1"));
    assert_eq!(w.phase(&id("R-1")), Ok(Phase::Finished));
    assert!(w.collisions().is_empty(), "{:?}", w.collisions());
    assert!(!w.state(&id("R-1")).unwrap().safety_state.field_violation);
}

#[test]
fn a_robot_beside_the_lane_does_not_stop_the_robot() {
    // R-2 stands on C (10, 5): five metres from the lane A → B
    let mut w = world(&[("R-1", "A"), ("R-2", "C")], true);
    go(&mut w, "R-1", "A", "A-B", "B", "o1");
    for _ in 0..1600 {
        w.step();
    }
    assert_eq!(w.phase(&id("R-1")), Ok(Phase::Finished));
    assert!((x(&w, "R-1") - 10.0).abs() < 1e-9);
}

#[test]
fn the_field_stops_the_robot_while_a_body_is_in_it_but_not_behind_it() {
    // R-2 stands behind R-1 on the lane (R-1 on B, R-2 on A, R-1 drives away to C
    // along B → C: R-2 is not in the way)
    let mut w = world(&[("R-1", "B"), ("R-2", "A")], true);
    go(&mut w, "R-1", "B", "B-C", "C", "o1");
    for _ in 0..1200 {
        w.step();
    }
    assert_eq!(w.phase(&id("R-1")), Ok(Phase::Finished));
    // and the bodies were never in contact
    let (s1, s2) = (spec(), spec());
    let (p1, p2) = (w.pose(&id("R-1")).unwrap(), w.pose(&id("R-2")).unwrap());
    assert!(!overlap(
        &place(&s1.shape, p1.x, p1.y, p1.heading),
        &place(&s2.shape, p2.x, p2.y, p2.heading)
    ));
}

#[test]
fn the_safety_stop_changes_nothing_when_nobody_is_in_the_way() {
    // one robot alone: the run is the same with and without the stop
    let run = |safety: bool| {
        let mut w = world(&[("R-1", "A")], safety);
        go(&mut w, "R-1", "A", "A-B", "B", "o1");
        (0..1200)
            .map(|_| {
                w.step();
                x(&w, "R-1").to_bits()
            })
            .collect::<Vec<_>>()
    };
    assert_eq!(run(true), run(false));
}

#[test]
fn the_same_run_gives_the_same_timeline() {
    let run = || {
        let mut w = world(&[("R-1", "A"), ("R-2", "B")], true);
        go(&mut w, "R-1", "A", "A-B", "B", "o1");
        (0..1000)
            .map(|_| {
                w.step();
                x(&w, "R-1").to_bits()
            })
            .collect::<Vec<_>>()
    };
    assert_eq!(run(), run());
}

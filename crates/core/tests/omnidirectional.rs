//! Omnidirectional robots (S63): they travel along an edge whatever their heading;
//! with no orientation in the order they keep their heading.

use botsim_core::actions::ActionTimes;
use botsim_core::drive::Phase;
use botsim_core::robot::Reception;
use botsim_core::world::{RobotId, World};
use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

const DT: f64 = 0.05;

fn id() -> RobotId {
    RobotId::new("ACME", "R-1")
}

fn spec(kinematics: &str) -> RobotSpec {
    let text = include_str!("data/diff_robot.factsheet.json").replace("DIFFERENTIAL", kinematics);
    let f: Factsheet = serde_json::from_str(&text).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

/// A (0,0) → B (10,0) → C (10,5); one robot on A facing east.
fn world(kinematics: &str) -> World {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    let g = Graph::from_lif(&lif).unwrap();
    let a = g.node_idx("A").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(id(), spec(kinematics), ActionTimes::new(1.0), a, 0.0)
        .unwrap();
    w
}

/// A → B → C; `bc` is extra JSON for the edge B-C (an orientation).
fn order(bc: &str) -> Order {
    serde_json::from_str(&format!(
        r#"{{"headerId":1,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{{"nodeId":"A","sequenceId":0,"released":true,"actions":[]}},
                 {{"nodeId":"B","sequenceId":2,"released":true,"actions":[]}},
                 {{"nodeId":"C","sequenceId":4,"released":true,"actions":[]}}],
        "edges":[{{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]}},
                 {{"edgeId":"B-C","sequenceId":3,"released":true,"actions":[]{bc}}}]}}"#
    ))
    .unwrap()
}

/// Run until the order is finished; returns (seconds, heading at the end, the
/// largest sideways speed seen, the smallest heading and the largest seen).
fn run(w: &mut World) -> (f64, f64, f64, (f64, f64)) {
    let mut max_vy: f64 = 0.0;
    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    for step in 0..6000 {
        w.step();
        let s = w.state(&id()).unwrap();
        max_vy = max_vy.max(s.velocity.and_then(|v| v.vy).unwrap_or(0.0).abs());
        let h = w.pose(&id()).unwrap().heading;
        lo = lo.min(h);
        hi = hi.max(h);
        if w.phase(&id()) == Ok(Phase::Finished) {
            return (step as f64 * DT, h, max_vy, (lo, hi));
        }
    }
    panic!("never finished");
}

#[test]
fn an_omnidirectional_robot_keeps_its_heading_when_the_order_names_none() {
    let mut w = world("OMNIDIRECTIONAL");
    assert_eq!(w.send_order(&order("")), Ok(Reception::Accepted));
    let (_, heading, max_vy, (lo, hi)) = run(&mut w);
    // it never turned: east all the way, also on B → C, which runs north
    assert_eq!((lo, hi, heading), (0.0, 0.0, 0.0));
    let p = w.pose(&id()).unwrap();
    assert_eq!((p.x, p.y), (10.0, 5.0));
    // going north while facing east is sideways: the state says so
    assert!(max_vy > 1.0, "sideways speed {max_vy}");
}

#[test]
fn it_is_faster_than_a_differential_robot_because_it_needs_no_turn() {
    let time = |kinematics: &str| {
        let mut w = world(kinematics);
        assert_eq!(w.send_order(&order("")), Ok(Reception::Accepted));
        run(&mut w).0
    };
    let (omni, diff) = (time("OMNIDIRECTIONAL"), time("DIFFERENTIAL"));
    // the differential robot turns 90° at B (about 1.6 s at 1 rad/s)
    assert!(diff - omni > 1.4, "omni {omni} s, differential {diff} s");
}

#[test]
fn a_named_orientation_is_reached_before_the_edge_and_then_kept() {
    // tangential π/2 on B → C (which runs north): heading north + π/2 = west (π)
    let bc = r#","orientation":1.5707963267948966"#;
    let mut w = world("OMNIDIRECTIONAL");
    assert_eq!(w.send_order(&order(bc)), Ok(Reception::Accepted));
    let (_, heading, max_vy, _) = run(&mut w);
    assert!(
        (heading.abs() - std::f64::consts::PI).abs() < 1e-9,
        "{heading}"
    );
    // facing west and driving north: the robot's left is south, so vy is negative
    assert!(max_vy > 1.0);
}

#[test]
fn a_global_orientation_is_valid_for_omnidirectional_robots_only() {
    let bc = r#","orientation":1.0,"orientationType":"GLOBAL""#;
    let mut w = world("OMNIDIRECTIONAL");
    assert_eq!(w.send_order(&order(bc)), Ok(Reception::Accepted));
    let (_, heading, _, _) = run(&mut w);
    assert!((heading - 1.0).abs() < 1e-9, "{heading}");

    let mut w = world("DIFFERENTIAL");
    match w.send_order(&order(bc)).unwrap() {
        Reception::Rejected(t) => {
            assert_eq!(t, botsim_core::errors::ErrorType::OrientationUnreachable)
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_differential_robot_reports_no_sideways_speed_and_turns_as_before() {
    let mut w = world("DIFFERENTIAL");
    assert_eq!(w.send_order(&order("")), Ok(Reception::Accepted));
    let (_, heading, max_vy, (lo, hi)) = run(&mut w);
    assert_eq!(max_vy, 0.0);
    assert!(hi > 1.5 && lo <= 0.0, "it turned north");
    assert!((heading - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
}

#[test]
fn the_observer_shows_the_speed_over_the_ground_of_a_sideways_robot() {
    let mut w = world("OMNIDIRECTIONAL");
    assert_eq!(w.send_order(&order("")), Ok(Reception::Accepted));
    let mut top: f64 = 0.0;
    for _ in 0..3000 {
        w.step();
        top = top.max(w.snapshot().robots[0].speed.abs());
    }
    assert!((top - 1.5).abs() < 1e-9, "{top}");
}

#[test]
fn the_state_with_a_sideways_speed_is_a_valid_state_message() {
    let schema: serde_json::Value =
        serde_json::from_str(include_str!("data/vda5050_state.schema.json")).unwrap();
    let validator = jsonschema::validator_for(&schema).unwrap();
    let mut w = world("OMNIDIRECTIONAL");
    assert_eq!(w.send_order(&order("")), Ok(Reception::Accepted));
    // wait for the part on B → C, facing east: moving sideways
    let mut s = w.state(&id()).unwrap();
    for _ in 0..600 {
        w.step();
        s = w.state(&id()).unwrap();
        if s.velocity.as_ref().unwrap().vy.unwrap().abs() > 0.1 {
            break;
        }
    }
    assert!(s.velocity.as_ref().unwrap().vy.unwrap().abs() > 0.1);
    let json = serde_json::to_value(&s).unwrap();
    assert!(validator.is_valid(&json), "{json}");
}

#[test]
fn the_protective_field_looks_the_way_the_robot_travels_not_the_way_it_faces() {
    // R-1 (omnidirectional, facing east) drives north along B → C; R-2 stands
    // north of it on C, R-3 stands east of B: only R-2 is in the way
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    let g = Graph::from_lif(&lif).unwrap();
    let (a, b, c) = (
        g.node_idx("A").unwrap(),
        g.node_idx("B").unwrap(),
        g.node_idx("C").unwrap(),
    );
    let mut w = World::new(g, DT, 0).unwrap();
    for (n, node) in [(1, b), (2, c)] {
        w.add_robot(
            RobotId::new("ACME", &format!("R-{n}")),
            spec("OMNIDIRECTIONAL"),
            ActionTimes::new(1.0),
            node,
            0.0,
        )
        .unwrap();
    }
    let _ = a;
    w.set_safety_stop(true);
    let o: Order = serde_json::from_str(
        r#"{"headerId":1,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"B","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"C","sequenceId":2,"released":true,"actions":[]}],
        "edges":[{"edgeId":"B-C","sequenceId":1,"released":true,"actions":[]}]}"#,
    )
    .unwrap();
    assert_eq!(w.send_order(&o), Ok(Reception::Accepted));
    for _ in 0..600 {
        w.step();
    }
    // it stopped before the robot on C (5 m north; both bodies 1.2 m long, facing east
    // — so their *widths* (0.8 m) meet in the north–south direction)
    let p = w.pose(&RobotId::new("ACME", "R-1")).unwrap();
    assert_eq!(p.x, 10.0);
    assert!(p.y > 3.0 && p.y < 4.2, "stopped at y = {}", p.y);
    assert!(w.collisions().is_empty());
    assert!(
        w.state(&RobotId::new("ACME", "R-1"))
            .unwrap()
            .safety_state
            .field_violation
    );
}

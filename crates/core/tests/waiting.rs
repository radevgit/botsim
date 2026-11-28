//! Who a waiting robot waits for, and why (U12 definition, S157, S158).

use botsim_core::actions::ActionTimes;
use botsim_core::disturbances::StopSpec;
use botsim_core::robot::Reception;
use botsim_core::waiting::{Place, Wait, WaitTotal};
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_model::waiting::{HeldBack, WaitReason};
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

mod vda_port;
use vda_port::VdaWorld;

const DT: f64 = 0.05;

/// A(0,0) → B(10,0), and two lone nodes just ahead of A: FAR(1.35, 0.55) and
/// NEAR(1.25, -0.55).
fn graph() -> Graph {
    let types = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let nodes: Vec<String> = [
        ("A", 0.0, 0.0),
        ("B", 10.0, 0.0),
        ("FAR", 1.35, 0.55),
        ("NEAR", 1.25, -0.55),
    ]
    .iter()
    .map(|(id, x, y)| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
        )
    })
    .collect();
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-25T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{}],"edges":[{{"edgeId":"A-B","startNodeId":"A","endNodeId":"B",
        "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":{types}}}]}}]}}]}}]}}"#,
        nodes.join(",")
    );
    Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap()).unwrap()
}

/// Robots D on A, F on FAR and N on NEAR, added in this order.
fn world() -> World {
    let g = graph();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let starts: Vec<_> = ["A", "FAR", "NEAR"]
        .iter()
        .map(|n| g.node_idx(n).unwrap())
        .collect();
    let mut w = World::new(g, DT, 0).unwrap();
    for (serial, node) in ["D", "F", "N"].into_iter().zip(starts) {
        let id = RobotId::new("ACME", serial);
        w.add_robot(id, spec.clone(), ActionTimes::new(1.0), node, 0.0)
            .unwrap();
    }
    w
}

/// An order for D along A → B, with B and the edge released or not.
fn go(w: &mut World, released: bool) {
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-25T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"D","orderId":"o","orderUpdateId":0,
        "nodes":[{{"nodeId":"A","sequenceId":0,"released":true,"actions":[]}},
                 {{"nodeId":"B","sequenceId":2,"released":{released},"actions":[]}}],
        "edges":[{{"edgeId":"A-B","sequenceId":1,"released":{released},"actions":[]}}]}}"#
    );
    let o: Order = serde_json::from_str(&json).unwrap();
    assert_eq!(w.send_order(&o), Ok(Reception::Accepted));
}

fn held(reason: WaitReason, blockers: &[usize]) -> Vec<Option<HeldBack>> {
    let first = HeldBack {
        reason,
        blockers: blockers.to_vec(),
    };
    vec![Some(first), None, None]
}

#[test]
fn a_safety_stop_names_the_nearest_robot_in_the_field() {
    let mut w = world();
    w.set_safety_stop(true);
    go(&mut w, true);
    for _ in 0..20 {
        w.step();
    }
    // both stand in the field of D; N (added last) is the nearer one
    let expected = Wait {
        reason: WaitReason::SafetyStop,
        blocker: Some(2),
    };
    assert_eq!(w.waits(), [Some(expected), None, None]);
}

#[test]
fn a_safety_stop_is_credited_to_the_robot_in_the_field() {
    let mut w = world();
    w.set_safety_stop(true);
    go(&mut w, true);
    for _ in 0..20 {
        w.step();
    }
    let a = w.graph().node_idx("A").unwrap();
    let expected = WaitTotal {
        waiter: 0,
        blocker: Some(2),
        reason: WaitReason::SafetyStop,
        place: Place::Node(a),
        ticks: 20,
    };
    assert_eq!(w.wait_totals(), [expected]);
    // the snapshot carries the same wait, for recordings
    let wait = Wait {
        reason: WaitReason::SafetyStop,
        blocker: Some(2),
    };
    let snapshot = w.snapshot();
    let waiting: Vec<_> = snapshot.robots.iter().map(|r| r.waiting).collect();
    assert_eq!(waiting, [Some(wait), None, None]);
}

#[test]
fn a_robot_that_stops_on_an_edge_is_counted_on_the_edge() {
    let g = graph();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let (a, b) = (g.node_idx("A").unwrap(), g.node_idx("B").unwrap());
    let mut w = World::new(g, DT, 0).unwrap();
    for (serial, node) in [("D", a), ("S", b)] {
        let id = RobotId::new("ACME", serial);
        w.add_robot(id, spec.clone(), ActionTimes::new(1.0), node, 0.0)
            .unwrap();
    }
    w.set_safety_stop(true);
    go(&mut w, true);
    for _ in 0..600 {
        w.step();
    }
    let edge = w.graph().edge_idx("A-B").unwrap();
    let totals = w.wait_totals();
    assert!(!totals.is_empty());
    assert!(
        totals
            .iter()
            .all(|t| t.blocker == Some(1) && t.place == Place::Edge(edge)),
        "{totals:?}"
    );
}

#[test]
fn a_robot_in_its_own_stop_is_not_counted_though_another_is_in_its_field() {
    let mut w = world();
    w.set_safety_stop(true);
    // a stop that begins as soon as the robot drives, for longer than the run
    let always = StopSpec {
        per_hour: 3600.0 / DT,
        min_seconds: 60.0,
        max_seconds: 60.0,
    };
    w.enable_random_stops(1, always).unwrap();
    go(&mut w, true);
    for _ in 0..20 {
        w.step();
    }
    assert_eq!(w.random_stops(), 1);
    // the deadlock detector still sees the wait
    assert!(w.waits()[0].is_some());
    assert_eq!(w.wait_totals(), []);
    assert_eq!(w.snapshot().robots[0].waiting, None);
}

#[test]
fn a_held_robot_waits_for_the_nearest_of_those_the_controller_names() {
    let mut w = world();
    go(&mut w, false);
    w.publish_held_back(held(WaitReason::Zone, &[1, 2]));
    w.step();
    let expected = Wait {
        reason: WaitReason::Zone,
        blocker: Some(2),
    };
    assert_eq!(w.waits()[0], Some(expected));
}

#[test]
fn a_controller_that_names_nobody_gives_an_unknown_blocker() {
    let mut w = world();
    go(&mut w, false);
    w.publish_held_back(held(WaitReason::Release, &[]));
    w.step();
    let expected = Wait {
        reason: WaitReason::Release,
        blocker: None,
    };
    assert_eq!(w.waits()[0], Some(expected));
}

#[test]
fn with_a_controller_only_its_word_counts_for_a_release_wait() {
    let mut w = world();
    go(&mut w, false);
    // it publishes, and does not hold D back
    w.publish_held_back(vec![None, None, None]);
    w.step();
    assert_eq!(w.waits()[0], None);
}

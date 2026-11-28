//! A deadlock whose cycle has a reservation link and a safety link (S76): robot X
//! stands on node P and waits for node Q, which robot O has released (reserved,
//! though O is far from it); O drives along a long edge that passes P, and is
//! stopped by its protective field because X is in it. X waits for O (reservation),
//! O waits for X (safety stop): neither stands on the other's next node.

use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_sim::actions::ActionTimes;
use botsim_sim::world::World;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

mod vda_port;
use vda_port::VdaWorld;

const DT: f64 = 0.05;

/// S(-10,0) → Q(10,0) is one long edge that passes P(0,0); P → Q is another.
fn graph() -> Graph {
    let types = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let node = |id: &str, x: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":0.0}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
        )
    };
    let edge = |id: &str, a: &str, b: &str| {
        format!(
            r#"{{"edgeId":"{id}","startNodeId":"{a}","endNodeId":"{b}",
            "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
        )
    };
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{}],"edges":[{},{}]}}]}}]}}"#,
        node("S", -10.0),
        node("P", 0.0),
        node("Q", 10.0),
        edge("S-Q", "S", "Q"),
        edge("P-Q", "P", "Q")
    );
    Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap()).unwrap()
}

fn order(serial: &str, first: &str, released: bool) -> Order {
    serde_json::from_str(&format!(
        r#"{{"headerId":1,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"{serial}","orderId":"o-{serial}","orderUpdateId":0,
        "nodes":[{{"nodeId":"{first}","sequenceId":0,"released":true,"actions":[]}},
                 {{"nodeId":"Q","sequenceId":2,"released":{released},"actions":[]}}],
        "edges":[{{"edgeId":"{first}-Q","sequenceId":1,"released":{released},"actions":[]}}]}}"#
    ))
    .unwrap()
}

fn world(safety_stop: bool) -> World {
    let g = graph();
    let (s, p) = (g.node_idx("S").unwrap(), g.node_idx("P").unwrap());
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(
        RobotId::new("ACME", "O"),
        spec.clone(),
        ActionTimes::new(1.0),
        s,
        0.0,
    )
    .unwrap();
    w.add_robot(
        RobotId::new("ACME", "X"),
        spec,
        ActionTimes::new(1.0),
        p,
        0.0,
    )
    .unwrap();
    w.set_safety_stop(safety_stop);
    w.set_deadlock_timeout(5.0);
    // O has the whole edge released, X waits for Q (released: false)
    w.send_order(&order("O", "S", true)).unwrap();
    w.send_order(&order("X", "P", false)).unwrap();
    w
}

#[test]
fn a_cycle_of_a_reservation_wait_and_a_safety_stop_is_a_deadlock() {
    let mut w = world(true);
    for _ in 0..(40.0 / DT) as usize {
        w.step();
    }
    let d = w.deadlocks();
    assert_eq!(d.len(), 1, "{d:?}");
    assert_eq!(
        d[0].robots,
        vec![RobotId::new("ACME", "O"), RobotId::new("ACME", "X")]
    );
    assert!(d[0].ended_tick.is_none());
    // nobody touched anybody
    assert!(w.collisions().is_empty());
}

#[test]
fn without_a_safety_stop_it_is_a_collision_not_a_deadlock() {
    let mut w = world(false);
    for _ in 0..(40.0 / DT) as usize {
        w.step();
    }
    assert!(w.deadlocks().is_empty());
    assert!(!w.collisions().is_empty());
}

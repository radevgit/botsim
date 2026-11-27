//! Shared setup: a corridor A - B - C with robot R-1 on A and an order A → B (C is
//! the horizon).

#![allow(dead_code)]

use botsim_core::actions::ActionTimes;
use botsim_core::robot::Reception::Accepted;
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

pub const DT: f64 = 0.05;
const DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../scenarios/");

pub fn read(name: &str) -> String {
    std::fs::read_to_string(format!("{DIR}{name}")).unwrap()
}

pub fn corridor_graph() -> Graph {
    let lif: Lif = serde_json::from_str(&read("corridor.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

/// The world with the robot and the order, no observer yet.
pub fn corridor_world() -> World {
    let graph = corridor_graph();
    let a = graph.node_idx("A").unwrap();
    let f: Factsheet = serde_json::from_str(&read("diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let mut w = World::new(graph, DT, 0).unwrap();
    w.add_robot(
        RobotId::new("ACME", "R-1"),
        spec,
        ActionTimes::new(2.0),
        a,
        0.0,
    )
    .unwrap();
    let order: Order = serde_json::from_str(
        r#"{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"A","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"B","sequenceId":2,"released":true,"actions":[]},
                 {"nodeId":"C","sequenceId":4,"released":false,"actions":[]}],
        "edges":[{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]},
                 {"edgeId":"B-C","sequenceId":3,"released":false,"actions":[]}]}"#,
    )
    .unwrap();
    assert_eq!(w.send_order(&order), Ok(Accepted));
    w
}

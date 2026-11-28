//! Layouts and robots the tests of the connection share.

use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use serde_json::{Value, json};
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

pub const DT: f64 = 0.05;

pub fn id(n: usize) -> RobotId {
    RobotId::new("ACME", &format!("R-{n:03}"))
}

/// A differential robot of type ACME/X1.
pub fn robot_spec() -> RobotSpec {
    let f: Factsheet = serde_json::from_str(include_str!(
        "../../../baseline/tests/data/diff_robot.factsheet.json"
    ))
    .unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

/// A layout for robots of that type from nodes (id, map, x, y) and directed edges
/// (from, to); edge ids are "from-to".
pub fn graph(nodes: &[(&str, &str, f64, f64)], edges: &[(&str, &str)]) -> Graph {
    let types = json!([{"manufacturer": "ACME", "seriesName": "X1"}]);
    let nodes: Vec<Value> = nodes
        .iter()
        .map(|(id, map, x, y)| {
            json!({"nodeId": id, "mapId": map, "nodePosition": {"x": x, "y": y},
                "mobileRobotTypeNodeProperties": [{"mobileRobotTypes": types}]})
        })
        .collect();
    let edges: Vec<Value> = edges
        .iter()
        .map(|(a, b)| {
            json!({"edgeId": format!("{a}-{b}"), "startNodeId": a, "endNodeId": b,
                "mobileRobotTypeEdgeProperties": [{"mobileRobotTypes": types,
                    "orientationType": "TANGENTIAL", "mobileRobotOrientations": [0.0]}]})
        })
        .collect();
    let lif: Lif = serde_json::from_value(json!({
        "metaInformation": {"lifId": "t", "creator": "t",
            "exportTimestamp": "2026-11-20T10:00:00.000Z", "lifVersion": "2.0.0"},
        "origins": [{"originId": "O", "layouts": [{"layoutId": "L",
            "layoutVersion": "1", "nodes": nodes, "edges": edges}]}],
    }))
    .unwrap();
    Graph::from_lif(&lif).unwrap()
}

/// N0 → N1 → N2 → N3 on map M, 10 m apart.
#[allow(dead_code)] // (not every test file uses it)
pub fn line() -> Graph {
    graph(
        &[
            ("N0", "M", 0.0, 0.0),
            ("N1", "M", 10.0, 0.0),
            ("N2", "M", 20.0, 0.0),
            ("N3", "M", 30.0, 0.0),
        ],
        &[("N0", "N1"), ("N1", "N2"), ("N2", "N3")],
    )
}

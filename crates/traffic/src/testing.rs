//! Small hand-built layouts and conflict maps for the unit tests.

use std::collections::BTreeSet;

use botsim_sweep::{Conflict, ConflictMap, TypePair};
use vdma_lif::Lif;

use crate::groups::Route;
use crate::spots::Piece;

pub const SMALL: &str = "ACME/Small";
pub const LARGE: &str = "ACME/Large";

/// A layout of these nodes and edges `(id, from, to)`. The module never uses
/// positions, so all nodes are at the origin.
pub fn lif(nodes: &[&str], edges: &[(&str, &str, &str)]) -> Lif {
    let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"Small"}]}"#;
    let nodes: Vec<String> = nodes
        .iter()
        .map(|id| {
            format!(
                r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":0,"y":0}},"mobileRobotTypeNodeProperties":[{ty}]}}"#
            )
        })
        .collect();
    let edges: Vec<String> = edges
        .iter()
        .map(|(id, a, b)| {
            format!(
                r#"{{"edgeId":"{id}","startNodeId":"{a}","endNodeId":"{b}","mobileRobotTypeEdgeProperties":[{ty}]}}"#
            )
        })
        .collect();
    serde_json::from_str(&format!(
        r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1","nodes":[{}],"edges":[{}]}}]}}]}}"#,
        nodes.join(","),
        edges.join(",")
    ))
    .expect("test LIF parses")
}

/// A route of a small robot through these nodes; the edge from `a` to `b` has the
/// id `a-b`.
pub fn route(nodes: &[&str]) -> Route {
    let nodes: Vec<String> = nodes.iter().map(|n| n.to_string()).collect();
    let edges: Vec<String> = nodes.windows(2).map(|w| w.join("-")).collect();
    Route::new(SMALL, &nodes, &edges).expect("route")
}

/// The layout with the nodes and edges these routes use.
pub fn layout(routes: &[&[&str]]) -> Lif {
    let nodes: BTreeSet<&str> = routes.iter().flat_map(|r| r.iter().copied()).collect();
    let edges: BTreeSet<(String, &str, &str)> = routes
        .iter()
        .flat_map(|r| r.windows(2))
        .map(|w| (w.join("-"), w[0], w[1]))
        .collect();
    let nodes: Vec<&str> = nodes.into_iter().collect();
    let edges: Vec<(&str, &str, &str)> = edges
        .iter()
        .map(|(id, a, b)| (id.as_str(), *a, *b))
        .collect();
    lif(&nodes, &edges)
}

/// A map for the two types without any listed conflict.
pub fn empty_map() -> ConflictMap {
    ConflictMap {
        sample_step: 0.25,
        min_overlap: 0.05,
        clearance: 0.1,
        types: vec![SMALL.into(), LARGE.into()],
        pairs: Vec::new(),
        node_pairs: Vec::new(),
        node_node_pairs: Vec::new(),
    }
}

/// One listed edge–edge conflict.
pub fn edge_pair(type_a: &str, edge_a: &str, type_b: &str, edge_b: &str) -> TypePair {
    TypePair {
        type_a: type_a.into(),
        type_b: type_b.into(),
        conflicts: vec![Conflict {
            edge_a: edge_a.into(),
            edge_b: edge_b.into(),
            shares_node: false,
            junction_nodes: None,
            depth: 0.2,
            distance: 0.0,
        }],
    }
}

pub fn node(id: &str) -> Piece {
    Piece::Node(id.into())
}

pub fn edge(id: &str) -> Piece {
    Piece::Edge(id.into())
}

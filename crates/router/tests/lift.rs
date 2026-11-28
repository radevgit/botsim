//! Routes through the waiting nodes of lift entrances (S49). Which edges are rides,
//! and the waiting node of each entrance, are given as data.

use std::collections::HashMap;

use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_model::spec::Kinematics;
use botsim_router::lift::route_through_waiting;
use botsim_router::route::{Arrival, Mover, Turns, straight_lengths};
use vdma_lif::Lif;

mod common;
use common::graph;

/// Two floors, two-way: `0` — `J` — `2` (the lift), with a lay-by `W` at `J`.
fn two_way() -> Graph {
    let node = |id: &str, map: &str, x: f64, y: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"{map}","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]}}]}}"#
        )
    };
    let edge = |a: &str, b: &str| {
        format!(
            r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}",
            "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}],
            "orientationType":"TANGENTIAL","mobileRobotOrientations":[0.0]}}]}}"#
        )
    };
    let floor = |p: &str, map: &str| -> (Vec<String>, Vec<String>) {
        let n = |s: &str| format!("{p}{s}");
        let nodes = vec![
            node(&n("0"), map, 0.0, 0.0),
            node(&n("J"), map, 10.0, 0.0),
            node(&n("2"), map, 20.0, 0.0),
            node(&n("W"), map, 10.0, 6.0),
        ];
        let edges = ["0", "2", "W"]
            .iter()
            .flat_map(|s| [edge(&n("J"), &n(s)), edge(&n(s), &n("J"))])
            .collect();
        (nodes, edges)
    };
    let (gn, mut ge) = floor("G", "L1");
    let (un, mut ue) = floor("U", "L2");
    ge.push(edge("G2", "U2"));
    ue.push(edge("U2", "G2"));
    let layout = |id: &str, n: Vec<String>, e: Vec<String>| {
        format!(
            r#"{{"originId":"O{id}","layouts":[{{"layoutId":"{id}","layoutVersion":"1",
            "nodes":[{}],"edges":[{}]}}]}}"#,
            n.join(","),
            e.join(",")
        )
    };
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{},{}]}}"#,
        layout("1", gn, ge),
        layout("2", un, ue)
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif).unwrap()
}

/// The ride map from (ride edge, waiting node of its entrance).
fn rides(g: &Graph, list: &[(&str, Option<&str>)]) -> HashMap<EdgeIdx, Option<NodeIdx>> {
    list.iter()
        .map(|&(e, w)| (g.edge_idx(e).unwrap(), w.map(|w| g.node_idx(w).unwrap())))
        .collect()
}

fn edge_ids(g: &Graph, edges: &[usize]) -> Vec<String> {
    edges.iter().map(|&e| g.edges[e].id.clone()).collect()
}

/// The edge ids of the route of a robot that may drive every edge and make every
/// turn.
fn route(
    g: &Graph,
    rides: &HashMap<EdgeIdx, Option<NodeIdx>>,
    from: &str,
    to: &str,
) -> Vec<String> {
    let all = vec![true; g.edges.len()]; // every edge may be used
    let cost = straight_lengths(g);
    let mover = Mover {
        ok: &all,
        cost: &cost,
        turns: None,
    };
    let (from, to) = (g.node_idx(from).unwrap(), g.node_idx(to).unwrap());
    let r = route_through_waiting(g, &mover, rides, Arrival::Free, from, to).unwrap();
    edge_ids(g, &r)
}

#[test]
fn a_route_to_the_lift_goes_through_the_waiting_node() {
    let g = two_way();

    let with = rides(&g, &[("G2-U2", Some("GW")), ("U2-G2", Some("UW"))]);
    // G0 → U0: to the lay-by, back, to the entrance, the ride, on the upper floor
    // (no detour there: the way after the ride has no ride in it)
    assert_eq!(
        route(&g, &with, "G0", "U0"),
        [
            "G0-GJ", "GJ-GW", "GW-GJ", "GJ-G2", "G2-U2", "U2-UJ", "UJ-U0"
        ]
    );
    // a robot that starts on the entrance cannot wait outside: no detour
    assert_eq!(route(&g, &with, "G2", "U0"), ["G2-U2", "U2-UJ", "UJ-U0"]);
    // a robot that starts on the lay-by has passed it already
    assert_eq!(route(&g, &with, "GW", "U2"), ["GW-GJ", "GJ-G2", "G2-U2"]);
    // no ride, no detour
    assert_eq!(route(&g, &with, "G0", "G2"), ["G0-GJ", "GJ-G2"]);

    // without a waiting node the route is the shortest one
    let without = rides(&g, &[("G2-U2", None), ("U2-G2", None)]);
    assert_eq!(
        route(&g, &without, "G0", "U0"),
        ["G0-GJ", "GJ-G2", "G2-U2", "U2-UJ", "UJ-U0"]
    );
}

#[test]
fn a_waiting_node_that_cannot_be_reached_means_no_detour() {
    let nodes = [
        ("A", 0.0, 0.0),
        ("B", 10.0, 0.0),
        ("C", 20.0, 0.0),
        ("W", 10.0, 5.0),
    ];
    // no way to the lay-by
    let g = graph(&nodes, &[("A", "B"), ("B", "C"), ("W", "B")]);
    let r = rides(&g, &[("B-C", Some("W"))]);
    assert_eq!(route(&g, &r, "A", "C"), ["A-B", "B-C"]);
    // a way to it, but none from it to the entrance
    let g = graph(&nodes, &[("A", "B"), ("B", "C"), ("A", "W")]);
    let r = rides(&g, &[("B-C", Some("W"))]);
    assert_eq!(route(&g, &r, "A", "C"), ["A-B", "B-C"]);
}

#[test]
fn a_waiting_node_outside_the_graph_means_no_detour() {
    let g = graph(
        &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("C", 20.0, 0.0)],
        &[("A", "B"), ("B", "C")],
    );
    let ride = g.edge_idx("B-C").unwrap();
    for outside in [g.nodes.len(), usize::MAX] {
        let r = HashMap::from([(ride, Some(outside))]);
        assert_eq!(route(&g, &r, "A", "C"), ["A-B", "B-C"]);
    }
}

#[test]
fn every_ride_of_a_route_is_planned_through_its_waiting_node() {
    // B-C and D-E are rides; W is the lay-by of B, V the one of D
    let g = graph(
        &[
            ("A", 0.0, 0.0),
            ("B", 10.0, 0.0),
            ("C", 20.0, 0.0),
            ("D", 30.0, 0.0),
            ("E", 40.0, 0.0),
            ("W", 5.0, 5.0),
            ("V", 25.0, 5.0),
        ],
        &[
            ("A", "B"),
            ("B", "C"),
            ("C", "D"),
            ("D", "E"),
            ("A", "W"),
            ("W", "B"),
            ("C", "V"),
            ("V", "D"),
        ],
    );
    let r = rides(&g, &[("B-C", Some("W")), ("D-E", Some("V"))]);
    assert_eq!(
        route(&g, &r, "A", "E"),
        ["A-W", "W-B", "B-C", "C-V", "V-D", "D-E"]
    );
}

#[test]
fn the_way_from_the_waiting_node_keeps_to_the_turns_the_robot_can_make() {
    // A-C-D is a straight line (C-D the ride); the way over the lay-by W turns at W
    let g = graph(
        &[
            ("A", 0.0, 0.0),
            ("C", 10.0, 10.0),
            ("D", 20.0, 20.0),
            ("W", 10.0, 0.0),
        ],
        &[("A", "C"), ("C", "D"), ("A", "W"), ("W", "C")],
    );
    let r = rides(&g, &[("C-D", Some("W"))]);
    // a robot that can turn on the spot waits at W
    assert_eq!(route(&g, &r, "A", "D"), ["A-W", "W-C", "C-D"]);
    // a three-wheel robot arrives at W heading east and cannot go on to C
    let all = vec![true; g.edges.len()];
    let cost = straight_lengths(&g);
    let turns = Turns::for_type(&g, Kinematics::ThreeWheel, "ACME", "X1").unwrap();
    let mover = Mover {
        ok: &all,
        cost: &cost,
        turns: Some(&turns),
    };
    let (a, d) = (g.node_idx("A").unwrap(), g.node_idx("D").unwrap());
    let path = route_through_waiting(&g, &mover, &r, Arrival::Free, a, d).unwrap();
    assert_eq!(edge_ids(&g, &path), ["A-C", "C-D"]);
}

use botsim_baseline::route::shortest_path;
use botsim_core::graph::Graph;
use vdma_lif::Lif;

/// A layout from nodes (id, x, y) and directed edges (from, to); edge ids are "from-to".
fn graph(nodes: &[(&str, f64, f64)], edges: &[(&str, &str)]) -> Graph {
    let ns: Vec<String> = nodes
        .iter()
        .map(|(id, x, y)| {
            format!(
                r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
                "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]}}]}}"#
            )
        })
        .collect();
    let es: Vec<String> = edges
        .iter()
        .map(|(a, b)| {
            format!(
                r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}",
                "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}],
                "orientationType":"TANGENTIAL","mobileRobotOrientations":[0.0]}}]}}"#
            )
        })
        .collect();
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-20T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{}],"edges":[{}]}}]}}]}}"#,
        ns.join(","),
        es.join(",")
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif).unwrap()
}

/// The route as edge ids, for readable asserts.
fn names(g: &Graph, from: &str, to: &str) -> Option<Vec<String>> {
    let p = shortest_path(g, g.node_idx(from).unwrap(), g.node_idx(to).unwrap())?;
    Some(p.iter().map(|&e| g.edges[e].id.clone()).collect())
}

fn strs(v: &[&str]) -> Option<Vec<String>> {
    Some(v.iter().map(|s| s.to_string()).collect())
}

#[test]
fn a_chain_and_the_same_node() {
    let g = graph(
        &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("C", 10.0, 5.0)],
        &[("A", "B"), ("B", "C")],
    );
    assert_eq!(names(&g, "A", "C"), strs(&["A-B", "B-C"]));
    assert_eq!(names(&g, "B", "C"), strs(&["B-C"]));
    assert_eq!(names(&g, "B", "B"), strs(&[]), "already there");
}

#[test]
fn edges_are_directed_and_unreachable_nodes_give_none() {
    let g = graph(
        &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("C", 20.0, 0.0)],
        &[("A", "B"), ("B", "C")],
    );
    assert_eq!(names(&g, "C", "A"), None, "one-way");
    assert_eq!(names(&g, "B", "A"), None);
}

#[test]
fn the_shortest_by_length_wins_not_the_fewest_edges() {
    // A→X→D is 2 hops but about 41 m; A→B→C→D is 3 hops and 12 m
    let g = graph(
        &[
            ("A", 0.0, 0.0),
            ("B", 4.0, 0.0),
            ("C", 8.0, 0.0),
            ("D", 12.0, 0.0),
            ("X", 6.0, 20.0),
        ],
        &[("A", "X"), ("X", "D"), ("A", "B"), ("B", "C"), ("C", "D")],
    );
    assert_eq!(names(&g, "A", "D"), strs(&["A-B", "B-C", "C-D"]));
}

#[test]
fn a_loop_does_not_confuse_it() {
    let g = graph(
        &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("C", 10.0, 10.0)],
        &[("A", "B"), ("B", "C"), ("C", "A")],
    );
    assert_eq!(names(&g, "B", "A"), strs(&["B-C", "C-A"]));
    assert_eq!(names(&g, "C", "B"), strs(&["C-A", "A-B"]));
}

#[test]
fn equal_routes_give_the_same_answer_every_time() {
    // two routes of equal length A→B→D and A→C→D
    let g = graph(
        &[
            ("A", 0.0, 0.0),
            ("B", 5.0, 5.0),
            ("C", 5.0, -5.0),
            ("D", 10.0, 0.0),
        ],
        &[("A", "B"), ("A", "C"), ("B", "D"), ("C", "D")],
    );
    let first = names(&g, "A", "D");
    assert!(first.is_some());
    for _ in 0..20 {
        assert_eq!(names(&g, "A", "D"), first);
    }
}

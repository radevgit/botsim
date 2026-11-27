//! Per-robot-type properties of the LIF (S55) in the graph. What they mean for a
//! robot is tested in `botsim-core`.

use botsim_model::graph::{Graph, GraphError};
use vdma_lif::Lif;

const X1: (&str, &str) = ("ACME", "X1");
const Y2: (&str, &str) = ("ACME", "Y2");

fn types(list: &[(&str, &str)]) -> String {
    let t: Vec<String> = list
        .iter()
        .map(|(m, s)| format!(r#"{{"manufacturer":"{m}","seriesName":"{s}"}}"#))
        .collect();
    format!("[{}]", t.join(","))
}

/// A layout A(0,0) – B(10,0) – C(20,0). Nodes A and B allow both types, node C
/// only X1. Edge A-B allows both types, with `y2_speed` for Y2; edge B-C only X1.
fn layout(y2_speed: Option<f64>) -> Result<Graph, GraphError> {
    let node = |id: &str, x: f64, allowed: &[(&str, &str)]| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":0.0}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{}}}]}}"#,
            types(allowed)
        )
    };
    let speed = y2_speed.map_or(String::new(), |v| format!(r#","maximumSpeed":{v}"#));
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{}],
        "edges":[
          {{"edgeId":"A-B","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[
            {{"mobileRobotTypes":{},"orientationType":"TANGENTIAL","mobileRobotOrientations":[0.0]}},
            {{"mobileRobotTypes":{},"orientationType":"TANGENTIAL","mobileRobotOrientations":[0.0]{speed}}}]}},
          {{"edgeId":"B-C","startNodeId":"B","endNodeId":"C","mobileRobotTypeEdgeProperties":[
            {{"mobileRobotTypes":{},"orientationType":"TANGENTIAL","mobileRobotOrientations":[0.0]}}]}}
        ]}}]}}]}}"#,
        node("A", 0.0, &[X1, Y2]),
        node("B", 10.0, &[X1, Y2]),
        node("C", 20.0, &[X1]),
        types(&[X1]),
        types(&[Y2]),
        types(&[X1]),
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif)
}

#[test]
fn the_graph_knows_which_types_may_use_what_and_the_speed_limits() {
    let g = layout(Some(0.5)).unwrap();
    let (a, c) = (g.node_idx("A").unwrap(), g.node_idx("C").unwrap());
    let (ab, bc) = (g.edge_idx("A-B").unwrap(), g.edge_idx("B-C").unwrap());

    assert!(g.node_allows(a, X1.0, X1.1) && g.node_allows(a, Y2.0, Y2.1));
    assert!(g.node_allows(c, X1.0, X1.1) && !g.node_allows(c, Y2.0, Y2.1));
    // a type that is not listed at all
    assert!(!g.node_allows(a, "ACME", "Z9") && !g.node_allows(a, "Other", "X1"));

    assert!(g.edge_allows(ab, X1.0, X1.1) && g.edge_allows(ab, Y2.0, Y2.1));
    assert!(g.edge_allows(bc, X1.0, X1.1) && !g.edge_allows(bc, Y2.0, Y2.1));

    assert_eq!(g.edge_max_speed(ab, X1.0, X1.1), None);
    assert_eq!(g.edge_max_speed(ab, Y2.0, Y2.1), Some(0.5));
    assert_eq!(
        g.edge_max_speed(bc, Y2.0, Y2.1),
        None,
        "not allowed: no entry"
    );
}

#[test]
fn a_maximum_speed_of_zero_or_less_is_refused_when_the_layout_is_read() {
    for v in [0.0, -1.0] {
        let e = layout(Some(v)).unwrap_err();
        assert_eq!(e, GraphError::BadMaximumSpeed { edge: "A-B".into() });
        assert!(e.to_string().contains("A-B"));
    }
}

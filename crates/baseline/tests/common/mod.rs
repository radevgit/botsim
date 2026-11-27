use botsim_model::graph::Graph;
use vdma_lif::Lif;

/// A layout from nodes (id, x, y) and directed edges (from, to); edge ids are "from-to".
pub fn graph(nodes: &[(&str, f64, f64)], edges: &[(&str, &str)]) -> Graph {
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

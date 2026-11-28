//! Rotation allowed at nodes (LIF `rotationAtStartNodeAllowed` /
//! `rotationAtEndNodeAllowed`, S64) in the graph. What it means for a robot is
//! tested in `botsim-sim`.

use botsim_model::graph::{Graph, Rotation};
use vdma_lif::Lif;

/// Layout A(0,0) → B(10,0) → C(10,5); `ab` and `bc` are extra JSON properties of
/// the two edges (e.g. `"rotationAtStartNodeAllowed":"NONE"`).
fn graph(ab: &str, bc: &str) -> Graph {
    let node = |id: &str, x: f64, y: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]}}]}}"#
        )
    };
    let edge = |id: &str, a: &str, b: &str, extra: &str| {
        let extra = if extra.is_empty() {
            String::new()
        } else {
            format!(",{extra}")
        };
        format!(
            r#"{{"edgeId":"{id}","startNodeId":"{a}","endNodeId":"{b}","mobileRobotTypeEdgeProperties":[
            {{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]{extra}}}]}}"#
        )
    };
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{}],"edges":[{},{}]}}]}}]}}"#,
        node("A", 0.0, 0.0),
        node("B", 10.0, 0.0),
        node("C", 10.0, 5.0),
        edge("A-B", "A", "B", ab),
        edge("B-C", "B", "C", bc)
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif).unwrap()
}

#[test]
fn the_layout_rules_are_read_and_combine_as_a_boolean_and() {
    let g = graph(
        r#""rotationAtStartNodeAllowed":"NONE","rotationAtEndNodeAllowed":"CW""#,
        "",
    );
    let (ab, bc) = (g.edge_idx("A-B").unwrap(), g.edge_idx("B-C").unwrap());
    assert_eq!(
        g.edge_rotation(ab, "ACME", "X1"),
        (Rotation::None, Rotation::Cw)
    );
    // nothing said: both ways
    assert_eq!(
        g.edge_rotation(bc, "ACME", "X1"),
        (Rotation::Both, Rotation::Both)
    );

    use Rotation::{Both, Ccw, Cw, None};
    for (a, b, want) in [
        (Both, Both, Both),
        (Both, Ccw, Ccw),
        (Cw, Both, Cw),
        (Ccw, Cw, None),
        (None, Both, None),
        (Ccw, Ccw, Ccw),
    ] {
        assert_eq!(a.and(b), want, "{a:?} and {b:?}");
        assert_eq!(b.and(a), want);
    }
    assert!(Both.allows(true) && Both.allows(false));
    assert!(Ccw.allows(true) && !Ccw.allows(false));
    assert!(!None.allows(true) && !None.allows(false));
}

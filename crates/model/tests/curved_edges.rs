//! Edges with a curved trajectory (S69) in the graph. How a robot drives them is
//! tested in `botsim-core`.

use std::f64::consts::FRAC_PI_2;

use botsim_model::graph::{Graph, GraphError};
use vdma_lif::Lif;

/// The trajectory JSON of a quarter circle of radius 5 from (0,0), starting east
/// and ending north at (5,5) (centre (0,5)).
const ARC: &str = r#""trajectory":{"degree":2,"controlPoints":[
    {"x":0.0,"y":0.0},{"x":5.0,"y":0.0,"weight":0.7071067811865476},{"x":5.0,"y":5.0}]}"#;

/// A(0,0) –[arc]→ B(5,5); B → C(5,12) straight on (north), B → D(10,5) turns right
/// (east). Edge A-B has the given extra properties for type X1; with `y2_straight`
/// the edge lists type Y2 as well, without a trajectory. All nodes allow both types.
fn graph(ab: &str, y2_straight: bool) -> Graph {
    let types =
        r#"[{"manufacturer":"ACME","seriesName":"X1"},{"manufacturer":"ACME","seriesName":"Y2"}]"#;
    let node = |id: &str, x: f64, y: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
        )
    };
    let x1 = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let y2 = r#"[{"manufacturer":"ACME","seriesName":"Y2"}]"#;
    let plain = |id: &str, a: &str, b: &str| {
        format!(
            r#"{{"edgeId":"{id}","startNodeId":"{a}","endNodeId":"{b}","mobileRobotTypeEdgeProperties":[
            {{"mobileRobotTypes":{types}}}]}}"#
        )
    };
    let ab_edge = format!(
        r#"{{"edgeId":"A-B","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[
        {{"mobileRobotTypes":{x1}{ab}}}{}]}}"#,
        if y2_straight {
            format!(r#",{{"mobileRobotTypes":{y2}}}"#)
        } else {
            String::new()
        }
    );
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{},{}],"edges":[{},{},{}]}}]}}]}}"#,
        node("A", 0.0, 0.0),
        node("B", 5.0, 5.0),
        node("C", 5.0, 12.0),
        node("D", 10.0, 5.0),
        ab_edge,
        plain("B-C", "B", "C"),
        plain("B-D", "B", "D")
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif).unwrap()
}

#[test]
fn the_travel_length_of_an_edge_is_the_length_of_its_curve_for_the_type_that_has_one() {
    // quarter circle of radius 5: 5π/2 = 7.854 m on a chord of 7.071 m (S74)
    let g = graph(&format!(",{ARC}"), true);
    let e = g.edge_idx("A-B").unwrap();
    let curved = g.edge_travel_length(e, "ACME", "X1");
    assert!((curved - 5.0 * FRAC_PI_2).abs() < 1e-3, "{curved}");
    // type Y2 drives the straight chord on the same edge
    let straight = g.edge_travel_length(e, "ACME", "Y2");
    assert!((straight - 50.0_f64.sqrt()).abs() < 1e-9, "{straight}");
    // an edge without a curve: the distance between the nodes
    let bc = g.edge_idx("B-C").unwrap();
    assert!((g.edge_travel_length(bc, "ACME", "X1") - 7.0).abs() < 1e-9);
}

#[test]
fn wrong_trajectories_are_refused_when_the_layout_is_read() {
    let read = |ab: &str| {
        let types = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
        let json = format!(
            r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
            "nodes":[{{"nodeId":"A","mapId":"M","nodePosition":{{"x":0.0,"y":0.0}},"mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{types}}}]}},
                     {{"nodeId":"B","mapId":"M","nodePosition":{{"x":5.0,"y":5.0}},"mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{types}}}]}}],
            "edges":[{{"edgeId":"A-B","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[
              {{"mobileRobotTypes":{types}{ab}}}]}}]}}]}}]}}"#
        );
        Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap())
    };
    assert!(read(&format!(",{ARC}")).is_ok());
    // it does not end on the end node (5, 5)
    let e =
        read(r#","trajectory":{"degree":1,"controlPoints":[{"x":0.0,"y":0.0},{"x":9.0,"y":9.0}]}"#)
            .unwrap_err();
    match &e {
        GraphError::BadTrajectory { edge, reason } => {
            assert_eq!(edge, "A-B");
            assert!(reason.contains("end node"), "{reason}");
        }
        other => panic!("{other:?}"),
    }
    // it does not start on the start node
    assert!(
        read(r#","trajectory":{"degree":1,"controlPoints":[{"x":1.0,"y":0.0},{"x":5.0,"y":5.0}]}"#)
            .is_err()
    );
    // not a curve: degree 3 with three points, a zero weight
    let e = read(r#","trajectory":{"degree":3,"controlPoints":[{"x":0.0,"y":0.0},{"x":1.0,"y":0.0},{"x":5.0,"y":5.0}]}"#)
        .unwrap_err()
        .to_string();
    assert!(e.contains("A-B") && e.contains("control points"), "{e}");
    assert!(read(r#","trajectory":{"degree":1,"controlPoints":[{"x":0.0,"y":0.0,"weight":0.0},{"x":5.0,"y":5.0}]}"#).is_err());
}

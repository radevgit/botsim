//! The curves of edges in the layout line, and the robots' types.

use std::fs::File;
use std::io::BufReader;

use botsim_model::graph::Graph;
use botsim_record::{LayoutLine, LayoutRobot, Recorder, read_recording};
use botsim_sim::observer::Observer;
use vdma_lif::Lif;

const TYPES: &str =
    r#"[{"manufacturer":"ACME","seriesName":"X1"},{"manufacturer":"ACME","seriesName":"Y2"}]"#;

/// A quarter circle of radius 5 from (0, 0) (east) to (5, 5) (north), centre (0, 5).
const ARC: &str = r#","trajectory":{"degree":2,"controlPoints":[
    {"x":0.0,"y":0.0},{"x":5.0,"y":0.0,"weight":0.7071067811865476},{"x":5.0,"y":5.0}]}"#;

/// A(0,0) → B(5,5) → C(5,12). With `arc`, the edge A-B has the arc for type X1 and
/// the straight line for type Y2; B-C is straight for both.
fn graph(arc: bool) -> Graph {
    let node = |id: &str, x: f64, y: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{TYPES}}}]}}"#
        )
    };
    let ab = if arc {
        format!(
            r#"[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]{ARC}}},
            {{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"Y2"}}]}}]"#
        )
    } else {
        format!(r#"[{{"mobileRobotTypes":{TYPES}}}]"#)
    };
    let bc = format!(r#"[{{"mobileRobotTypes":{TYPES}}}]"#);
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{}],
        "edges":[{{"edgeId":"A-B","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":{ab}}},
                 {{"edgeId":"B-C","startNodeId":"B","endNodeId":"C","mobileRobotTypeEdgeProperties":{bc}}}]}}]}}]}}"#,
        node("A", 0.0, 0.0),
        node("B", 5.0, 5.0),
        node("C", 5.0, 12.0),
    );
    Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap()).unwrap()
}

#[test]
fn an_edge_without_a_curve_has_no_paths() {
    let line = LayoutLine::from_graph(&graph(false));
    assert!(line.edges.iter().all(|e| e.paths.is_empty()));
    assert!(line.robots.is_empty());
}

#[test]
fn each_group_of_types_gets_its_own_path() {
    let line = LayoutLine::from_graph(&graph(true));
    let (ab, bc) = (&line.edges[0], &line.edges[1]);
    assert!(bc.paths.is_empty(), "a straight edge keeps no paths");

    assert_eq!(ab.paths.len(), 2);
    let x1 = ab.paths.iter().find(|p| p.types == ["ACME/X1"]).unwrap();
    let y2 = ab.paths.iter().find(|p| p.types == ["ACME/Y2"]).unwrap();

    // Y2 has no curve: the straight line, two points
    assert_eq!(y2.points, [[0.0, 0.0], [5.0, 5.0]]);

    // X1 has the arc: it starts and ends at the nodes, lies on the circle round
    // (0, 5), and the points are not further apart than the sampling step
    assert_eq!(x1.points.first(), Some(&[0.0, 0.0]));
    let last = x1.points.last().unwrap();
    assert!((last[0] - 5.0).abs() < 1e-9 && (last[1] - 5.0).abs() < 1e-9);
    assert!(x1.points.len() >= 39, "{} points", x1.points.len());
    for p in &x1.points {
        let radius = p[0].hypot(p[1] - 5.0);
        assert!(
            // the curve is cut into short straight pieces, so a little off is fine
            (radius - 5.0).abs() < 1e-3,
            "{p:?} is {radius} from the centre"
        );
    }
    for w in x1.points.windows(2) {
        let step = (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]);
        assert!(step <= 0.2 + 1e-9 && step > 0.0, "step {step}");
    }
}

#[test]
fn types_that_share_a_curve_are_one_entry() {
    // both types listed together on the edge with the arc
    let json_types = TYPES;
    let node = |id: &str, x: f64, y: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{json_types}}}]}}"#
        )
    };
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{}],
        "edges":[{{"edgeId":"A-B","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":
            [{{"mobileRobotTypes":{json_types}{ARC}}}]}}]}}]}}]}}"#,
        node("A", 0.0, 0.0),
        node("B", 5.0, 5.0),
    );
    let graph = Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap()).unwrap();
    let line = LayoutLine::from_graph(&graph);
    assert_eq!(line.edges[0].paths.len(), 1);
    assert_eq!(line.edges[0].paths[0].types, ["ACME/X1", "ACME/Y2"]);
}

#[test]
fn the_json_round_trips_and_old_lines_still_read() {
    let line = LayoutLine::from_graph(&graph(true)).with_robots(vec![LayoutRobot {
        id: "ACME/R-001".into(),
        robot_type: "ACME/X1".into(),
    }]);
    let text = serde_json::to_string(&line).unwrap();
    assert!(
        text.contains(r#""robots":[{"id":"ACME/R-001","type":"ACME/X1"}]"#),
        "{text}"
    );
    assert!(
        text.contains(r#""paths":[{"types":["ACME/X1"],"points":[[0.0,0.0],"#),
        "{text}"
    );
    assert_eq!(serde_json::from_str::<LayoutLine>(&text).unwrap(), line);

    // a line from before the curves: no `paths`, no `robots`
    // (one line, as in a recording)
    let old = r#"{"type":"layout","version":1,"maps":["M"],"nodes":[{"id":"A","map":0,"x":0.0,"y":0.0},{"id":"B","map":0,"x":1.0,"y":0.0}],"edges":[{"id":"A-B","start":0,"end":1,"length":1.0,"map_transition":false}]}"#;
    let rec = read_recording(old.as_bytes()).unwrap();
    assert!(rec.layout.edges[0].paths.is_empty() && rec.layout.robots.is_empty());
}

#[test]
fn the_recorder_writes_the_curves_and_the_robot_types() {
    let path = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("curves.jsonl");
    let (recorder, handle) = Recorder::new(File::create(&path).unwrap());
    let mut recorder = recorder.with_robots(vec![
        LayoutRobot {
            id: "ACME/R-001".into(),
            robot_type: "ACME/X1".into(),
        },
        LayoutRobot {
            id: "ACME/R-002".into(),
            robot_type: "ACME/Y2".into(),
        },
    ]);
    recorder.on_layout(&graph(true));
    handle.finish().unwrap();
    let rec = read_recording(BufReader::new(File::open(&path).unwrap())).unwrap();
    assert_eq!(rec.layout.robots.len(), 2);
    assert_eq!(rec.layout.robots[1].robot_type, "ACME/Y2");
    assert_eq!(rec.layout.edges[0].paths.len(), 2);
    assert_eq!(
        rec.layout,
        LayoutLine::from_graph(&graph(true)).with_robots(rec.layout.robots.clone())
    );
}

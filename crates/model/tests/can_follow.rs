//! `Graph::can_follow` / `can_start` (S71) in a few cases. That the robot itself
//! answers the same is tested in `botsim-sim`.

use botsim_model::graph::{Graph, Rotation};
use botsim_model::spec::Kinematics;
use vdma_lif::Lif;

const X1: &str = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;

/// A(0,0) → B(10,0), then B → C(10,5) (a left turn of 90°), B → D(20,0) (straight
/// on) and B → E(10,-5) (a right turn); `ab` is extra JSON for A-B, `bx` for the
/// three edges after it (e.g. `"rotationAtStartNodeAllowed":"NONE"`).
fn graph(ab: &str, bx: &str) -> Graph {
    let node = |id: &str, x: f64, y: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{X1}}}]}}"#
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
            {{"mobileRobotTypes":{X1}{extra}}}]}}"#
        )
    };
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{},{},{}],"edges":[{},{},{},{}]}}]}}]}}"#,
        node("A", 0.0, 0.0),
        node("B", 10.0, 0.0),
        node("C", 10.0, 5.0),
        node("D", 20.0, 0.0),
        node("E", 10.0, -5.0),
        edge("A-B", "A", "B", ab),
        edge("B-C", "B", "C", bx),
        edge("B-D", "B", "D", bx),
        edge("B-E", "B", "E", bx)
    );
    Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap()).unwrap()
}

/// What the graph says about B-`next` after A-B.
fn graph_says(ab: &str, bx: &str, kinematics: Kinematics, next: &str) -> bool {
    let g = graph(ab, bx);
    let e = |id: &str| g.edge_idx(id).unwrap();
    g.can_follow(e("A-B"), e(&format!("B-{next}")), kinematics, "ACME", "X1")
}

#[test]
fn the_answers_in_a_few_plain_cases() {
    // a turn of 90° is allowed for a differential robot, not for a car-like one;
    // straight on is allowed for both; an omnidirectional robot never turns
    assert!(graph_says("", "", Kinematics::Differential, "C"));
    assert!(!graph_says("", "", Kinematics::ThreeWheel, "C"));
    assert!(graph_says("", "", Kinematics::ThreeWheel, "D"));
    assert!(graph_says("", "", Kinematics::Omnidirectional, "C"));
    // no rotation at the node: a differential robot cannot take the turn either,
    // but can go straight on
    let none = r#""rotationAtStartNodeAllowed":"NONE""#;
    assert!(!graph_says("", none, Kinematics::Differential, "C"));
    assert!(graph_says("", none, Kinematics::Differential, "D"));
    // only clockwise allowed: the left turn is made the long way round
    let cw = r#""rotationAtStartNodeAllowed":"CW""#;
    assert!(graph_says("", cw, Kinematics::Differential, "C"));
}

#[test]
fn can_start_looks_at_the_heading_the_robot_has() {
    let g = graph("", "");
    let a_b = g.edge_idx("A-B").unwrap();
    let ask = |heading: f64, k: Kinematics| g.can_start(heading, None, a_b, k, "ACME", "X1");
    assert!(ask(0.0, Kinematics::ThreeWheel));
    assert!(ask(0.01, Kinematics::ThreeWheel)); // within about a degree
    assert!(!ask(0.1, Kinematics::ThreeWheel));
    assert!(!ask(std::f64::consts::PI, Kinematics::ThreeWheel));
    // a robot that turns on the spot starts in any heading
    assert!(ask(std::f64::consts::PI, Kinematics::Differential));
    // unless the layout allows no rotation at the start node
    let g = graph(r#""rotationAtStartNodeAllowed":"NONE""#, "");
    let a_b = g.edge_idx("A-B").unwrap();
    assert!(!g.can_start(
        1.0,
        Some(Rotation::Both),
        a_b,
        Kinematics::Differential,
        "ACME",
        "X1"
    ));
    assert!(g.can_start(0.0, None, a_b, Kinematics::Differential, "ACME", "X1"));
}

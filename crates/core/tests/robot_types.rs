//! Per-robot-type properties of the LIF (S55): which robot types may use a node
//! or an edge, and the speed limit of an edge for a type.

use botsim_core::actions::ActionTimes;
use botsim_core::errors::ErrorType;
use botsim_core::robot::Reception;
use botsim_core::world::{World, WorldError};
use botsim_model::graph::{Graph, GraphError};
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

const DT: f64 = 0.05;
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

fn spec(series: &str) -> RobotSpec {
    let text =
        include_str!("data/diff_robot.factsheet.json").replace("\"X1\"", &format!("\"{series}\""));
    let f: Factsheet = serde_json::from_str(&text).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn id(serial: &str) -> RobotId {
    RobotId::new("ACME", serial)
}

fn order(serial: &str, from: &str, edge: &str, to: &str) -> Order {
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"{serial}","orderId":"o1","orderUpdateId":0,
        "nodes":[{{"nodeId":"{from}","sequenceId":0,"released":true,"actions":[]}},
                 {{"nodeId":"{to}","sequenceId":2,"released":true,"actions":[]}}],
        "edges":[{{"edgeId":"{edge}","sequenceId":1,"released":true,"actions":[]}}]}}"#
    );
    serde_json::from_str(&json).unwrap()
}

#[test]
fn a_robot_cannot_be_placed_on_a_node_its_type_may_not_use() {
    let g = layout(None).unwrap();
    let c = g.node_idx("C").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let r = w.add_robot(id("R-Y"), spec("Y2"), ActionTimes::new(1.0), c, 0.0);
    match r {
        Err(WorldError::Start(_)) => {}
        other => panic!("{other:?}"),
    }
    assert!(r.unwrap_err().to_string().contains("node 'C'"));
    // the other type may stand there
    assert!(
        w.add_robot(id("R-X"), spec("X1"), ActionTimes::new(1.0), c, 0.0)
            .is_ok()
    );
}

#[test]
fn an_order_over_an_edge_or_node_the_type_may_not_use_is_refused() {
    let g = layout(None).unwrap();
    let b = g.node_idx("B").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(id("R-Y"), spec("Y2"), ActionTimes::new(1.0), b, 0.0)
        .unwrap();
    w.add_robot(id("R-X"), spec("X1"), ActionTimes::new(1.0), b, 0.0)
        .unwrap();
    // B → C: edge and node are X1 only
    let r = w.send_order(&order("R-Y", "B", "B-C", "C")).unwrap();
    assert_eq!(r, Reception::Rejected(ErrorType::NoRouteToTarget));
    let s = w.state(&id("R-Y")).unwrap();
    let text = format!("{:?}", s.errors);
    assert!(text.contains("does not allow this robot type"), "{text}");
    // X1 may
    assert_eq!(
        w.send_order(&order("R-X", "B", "B-C", "C")).unwrap(),
        Reception::Accepted
    );
}

/// The top speed (m/s) reached while a robot of `series` drives A → B, and the
/// time it needs (s).
fn drive_a_to_b(series: &str, y2_speed: Option<f64>) -> (f64, f64) {
    let g = layout(y2_speed).unwrap();
    let a = g.node_idx("A").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(id("R-1"), spec(series), ActionTimes::new(1.0), a, 0.0)
        .unwrap();
    assert_eq!(
        w.send_order(&order("R-1", "A", "A-B", "B")).unwrap(),
        Reception::Accepted
    );
    let mut top: f64 = 0.0;
    let mut time = None;
    for step in 0..4000 {
        w.step();
        top = top.max(w.snapshot().robots[0].speed.abs());
        if time.is_none() && (w.pose(&id("R-1")).unwrap().x - 10.0).abs() < 1e-9 {
            time = Some(step as f64 * DT);
        }
    }
    (top, time.expect("it arrived"))
}

#[test]
fn an_edge_speed_limit_applies_to_the_type_it_is_given_for() {
    let (fast_top, fast_time) = drive_a_to_b("X1", Some(0.5));
    let (slow_top, slow_time) = drive_a_to_b("Y2", Some(0.5));
    // X1 has no limit on this edge: it reaches its own top speed (1.5 m/s)
    assert!(fast_top > 1.4, "{fast_top}");
    // Y2 is limited to 0.5 m/s there, never above it, and it takes much longer
    assert!(slow_top <= 0.5 + 1e-9 && slow_top > 0.49, "{slow_top}");
    assert!(
        slow_time > 2.0 * fast_time,
        "{slow_time} against {fast_time}"
    );
    // and a limit above the robot's own top speed changes nothing
    let (top, time) = drive_a_to_b("Y2", Some(9.0));
    assert!((top - fast_top).abs() < 1e-9 && (time - fast_time).abs() < 1e-9);
}

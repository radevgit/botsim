//! Edges with a curved trajectory (S69): the robot follows the curve, its heading
//! follows the direction of the curve, and it drives through nodes where curves
//! join smoothly.

use std::f64::consts::{FRAC_PI_2, PI};

use botsim_core::actions::ActionTimes;
use botsim_core::drive::{Driver, Phase, RouteEdge};
use botsim_core::robot::Reception;
use botsim_core::world::{RobotId, World};
use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

const DT: f64 = 0.05;
/// Radius of the quarter circle A → B.
const R: f64 = 5.0;

fn id() -> RobotId {
    RobotId::new("ACME", "R-1")
}

fn spec(series: &str, kinematics: &str) -> RobotSpec {
    let text = include_str!("data/diff_robot.factsheet.json")
        .replace("DIFFERENTIAL", kinematics)
        .replace("\"X1\"", &format!("\"{series}\""));
    RobotSpec::from_factsheet(&serde_json::from_str::<Factsheet>(&text).unwrap()).unwrap()
}

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

fn arc_graph() -> Graph {
    graph(&format!(",{ARC}"), false)
}

fn world(g: Graph, series: &str, kinematics: &str, pass_through: bool) -> World {
    let a = g.node_idx("A").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(
        id(),
        spec(series, kinematics),
        ActionTimes::new(1.0),
        a,
        0.0,
    )
    .unwrap();
    w.set_pass_through(pass_through);
    w
}

/// An order from A along the edges named in `via` (`A-B`, then `B-C` or `B-D`);
/// `ab` is extra JSON for the first edge (an orientation).
fn order(via: &[&str], ab: &str) -> Order {
    let node_of = |edge: &str| edge.split('-').nth(1).unwrap().to_string();
    let mut nodes =
        vec![r#"{"nodeId":"A","sequenceId":0,"released":true,"actions":[]}"#.to_string()];
    let mut edges = Vec::new();
    for (i, e) in via.iter().enumerate() {
        let extra = if i == 0 { ab } else { "" };
        edges.push(format!(
            r#"{{"edgeId":"{e}","sequenceId":{},"released":true,"actions":[]{extra}}}"#,
            2 * i + 1
        ));
        nodes.push(format!(
            r#"{{"nodeId":"{}","sequenceId":{},"released":true,"actions":[]}}"#,
            node_of(e),
            2 * i + 2
        ));
    }
    serde_json::from_str(&format!(
        r#"{{"headerId":1,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{}],"edges":[{}]}}"#,
        nodes.join(","),
        edges.join(",")
    ))
    .unwrap()
}

/// Distance of the robot from the centre (0, 5) of the arc.
fn radius(w: &World) -> f64 {
    let p = w.pose(&id()).unwrap();
    p.x.hypot(p.y - R)
}

/// The angle a - b folded into (-π, π].
fn diff(a: f64, b: f64) -> f64 {
    (a - b + PI).rem_euclid(2.0 * PI) - PI
}

#[test]
fn a_robot_follows_the_curve_and_its_heading_follows_the_direction_of_the_curve() {
    let mut w = world(arc_graph(), "X1", "DIFFERENTIAL", false);
    assert_eq!(w.send_order(&order(&["A-B"], "")), Ok(Reception::Accepted));
    let (mut last_heading, mut worst_radius, mut worst_heading, mut biggest_jump) =
        (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    let mut steps_on_the_arc = 0;
    for _ in 0..3000 {
        w.step();
        let p = w.pose(&id()).unwrap();
        // on the arc: x > 0.01 and y < 4.99
        if p.x > 0.01 && p.y < R - 0.01 {
            steps_on_the_arc += 1;
            worst_radius = worst_radius.max((radius(&w) - R).abs());
            // the tangent of the circle at the robot: perpendicular to the radius
            let tangent = (p.y - R).atan2(p.x) + FRAC_PI_2;
            worst_heading = worst_heading.max(diff(p.heading, tangent).abs());
            biggest_jump = biggest_jump.max(diff(p.heading, last_heading).abs());
        }
        last_heading = p.heading;
        if w.phase(&id()) == Ok(Phase::Finished) {
            break;
        }
    }
    assert!(steps_on_the_arc > 100);
    assert!(worst_radius < 0.01, "off the circle by {worst_radius}");
    assert!(
        worst_heading < 0.03,
        "heading off the tangent by {worst_heading}"
    );
    // the heading changes smoothly (a quarter turn over about 5 s: < 0.1 rad a step)
    assert!(biggest_jump < 0.1, "a jump of {biggest_jump} rad");
    // it ends on B with the heading of the end of the curve: north
    let p = w.pose(&id()).unwrap();
    assert!((p.x - 5.0).abs() < 1e-9 && (p.y - 5.0).abs() < 1e-9);
    assert!((p.heading - FRAC_PI_2).abs() < 1e-9, "{}", p.heading);
}

#[test]
fn the_state_reports_the_turning_and_the_trip_takes_the_length_of_the_arc() {
    let mut w = world(arc_graph(), "X1", "DIFFERENTIAL", false);
    w.send_order(&order(&["A-B"], "")).unwrap();
    let mut turning = Vec::new();
    let mut seconds = 0.0;
    for step in 0..3000 {
        w.step();
        let v = w.state(&id()).unwrap().velocity.unwrap();
        if v.vx.unwrap_or(0.0) > 1.4 {
            turning.push(v.omega.unwrap());
        }
        if w.phase(&id()) == Ok(Phase::Finished) {
            seconds = step as f64 * DT;
            break;
        }
    }
    // at the full speed of 1.5 m/s on a radius of 5 m: ω = v / R = 0.3 rad/s
    assert!(turning.len() > 20);
    assert!(
        turning.iter().all(|w| (w - 0.3).abs() < 0.03),
        "{turning:?}"
    );
    // the same trip on the straight chord (7.07 m) is shorter than on the arc (7.85 m)
    let chord = {
        let mut w = world(graph("", false), "X1", "DIFFERENTIAL", false);
        // the robot faces east, the chord points north-east: it turns first
        w.send_order(&order(&["A-B"], "")).unwrap();
        let mut t = 0.0;
        for step in 0..3000 {
            w.step();
            if w.phase(&id()) == Ok(Phase::Finished) {
                t = step as f64 * DT;
                break;
            }
        }
        t
    };
    assert!(seconds > 0.0 && chord > 0.0);
    // on the arc the robot starts facing east already: no turn at A, so the longer
    // path is the faster trip; the arc is 0.78 m longer, the turn on the chord 0.8 s
    assert!(seconds < chord + 1.0, "arc {seconds} s, chord {chord} s");
}

#[test]
fn driving_backwards_along_a_curve_keeps_the_heading_against_the_direction() {
    let mut w = world(arc_graph(), "X1", "DIFFERENTIAL", false);
    // tangential orientation π: backwards
    let ab = r#","orientation":3.141592653589793,"orientationType":"TANGENTIAL""#;
    assert_eq!(w.send_order(&order(&["A-B"], ab)), Ok(Reception::Accepted));
    let mut worst = 0.0_f64;
    for _ in 0..4000 {
        w.step();
        let p = w.pose(&id()).unwrap();
        if p.x > 0.01 && p.y < R - 0.01 {
            let tangent = (p.y - R).atan2(p.x) + FRAC_PI_2;
            worst = worst.max(diff(p.heading, tangent + PI).abs());
            assert!((radius(&w) - R).abs() < 0.01);
        }
        if w.phase(&id()) == Ok(Phase::Finished) {
            break;
        }
    }
    assert!(worst < 0.03, "{worst}");
    let p = w.pose(&id()).unwrap();
    assert!(
        (p.heading - (-FRAC_PI_2)).abs() < 1e-9,
        "south at the end: {}",
        p.heading
    );
}

#[test]
fn it_drives_through_a_node_where_curves_join_smoothly_and_stops_at_a_kink() {
    let speed_at_b = |next: &str| {
        let mut w = world(arc_graph(), "X1", "DIFFERENTIAL", true);
        assert_eq!(
            w.send_order(&order(&["A-B", next], "")),
            Ok(Reception::Accepted)
        );
        for _ in 0..3000 {
            w.step();
            let s = w.state(&id()).unwrap();
            if s.last_node_id == "B" {
                return s.velocity.and_then(|v| v.vx).unwrap_or(0.0).abs();
            }
        }
        panic!("never reached B");
    };
    // the arc ends going north and B → C goes north: straight through
    assert!(speed_at_b("B-C") > 1.0);
    // B → D goes east: a corner
    assert!(speed_at_b("B-D") < 0.05);
}

#[test]
fn the_edge_is_curved_only_for_the_type_whose_properties_have_the_trajectory() {
    // type Y2 is listed on A-B without a trajectory: it drives the straight chord
    let mut w = world(graph(&format!(",{ARC}"), true), "Y2", "DIFFERENTIAL", false);
    assert_eq!(w.send_order(&order(&["A-B"], "")), Ok(Reception::Accepted));
    let mut max_off_chord = 0.0_f64;
    for _ in 0..3000 {
        w.step();
        let p = w.pose(&id()).unwrap();
        // the chord is the line y = x
        max_off_chord = max_off_chord.max((p.y - p.x).abs() / 2.0_f64.sqrt());
        if w.phase(&id()) == Ok(Phase::Finished) {
            break;
        }
    }
    assert!(max_off_chord < 1e-9, "{max_off_chord}");
    // and X1 on the same edge leaves the chord by up to the sagitta (≈ 1.46 m)
    let mut w = world(graph(&format!(",{ARC}"), true), "X1", "DIFFERENTIAL", false);
    w.send_order(&order(&["A-B"], "")).unwrap();
    let mut max_off_chord = 0.0_f64;
    for _ in 0..3000 {
        w.step();
        let p = w.pose(&id()).unwrap();
        max_off_chord = max_off_chord.max((p.y - p.x).abs() / 2.0_f64.sqrt());
    }
    assert!((max_off_chord - 1.4645).abs() < 0.02, "{max_off_chord}");
}

#[test]
fn an_omnidirectional_robot_follows_the_curve_with_the_heading_the_order_asks_for() {
    // no orientation: it keeps its heading (east) and drives the curve sideways
    let mut w = world(arc_graph(), "X1", "OMNIDIRECTIONAL", false);
    w.send_order(&order(&["A-B"], "")).unwrap();
    let mut max_vy: f64 = 0.0;
    for _ in 0..3000 {
        w.step();
        assert_eq!(w.pose(&id()).unwrap().heading, 0.0, "it never turns");
        let v = w.state(&id()).unwrap().velocity.unwrap();
        max_vy = max_vy.max(v.vy.unwrap_or(0.0));
        if w.phase(&id()) == Ok(Phase::Finished) {
            break;
        }
    }
    assert!(
        max_vy > 1.0,
        "at the end it moves north, sideways to its heading"
    );
    assert_eq!(w.pose(&id()).unwrap().x, 5.0);

    // tangential 0: the heading follows the curve, so no sideways speed
    let mut w = world(arc_graph(), "X1", "OMNIDIRECTIONAL", false);
    w.send_order(&order(
        &["A-B"],
        r#","orientation":0.0,"orientationType":"TANGENTIAL""#,
    ))
    .unwrap();
    let mut max_vy: f64 = 0.0;
    for _ in 0..3000 {
        w.step();
        let v = w.state(&id()).unwrap().velocity.unwrap();
        max_vy = max_vy.max(v.vy.unwrap_or(0.0).abs());
        if w.phase(&id()) == Ok(Phase::Finished) {
            break;
        }
    }
    assert!(max_vy < 0.05, "{max_vy}");
    assert!((w.pose(&id()).unwrap().heading - FRAC_PI_2).abs() < 1e-9);

    // global 0: the heading stays east on the whole curve
    let mut w = world(arc_graph(), "X1", "OMNIDIRECTIONAL", false);
    w.send_order(&order(
        &["A-B"],
        r#","orientation":0.0,"orientationType":"GLOBAL""#,
    ))
    .unwrap();
    for _ in 0..3000 {
        w.step();
        assert_eq!(w.pose(&id()).unwrap().heading, 0.0);
        if w.phase(&id()) == Ok(Phase::Finished) {
            break;
        }
    }
}

#[test]
fn the_direction_the_robot_travels_in_turns_with_the_curve() {
    // the protective field and the door/zone checks look the way the robot goes
    let g = arc_graph();
    let a = g.node_idx("A").unwrap();
    let edge = g.edge_idx("A-B").unwrap();
    let mut d = Driver::new(
        &g,
        &spec("X1", "DIFFERENTIAL"),
        a,
        0.0,
        &[RouteEdge::new(edge)],
    )
    .unwrap();
    let mut first = None;
    let mut last = (0.0, 0.0);
    for _ in 0..3000 {
        d.step(DT);
        if let Some(t) = d.travel_direction() {
            first.get_or_insert(t);
            last = t;
        }
        if d.is_finished() {
            break;
        }
    }
    let (f, l) = (first.unwrap(), last);
    assert!(f.0 > 0.99 && f.1.abs() < 0.1, "starts east: {f:?}");
    assert!(l.1 > 0.99 && l.0.abs() < 0.1, "ends north: {l:?}");
}

#[test]
fn a_run_on_a_curve_is_repeatable() {
    let run = || {
        let mut w = world(arc_graph(), "X1", "DIFFERENTIAL", true);
        w.send_order(&order(&["A-B", "B-C"], "")).unwrap();
        (0..1500)
            .map(|_| {
                w.step();
                let p = w.pose(&id()).unwrap();
                (p.x.to_bits(), p.y.to_bits(), p.heading.to_bits())
            })
            .collect::<Vec<_>>()
    };
    assert_eq!(run(), run());
}

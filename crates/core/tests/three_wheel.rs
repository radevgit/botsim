//! The three-wheel (car-like) robot (S70): it cannot turn on the spot, so an order
//! that starts an edge in another direction than the robot heads is refused; it
//! drives curves, and a curve so tight that it would have to turn faster than it
//! can (at the lowest speed it can drive) is refused. The limits are the
//! simulator's own choice: the factsheet has no field for a turning radius.

use std::f64::consts::{FRAC_PI_2, PI};

use botsim_core::actions::ActionTimes;
use botsim_core::drive::Phase;
use botsim_core::errors::ErrorType;
use botsim_core::graph::Graph;
use botsim_core::robot::Reception;
use botsim_core::spec::RobotSpec;
use botsim_core::world::{RobotId, World, WorldError};
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

const DT: f64 = 0.05;
const NO_ROUTE: Result<Reception, WorldError> = Ok(Reception::Rejected(ErrorType::NoRouteToTarget));
const ACCEPTED: Result<Reception, WorldError> = Ok(Reception::Accepted);

fn id() -> RobotId {
    RobotId::new("ACME", "R-1")
}

/// Layout: A(0,0); E(10,0) east of it and F(10,10) north of E (a corner); an arc
/// A → B(r,r) of radius `r` starting east and ending north, then B → C(r,r+10)
/// straight on north and B → D(r+10,r) turning right.
fn graph(r: f64) -> Graph {
    let x1 = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let node = |id: &str, x: f64, y: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{x1}}}]}}"#
        )
    };
    let edge = |id: &str, a: &str, b: &str, extra: &str| {
        format!(
            r#"{{"edgeId":"{id}","startNodeId":"{a}","endNodeId":"{b}","mobileRobotTypeEdgeProperties":[
            {{"mobileRobotTypes":{x1}{extra}}}]}}"#
        )
    };
    let arc = format!(
        r#","trajectory":{{"degree":2,"controlPoints":[
        {{"x":0.0,"y":0.0}},{{"x":{r},"y":0.0,"weight":0.7071067811865476}},{{"x":{r},"y":{r}}}]}}"#
    );
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{},{},{},{}],"edges":[{},{},{},{},{}]}}]}}]}}"#,
        node("A", 0.0, 0.0),
        node("B", r, r),
        node("C", r, r + 10.0),
        node("D", r + 10.0, r),
        node("E", 10.0, 0.0),
        node("F", 10.0, 10.0),
        edge("A-B", "A", "B", &arc),
        edge("B-C", "B", "C", ""),
        edge("B-D", "B", "D", ""),
        edge("A-E", "A", "E", ""),
        edge("E-F", "E", "F", "")
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif).unwrap()
}

/// The test factsheet (min speed 0.1 m/s, max 1.5, angular speed 1 rad/s) with the
/// given kinematics; `tune` can change the limits.
fn spec(kinematics: &str, tune: impl Fn(&mut RobotSpec)) -> RobotSpec {
    let text = include_str!("data/diff_robot.factsheet.json").replace("DIFFERENTIAL", kinematics);
    let mut s =
        RobotSpec::from_factsheet(&serde_json::from_str::<Factsheet>(&text).unwrap()).unwrap();
    tune(&mut s);
    s
}

fn world(r: f64, kinematics: &str, heading: f64, tune: impl Fn(&mut RobotSpec)) -> World {
    let g = graph(r);
    let a = g.node_idx("A").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(
        id(),
        spec(kinematics, tune),
        ActionTimes::new(1.0),
        a,
        heading,
    )
    .unwrap();
    w
}

fn three_wheel(heading: f64) -> World {
    world(5.0, "THREE_WHEEL", heading, |_| {})
}

/// An order from A along the edges of `via` (each starts where the one before
/// ended); `first` is extra JSON for the first edge (an orientation).
fn order(via: &[&str], first: &str) -> Order {
    let node_of = |edge: &str| edge.split('-').nth(1).unwrap().to_string();
    let mut nodes =
        vec![r#"{"nodeId":"A","sequenceId":0,"released":true,"actions":[]}"#.to_string()];
    let mut edges = Vec::new();
    for (i, e) in via.iter().enumerate() {
        let extra = if i == 0 { first } else { "" };
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

/// Run until the robot has finished (at most 200 s); the poses seen on the way.
fn run(w: &mut World) -> Vec<(f64, f64, f64)> {
    let mut seen = Vec::new();
    for _ in 0..4000 {
        w.step();
        let p = w.pose(&id()).unwrap();
        seen.push((p.x, p.y, p.heading));
        if w.phase(&id()) == Ok(Phase::Finished) {
            break;
        }
    }
    assert_eq!(w.phase(&id()), Ok(Phase::Finished));
    seen
}

#[test]
fn it_drives_straight_when_it_heads_the_way_the_edge_starts() {
    let mut w = three_wheel(0.0);
    assert_eq!(w.send_order(&order(&["A-E"], "")), Ok(Reception::Accepted));
    let seen = run(&mut w);
    assert!(
        seen.iter().all(|p| p.2.abs() < 1e-12),
        "the heading changed"
    );
    let p = w.pose(&id()).unwrap();
    assert!((p.x - 10.0).abs() < 1e-9 && p.y.abs() < 1e-9);
}

#[test]
fn it_refuses_a_corner_because_it_cannot_turn_on_the_spot() {
    // east to E, then north to F: a 90 degree turn at E
    let mut w = three_wheel(0.0);
    assert_eq!(w.send_order(&order(&["A-E", "E-F"], "")), NO_ROUTE);
    // the same corner with a robot that turns on the spot is driven
    let mut w = world(5.0, "DIFFERENTIAL", 0.0, |_| {});
    assert_eq!(w.send_order(&order(&["A-E", "E-F"], "")), ACCEPTED);
    run(&mut w);
}

#[test]
fn it_drives_a_smooth_join_and_never_turns_without_moving() {
    // the arc ends north and B → C goes north: no turn at B
    let mut w = three_wheel(0.0);
    assert_eq!(w.send_order(&order(&["A-B", "B-C"], "")), ACCEPTED);
    let seen = run(&mut w);
    let mut last = (0.0, 0.0, 0.0);
    for p in &seen {
        let turned = (p.2 - last.2).abs();
        let moved = (p.0 - last.0).hypot(p.1 - last.1);
        assert!(
            turned < 1e-9 || moved > 1e-9,
            "turned {turned} rad while standing still at {p:?}"
        );
        last = *p;
    }
    let p = w.pose(&id()).unwrap();
    assert!((p.x - 5.0).abs() < 1e-9 && (p.y - 15.0).abs() < 1e-9);
    assert!((p.heading - FRAC_PI_2).abs() < 1e-9);
    // and the arc and the corner after it: B → D goes east, a corner, refused
    let mut w = three_wheel(0.0);
    assert_eq!(w.send_order(&order(&["A-B", "B-D"], "")), NO_ROUTE);
}

#[test]
fn a_robot_that_does_not_head_the_way_the_first_edge_starts_is_refused() {
    // heading north, the edge A → E starts east
    let mut w = three_wheel(FRAC_PI_2);
    assert_eq!(w.send_order(&order(&["A-E"], "")), NO_ROUTE);
    // a degree or so off counts as aligned: it drives, with the heading set right
    let mut w = three_wheel(0.01);
    assert_eq!(w.send_order(&order(&["A-E"], "")), Ok(Reception::Accepted));
    run(&mut w);
    let p = w.pose(&id()).unwrap();
    assert!(p.heading.abs() < 1e-12, "{}", p.heading);
    // three degrees off does not
    let mut w = three_wheel(0.06);
    assert_eq!(w.send_order(&order(&["A-E"], "")), NO_ROUTE);
}

#[test]
fn it_drives_backwards_only_if_it_already_heads_backwards() {
    let backwards = r#","orientation":3.141592653589793,"orientationType":"TANGENTIAL""#;
    // facing east, the edge wants it facing west: a half turn on the spot
    let mut w = three_wheel(0.0);
    assert_eq!(w.send_order(&order(&["A-E"], backwards)), NO_ROUTE);
    // facing west already: it reverses to E, keeping its heading
    let mut w = three_wheel(PI);
    assert_eq!(
        w.send_order(&order(&["A-E"], backwards)),
        Ok(Reception::Accepted)
    );
    run(&mut w);
    let p = w.pose(&id()).unwrap();
    assert!((p.x - 10.0).abs() < 1e-9 && p.y.abs() < 1e-9);
    assert!((p.heading.abs() - PI).abs() < 1e-9, "{}", p.heading);
}

#[test]
fn a_curve_tighter_than_the_lowest_speed_allows_is_refused() {
    // lowest speed 0.5 m/s, at most 0.5 rad/s: the smallest radius is 0.5 / 0.5 = 1 m
    let tune = |s: &mut RobotSpec| {
        s.min_speed = 0.5;
        s.limits.max_angular_speed = 0.5;
    };
    let mut w = world(0.5, "THREE_WHEEL", 0.0, tune);
    assert_eq!(w.send_order(&order(&["A-B"], "")), NO_ROUTE);
    let mut w = world(1.5, "THREE_WHEEL", 0.0, tune);
    assert_eq!(w.send_order(&order(&["A-B"], "")), Ok(Reception::Accepted));
    run(&mut w);
    // a robot that turns on the spot just drives the tight curve slowly
    let mut w = world(0.5, "DIFFERENTIAL", 0.0, tune);
    assert_eq!(w.send_order(&order(&["A-B"], "")), Ok(Reception::Accepted));
    run(&mut w);
}

#[test]
fn the_speed_on_a_curve_keeps_the_turning_within_the_angular_speed() {
    // radius 1 m, at most 0.5 rad/s: at most 0.5 m/s on the curve, where the
    // trip of 1.57 m would otherwise reach nearly 1 m/s (and 1 rad/s)
    let tune = |s: &mut RobotSpec| s.limits.max_angular_speed = 0.5;
    for kinematics in ["DIFFERENTIAL", "THREE_WHEEL"] {
        let mut w = world(1.0, kinematics, 0.0, tune);
        assert_eq!(w.send_order(&order(&["A-B"], "")), Ok(Reception::Accepted));
        let (mut top_speed, mut top_turn) = (0.0_f64, 0.0_f64);
        for _ in 0..4000 {
            w.step();
            let v = w.state(&id()).unwrap().velocity.unwrap();
            top_speed = top_speed.max(v.vx.unwrap_or(0.0).abs());
            top_turn = top_turn.max(v.omega.unwrap_or(0.0).abs());
            if w.phase(&id()) == Ok(Phase::Finished) {
                break;
            }
        }
        assert_eq!(w.phase(&id()), Ok(Phase::Finished), "{kinematics}");
        assert!(top_speed < 0.52, "{kinematics}: {top_speed} m/s");
        assert!(top_speed > 0.4, "{kinematics}: {top_speed} m/s");
        assert!(top_turn < 0.55, "{kinematics}: {top_turn} rad/s");
    }
}

#[test]
fn a_wide_curve_is_not_slowed() {
    // radius 5 m at 1 rad/s allows 5 m/s: the 1.5 m/s of the robot is the limit
    let mut w = three_wheel(0.0);
    w.send_order(&order(&["A-B"], "")).unwrap();
    let mut top_speed = 0.0_f64;
    for _ in 0..4000 {
        w.step();
        top_speed = top_speed.max(w.state(&id()).unwrap().velocity.unwrap().vx.unwrap_or(0.0));
        if w.phase(&id()) == Ok(Phase::Finished) {
            break;
        }
    }
    assert!(top_speed > 1.4, "{top_speed}");
}

#[test]
fn it_is_deterministic() {
    let once = || {
        let mut w = three_wheel(0.0);
        w.send_order(&order(&["A-B", "B-C"], "")).unwrap();
        run(&mut w)
    };
    assert_eq!(once(), once());
}

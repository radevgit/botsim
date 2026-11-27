//! Rotation allowed at nodes (LIF `rotationAtStartNodeAllowed` /
//! `rotationAtEndNodeAllowed`, S64).

use botsim_core::actions::ActionTimes;
use botsim_core::drive::Phase;
use botsim_core::errors::ErrorType;
use botsim_core::robot::Reception;
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

const DT: f64 = 0.05;

fn id() -> RobotId {
    RobotId::new("ACME", "R-1")
}

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

fn spec(kinematics: &str) -> RobotSpec {
    let text = include_str!("data/diff_robot.factsheet.json").replace("DIFFERENTIAL", kinematics);
    RobotSpec::from_factsheet(&serde_json::from_str::<Factsheet>(&text).unwrap()).unwrap()
}

/// A robot on A with the given heading.
fn world(ab: &str, bc: &str, kinematics: &str, heading: f64) -> World {
    let g = graph(ab, bc);
    let a = g.node_idx("A").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(id(), spec(kinematics), ActionTimes::new(1.0), a, heading)
        .unwrap();
    w
}

/// A → B → C released as far as `through` says: "B" (B-C is the horizon) or "C".
fn order(through: &str, bc_extra: &str) -> Order {
    let c = through == "C";
    serde_json::from_str(&format!(
        r#"{{"headerId":1,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{{"nodeId":"A","sequenceId":0,"released":true,"actions":[]}},
                 {{"nodeId":"B","sequenceId":2,"released":true,"actions":[]}},
                 {{"nodeId":"C","sequenceId":4,"released":{c},"actions":[]}}],
        "edges":[{{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]}},
                 {{"edgeId":"B-C","sequenceId":3,"released":{c},"actions":[]{bc_extra}}}]}}"#
    ))
    .unwrap()
}

/// The update that releases B → C.
fn update() -> Order {
    serde_json::from_str(
        r#"{"headerId":2,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":1,
        "nodes":[{"nodeId":"B","sequenceId":2,"released":true,"actions":[]},
                 {"nodeId":"C","sequenceId":4,"released":true,"actions":[]}],
        "edges":[{"edgeId":"B-C","sequenceId":3,"released":true,"actions":[]}]}"#,
    )
    .unwrap()
}

const NO_ROUTE: Reception = Reception::Rejected(ErrorType::NoRouteToTarget);

/// Run until finished; returns (seconds, the total turning in radians, positive =
/// counter-clockwise, how much it turned in each direction).
fn run(w: &mut World) -> (f64, f64, f64, f64) {
    let mut last = w.pose(&id()).unwrap().heading;
    let (mut net, mut ccw, mut cw) = (0.0, 0.0, 0.0);
    for step in 0..8000 {
        w.step();
        let h = w.pose(&id()).unwrap().heading;
        let d = botsim_model::angle::angle_diff(last, h);
        net += d;
        if d > 0.0 {
            ccw += d;
        } else {
            cw -= d;
        }
        last = h;
        if w.phase(&id()) == Ok(Phase::Finished) {
            return (step as f64 * DT, net, ccw, cw);
        }
    }
    panic!("never finished");
}

#[test]
fn a_turn_the_layout_does_not_allow_makes_the_order_refused() {
    // facing east on A → B (east), then B → C (north): a 90° turn on B
    let none = r#""rotationAtStartNodeAllowed":"NONE""#;
    let mut w = world("", none, "DIFFERENTIAL", 0.0);
    assert_eq!(w.send_order(&order("C", "")), Ok(NO_ROUTE));
    let text = format!("{:?}", w.state(&id()).unwrap().errors);
    assert!(text.contains("turn at node 'B'"), "{text}");

    // no turn needed (A → B alone): the rule does not matter
    let mut w = world(none, none, "DIFFERENTIAL", 0.0);
    assert_eq!(w.send_order(&order("B", "")), Ok(Reception::Accepted));
}

#[test]
fn what_the_edge_before_allows_at_its_end_counts_too() {
    // A → B says NONE at its end node, B → C says BOTH at its start: together NONE
    let mut w = world(
        r#""rotationAtEndNodeAllowed":"NONE""#,
        "",
        "DIFFERENTIAL",
        0.0,
    );
    assert_eq!(w.send_order(&order("C", "")), Ok(NO_ROUTE));
    // CW at the end of the first edge, CCW at the start of the second: nothing in common
    let mut w = world(
        r#""rotationAtEndNodeAllowed":"CW""#,
        r#""rotationAtStartNodeAllowed":"CCW""#,
        "DIFFERENTIAL",
        0.0,
    );
    assert_eq!(w.send_order(&order("C", "")), Ok(NO_ROUTE));
}

#[test]
fn one_allowed_direction_is_taken_even_the_long_way() {
    let turn = |bc: &str| {
        let mut w = world("", bc, "DIFFERENTIAL", 0.0);
        assert_eq!(w.send_order(&order("C", "")), Ok(Reception::Accepted));
        run(&mut w)
    };
    let (t_both, net_both, ccw_both, cw_both) = turn("");
    // east → north is 90° counter-clockwise
    assert!((net_both - std::f64::consts::FRAC_PI_2).abs() < 1e-6);
    assert!(cw_both < 1e-9 && (ccw_both - net_both).abs() < 1e-9);

    // CCW allowed: the shorter way, nothing changes
    let (t_ccw, net_ccw, ..) = turn(r#""rotationAtStartNodeAllowed":"CCW""#);
    assert_eq!((t_ccw, net_ccw), (t_both, net_both));

    // only CW allowed: 270° clockwise, which takes longer
    let (t_cw, net_cw, ccw, cw) = turn(r#""rotationAtStartNodeAllowed":"CW""#);
    assert!(ccw < 1e-9, "it never turned counter-clockwise");
    assert!(
        (cw - 3.0 * std::f64::consts::FRAC_PI_2).abs() < 0.06,
        "turned {cw} rad clockwise"
    );
    assert!((net_cw + 3.0 * std::f64::consts::FRAC_PI_2).abs() < 0.06);
    // at 1 rad/s: 3π/2 against π/2 is about 3.1 s longer
    assert!(
        (t_cw - t_both - std::f64::consts::PI).abs() < 0.3,
        "{t_cw} against {t_both}"
    );
}

#[test]
fn the_first_edge_of_an_order_follows_its_start_rule() {
    // the robot faces north and A → B runs east: 90° clockwise
    let half = std::f64::consts::FRAC_PI_2;
    for (rule, accepted) in [
        ("", true),
        (r#""rotationAtStartNodeAllowed":"CW""#, true),
        (r#""rotationAtStartNodeAllowed":"CCW""#, true), // the long way round
        (r#""rotationAtStartNodeAllowed":"NONE""#, false),
    ] {
        let mut w = world(rule, "", "DIFFERENTIAL", half);
        let r = w.send_order(&order("B", "")).unwrap();
        assert_eq!(r == Reception::Accepted, accepted, "{rule}: {r:?}");
    }
}

#[test]
fn an_order_update_continues_from_what_the_order_before_it_planned() {
    // A → B is the base, B → C the horizon; A → B allows nothing at its end node
    let mut w = world(
        r#""rotationAtEndNodeAllowed":"NONE""#,
        "",
        "DIFFERENTIAL",
        0.0,
    );
    assert_eq!(w.send_order(&order("B", "")), Ok(Reception::Accepted));
    for _ in 0..400 {
        w.step();
    }
    assert_eq!(w.phase(&id()), Ok(Phase::Waiting));
    // the update needs a turn on B, which A → B forbids: refused, the robot stays
    assert_eq!(w.send_order(&update()), Ok(NO_ROUTE));
    assert_eq!(w.phase(&id()), Ok(Phase::Waiting));

    // with both ways allowed the same update is accepted and the robot goes on
    let mut w = world("", "", "DIFFERENTIAL", 0.0);
    w.send_order(&order("B", "")).unwrap();
    for _ in 0..400 {
        w.step();
    }
    assert_eq!(w.send_order(&update()), Ok(Reception::Accepted));
    for _ in 0..600 {
        w.step();
    }
    assert_eq!(w.pose(&id()).unwrap().y, 5.0);
}

#[test]
fn omnidirectional_robots_follow_the_rules_when_they_must_turn() {
    let none = r#""rotationAtStartNodeAllowed":"NONE""#;
    // no orientation: the robot keeps its heading and needs no turn
    let mut w = world("", none, "OMNIDIRECTIONAL", 0.0);
    assert_eq!(w.send_order(&order("C", "")), Ok(Reception::Accepted));
    // a global orientation on B → C needs a turn at B: refused
    let mut w = world("", none, "OMNIDIRECTIONAL", 0.0);
    let global = r#","orientation":1.0,"orientationType":"GLOBAL""#;
    assert_eq!(w.send_order(&order("C", global)), Ok(NO_ROUTE));
}

#[test]
fn the_turn_direction_is_reported_in_the_state() {
    let mut w = world(
        "",
        r#""rotationAtStartNodeAllowed":"CW""#,
        "DIFFERENTIAL",
        0.0,
    );
    w.send_order(&order("C", "")).unwrap();
    // wait for the turn on B
    for _ in 0..2000 {
        w.step();
        let omega = w.state(&id()).unwrap().velocity.and_then(|v| v.omega);
        if omega.is_some_and(|o| o != 0.0) && w.pose(&id()).unwrap().x == 10.0 {
            assert!(omega.unwrap() < 0.0, "clockwise is negative: {omega:?}");
            return;
        }
    }
    panic!("it never turned on B");
}

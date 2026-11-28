//! Driving through nodes without stopping (S66).

use botsim_core::actions::ActionTimes;
use botsim_core::door::DoorSpec;
use botsim_core::drive::Phase;
use botsim_core::robot::Reception;
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::instant_actions::InstantActions;
use vda5050::order::Order;
use vdma_lif::Lif;

mod vda_port;
use vda_port::VdaWorld;

const DT: f64 = 0.05;

fn id() -> RobotId {
    RobotId::new("ACME", "R-1")
}

fn spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

/// Nodes P0, P1, … at the given points, joined by edges `P0-P1`, `P1-P2`, …;
/// `extra[i]` is extra JSON for edge i (e.g. `"maximumSpeed":0.5`).
fn graph(points: &[(f64, f64)], extra: &[(usize, &str)]) -> Graph {
    let node = |i: usize, (x, y): (f64, f64)| {
        format!(
            r#"{{"nodeId":"P{i}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]}}]}}"#
        )
    };
    let edge = |i: usize| {
        let e: String = extra
            .iter()
            .filter(|(k, _)| *k == i)
            .map(|(_, s)| format!(",{s}"))
            .collect();
        format!(
            r#"{{"edgeId":"P{i}-P{}","startNodeId":"P{i}","endNodeId":"P{}","mobileRobotTypeEdgeProperties":[
            {{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]{e}}}]}}"#,
            i + 1,
            i + 1
        )
    };
    let nodes: Vec<String> = points
        .iter()
        .enumerate()
        .map(|(i, &p)| node(i, p))
        .collect();
    let edges: Vec<String> = (0..points.len() - 1).map(edge).collect();
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{}],"edges":[{}]}}]}}]}}"#,
        nodes.join(","),
        edges.join(",")
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif).unwrap()
}

/// Five nodes on a line, 6 m apart.
fn line() -> Vec<(f64, f64)> {
    (0..5).map(|i| (6.0 * i as f64, 0.0)).collect()
}

fn world(g: Graph, pass_through: bool) -> World {
    let start = g.node_idx("P0").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(
        id(),
        spec(),
        ActionTimes::new(1.0).with("pick", 2.0),
        start,
        0.0,
    )
    .unwrap();
    w.set_pass_through(pass_through);
    w
}

/// An order along P0 … P(n-1), everything released up to node `released` (the rest
/// is the horizon); `actions[i]` is the JSON action list of node i.
fn order(n: usize, released: usize, actions: &[(usize, &str)]) -> Order {
    let acts = |i: usize| {
        actions
            .iter()
            .find(|(k, _)| *k == i)
            .map_or("[]".to_string(), |(_, a)| a.to_string())
    };
    let nodes: Vec<String> = (0..n)
        .map(|i| {
            format!(
                r#"{{"nodeId":"P{i}","sequenceId":{},"released":{},"actions":{}}}"#,
                2 * i,
                i <= released,
                acts(i)
            )
        })
        .collect();
    let edges: Vec<String> = (0..n - 1)
        .map(|i| {
            format!(
                r#"{{"edgeId":"P{i}-P{}","sequenceId":{},"released":{},"actions":[]}}"#,
                i + 1,
                2 * i + 1,
                i < released
            )
        })
        .collect();
    serde_json::from_str(&format!(
        r#"{{"headerId":1,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{}],"edges":[{}]}}"#,
        nodes.join(","),
        edges.join(",")
    ))
    .unwrap()
}

/// The speed (m/s, along the heading) the robot has at the moment its last node
/// changes to each of `P1`, `P2`, …; and the time (s) when it finished.
fn speeds_at_nodes(w: &mut World) -> (Vec<(String, f64)>, f64) {
    let mut seen: Vec<(String, f64)> = Vec::new();
    let mut last = String::new();
    for step in 0..12000 {
        w.step();
        let s = w.state(&id()).unwrap();
        if s.last_node_id != last {
            last = s.last_node_id.clone();
            let v = s.velocity.and_then(|v| v.vx).unwrap_or(0.0).abs();
            seen.push((last.clone(), v));
        }
        if w.phase(&id()) == Ok(Phase::Finished) {
            return (seen, step as f64 * DT);
        }
    }
    panic!("never finished: {seen:?}");
}

fn at(seen: &[(String, f64)], node: &str) -> f64 {
    seen.iter().find(|(n, _)| n == node).unwrap().1
}

#[test]
fn a_robot_drives_through_the_nodes_on_a_straight_line() {
    let run = |on: bool| {
        let mut w = world(graph(&line(), &[]), on);
        assert_eq!(w.send_order(&order(5, 4, &[])), Ok(Reception::Accepted));
        speeds_at_nodes(&mut w)
    };
    let (off, t_off) = run(false);
    let (on, t_on) = run(true);
    // off: it stands still at each node (the last node changes as it arrives)
    for node in ["P1", "P2", "P3"] {
        assert!(at(&off, node) < 0.05, "{node} off: {}", at(&off, node));
        assert!(at(&on, node) > 1.0, "{node} on: {}", at(&on, node));
    }
    // it stops at the end
    assert!(at(&on, "P4") < 0.05);
    // and the whole trip is much faster
    assert!(t_on + 4.0 < t_off, "on {t_on} s, off {t_off} s");
}

#[test]
fn it_still_stops_where_there_is_something_to_do_or_it_cannot_go_on() {
    let pick = r#"[{"actionId":"a1","actionType":"pick","blockingType":"HARD"}]"#;
    // an action on P2
    let mut w = world(graph(&line(), &[]), true);
    assert_eq!(
        w.send_order(&order(5, 4, &[(2, pick)])),
        Ok(Reception::Accepted)
    );
    let (seen, _) = speeds_at_nodes(&mut w);
    assert!(at(&seen, "P1") > 1.0 && at(&seen, "P2") < 0.05, "{seen:?}");

    // the end of the released route (P3 is the last released node)
    let mut w = world(graph(&line(), &[]), true);
    w.send_order(&order(5, 3, &[])).unwrap();
    for _ in 0..2000 {
        w.step();
    }
    assert_eq!(w.phase(&id()), Ok(Phase::Waiting));
    assert_eq!(w.pose(&id()).unwrap().x, 18.0, "it stopped on P3");

    // a corner: the turn needs a standstill
    let corner = vec![
        (0.0, 0.0),
        (6.0, 0.0),
        (12.0, 0.0),
        (12.0, 6.0),
        (12.0, 12.0),
    ];
    let mut w = world(graph(&corner, &[]), true);
    w.send_order(&order(5, 4, &[])).unwrap();
    let (seen, _) = speeds_at_nodes(&mut w);
    assert!(at(&seen, "P1") > 1.0, "straight on: through");
    assert!(at(&seen, "P2") < 0.05, "the corner: stop and turn");
    assert!(at(&seen, "P3") > 1.0, "straight on again");
}

#[test]
fn a_lower_speed_limit_on_the_next_edge_and_a_short_next_edge_slow_it_down() {
    // P1-P2 is limited to 0.5 m/s
    let mut w = world(graph(&line(), &[(1, r#""maximumSpeed":0.5"#)]), true);
    w.send_order(&order(5, 4, &[])).unwrap();
    let (seen, _) = speeds_at_nodes(&mut w);
    let v = at(&seen, "P1");
    assert!(v > 0.3 && v <= 0.5 + 1e-9, "enters the slow edge at {v}");

    // the next edge is 0.4 m: it must be able to stop at its end (v² ≤ 2 d s)
    let short = vec![(0.0, 0.0), (6.0, 0.0), (6.4, 0.0), (12.4, 0.0)];
    let mut w = world(graph(&short, &[]), true);
    w.send_order(&order(4, 3, &[])).unwrap();
    let (seen, _) = speeds_at_nodes(&mut w);
    let v = at(&seen, "P1");
    let limit = (2.0 * 0.8 * 0.4_f64).sqrt();
    assert!(v > 0.3 && v <= limit + 1e-9, "{v} against {limit}");
}

#[test]
fn it_does_not_drive_through_to_an_edge_it_may_not_start_yet() {
    // a door on P1-P2: the robot stops on P1 and waits for it to be open
    let g = graph(&line(), &[]);
    let e = g.edge_idx("P1-P2").unwrap();
    let mut w = world(g, true);
    w.add_door(DoorSpec {
        id: "D".into(),
        edges: vec![e],
        passage_width: None,
        open_seconds: 2.0,
        close_seconds: 2.0,
        hold_seconds: None,
    })
    .unwrap();
    w.send_order(&order(5, 4, &[])).unwrap();
    for _ in 0..1000 {
        w.step();
    }
    assert_eq!(w.phase(&id()), Ok(Phase::WaitingForClearance));
    assert_eq!(
        w.pose(&id()).unwrap().x,
        6.0,
        "stopped on P1, before the door edge"
    );
    // opened: it goes on and does the rest (P2 → P4) without stopping
    w.open_door("D").unwrap();
    let mut passed_p3 = None;
    for _ in 0..3000 {
        w.step();
        let s = w.state(&id()).unwrap();
        if s.last_node_id == "P3" && passed_p3.is_none() {
            passed_p3 = s.velocity.and_then(|v| v.vx);
        }
    }
    assert!(passed_p3.unwrap() > 1.0);
    assert_eq!(w.pose(&id()).unwrap().x, 24.0);
}

#[test]
fn cancelling_while_driving_through_stops_the_robot_at_the_next_node() {
    let mut w = world(graph(&line(), &[]), true);
    w.send_order(&order(5, 4, &[])).unwrap();
    // drive until it is between P1 and P2 at speed
    for _ in 0..4000 {
        w.step();
        let s = w.state(&id()).unwrap();
        if s.last_node_id == "P1" && s.velocity.and_then(|v| v.vx).unwrap_or(0.0) > 1.0 {
            break;
        }
    }
    let cancel: InstantActions = serde_json::from_str(
        r#"{"headerId":2,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1",
        "actions":[{"actionType":"cancelOrder","actionId":"c1","blockingType":"NONE"}]}"#,
    )
    .unwrap();
    w.send_instant_actions(&cancel).unwrap();
    for _ in 0..800 {
        w.step();
    }
    assert_eq!(w.phase(&id()), Ok(Phase::Finished));
    assert_eq!(w.pose(&id()).unwrap().x, 12.0, "it ended on P2");
    assert_eq!(w.state(&id()).unwrap().last_node_id, "P2");
}

#[test]
fn the_state_stays_valid_and_consistent_while_driving_through() {
    let schema: serde_json::Value =
        serde_json::from_str(include_str!("../../vda/tests/data/state.schema")).unwrap();
    let validator = jsonschema::validator_for(&schema).unwrap();
    let mut w = world(graph(&line(), &[]), true);
    w.send_order(&order(5, 4, &[])).unwrap();
    let mut nodes_seen = Vec::new();
    for _ in 0..1200 {
        w.step();
        let s = w.state(&id()).unwrap();
        // the nodes still to be driven are the ones after the last node
        let after: Vec<&str> = s.node_states.iter().map(|n| n.node_id.as_str()).collect();
        let idx: usize = s.last_node_id.trim_start_matches('P').parse().unwrap_or(0);
        assert!(
            after.iter().all(|n| n[1..].parse::<usize>().unwrap() > idx),
            "{after:?}"
        );
        if nodes_seen.last() != Some(&s.last_node_id) {
            nodes_seen.push(s.last_node_id.clone());
            assert!(validator.is_valid(&serde_json::to_value(&s).unwrap()));
        }
    }
    assert_eq!(nodes_seen, vec!["P0", "P1", "P2", "P3", "P4"]);
}

#[test]
fn a_run_with_driving_through_is_repeatable() {
    let run = || {
        let mut w = world(graph(&line(), &[]), true);
        w.send_order(&order(5, 4, &[])).unwrap();
        (0..1500)
            .map(|_| {
                w.step();
                w.pose(&id()).unwrap().x.to_bits()
            })
            .collect::<Vec<_>>()
    };
    assert_eq!(run(), run());
}

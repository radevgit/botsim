//! The server with the traffic module behind its slot, on a hand-written layout
//! and robot type.

use botsim_model::graph::Graph;
use botsim_model::spec::{Kinematics, Shape};
use botsim_model::waiting::{HeldBack, WaitReason};
use botsim_server::controller::Controller;
use botsim_server::traffic_policy::TrafficAdapter;
use botsim_space::{MapOptions, TypeRef};
use vdma_lif::Lif;

mod hand;
use hand::{config, cycle, job, orders, report};

/// A layout of these nodes (id, x, y) and directed edges (from, to) for the type
/// ACME/X1; edge ids are "from-to".
fn lif(nodes: &[(&str, f64, f64)], edges: &[(&str, &str)]) -> Lif {
    let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let nodes: Vec<String> = nodes
        .iter()
        .map(|(id, x, y)| {
            format!(
                r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
                "mobileRobotTypeNodeProperties":[{ty}}}]}}"#
            )
        })
        .collect();
    let edges: Vec<String> = edges
        .iter()
        .map(|(a, b)| {
            format!(
                r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}",
                "mobileRobotTypeEdgeProperties":[{ty},
                "orientationType":"TANGENTIAL","mobileRobotOrientations":[0.0]}}]}}"#
            )
        })
        .collect();
    serde_json::from_str(&format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-20T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{}],"edges":[{}]}}]}}]}}"#,
        nodes.join(","),
        edges.join(",")
    ))
    .unwrap()
}

/// The type ACME/X1, 1.2 m by 0.8 m.
fn x1() -> TypeRef {
    TypeRef {
        name: "ACME/X1".into(),
        manufacturer: "ACME".into(),
        series: "X1".into(),
        kinematics: Kinematics::Differential,
        length: 1.2,
        width: 0.8,
        field_margin: 0.3,
        field_from_factsheet: true,
        shape: Shape::Rect {
            cx: 0.0,
            cy: 0.0,
            half_length: 0.6,
            half_width: 0.4,
        },
    }
}

#[test]
fn under_the_traffic_rules_a_silent_robot_keeps_its_way_and_gets_no_more() {
    let layout = lif(
        &[
            ("A", -10.0, 0.0),
            ("M", 0.0, 0.0),
            ("B", 10.0, 0.0),
            ("C", 0.0, -10.0),
            ("D", 0.0, 10.0),
        ],
        &[("A", "M"), ("M", "B"), ("C", "M"), ("M", "D")],
    );
    let g = Graph::from_lif(&layout).unwrap();
    let [a, m, b, c, d] = ["A", "M", "B", "C", "D"].map(|id| g.node_idx(id).unwrap());
    let adapter = TrafficAdapter::new(&layout, &[x1()], &MapOptions::default()).unwrap();
    let mut ctl = Controller::with_policy(&config(g.clone()), adapter);
    let (first, second) = ([a, m, b], [c, m, d]);
    let idle = [
        report(&g, 0, "", &[a], 0, false),
        report(&g, 1, "", &[c], 0, false),
    ];
    cycle(&mut ctl, 0, &idle, [job(1, a, b), job(2, c, d)]);

    // the first is silent: the second waits for its way
    let waiting = [report(&g, 1, "job-2", &second, 0, false)];
    for tick in [1, 2, 100_000] {
        let out = cycle(&mut ctl, tick, &waiting, []);
        assert!(out.messages.is_empty(), "tick {tick}: {:?}", out.messages);
        let held = HeldBack {
            reason: WaitReason::Release,
            blockers: vec![0],
        };
        assert_eq!(out.held_back, vec![None, Some(held)], "tick {tick}");
    }

    // the second is silent while the first leaves the crossing and arrives:
    // nothing for the second
    let gone = [report(&g, 0, "job-1", &first, 2, false)];
    let out = cycle(&mut ctl, 100_001, &gone, []);
    assert!(out.messages.is_empty(), "{:?}", out.messages);
    let out = cycle(&mut ctl, 100_002, &waiting, []);
    assert_eq!(orders(&out), [(1, 1, vec![c, m, d])]);
}

#[test]
fn the_second_robot_waits_for_the_first_at_a_crossing() {
    // A → M → B and C → M → D cross at M
    let layout = lif(
        &[
            ("A", -10.0, 0.0),
            ("M", 0.0, 0.0),
            ("B", 10.0, 0.0),
            ("C", 0.0, -10.0),
            ("D", 0.0, 10.0),
        ],
        &[("A", "M"), ("M", "B"), ("C", "M"), ("M", "D")],
    );
    let g = Graph::from_lif(&layout).unwrap();
    let [a, m, b, c, d] = ["A", "M", "B", "C", "D"].map(|id| g.node_idx(id).unwrap());
    let adapter = TrafficAdapter::new(&layout, &[x1()], &MapOptions::default()).unwrap();
    let mut ctl = Controller::with_policy(&config(g.clone()), adapter);
    let (first, second) = ([a, m, b], [c, m, d]);

    let idle = [
        report(&g, 0, "", &[a], 0, false),
        report(&g, 1, "", &[c], 0, false),
    ];
    let out = cycle(&mut ctl, 0, &idle, [job(1, a, b), job(2, c, d)]);
    assert_eq!(orders(&out), [(0, 0, vec![a, m, b]), (1, 0, vec![c])]);

    // the first drives to the crossing and onto it: the second waits for it
    for (tick, at) in [(1, 0), (2, 1)] {
        let states = [
            report(&g, 0, "job-1", &first, at, true),
            report(&g, 1, "job-2", &second, 0, false),
        ];
        let out = cycle(&mut ctl, tick, &states, []);
        assert!(out.messages.is_empty(), "tick {tick}: {:?}", out.messages);
        let held = HeldBack {
            reason: WaitReason::Release,
            blockers: vec![0],
        };
        assert_eq!(out.held_back, vec![None, Some(held)], "tick {tick}");
    }

    // it has left the crossing
    let states = [
        report(&g, 0, "job-1", &first, 2, false),
        report(&g, 1, "job-2", &second, 0, false),
    ];
    let out = cycle(&mut ctl, 3, &states, []);
    assert_eq!(orders(&out), [(1, 1, vec![c, m, d])]);
    assert_eq!(ctl.completed().len(), 1);
}

#[test]
fn a_layout_that_gives_no_graph_is_an_error() {
    // an edge to a node that is not there
    let layout = lif(&[("A", 0.0, 0.0)], &[("A", "Z")]);
    let error = TrafficAdapter::new(&layout, &[x1()], &MapOptions::default()).unwrap_err();
    assert_eq!(
        error,
        "no conflict map: the layout cannot be turned into a graph"
    );
}

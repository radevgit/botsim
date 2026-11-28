//! A report that is further along its route than what has been released to the
//! robot does not fit what was sent. Under both traffic rules the controller then
//! releases nothing of that robot's route, and goes on when a report fits again.

use botsim_baseline::baseline_policy::BaselinePolicy;
use botsim_model::graph::Graph;
use botsim_server::controller::Controller;
use botsim_server::policy::TrafficPolicy;
use botsim_server::traffic_policy::TrafficAdapter;
use botsim_space::{MapOptions, TypeRef};
use vdma_lif::Lif;

#[path = "../../server/tests/hand/mod.rs"]
mod hand;
use hand::{config, cycle, job, orders, report};

/// The line A → B → C → D → E, and the node F beside it, for the type ACME/X1.
fn layout() -> Lif {
    let types = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let nodes: Vec<String> = [
        ("A", 0.0, 0.0),
        ("B", 10.0, 0.0),
        ("C", 20.0, 0.0),
        ("D", 30.0, 0.0),
        ("E", 40.0, 0.0),
        ("F", 0.0, 20.0),
    ]
    .iter()
    .map(|(id, x, y)| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
        )
    })
    .collect();
    let edges: Vec<String> = ["AB", "BC", "CD", "DE"]
        .iter()
        .map(|id| {
            format!(
                r#"{{"edgeId":"{id}","startNodeId":"{}","endNodeId":"{}",
                "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":{types}}}]}}"#,
                &id[..1],
                &id[1..]
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

/// R-1 gets the job from A to E, released as far as C; then it reports D and E,
/// then B.
fn nothing_beyond_the_release<P: TrafficPolicy>(
    mut ctl: Controller<P>,
    graph: &Graph,
    held: impl Fn(&Controller<P>) -> Vec<(usize, Vec<usize>)>,
) {
    let line = ["A", "B", "C", "D", "E"].map(|id| graph.node_idx(id).unwrap());
    let aside = graph.node_idx("F").unwrap();
    let other = report(graph, 1, "", &[aside], 0, false);
    let idle = [report(graph, 0, "", &line[..1], 0, false), other.clone()];
    let out = cycle(&mut ctl, 0, &idle, [job(1, line[0], line[4])]);
    assert_eq!(orders(&out), [(0, 0, line[..3].to_vec())]);
    let before = held(&ctl);

    // one node beyond what is released, and two (the last): driving, so the job
    // is not over
    for (tick, step) in [(1, 3), (2, 4), (3, 3)] {
        let states = [report(graph, 0, "job-1", &line, step, true), other.clone()];
        let out = cycle(&mut ctl, tick, &states, []);
        assert!(out.messages.is_empty(), "at {step}: {:?}", out.messages);
        assert_eq!(held(&ctl), before, "at {step}");
    }
    assert!(ctl.completed().is_empty() && ctl.rejected().is_empty());
    assert_eq!(ctl.busy(), 1);

    // a report within the release: the way goes on from what was released
    let states = [report(graph, 0, "job-1", &line, 1, true), other];
    let out = cycle(&mut ctl, 4, &states, []);
    assert_eq!(orders(&out), [(0, 1, line[2..4].to_vec())]);
}

#[test]
fn under_the_baseline_rules_no_route_is_released_beyond_the_release() {
    let graph = Graph::from_lif(&layout()).unwrap();
    let ctl = Controller::with_policy(&config(graph.clone()), BaselinePolicy::new(&graph));
    nothing_beyond_the_release(ctl, &graph, |c| {
        let held = c.policy().reservations();
        held.into_iter().map(|r| (r.robot, r.nodes)).collect()
    });
}

#[test]
fn under_the_traffic_rules_no_route_is_released_beyond_the_release() {
    let lif = layout();
    let graph = Graph::from_lif(&lif).unwrap();
    let x1 = TypeRef::test_rect(
        "ACME/X1",
        botsim_model::spec::Kinematics::Differential,
        1.2,
        0.8,
    );
    let adapter = TrafficAdapter::new(&lif, &[x1], &MapOptions::default()).unwrap();
    let ctl = Controller::with_policy(&config(graph.clone()), adapter);
    // (the module says nothing of what is held)
    nothing_beyond_the_release(ctl, &graph, |_| Vec::new());
}

use botsim_baseline::order_builder::{
    OrderHeader, Plan, RELEASE_WINDOW, build_order as try_build_order,
};
use botsim_core::actions::ActionTimes;
use botsim_core::graph::Graph;
use botsim_core::jobs::Job;
use botsim_core::robot::Reception::Accepted;
use botsim_core::spec::RobotSpec;
use botsim_core::world::{RobotId, World};
use vda5050::factsheet::Factsheet;
use vda5050::state::ActionStatus::{Finished, Running};

mod common;
use common::graph;

/// The order message; the tests build only valid ones.
fn build_order(
    g: &Graph,
    h: &OrderHeader,
    p: &Plan,
    from: usize,
    through: usize,
) -> vda5050::order::Order {
    try_build_order(g, h, p, from, through).unwrap()
}

const DT: f64 = 0.05;

/// N0 – N1 – … – N6, 5 m apart, one way.
fn chain() -> Graph {
    let nodes: Vec<(String, f64, f64)> = (0..7)
        .map(|i| (format!("N{i}"), 5.0 * i as f64, 0.0))
        .collect();
    let nodes: Vec<(&str, f64, f64)> = nodes.iter().map(|(n, x, y)| (n.as_str(), *x, *y)).collect();
    let edges: Vec<(String, String)> = (0..6)
        .map(|i| (format!("N{i}"), format!("N{}", i + 1)))
        .collect();
    let edges: Vec<(&str, &str)> = edges
        .iter()
        .map(|(a, b)| (a.as_str(), b.as_str()))
        .collect();
    graph(&nodes, &edges)
}

fn job(g: &Graph, pickup: &str, drop: &str) -> Job {
    Job {
        id: 1,
        pickup: g.node_idx(pickup).unwrap(),
        drop: g.node_idx(drop).unwrap(),
        created_tick: 0,
    }
}

fn action_nodes(plan: &Plan) -> Vec<(usize, Vec<&str>)> {
    plan.actions
        .iter()
        .enumerate()
        .filter(|(_, a)| !a.is_empty())
        .map(|(i, a)| (i, a.iter().map(String::as_str).collect()))
        .collect()
}

#[test]
fn a_job_plan_drives_to_the_pickup_then_the_drop() {
    let g = chain();
    let p = Plan::for_job(&g, g.node_idx("N0").unwrap(), &job(&g, "N2", "N5")).unwrap();
    assert_eq!(p.nodes.len(), 6, "N0 … N5");
    assert_eq!(action_nodes(&p), vec![(2, vec!["pick"]), (5, vec!["drop"])]);

    // the robot already stands on the pickup node: pick is on the first node
    let p = Plan::for_job(&g, g.node_idx("N2").unwrap(), &job(&g, "N2", "N4")).unwrap();
    assert_eq!(action_nodes(&p), vec![(0, vec!["pick"]), (2, vec!["drop"])]);

    // unreachable (the chain is one-way): no plan
    assert!(Plan::for_job(&g, g.node_idx("N4").unwrap(), &job(&g, "N1", "N5")).is_none());
    assert!(Plan::for_job(&g, g.node_idx("N0").unwrap(), &job(&g, "N5", "N1")).is_none());
}

#[test]
fn the_order_carries_the_actions_with_ids_and_hard_blocking() {
    let g = chain();
    let p = Plan::for_job(&g, g.node_idx("N0").unwrap(), &job(&g, "N2", "N5")).unwrap();
    let r = RobotId::new("ACME", "R-001");
    let h = OrderHeader {
        robot: &r,
        header_id: 1,
        timestamp: "2026-11-20T10:00:00.000Z",
        order_id: "j1",
        order_update_id: 0,
    };
    let o = build_order(&g, &h, &p, 0, p.released_through(0, RELEASE_WINDOW));
    let acts = |i: usize| -> Vec<(String, String)> {
        o.nodes[i]
            .actions
            .iter()
            .map(|a| (a.action_id.clone(), a.action_type.clone()))
            .collect()
    };
    assert!(acts(0).is_empty() && acts(1).is_empty());
    assert_eq!(acts(2), vec![("pick-2".to_string(), "pick".to_string())]);
    assert_eq!(acts(5), vec![("drop-5".to_string(), "drop".to_string())]);
    assert!(
        o.nodes[2]
            .actions
            .iter()
            .all(|a| a.blocking_type == vda5050::common::BlockingType::Hard)
    );
}

#[test]
fn a_robot_runs_a_whole_job_in_the_world() {
    let g = chain();
    let n0 = g.node_idx("N0").unwrap();
    let p = Plan::for_job(&g, n0, &job(&g, "N2", "N5")).unwrap();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let r = RobotId::new("ACME", "R-001");
    let times = ActionTimes::new(1.0).with("pick", 5.0).with("drop", 5.0);
    w.add_robot(
        r.clone(),
        RobotSpec::from_factsheet(&f).unwrap(),
        times,
        n0,
        0.0,
    )
    .unwrap();

    let header = |update| OrderHeader {
        robot: &r,
        header_id: 1 + update,
        timestamp: "2026-11-20T10:00:00.000Z",
        order_id: "j1",
        order_update_id: update,
    };
    let o = build_order(
        w.graph(),
        &header(0),
        &p,
        0,
        p.released_through(0, RELEASE_WINDOW),
    );
    assert_eq!(w.send_order(&o), Ok(Accepted));

    // it reaches N2 and picks for 5 s: it stands still there
    let mut ticks_picking = 0;
    for _ in 0..6000 {
        w.step();
        let s = w.state(&r).unwrap();
        if s.action_states
            .iter()
            .any(|a| a.action_id == "pick-2" && a.action_status == Running)
        {
            assert_eq!(s.last_node_id, "N2");
            assert!(!s.driving);
            ticks_picking += 1;
        }
    }
    assert_eq!(ticks_picking, 100, "5 s at 50 ms");

    // the window (2) ended at N2, where the pick was done: update 1 releases up
    // to N4 (the robot is at N2, the decision node)
    let s = w.state(&r).unwrap();
    assert_eq!(s.last_node_id, "N2");
    let u = build_order(
        w.graph(),
        &header(1),
        &p,
        2,
        p.released_through(2, RELEASE_WINDOW),
    );
    assert_eq!(w.send_order(&u), Ok(Accepted));
    for _ in 0..6000 {
        w.step();
    }
    assert_eq!(w.state(&r).unwrap().last_node_id, "N4", "waits at N4");

    // update 2 releases the last node, N5, where the drop is done
    let u = build_order(
        w.graph(),
        &header(2),
        &p,
        4,
        p.released_through(4, RELEASE_WINDOW),
    );
    assert_eq!(w.send_order(&u), Ok(Accepted));
    for _ in 0..6000 {
        w.step();
    }
    let s = w.state(&r).unwrap();
    assert_eq!(s.last_node_id, "N5");
    let status = |id: &str| {
        s.action_states
            .iter()
            .find(|a| a.action_id == id)
            .unwrap()
            .action_status
    };
    assert_eq!(status("pick-2"), Finished);
    assert_eq!(status("drop-5"), Finished);
    assert!(w.collisions().is_empty());
}

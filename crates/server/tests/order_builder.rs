use std::sync::LazyLock;

use botsim_core::actions::ActionTimes;
use botsim_core::robot::Reception::Accepted;
use botsim_core::world::World;
use botsim_link::ids::OrderId;
use botsim_link::message::Millis;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_model::time::unix_ms_from_utc;
use botsim_router::route::shortest_path;
use botsim_server::cycle::robot_no;
use botsim_server::order_builder::{
    OrderHeader, Plan, RELEASE_WINDOW, build_order as try_build_order,
};
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;

mod common;
use common::graph;
mod translate;

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

fn robot() -> RobotId {
    RobotId::new("ACME", "R-001")
}

/// The header of an order of job 1 to [`robot`], the first of its fleet.
fn header(update: u32) -> OrderHeader<'static> {
    static JOB_1: LazyLock<OrderId> = LazyLock::new(|| OrderId::new("job-1"));
    OrderHeader {
        robot: robot_no(0),
        seq: 1 + update,
        time: Millis(unix_ms_from_utc(2026, 11, 20, 10, 0, 0)),
        order_id: &JOB_1,
        order_update_id: update,
    }
}

fn plan(g: &Graph, from: &str, to: &str) -> Plan {
    let start = g.node_idx(from).unwrap();
    let edges = shortest_path(g, start, g.node_idx(to).unwrap()).unwrap();
    Plan::new(g, start, edges)
}

fn nodes(o: &Order) -> Vec<(&str, u32, bool)> {
    o.nodes
        .iter()
        .map(|n| (n.node_id.as_str(), n.sequence_id, n.released))
        .collect()
}

fn edges(o: &Order) -> Vec<(&str, u32, bool)> {
    o.edges
        .iter()
        .map(|e| (e.edge_id.as_str(), e.sequence_id, e.released))
        .collect()
}

#[test]
fn the_window_is_two_nodes() {
    assert_eq!(RELEASE_WINDOW, 2);
}

#[test]
fn a_new_order_releases_the_robot_node_and_two_more() {
    let g = chain();
    let p = plan(&g, "N0", "N6");
    let o = build_order(&g, &header(0), &p, 0, p.released_through(0, RELEASE_WINDOW));

    assert_eq!((o.order_id.as_str(), o.order_update_id), ("job-1", 0));
    assert_eq!(
        (o.manufacturer.as_str(), o.serial_number.as_str()),
        ("ACME", "R-001")
    );
    assert_eq!(
        nodes(&o),
        vec![
            ("N0", 0, true),
            ("N1", 2, true),
            ("N2", 4, true),
            ("N3", 6, false),
            ("N4", 8, false),
            ("N5", 10, false),
            ("N6", 12, false),
        ]
    );
    assert_eq!(
        edges(&o),
        vec![
            ("N0-N1", 1, true),
            ("N1-N2", 3, true),
            ("N2-N3", 5, false),
            ("N3-N4", 7, false),
            ("N4-N5", 9, false),
            ("N5-N6", 11, false),
        ]
    );
}

#[test]
fn an_update_starts_at_the_decision_node_and_keeps_the_numbering() {
    let g = chain();
    let p = plan(&g, "N0", "N6");
    // robot is at N2 (the decision node): release through N4, N5 and N6 stay horizon
    let o = build_order(&g, &header(1), &p, 2, p.released_through(2, RELEASE_WINDOW));
    assert_eq!(o.order_update_id, 1);
    assert_eq!(
        nodes(&o),
        vec![
            ("N2", 4, true),
            ("N3", 6, true),
            ("N4", 8, true),
            ("N5", 10, false),
            ("N6", 12, false)
        ]
    );
    assert_eq!(
        edges(&o),
        vec![
            ("N2-N3", 5, true),
            ("N3-N4", 7, true),
            ("N4-N5", 9, false),
            ("N5-N6", 11, false)
        ]
    );
}

#[test]
fn a_short_route_has_no_horizon_and_a_one_node_order_is_valid() {
    let g = chain();
    let p = plan(&g, "N0", "N2");
    let o = build_order(&g, &header(0), &p, 0, p.released_through(0, RELEASE_WINDOW));
    assert!(o.nodes.iter().all(|n| n.released) && o.edges.iter().all(|e| e.released));

    let p = plan(&g, "N4", "N4"); // already there: just the start node
    let o = build_order(&g, &header(0), &p, 0, 0);
    assert_eq!(nodes(&o), vec![("N4", 0, true)]);
    assert!(o.edges.is_empty());
}

/// The order message as VDA 5050; the tests build only valid ones.
fn build_order(g: &Graph, h: &OrderHeader, p: &Plan, from: usize, through: usize) -> Order {
    let message = try_build_order(g, h, p, from, through).unwrap();
    translate::vda_order(g, &robot(), &message)
}

#[test]
fn the_first_node_must_be_released() {
    let g = chain();
    let p = plan(&g, "N0", "N6");
    // from node 4, released only through node 2: no message
    assert!(try_build_order(&g, &header(1), &p, 4, 2).is_none());
}

#[test]
fn a_robot_drives_the_order_and_the_update_in_the_world() {
    let g = chain();
    let n0 = g.node_idx("N0").unwrap();
    let p = plan(&g, "N0", "N6");
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let r = robot();
    w.add_robot(
        r.clone(),
        RobotSpec::from_factsheet(&f).unwrap(),
        ActionTimes::new(1.0),
        n0,
        0.0,
    )
    .unwrap();

    // the order is accepted by the robot as it is
    let o = build_order(
        w.graph(),
        &header(0),
        &p,
        0,
        p.released_through(0, RELEASE_WINDOW),
    );
    assert_eq!(w.send_order(&o), Ok(Accepted));
    for _ in 0..4000 {
        w.step();
    }
    // it stopped at the end of the base (N2) and waits for the update
    let s = w.state(&r).unwrap();
    assert_eq!(s.last_node_id, "N2");
    assert!(!s.driving);
    assert_eq!(s.node_states.len(), 4, "N3 … N6 are the horizon");

    // update 1 (from N2) releases N3 and N4; the robot stops at N4
    for (update, from) in [(1, 2), (2, 4)] {
        let u = build_order(
            w.graph(),
            &header(update),
            &p,
            from,
            p.released_through(from, RELEASE_WINDOW),
        );
        assert_eq!(w.send_order(&u), Ok(Accepted));
        for _ in 0..4000 {
            w.step();
        }
    }
    // update 2 (from N4) released N5 and N6
    let s = w.state(&r).unwrap();
    assert_eq!((s.last_node_id.as_str(), s.order_update_id), ("N6", 2));
    assert!((w.pose(&r).unwrap().x - 30.0).abs() < 1e-9);
}

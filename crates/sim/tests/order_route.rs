// 3.14159 is deliberate: an orientation "about π" as written in real orders.
#![allow(clippy::approx_constant)]

use botsim_model::graph::Graph;
use botsim_model::orientation::OrientationType;
use botsim_sim::drive::RouteEdge;
use botsim_sim::order_route::OrderError;
use vda5050::order::Order;
use vdma_lif::Lif;

mod vda_port;
use vda_port::{Refused, route_from_order};

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

/// Order A -> B -> C with the given release flags
/// (node A, edge A-B, node B, edge B-C, node C).
fn order(rel: [bool; 5]) -> Order {
    let json = format!(
        r#"{{
        "headerId": 1, "timestamp": "2026-11-20T10:00:00.000Z", "version": "3.0.1",
        "manufacturer": "ACME", "serialNumber": "R-001",
        "orderId": "o1", "orderUpdateId": 0,
        "nodes": [
            {{"nodeId": "A", "sequenceId": 0, "released": {}, "actions": []}},
            {{"nodeId": "B", "sequenceId": 2, "released": {}, "actions": []}},
            {{"nodeId": "C", "sequenceId": 4, "released": {}, "actions": []}}
        ],
        "edges": [
            {{"edgeId": "A-B", "sequenceId": 1, "released": {}, "orientation": 0.0, "orientationType": "TANGENTIAL", "actions": []}},
            {{"edgeId": "B-C", "sequenceId": 3, "released": {}, "orientation": 3.14159, "actions": []}}
        ]
    }}"#,
        rel[0], rel[2], rel[4], rel[1], rel[3]
    );
    serde_json::from_str(&json).unwrap()
}

#[test]
fn fully_released_order_becomes_a_route_with_orientations() {
    let g = graph();
    let r = route_from_order(&g, &order([true; 5])).unwrap();
    assert_eq!(r.start, g.node_idx("A").unwrap());
    assert!(!r.has_horizon);
    assert_eq!(
        r.edges,
        vec![
            RouteEdge {
                edge: g.edge_idx("A-B").unwrap(),
                orientation: Some(0.0),
                orientation_type: Some(OrientationType::Tangential),
            },
            RouteEdge {
                edge: g.edge_idx("B-C").unwrap(),
                orientation: Some(3.14159),
                orientation_type: None,
            },
        ]
    );
}

#[test]
fn only_the_released_base_is_in_the_route() {
    let g = graph();
    // A, A-B, B released; B-C and C are the horizon
    let r = route_from_order(&g, &order([true, true, true, false, false])).unwrap();
    assert!(r.has_horizon);
    assert_eq!(r.edges.len(), 1);
    assert_eq!(r.edges[0].edge, g.edge_idx("A-B").unwrap());

    // only the first node released: empty base, everything else horizon
    let r = route_from_order(&g, &order([true, false, false, false, false])).unwrap();
    assert!(r.has_horizon && r.edges.is_empty());
}

// (the structure of the lists and the release rules are the translator's checks:
// `botsim-vda`, `tests/refused_orders.rs`)

#[test]
fn a_new_order_starts_at_sequence_id_zero_and_an_update_may_continue() {
    let g = graph();

    // a new order must start at sequenceId 0 ...
    let mut o = order([true; 5]);
    for (i, n) in o.nodes.iter_mut().enumerate() {
        n.sequence_id = 10 + 2 * i as u32;
    }
    for (i, e) in o.edges.iter_mut().enumerate() {
        e.sequence_id = 11 + 2 * i as u32;
    }
    assert_eq!(
        route_from_order(&g, &o),
        Err(Refused::Simulator(OrderError::FirstNodeNotZero {
            found: 10
        }))
    );
    // ... an update may continue the numbering
    o.order_update_id = 1;
    assert!(route_from_order(&g, &o).is_ok());
}

#[test]
fn ids_and_connections_are_checked_against_the_layout() {
    let g = graph();

    // (names the layout does not have are refused by the translator)
    let mut o = order([true; 5]);
    o.nodes[2].node_id = "Z".into();
    assert!(matches!(
        route_from_order(&g, &o),
        Err(Refused::Translator(_))
    ));

    // edge C-B exists but runs the other way than B -> C
    let mut o = order([true; 5]);
    o.edges[1].edge_id = "C-B".into();
    assert_eq!(
        route_from_order(&g, &o),
        Err(Refused::Simulator(OrderError::EdgeDoesNotConnect {
            edge: "C-B".into()
        }))
    );
}

#[test]
fn numbers_outside_the_layout_are_refused() {
    use botsim_link::ids::OrderId;
    use botsim_link::order::{Leg, Order};
    use botsim_sim::order_route;

    let g = graph();
    let leg = |edge, end| Leg {
        edge,
        end,
        orientation: None,
        orientation_type: None,
    };
    let refused = |start, legs: Vec<Leg>| {
        // (the horizon is checked like the base)
        let order = Order::new(OrderId::new("o"), 0, 0, start, legs, 0, Vec::new()).unwrap();
        order_route::route_from_order(&g, &order).unwrap_err()
    };
    let outside = |what, number| OrderError::OutsideLayout { what, number };
    assert_eq!(refused(99, Vec::new()), outside("node", 99));
    assert_eq!(refused(0, vec![leg(0, 99)]), outside("node", 99));
    assert_eq!(
        refused(0, vec![leg(usize::MAX, 1)]),
        outside("edge", usize::MAX)
    );
}

#[test]
fn route_from_an_order_can_be_driven() {
    use botsim_model::spec::RobotSpec;
    use botsim_sim::drive::Driver;
    use vda5050::factsheet::Factsheet;

    let g = graph();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();

    // B-C has orientation ≈ π: the robot drives it backwards and ends facing south
    let r = route_from_order(&g, &order([true; 5])).unwrap();
    let mut d = Driver::new(&g, &spec, r.start, 0.0, &r.edges).unwrap();
    let mut steps = 0;
    while !d.is_finished() {
        d.step(0.05);
        steps += 1;
        assert!(steps < 10_000);
    }
    let p = d.pose();
    assert!((p.x - 10.0).abs() < 1e-9 && (p.y - 5.0).abs() < 1e-9);
    assert!(
        (p.heading + std::f64::consts::FRAC_PI_2).abs() < 0.01,
        "{}",
        p.heading
    );
}

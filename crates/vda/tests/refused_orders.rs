//! The refusals of an order that the simulator made itself while it read VDA 5050
//! messages (their tests were in `botsim-sim`): the structure of the lists, the
//! release rules and names the layout does not have. The simulator reads `link`
//! orders, which cannot be written that way, so the translator makes these
//! checks, with the error type and references a robot refuses with.

// 3.14159 is deliberate: an orientation "about π" as written in real orders.
#![allow(clippy::approx_constant)]

use botsim_model::errors::ErrorType;
use botsim_vda::report::{Issue, Item, Problem};
use vda5050::order::Order;

mod common;
use common::translator;

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

fn refused(order: &Order) -> Issue {
    translator().order_from_vda(order).unwrap_err()
}

#[test]
fn release_rules_are_checked() {
    let problem = |rel| refused(&order(rel)).problem;
    assert_eq!(
        problem([false, false, false, false, false]),
        Problem::FirstNodeNotReleased
    );
    // edge B-C released but node C is not
    assert_eq!(
        problem([true, true, true, true, false]),
        Problem::EdgeReleasedWithoutNode { edge: "B-C".into() }
    );
    // unreleased edge A-B followed by a released node B (and edge B-C)
    assert_eq!(
        problem([true, false, true, true, true]),
        Problem::ReleasedAfterHorizon { id: "B".into() }
    );
    // unreleased edge B-C followed by a released node C
    assert_eq!(
        problem([true, true, true, false, true]),
        Problem::ReleasedAfterHorizon { id: "C".into() }
    );
}

#[test]
fn structure_is_checked() {
    let mut o = order([true; 5]);
    o.edges.pop();
    assert_eq!(
        refused(&o).problem,
        Problem::WrongCounts { nodes: 3, edges: 1 }
    );

    let mut o = order([true; 5]);
    o.nodes.clear();
    o.edges.clear();
    assert!(matches!(refused(&o).problem, Problem::WrongCounts { .. }));

    let mut o = order([true; 5]);
    o.edges[0].sequence_id = 5;
    assert_eq!(
        refused(&o).problem,
        Problem::BadSequenceId {
            field: "edges[].sequenceId",
            id: "A-B".into(),
            expected: 1,
            found: 5
        }
    );
}

#[test]
fn ids_are_checked_against_the_layout() {
    let mut o = order([true; 5]);
    o.nodes[2].node_id = "Z".into();
    assert_eq!(
        refused(&o).problem,
        Problem::UnknownName {
            item: Item::Node,
            name: "Z".into()
        }
    );

    let mut o = order([true; 5]);
    o.edges[1].edge_id = "X-Y".into();
    assert_eq!(
        refused(&o).problem,
        Problem::UnknownName {
            item: Item::Edge,
            name: "X-Y".into()
        }
    );
}

#[test]
fn order_refusals_have_the_predefined_types_with_the_prescribed_references() {
    let references = |issue: &Issue| {
        issue
            .references
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect::<Vec<_>>()
    };
    let mut o = order([false; 5]);
    o.order_update_id = 3;
    let r = refused(&o);
    assert_eq!(r.error, Some(ErrorType::ValidationFailure));
    assert_eq!(
        references(&r),
        vec![
            ("orderId".to_string(), "o1".to_string()),
            ("orderUpdateId".to_string(), "3".to_string())
        ]
    );

    let mut o = order([true; 5]);
    o.nodes[2].node_id = "Z".into();
    o.order_update_id = 3;
    let r = refused(&o);
    assert_eq!(r.error, Some(ErrorType::NoRouteToTarget));
    assert_eq!(
        references(&r),
        vec![("orderId".to_string(), "o1".to_string())]
    );
}

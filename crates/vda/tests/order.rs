//! Orders that are refused, each with its error type and references, and what of an
//! order is left to the robot.

// 3.14159 is deliberate: an orientation "about π" as written in real orders.
#![allow(clippy::approx_constant)]

mod common;

use botsim_link::ids::{ActionId, ActionType, OrderId};
use botsim_link::message::{Millis, ToRobot, ToRobotBody};
use botsim_link::order::{Leg, NodeAction, Order, OrientationType};
use botsim_model::errors::ErrorType;
use botsim_model::robot::RobotNo;
use botsim_vda::report::{Issue, Item, Problem};
use common::*;
use vda5050::order as vda;

/// Order A -> B -> C with the given release flags
/// (node A, edge A-B, node B, edge B-C, node C), as the tests of the robot's order
/// checks have it.
fn message(rel: [bool; 5]) -> vda::Order {
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

fn accepted(m: &vda::Order) -> Order {
    let out = translator().order_from_vda(m).unwrap();
    assert_eq!(out.issues, Vec::new());
    match out.message.body {
        ToRobotBody::Order(o) => o,
        other => panic!("{other:?}"),
    }
}

fn refused(m: &vda::Order) -> Issue {
    translator().order_from_vda(m).unwrap_err()
}

fn references(pairs: &[(&str, &str)]) -> Vec<(std::sync::Arc<str>, std::sync::Arc<str>)> {
    pairs.iter().map(|&(k, v)| (k.into(), v.into())).collect()
}

/// `VALIDATION_FAILURE` names the order and the update.
#[track_caller]
fn invalid(m: &vda::Order, problem: Problem) {
    let issue = refused(m);
    assert_eq!(issue.problem, problem);
    assert_eq!(issue.error, Some(ErrorType::ValidationFailure));
    let update = m.order_update_id.to_string();
    assert_eq!(
        issue.references,
        references(&[("orderId", &m.order_id), ("orderUpdateId", &update)])
    );
}

/// `NO_ROUTE_TO_TARGET` names the order.
#[track_caller]
fn no_route(m: &vda::Order, item: Item, name: &str) {
    let issue = refused(m);
    assert_eq!(
        issue.problem,
        Problem::UnknownName {
            item,
            name: name.to_string()
        }
    );
    assert_eq!(issue.error, Some(ErrorType::NoRouteToTarget));
    assert_eq!(issue.references, references(&[("orderId", &m.order_id)]));
}

#[test]
fn a_released_order_is_taken_with_its_orientations() {
    let o = accepted(&message([true; 5]));
    assert_eq!(o.start(), 0);
    assert_eq!(o.released(), 2);
    assert_eq!(
        o.legs(),
        [
            Leg {
                edge: 0,
                end: 1,
                orientation: Some(0.0),
                orientation_type: Some(OrientationType::Tangential),
            },
            Leg {
                edge: 1,
                end: 2,
                orientation: Some(3.14159),
                orientation_type: None,
            },
        ]
    );
}

#[test]
fn the_released_part_is_counted() {
    assert_eq!(
        accepted(&message([true, true, true, false, false])).released(),
        1
    );
    assert_eq!(
        accepted(&message([true, false, false, false, false])).released(),
        0
    );
}

#[test]
fn release_rules_are_checked() {
    invalid(
        &message([false, false, false, false, false]),
        Problem::FirstNodeNotReleased,
    );
    invalid(
        &message([true, true, true, true, false]),
        Problem::EdgeReleasedWithoutNode { edge: "B-C".into() },
    );
    invalid(
        &message([true, false, true, true, true]),
        Problem::ReleasedAfterHorizon { id: "B".into() },
    );
    invalid(
        &message([true, true, true, false, true]),
        Problem::ReleasedAfterHorizon { id: "C".into() },
    );
}

#[test]
fn counts_are_checked() {
    let mut m = message([true; 5]);
    m.edges.pop();
    invalid(&m, Problem::WrongCounts { nodes: 3, edges: 1 });

    let mut m = message([true; 5]);
    m.nodes.clear();
    m.edges.clear();
    invalid(&m, Problem::WrongCounts { nodes: 0, edges: 0 });
}

/// The same order numbered from `first` on.
fn numbered(first: u32) -> vda::Order {
    let mut m = message([true; 5]);
    for (i, n) in m.nodes.iter_mut().enumerate() {
        n.sequence_id = first.wrapping_add(2 * i as u32);
    }
    for (i, e) in m.edges.iter_mut().enumerate() {
        e.sequence_id = first.wrapping_add(2 * i as u32 + 1);
    }
    m
}

#[test]
fn sequence_ids_are_checked() {
    let mut m = message([true; 5]);
    m.edges[0].sequence_id = 5;
    invalid(
        &m,
        Problem::BadSequenceId {
            field: "edges[].sequenceId",
            id: "A-B".into(),
            expected: 1,
            found: 5,
        },
    );

    let mut m = message([true; 5]);
    m.nodes[2].sequence_id = 6;
    invalid(
        &m,
        Problem::BadSequenceId {
            field: "nodes[].sequenceId",
            id: "C".into(),
            expected: 4,
            found: 6,
        },
    );

    // a node's number is even
    let mut m = numbered(11);
    m.order_update_id = 1;
    invalid(
        &m,
        Problem::BadSequenceId {
            field: "nodes[].sequenceId",
            id: "A".into(),
            expected: 10,
            found: 11,
        },
    );
}

#[test]
fn a_first_number_other_than_zero_is_left_to_the_robot() {
    // also for a new order, which the robot refuses
    assert_eq!(accepted(&numbered(10)).first_step(), 5);
    let mut m = numbered(10);
    m.order_update_id = 1;
    assert_eq!(accepted(&m).node_sequence_id(0), Some(10));
}

#[test]
fn the_highest_sequence_ids_do_not_overflow() {
    let last = u32::MAX - 5;
    assert_eq!(
        accepted(&numbered(last)).node_sequence_id(2),
        Some(u32::MAX - 1)
    );
    // the numbers would go on past 32 bits
    let m = numbered(u32::MAX - 1);
    assert_eq!(
        refused(&m).problem,
        Problem::BadSequenceId {
            field: "nodes[].sequenceId",
            id: "B".into(),
            expected: u64::from(u32::MAX) + 1,
            found: 0,
        }
    );
}

#[test]
fn unknown_names_are_refused() {
    let mut m = message([true; 5]);
    m.nodes[2].node_id = "Z".into();
    no_route(&m, Item::Node, "Z");

    let mut m = message([true; 5]);
    m.nodes[0].node_id = "".into();
    no_route(&m, Item::Node, "");

    // also in the horizon
    let mut m = message([true, false, false, false, false]);
    m.edges[1].edge_id = "X-Y".into();
    no_route(&m, Item::Edge, "X-Y");
}

#[test]
fn an_edge_that_does_not_connect_is_left_to_the_robot() {
    // edge C-B exists but runs the other way than B -> C
    let mut m = message([true; 5]);
    m.edges[1].edge_id = "C-B".into();
    assert_eq!(accepted(&m).legs()[1].edge, 2);
}

#[test]
fn unusable_ids_and_numbers_are_refused() {
    let mut m = message([true; 5]);
    m.order_id = String::new();
    invalid(&m, Problem::UnusableId { field: "orderId" });

    for (id, kind, field) in [
        ("", "pick", "actions[].actionId"),
        ("a", "", "actions[].actionType"),
    ] {
        let mut m = message([true; 5]);
        m.nodes[1].actions.push(vda::Action {
            action_type: kind.into(),
            action_id: id.into(),
            action_descriptor: None,
            blocking_type: vda::BlockingType::Hard,
            action_parameters: None,
            retriable: None,
        });
        invalid(&m, Problem::UnusableId { field });
    }

    for angle in [f64::NAN, f64::INFINITY] {
        let mut m = message([true; 5]);
        m.edges[0].orientation = Some(angle);
        invalid(
            &m,
            Problem::NotFinite {
                field: "edges[].orientation",
            },
        );
    }
}

#[test]
fn any_finite_angle_is_taken_as_it_is() {
    let mut m = message([true; 5]);
    m.edges[0].orientation = Some(7.0);
    assert_eq!(accepted(&m).legs()[0].orientation, Some(7.0));
}

#[test]
fn the_header_is_checked() {
    let mut m = message([true; 5]);
    m.version = "2.1.0".into();
    invalid(&m, Problem::Version("2.1.0".into()));

    // another 3.x version is read
    let mut m = message([true; 5]);
    m.version = "3.1.0".into();
    accepted(&m);

    for time in [
        "",
        "2026-11-20 10:00:00",
        "2026-11-20T10:00:00+01:00",
        "99999999999999999-01-01T00:00:00Z",
    ] {
        let mut m = message([true; 5]);
        m.timestamp = time.into();
        invalid(&m, Problem::BadTime { field: "timestamp" });
    }

    // an unknown robot refuses nothing
    let mut m = message([true; 5]);
    m.serial_number = "R-002".into();
    let issue = refused(&m);
    assert!(matches!(issue.problem, Problem::UnknownRobot(_)));
    assert_eq!((issue.error, issue.references), (None, Vec::new()));
}

fn outgoing(order: Order) -> Result<vda::Order, Issue> {
    translator()
        .to_robot_to_vda(&to_robot(ToRobotBody::Order(order)))
        .map(|out| match out.message {
            botsim_vda::translator::RobotMessage::Order(o) => o,
            other => panic!("{other:?}"),
        })
}

fn built(id: &str, start: usize, legs: Vec<Leg>, actions: Vec<NodeAction>) -> Order {
    Order::new(OrderId::new(id), 4, 0, start, legs, 0, actions).unwrap()
}

#[test]
fn numbers_outside_the_layout_are_not_sent() {
    for (start, legs, item, number) in [
        (3, vec![], Item::Node, 3),
        (usize::MAX, vec![], Item::Node, usize::MAX),
        (0, vec![leg(3, 1)], Item::Edge, 3),
        (0, vec![leg(0, 1), leg(1, 3)], Item::Node, 3),
    ] {
        let issue = outgoing(built("o1", start, legs, vec![])).unwrap_err();
        assert_eq!(issue.problem, Problem::OutsideLayout { item, number });
        assert_eq!(issue.error, Some(ErrorType::NoRouteToTarget));
        assert_eq!(issue.references, references(&[("orderId", "o1")]));
    }
}

#[test]
fn unusable_ids_and_numbers_are_not_sent() {
    let bad_action = |id: &str, kind: &str| {
        let mut a = action(id, botsim_link::order::Blocking::Hard);
        a.id = ActionId::new(id);
        a.kind = ActionType::new(kind);
        built("o1", 0, vec![], vec![NodeAction { node: 0, action: a }])
    };
    let nan = Leg {
        orientation: Some(f64::NAN),
        ..leg(0, 1)
    };
    for (order, problem) in [
        (
            built("", 0, vec![], vec![]),
            Problem::UnusableId { field: "orderId" },
        ),
        (
            bad_action("", "pick"),
            Problem::UnusableId {
                field: "actions[].actionId",
            },
        ),
        (
            bad_action("a", ""),
            Problem::UnusableId {
                field: "actions[].actionType",
            },
        ),
        (
            built("o1", 0, vec![nan], vec![]),
            Problem::NotFinite {
                field: "edges[].orientation",
            },
        ),
    ] {
        let issue = outgoing(order).unwrap_err();
        assert_eq!(issue.problem, problem);
        assert_eq!(issue.error, Some(ErrorType::ValidationFailure));
        assert_eq!(issue.references.len(), 2);
    }
}

#[test]
fn an_angle_outside_the_range_is_sent_as_the_same_direction() {
    let legs: Vec<Leg> = [3.1416, -3.1416, 4.71238898038469]
        .into_iter()
        .zip([(0, 1), (1, 2), (2, 1)])
        .map(|(angle, (edge, end))| Leg {
            orientation: Some(angle),
            ..leg(edge, end)
        })
        .collect();
    let out = translator()
        .to_robot_to_vda(&to_robot(ToRobotBody::Order(built("o1", 0, legs, vec![]))))
        .unwrap();
    let botsim_vda::translator::RobotMessage::Order(m) = out.message else {
        panic!("not an order");
    };
    // within the range of the schema: unchanged
    assert_eq!(m.edges[0].orientation, Some(3.1416));
    assert_eq!(m.edges[1].orientation, Some(-3.1416));
    let changed = m.edges[2].orientation.unwrap();
    assert!(
        (changed + std::f64::consts::FRAC_PI_2).abs() < 1e-12,
        "{changed}"
    );
    assert_eq!(out.issues.len(), 1);
    assert_eq!(out.issues[0].entry, Some(2));
    assert_eq!(out.issues[0].error, None);
    assert_eq!(
        out.issues[0].problem,
        Problem::AngleNormalized {
            field: "edges[].orientation",
            from: 4.71238898038469
        }
    );
}

#[test]
fn a_header_that_cannot_be_written_refuses_the_message() {
    let t = translator();
    let send = |change: &dyn Fn(&mut ToRobot)| {
        let mut m = to_robot(ToRobotBody::Order(order(0, 1)));
        change(&mut m);
        t.to_robot_to_vda(&m).unwrap_err()
    };
    let issue = send(&|m| m.header.time = Millis(i64::MAX));
    assert_eq!(issue.problem, Problem::BadTime { field: "timestamp" });
    assert_eq!(issue.error, Some(ErrorType::ValidationFailure));

    let unknown = RobotNo::new(u32::MAX, 0);
    let issue = send(&|m| m.header.robot = unknown);
    assert_eq!(issue.problem, Problem::UnknownRobotNo(unknown));
    assert_eq!(issue.error, None);

    let old = RobotNo::new(ROBOT.slot(), 0);
    let issue = send(&|m| m.header.robot = old);
    assert_eq!(issue.problem, Problem::OldRobotNo(old));
}

#[test]
fn an_issue_reads_as_a_sentence() {
    let mut m = message([true; 5]);
    m.nodes[2].node_id = "Z".into();
    assert_eq!(
        refused(&m).to_string(),
        "node 'Z' is not in the layout [NO_ROUTE_TO_TARGET]"
    );
}

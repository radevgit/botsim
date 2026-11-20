use botsim_core::actions::ActionTimes;
use botsim_core::errors::ErrorType::{self, *};
use botsim_core::graph::Graph;
use botsim_core::robot::{Reception, Reception::Accepted, Robot};
use botsim_core::spec::RobotSpec;
use botsim_core::state_report::Header;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vda5050::state::State;
use vdma_lif::Lif;

const DT: f64 = 0.05;

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn robot(at: &str) -> Robot {
    let g = graph();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    Robot::new(
        &g,
        spec,
        ActionTimes::new(1.0),
        g.node_idx(at).unwrap(),
        0.0,
    )
    .unwrap()
}

type N<'a> = &'a [(&'a str, u32, bool)];

fn order(id: &str, update: u32, nodes: N, edges: N) -> Order {
    let n: Vec<String> = nodes
        .iter()
        .map(|(i, s, r)| {
            format!(r#"{{"nodeId":"{i}","sequenceId":{s},"released":{r},"actions":[]}}"#)
        })
        .collect();
    let e: Vec<String> = edges
        .iter()
        .map(|(i, s, r)| {
            format!(r#"{{"edgeId":"{i}","sequenceId":{s},"released":{r},"actions":[]}}"#)
        })
        .collect();
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001","orderId":"{id}","orderUpdateId":{update},
        "nodes":[{}],"edges":[{}]}}"#,
        n.join(","),
        e.join(",")
    );
    serde_json::from_str(&json).unwrap()
}

/// A → B, fully released.
fn a_to_b(id: &str) -> Order {
    order(
        id,
        0,
        &[("A", 0, true), ("B", 2, true)],
        &[("A-B", 1, true)],
    )
}

/// B → C, fully released.
fn b_to_c(id: &str) -> Order {
    order(
        id,
        0,
        &[("B", 0, true), ("C", 2, true)],
        &[("B-C", 1, true)],
    )
}

fn header() -> Header {
    Header {
        header_id: 1,
        timestamp: "2026-11-20T10:00:00.000Z".into(),
        version: "3.0.1".into(),
        manufacturer: "ACME".into(),
        serial_number: "R-001".into(),
    }
}

fn state(r: &Robot) -> State {
    r.state(&graph(), &header())
}

fn error_types(r: &Robot) -> Vec<ErrorType> {
    r.errors().iter().map(|e| e.error_type).collect()
}

fn run_until_idle(r: &mut Robot) {
    for _ in 0..100_000 {
        r.step(DT);
        let s = state(r);
        if !s.driving
            && s.node_states.is_empty()
            && s.action_states
                .iter()
                .all(|a| a.action_status == vda5050::state::ActionStatus::Finished)
        {
            return;
        }
    }
    panic!("robot never became idle");
}

#[test]
fn an_idle_robot_without_an_order_reports_no_order() {
    let r = robot("A");
    let s = state(&r);
    assert_eq!((s.order_id.as_str(), s.last_node_id.as_str()), ("", ""));
    assert!(!s.driving && s.errors.is_empty());
}

#[test]
fn accepts_a_new_order_and_executes_it() {
    let g = graph();
    let mut r = robot("A");
    assert_eq!(r.receive_order(&g, &a_to_b("o1")), Accepted);
    assert_eq!(state(&r).order_id, "o1");
    run_until_idle(&mut r);
    let s = state(&r);
    assert_eq!(s.last_node_id, "B");
    assert!((r.pose().x - 10.0).abs() < 1e-9);
}

#[test]
fn a_second_order_while_one_is_active_is_rejected_and_the_first_continues() {
    let g = graph();
    let mut r = robot("A");
    assert_eq!(r.receive_order(&g, &a_to_b("o1")), Accepted);
    for _ in 0..20 {
        r.step(DT);
    }
    assert_eq!(
        r.receive_order(&g, &b_to_c("o2")),
        Reception::Rejected(OtherOrderActive)
    );
    let s = state(&r);
    assert_eq!(s.order_id, "o1", "still the first order");
    assert_eq!(s.errors.len(), 1);
    assert_eq!(s.errors[0].error_type, "OTHER_ORDER_ACTIVE");
    run_until_idle(&mut r);
    assert_eq!(state(&r).last_node_id, "B");
}

#[test]
fn after_finishing_a_new_order_must_start_where_the_robot_stands() {
    let g = graph();
    let mut r = robot("A");
    r.receive_order(&g, &a_to_b("o1"));
    run_until_idle(&mut r);

    // the robot stands at B; an order that starts at C is rejected
    let from_c = order(
        "o2",
        0,
        &[("C", 0, true), ("B", 2, true)],
        &[("C-B", 1, true)],
    );
    assert_eq!(
        r.receive_order(&g, &from_c),
        Reception::Rejected(StartNodeOutOfRange)
    );
    assert_eq!(state(&r).order_id, "o1");

    // one that starts at B is accepted, and clears the error
    assert_eq!(r.receive_order(&g, &b_to_c("o2")), Accepted);
    let s = state(&r);
    assert_eq!(s.order_id, "o2");
    assert!(s.errors.is_empty());
    run_until_idle(&mut r);
    assert!((r.pose().y - 5.0).abs() < 1e-9);
}

#[test]
fn a_new_order_must_have_update_id_zero() {
    let g = graph();
    let mut r = robot("A");
    let o = order(
        "o1",
        4,
        &[("A", 0, true), ("B", 2, true)],
        &[("A-B", 1, true)],
    );
    assert_eq!(
        r.receive_order(&g, &o),
        Reception::Rejected(ValidationFailure)
    );
}

#[test]
fn invalid_orders_are_rejected_with_the_matching_error() {
    let g = graph();
    let mut r = robot("A");
    // unknown node
    let o = order(
        "o1",
        0,
        &[("A", 0, true), ("Z", 2, true)],
        &[("A-B", 1, true)],
    );
    assert_eq!(
        r.receive_order(&g, &o),
        Reception::Rejected(NoRouteToTarget)
    );
    // first node not released
    let o = order(
        "o1",
        0,
        &[("A", 0, false), ("B", 2, false)],
        &[("A-B", 1, false)],
    );
    assert_eq!(
        r.receive_order(&g, &o),
        Reception::Rejected(ValidationFailure)
    );
    assert_eq!(error_types(&r), vec![NoRouteToTarget, ValidationFailure]);
    assert_eq!(state(&r).order_id, "", "nothing was accepted");
}

#[test]
fn order_updates_are_checked_for_their_update_id_and_start() {
    let g = graph();
    let mut r = robot("A");
    // o1: A-B released, B-C horizon
    let first = order(
        "o1",
        0,
        &[("A", 0, true), ("B", 2, true), ("C", 4, false)],
        &[("A-B", 1, true), ("B-C", 3, false)],
    );
    assert_eq!(r.receive_order(&g, &first), Accepted);
    for _ in 0..10 {
        r.step(DT);
    }

    let update = |upd: u32, from: &str| match from {
        "B" => order(
            "o1",
            upd,
            &[("B", 2, true), ("C", 4, true)],
            &[("B-C", 3, true)],
        ),
        _ => order(
            "o1",
            upd,
            &[("C", 4, true), ("B", 6, true)],
            &[("C-B", 5, true)],
        ),
    };

    // the same message again (update id 0 is the current one)
    assert_eq!(
        r.receive_order(&g, &first.clone()),
        Reception::Rejected(SameOrderUpdateId)
    );
    // does not start at the decision point (B)
    assert_eq!(
        r.receive_order(&g, &update(1, "C")),
        Reception::Rejected(ValidationFailure)
    );
    // the valid update is accepted, clears all errors, and the order completes
    assert_eq!(r.receive_order(&g, &update(1, "B")), Accepted);
    assert!(r.errors().is_empty());
    assert_eq!(state(&r).order_update_id, 1);
    // now the first message (update id 0) is outdated
    assert_eq!(
        r.receive_order(&g, &first),
        Reception::Rejected(OutdatedOrderUpdate)
    );
    run_until_idle(&mut r);
    assert_eq!(state(&r).last_node_id, "C");
}

#[test]
fn the_state_with_errors_is_schema_valid() {
    let g = graph();
    let mut r = robot("A");
    r.receive_order(&g, &a_to_b("o1"));
    r.receive_order(&g, &b_to_c("o2")); // OTHER_ORDER_ACTIVE
    let schema: serde_json::Value =
        serde_json::from_str(include_str!("data/vda5050_state.schema.json")).unwrap();
    let v = jsonschema::options()
        .should_validate_formats(true)
        .build(&schema)
        .unwrap();
    let instance = serde_json::to_value(state(&r)).unwrap();
    let errs: Vec<String> = v.iter_errors(&instance).map(|e| e.to_string()).collect();
    assert!(errs.is_empty(), "{errs:?}");
}

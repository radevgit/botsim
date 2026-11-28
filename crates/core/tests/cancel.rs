use botsim_core::actions::ActionTimes;
use botsim_core::robot::{Reception::Accepted, Reception::Rejected, Robot};
use botsim_model::errors::ErrorType::*;
use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::instant_actions::InstantActions;
use vda5050::order::Order;
use vda5050::state::ActionStatus::{self, Failed, Finished, Running};
use vda5050::state::State;
use vdma_lif::Lif;

mod vda_port;
use vda_port::{Header, VdaRobot};

const DT: f64 = 0.05;

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn robot() -> Robot {
    let g = graph();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    // every action takes 5 s
    Robot::new(
        &g,
        spec,
        ActionTimes::new(5.0),
        g.node_idx("A").unwrap(),
        0.0,
    )
    .unwrap()
}

/// Order A→B→C, fully released; actions (JSON arrays) on B and C.
fn order(update: u32, b_actions: &str, c_actions: &str) -> Order {
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001","orderId":"o1","orderUpdateId":{update},
        "nodes":[
          {{"nodeId":"A","sequenceId":0,"released":true,"actions":[]}},
          {{"nodeId":"B","sequenceId":2,"released":true,"actions":{b_actions}}},
          {{"nodeId":"C","sequenceId":4,"released":true,"actions":{c_actions}}}],
        "edges":[
          {{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]}},
          {{"edgeId":"B-C","sequenceId":3,"released":true,"actions":[]}}]}}"#
    );
    serde_json::from_str(&json).unwrap()
}

fn instant(actions: &str) -> InstantActions {
    let json = format!(
        r#"{{"headerId":2,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001","actions":{actions}}}"#
    );
    serde_json::from_str(&json).unwrap()
}

fn cancel(id: &str) -> InstantActions {
    instant(&format!(
        r#"[{{"actionType":"cancelOrder","actionId":"{id}","blockingType":"NONE"}}]"#
    ))
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
    let s = r.state(&graph(), &header());
    // every state must be valid against the official schema
    let schema: serde_json::Value =
        serde_json::from_str(include_str!("../../vda/tests/data/state.schema")).unwrap();
    let v = jsonschema::options()
        .should_validate_formats(true)
        .build(&schema)
        .unwrap();
    let instance = serde_json::to_value(&s).unwrap();
    let errs: Vec<String> = v.iter_errors(&instance).map(|e| e.to_string()).collect();
    assert!(errs.is_empty(), "{errs:?}");
    s
}

fn instant_status(s: &State, id: &str) -> ActionStatus {
    s.instant_action_states
        .iter()
        .find(|a| a.action_id == id)
        .unwrap_or_else(|| panic!("no instant action {id}"))
        .action_status
}

fn action_status(s: &State, id: &str) -> ActionStatus {
    s.action_states
        .iter()
        .find(|a| a.action_id == id)
        .unwrap_or_else(|| panic!("no action {id}"))
        .action_status
}

fn steps(r: &mut Robot, n: usize) {
    for _ in 0..n {
        r.step(DT);
    }
}

#[test]
fn cancel_while_driving_stops_at_the_next_node_and_drops_the_rest() {
    let g = graph();
    let mut r = robot();
    assert_eq!(r.receive_order(&g, &order(0, "[]", "[]")), Accepted);
    steps(&mut r, 60); // 3 s: on A-B, moving
    assert!(state(&r).driving);

    r.receive_instant_actions(&graph(), &cancel("c1"));
    let s = state(&r);
    assert_eq!(instant_status(&s, "c1"), Running, "still moving");
    assert_eq!(s.last_node_id, "A");
    // it is heading for B only; the rest of the order is gone
    assert_eq!(s.node_states.len(), 1);
    assert_eq!(s.node_states[0].node_id, "B");
    assert_eq!(s.edge_states.len(), 1);

    steps(&mut r, 400);
    let s = state(&r);
    assert_eq!(instant_status(&s, "c1"), Finished);
    assert_eq!(s.last_node_id, "B", "stopped at B, did not go on to C");
    assert!(!s.driving);
    assert!(s.node_states.is_empty() && s.edge_states.is_empty());
    assert_eq!(
        (s.order_id.as_str(), s.order_update_id),
        ("o1", 0),
        "ids are kept"
    );
    assert!((r.pose().x - 10.0).abs() < 1e-9 && r.pose().y == 0.0);

    // idle again: a new order from B is accepted
    let from_b = {
        let json = r#"{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
            "manufacturer":"ACME","serialNumber":"R-001","orderId":"o2","orderUpdateId":0,
            "nodes":[{"nodeId":"B","sequenceId":0,"released":true,"actions":[]},
                     {"nodeId":"C","sequenceId":2,"released":true,"actions":[]}],
            "edges":[{"edgeId":"B-C","sequenceId":1,"released":true,"actions":[]}]}"#;
        serde_json::from_str::<Order>(json).unwrap()
    };
    assert_eq!(r.receive_order(&g, &from_b), Accepted);
}

#[test]
fn cancel_at_a_node_fails_scheduled_and_running_actions_and_stops_at_once() {
    let g = graph();
    let mut r = robot();
    let b = r#"[{"actionId":"pick","actionType":"pick","blockingType":"HARD"},
                {"actionId":"beep","actionType":"beep","blockingType":"NONE"}]"#;
    let c = r#"[{"actionId":"drop","actionType":"drop","blockingType":"HARD"}]"#;
    assert_eq!(r.receive_order(&g, &order(0, b, c)), Accepted);
    while !state(&r)
        .action_states
        .iter()
        .any(|a| a.action_status == Running)
    {
        r.step(DT);
    }
    // acting at B: "pick" runs, "beep" waits behind it, "drop" (at C) is not triggered
    let s = state(&r);
    assert_eq!(s.last_node_id, "B");
    assert_eq!(action_status(&s, "pick"), Running);

    r.receive_instant_actions(&graph(), &cancel("c1"));
    let s = state(&r);
    // all three FAILED, the robot is idle at once, cancelOrder is FINISHED
    assert_eq!(action_status(&s, "pick"), Failed);
    assert_eq!(action_status(&s, "beep"), Failed);
    assert_eq!(action_status(&s, "drop"), Failed);
    assert_eq!(s.action_states.len(), 3, "no duplicates");
    assert_eq!(instant_status(&s, "c1"), Finished);
    assert!(!s.driving && s.node_states.is_empty());
    assert_eq!(s.last_node_id, "B");

    // and it stays where it is
    steps(&mut r, 400);
    assert_eq!(state(&r).last_node_id, "B");
}

#[test]
fn cancel_without_an_active_order_fails_with_no_order_to_cancel() {
    let mut r = robot(); // idle, never had an order
    r.receive_instant_actions(&graph(), &cancel("c1"));
    let s = state(&r);
    assert_eq!(instant_status(&s, "c1"), Failed);
    assert_eq!(s.errors.len(), 1);
    assert_eq!(s.errors[0].error_type, "NO_ORDER_TO_CANCEL");
    let refs = s.errors[0].error_references.as_ref().unwrap();
    assert_eq!(
        (
            refs[0].reference_key.as_str(),
            refs[0].reference_value.as_str()
        ),
        ("actionId", "c1")
    );
}

#[test]
fn cancel_with_another_order_id_fails_and_the_order_goes_on() {
    let g = graph();
    let mut r = robot();
    r.receive_order(&g, &order(0, "[]", "[]"));
    steps(&mut r, 20);
    r.receive_instant_actions(
        &graph(),
        &instant(
            r#"[{"actionType":"cancelOrder","actionId":"c1","blockingType":"NONE",
                 "actionParameters":[{"key":"orderId","value":"other"}]}]"#,
        ),
    );
    let s = state(&r);
    assert_eq!(instant_status(&s, "c1"), Failed);
    assert_eq!(s.errors[0].error_type, "NO_ORDER_TO_CANCEL");
    steps(&mut r, 2000);
    assert_eq!(state(&r).last_node_id, "C", "the order ran to its end");

    // the right id works while the order is active
    let mut r = robot();
    r.receive_order(&g, &order(0, "[]", "[]"));
    steps(&mut r, 20);
    r.receive_instant_actions(
        &graph(),
        &instant(
            r#"[{"actionType":"cancelOrder","actionId":"c2","blockingType":"NONE",
                 "actionParameters":[{"key":"orderId","value":"o1"}]}]"#,
        ),
    );
    assert_eq!(instant_status(&state(&r), "c2"), Running);
}

#[test]
fn an_update_after_a_cancel_is_rejected() {
    let g = graph();
    let mut r = robot();
    r.receive_order(&g, &order(0, "[]", "[]"));
    steps(&mut r, 20);
    r.receive_instant_actions(&graph(), &cancel("c1"));
    // an update (update id 1) for the cancelled order
    let update = {
        let json = r#"{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
            "manufacturer":"ACME","serialNumber":"R-001","orderId":"o1","orderUpdateId":1,
            "nodes":[{"nodeId":"B","sequenceId":2,"released":true,"actions":[]},
                     {"nodeId":"C","sequenceId":4,"released":true,"actions":[]}],
            "edges":[{"edgeId":"B-C","sequenceId":3,"released":true,"actions":[]}]}"#;
        serde_json::from_str::<Order>(json).unwrap()
    };
    assert_eq!(
        r.receive_order(&g, &update),
        Rejected(OrderUpdateFollowingCancel)
    );
    steps(&mut r, 400);
    assert_eq!(state(&r).last_node_id, "B", "still stopped at B");
}

#[test]
fn unsupported_instant_actions_fail_with_invalid_instant_action() {
    let mut r = robot();
    r.receive_instant_actions(
        &graph(),
        &instant(r#"[{"actionType":"startPause","actionId":"p1","blockingType":"NONE"}]"#),
    );
    let s = state(&r);
    assert_eq!(instant_status(&s, "p1"), Failed);
    assert_eq!(s.errors[0].error_type, "INVALID_INSTANT_ACTION");
}

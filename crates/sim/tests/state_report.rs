use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use botsim_sim::actions::ActionTimes;
use botsim_sim::drive::{Driver, Phase};
use serde_json::Value;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vda5050::state::ActionStatus::{Finished, Running, Waiting};
use vda5050::state::State;
use vdma_lif::Lif;

mod vda_port;
use vda_port::{Header, build_state, route_from_order};

const DT: f64 = 0.05;

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn header() -> Header {
    Header {
        header_id: 7,
        timestamp: "2026-11-20T10:00:00.000Z".into(),
        version: "3.0.1".into(),
        manufacturer: "ACME".into(),
        serial_number: "R-001".into(),
    }
}

/// Order A→B→C. `rel` = released flags (A, A-B, B, B-C, C); `b_actions` = actions
/// on B as JSON; `bc_orientation` = orientation of B-C as JSON (or "null").
fn order(update_id: u32, rel: [bool; 5], b_actions: &str, bc_orientation: &str) -> Order {
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001","orderId":"o1","orderUpdateId":{update_id},
        "nodes":[
          {{"nodeId":"A","sequenceId":0,"released":{},"actions":[]}},
          {{"nodeId":"B","sequenceId":2,"released":{},"actions":{b_actions}}},
          {{"nodeId":"C","sequenceId":4,"released":{},"actions":[]}}],
        "edges":[
          {{"edgeId":"A-B","sequenceId":1,"released":{},"actions":[]}},
          {{"edgeId":"B-C","sequenceId":3,"released":{},"orientation":{bc_orientation},"actions":[]}}]}}"#,
        rel[0], rel[2], rel[4], rel[1], rel[3]
    );
    serde_json::from_str(&json).unwrap()
}

fn driver(o: &Order) -> Driver {
    let g = graph();
    let r = route_from_order(&g, o).unwrap();
    Driver::from_order_route(&g, &spec(), 0.0, &r, ActionTimes::new(2.0)).unwrap()
}

fn state(d: &Driver) -> State {
    build_state(&graph(), &header(), &d.progress(), &[], &[])
}

/// The message must be valid against the official VDA 5050 state schema.
fn assert_schema_valid(s: &State) {
    let schema: Value =
        serde_json::from_str(include_str!("../../vda/tests/data/state.schema")).unwrap();
    let validator = jsonschema::options()
        .should_validate_formats(true)
        .build(&schema)
        .unwrap();
    let instance = serde_json::to_value(s).unwrap();
    let errors: Vec<String> = validator
        .iter_errors(&instance)
        .map(|e| format!("{e} at {}", e.instance_path()))
        .collect();
    assert!(errors.is_empty(), "schema errors: {errors:?}\n{instance}");
}

fn node_ids(s: &State) -> Vec<(&str, u32, bool)> {
    s.node_states
        .iter()
        .map(|n| (n.node_id.as_str(), n.sequence_id, n.released))
        .collect()
}

fn edge_ids(s: &State) -> Vec<(&str, u32, bool)> {
    s.edge_states
        .iter()
        .map(|e| (e.edge_id.as_str(), e.sequence_id, e.released))
        .collect()
}

#[test]
fn state_at_the_start_of_an_order_with_a_horizon() {
    // A, A-B, B released; B-C and C are the horizon
    let d = driver(&order(0, [true, true, true, false, false], "[]", "null"));
    let s = state(&d);
    assert_schema_valid(&s);

    assert_eq!((s.header_id, s.manufacturer.as_str()), (7, "ACME"));
    assert_eq!((s.order_id.as_str(), s.order_update_id), ("o1", 0));
    // standing on the first node: it is the last node, and not in nodeStates
    assert_eq!((s.last_node_id.as_str(), s.last_node_sequence_id), ("A", 0));
    assert_eq!(node_ids(&s), vec![("B", 2, true), ("C", 4, false)]);
    assert_eq!(edge_ids(&s), vec![("A-B", 1, true), ("B-C", 3, false)]);
    assert!(s.driving);
    assert!(s.action_states.is_empty());
    let pos = s.mobile_robot_position.unwrap();
    assert_eq!((pos.x, pos.y, pos.theta), (0.0, 0.0, 0.0));
    assert_eq!(pos.map_id, "M");
}

#[test]
fn nodes_and_edges_leave_the_lists_as_the_robot_traverses_them() {
    let mut d = driver(&order(0, [true; 5], "[]", "null"));
    let mut seen_turning_at_b = false;

    while !d.is_finished() {
        d.step(DT);
        let s = state(&d);
        assert_schema_valid(&s);

        // invariants at every step
        assert_eq!(
            s.edge_states.len(),
            s.node_states.len(),
            "one edge per upcoming node"
        );
        let p = d.pose();
        match (s.last_node_id.as_str(), s.node_states.len()) {
            ("A", 2) => assert!(p.x <= 10.0 && p.y == 0.0, "on A-B"),
            ("B", 1) => assert!(p.x == 10.0, "reached B"),
            ("C", 0) => assert!((p.y - 5.0).abs() < 1e-9),
            other => panic!("unexpected {other:?}"),
        }
        if s.last_node_id == "B" && d.phase() == Phase::Rotating {
            assert!(s.driving, "turning on the spot counts as driving");
            seen_turning_at_b = true;
        }
    }
    assert!(seen_turning_at_b);

    let s = state(&d);
    // finished: lists empty, order id kept, standing at the last node
    assert_eq!((s.last_node_id.as_str(), s.last_node_sequence_id), ("C", 4));
    assert!(s.node_states.is_empty() && s.edge_states.is_empty());
    assert_eq!(s.order_id, "o1");
    assert!(!s.driving);
}

#[test]
fn waiting_at_the_end_of_the_base_shows_the_horizon() {
    let mut d = driver(&order(0, [true, true, true, false, false], "[]", "null"));
    while d.phase() != Phase::Waiting {
        d.step(DT);
    }
    let s = state(&d);
    assert_schema_valid(&s);
    assert_eq!((s.last_node_id.as_str(), s.last_node_sequence_id), ("B", 2));
    assert_eq!(node_ids(&s), vec![("C", 4, false)]);
    assert_eq!(edge_ids(&s), vec![("B-C", 3, false)]);
    assert!(!s.driving);
    let v = s.velocity.unwrap();
    assert_eq!((v.vx, v.omega), (Some(0.0), Some(0.0)));
}

#[test]
fn an_order_update_shows_in_the_state() {
    let g = graph();
    let mut d = driver(&order(0, [true, true, true, false, false], "[]", "null"));
    while d.phase() != Phase::Waiting {
        d.step(DT);
    }
    // update: B (decision node) and C released
    let update = {
        let mut u = order(1, [true; 5], "[]", "null");
        u.nodes.remove(0);
        u.edges.remove(0);
        u
    };
    d.extend(&g, &route_from_order(&g, &update).unwrap())
        .unwrap();
    let s = state(&d);
    assert_schema_valid(&s);
    assert_eq!(s.order_update_id, 1);
    assert_eq!((s.last_node_id.as_str(), s.last_node_sequence_id), ("B", 2));
    assert_eq!(node_ids(&s), vec![("C", 4, true)], "C is released now");
    assert_eq!(edge_ids(&s), vec![("B-C", 3, true)]);
}

#[test]
fn action_states_go_waiting_running_finished() {
    let hard = r#"[{"actionId":"p","actionType":"pick","blockingType":"HARD"}]"#;
    let mut d = driver(&order(0, [true; 5], hard, "null"));

    let s = state(&d);
    assert_schema_valid(&s);
    // not triggered yet: reported as WAITING
    assert_eq!(s.action_states.len(), 1);
    assert_eq!(s.action_states[0].action_id, "p");
    assert_eq!(s.action_states[0].action_type.as_deref(), Some("pick"));
    assert_eq!(s.action_states[0].action_status, Waiting);

    while d.phase() != Phase::Acting {
        d.step(DT);
    }
    let s = state(&d);
    assert_schema_valid(&s);
    assert_eq!(s.action_states.len(), 1, "no duplicates once queued");
    assert_eq!(s.action_states[0].action_status, Running);
    assert!(!s.driving);
    assert_eq!(
        s.last_node_id, "B",
        "node counts as traversed before its actions run"
    );

    while !d.is_finished() {
        d.step(DT);
    }
    let s = state(&d);
    assert_eq!(s.action_states.len(), 1);
    assert_eq!(s.action_states[0].action_status, Finished);
}

#[test]
fn driving_backwards_gives_negative_vx_and_turning_gives_omega() {
    // B-C backwards (orientation π): at B the robot turns from east to south
    let mut d = driver(&order(0, [true; 5], "[]", "3.14159"));
    let mut saw_back = false;
    let mut saw_turn = false;
    while !d.is_finished() {
        d.step(DT);
        let s = state(&d);
        assert_schema_valid(&s);
        let v = s.velocity.as_ref().unwrap();
        if d.phase() == Phase::Rotating {
            saw_turn = true;
            assert_eq!(v.vx, Some(0.0));
            // east → south is a clockwise turn: negative omega, at the max angular speed
            assert_eq!(v.omega, Some(-1.0));
            assert!(s.driving);
        }
        if d.phase() == Phase::Driving && s.last_node_id == "B" {
            // the first step after the turn still has speed 0
            assert!(v.vx.unwrap() <= 0.0, "backwards: {:?}", v.vx);
            saw_back |= v.vx.unwrap() < 0.0;
        }
        if d.phase() == Phase::Driving && s.last_node_id == "A" {
            assert!(v.vx.unwrap() >= 0.0);
        }
    }
    assert!(saw_back && saw_turn);
}

#[test]
fn a_driver_without_an_order_reports_empty_ids() {
    use botsim_sim::drive::RouteEdge;
    let g = graph();
    let d = Driver::new(
        &g,
        &spec(),
        g.node_idx("A").unwrap(),
        0.0,
        &[RouteEdge::new(g.edge_idx("A-B").unwrap())],
    )
    .unwrap();
    let s = state(&d);
    assert_schema_valid(&s);
    assert_eq!((s.order_id.as_str(), s.order_update_id), ("", 0));
    assert_eq!((s.last_node_id.as_str(), s.last_node_sequence_id), ("", 0));
    assert!(s.node_states.is_empty() && s.edge_states.is_empty());
}

#[test]
fn state_round_trips_through_json() {
    let d = driver(&order(0, [true; 5], "[]", "null"));
    let s = state(&d);
    let json = serde_json::to_string(&s).unwrap();
    let back: State = serde_json::from_str(&json).unwrap();
    assert_eq!(s, back);
}

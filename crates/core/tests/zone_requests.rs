use botsim_core::actions::ActionTimes;
use botsim_core::robot::Reception::Accepted;
use botsim_core::world::World;
use botsim_core::zone_requests::ZoneRequestTiming::{self, OnHorizon, OnRelease};
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_model::zones::Zones;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vda5050::responses::{GrantType, Responses};
use vda5050::state::RequestStatus;
use vda5050::zone_set::ZoneSet;
use vdma_lif::Lif;

mod vda_port;
use vda_port::VdaWorld;

const DT: f64 = 0.05;

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap() // A (0,0) → B (10,0) → C (10,5)
}

fn spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn id(n: &str) -> RobotId {
    RobotId::new("ACME", n)
}

fn square(x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!(
        r#""vertices":[{{"x":{x0},"y":{y0}}},{{"x":{x1},"y":{y0}}},{{"x":{x1},"y":{y1}}},{{"x":{x0},"y":{y1}}}]"#
    )
}

fn zones(zones: &[String]) -> Zones {
    let set: ZoneSet = serde_json::from_str(&format!(
        r#"{{"mapId":"M","zoneSetId":"zs1","zones":[{}]}}"#,
        zones.join(",")
    ))
    .unwrap();
    Zones::new(&graph(), set).unwrap()
}

/// A RELEASE zone on the way from A to B (x 4 … 7).
fn gate() -> String {
    format!(
        r#"{{"zoneId":"gate","zoneType":"RELEASE",{}}}"#,
        square(4.0, -1.0, 7.0, 1.0)
    )
}

/// R-1 on A with the zones and the given timing; no zones if `z` is empty.
fn world(z: &[String], timing: ZoneRequestTiming) -> World {
    let g = graph();
    let a = g.node_idx("A").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    if !z.is_empty() {
        w.set_zones(zones(z));
    }
    w.add_robot(id("R-1"), spec(), ActionTimes::new(1.0), a, 0.0)
        .unwrap();
    w.set_zone_request_timing(&id("R-1"), timing).unwrap();
    w
}

/// A → B → C; `released` = (A-B and B released, B-C and C released).
fn order(update: u32, from: &str, ab_released: bool, bc_released: bool) -> Order {
    let (nodes, edges) = if from == "A" {
        (
            format!(
                r#"[{{"nodeId":"A","sequenceId":0,"released":true,"actions":[]}},
                    {{"nodeId":"B","sequenceId":2,"released":{ab_released},"actions":[]}},
                    {{"nodeId":"C","sequenceId":4,"released":{bc_released},"actions":[]}}]"#
            ),
            format!(
                r#"[{{"edgeId":"A-B","sequenceId":1,"released":{ab_released},"actions":[]}},
                    {{"edgeId":"B-C","sequenceId":3,"released":{bc_released},"actions":[]}}]"#
            ),
        )
    } else {
        // an update starting at the decision node A, releasing what is asked
        (
            format!(
                r#"[{{"nodeId":"A","sequenceId":0,"released":true,"actions":[]}},
                    {{"nodeId":"B","sequenceId":2,"released":{ab_released},"actions":[]}},
                    {{"nodeId":"C","sequenceId":4,"released":{bc_released},"actions":[]}}]"#
            ),
            format!(
                r#"[{{"edgeId":"A-B","sequenceId":1,"released":{ab_released},"actions":[]}},
                    {{"edgeId":"B-C","sequenceId":3,"released":{bc_released},"actions":[]}}]"#
            ),
        )
    };
    serde_json::from_str(&format!(
        r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":{update},
        "nodes":{nodes},"edges":{edges}}}"#
    ))
    .unwrap()
}

/// The controller answers one request of R-1.
fn respond(w: &mut World, request_id: &str, grant: GrantType) {
    let msg: Responses = serde_json::from_value(serde_json::json!({
        "headerId": 1, "timestamp": "2026-11-20T10:00:00.000Z", "version": "3.0.1",
        "manufacturer": "ACME", "serialNumber": "R-1",
        "responses": [{"requestId": request_id, "grantType": grant}]
    }))
    .unwrap();
    w.send_responses(&msg).unwrap();
}

fn requests(w: &World) -> Vec<(String, String)> {
    w.zone_requests(&id("R-1"))
        .unwrap()
        .iter()
        .map(|r| (r.request_id.clone(), r.zone_id.clone()))
        .collect()
}

// ---- geometry ----

#[test]
fn an_edge_enters_a_release_zone_when_it_starts_outside_and_reaches_it() {
    let z = zones(&[
        gate(),
        format!(
            r#"{{"zoneId":"slow","zoneType":"SPEED_LIMIT","maximumSpeed":0.5,{}}}"#,
            square(4.0, -1.0, 7.0, 1.0)
        ),
    ]);
    // zone 0 is the RELEASE zone (4..7 × -1..1), zone 1 is not a RELEASE zone
    assert_eq!(
        z.release_zones_entered((0.0, 0.0), (10.0, 0.0)),
        vec![0],
        "passes through"
    );
    assert_eq!(
        z.release_zones_entered((0.0, 0.0), (5.0, 0.0)),
        vec![0],
        "ends inside"
    );
    assert_eq!(
        z.release_zones_entered((0.0, 0.0), (3.0, 0.0)),
        Vec::<usize>::new(),
        "stops short"
    );
    assert_eq!(
        z.release_zones_entered((5.0, 0.0), (10.0, 0.0)),
        Vec::<usize>::new(),
        "starts inside: leaving"
    );
    assert_eq!(
        z.release_zones_entered((5.0, 0.2), (6.0, 0.2)),
        Vec::<usize>::new(),
        "inside the whole time"
    );
    assert_eq!(
        z.release_zones_entered((0.0, 5.0), (10.0, 5.0)),
        Vec::<usize>::new(),
        "passes beside it"
    );
    assert_eq!(
        z.release_zones_entered((0.0, 1.0), (10.0, 1.0)),
        vec![0],
        "touching the border counts"
    );
}

// ---- requests in the state ----

#[test]
fn without_zones_the_state_has_no_zone_requests_and_with_zones_an_empty_list() {
    let mut w = world(&[], OnRelease);
    assert!(w.state(&id("R-1")).unwrap().zone_requests.is_none());

    let mut w = world(&[gate()], OnRelease);
    assert_eq!(w.state(&id("R-1")).unwrap().zone_requests, Some(vec![]));
}

#[test]
fn a_released_entering_edge_is_requested_at_once_with_either_timing() {
    for timing in [OnRelease, OnHorizon] {
        let mut w = world(&[gate()], timing);
        assert_eq!(w.send_order(&order(0, "A", true, true)), Ok(Accepted));
        let s = w.state(&id("R-1")).unwrap();
        let r = &s.zone_requests.unwrap()[0];
        assert_eq!(r.request_id, "req-1");
        assert_eq!(
            (r.zone_id.as_str(), r.zone_set_id.as_str()),
            ("gate", "zs1")
        );
        assert_eq!(r.request_status, RequestStatus::Requested);
        assert_eq!(format!("{:?}", r.request_type), "Access");
    }
}

#[test]
fn the_timing_decides_whether_a_horizon_edge_is_requested() {
    // A-B is in the horizon (only A is released)
    let mut early = world(&[gate()], OnHorizon);
    early.send_order(&order(0, "A", false, false)).unwrap();
    assert_eq!(requests(&early), vec![("req-1".into(), "gate".into())]);

    let mut late = world(&[gate()], OnRelease);
    late.send_order(&order(0, "A", false, false)).unwrap();
    assert!(requests(&late).is_empty(), "not released yet: no request");

    // the update releases A-B: now the late robot asks, the early one keeps its request
    assert_eq!(late.send_order(&order(1, "U", true, false)), Ok(Accepted));
    assert_eq!(requests(&late), vec![("req-1".into(), "gate".into())]);
    early.send_order(&order(1, "U", true, false)).unwrap();
    assert_eq!(
        requests(&early),
        vec![("req-1".into(), "gate".into())],
        "same request, no second one"
    );
}

#[test]
fn a_request_stays_while_the_robot_is_inside_and_goes_when_it_has_left() {
    let mut w = world(&[gate()], OnRelease);
    w.send_order(&order(0, "A", true, true)).unwrap();
    respond(&mut w, "req-1", GrantType::Granted); // the robot waits until it is granted
    let (mut before, mut inside, mut after) = (0, 0, 0);
    for _ in 0..3000 {
        w.step();
        let x = w.pose(&id("R-1")).unwrap();
        let has = !requests(&w).is_empty();
        let in_zone = (4.0..=7.0).contains(&x.x) && x.y.abs() <= 1.0;
        match (x.x < 4.0 && x.y == 0.0, in_zone, has) {
            (true, _, true) => before += 1,
            (_, true, true) => inside += 1,
            (_, true, false) => panic!("inside the zone without a request at x = {}", x.x),
            (false, false, true) if x.x > 7.0 || x.y > 0.0 => {
                panic!("request still there after leaving at ({}, {})", x.x, x.y)
            }
            _ => after += 1,
        }
        if requests(&w).first().is_some_and(|r| r.0 != "req-1") {
            panic!("the request id changed");
        }
    }
    assert!(
        before > 0 && inside > 0 && after > 0,
        "{before} {inside} {after}"
    );
    assert!(requests(&w).is_empty());
}

#[test]
fn edges_that_do_not_enter_a_release_zone_make_no_request() {
    // a zone around A: A-B starts inside it and leaves
    let around_a = format!(
        r#"{{"zoneId":"home","zoneType":"RELEASE",{}}}"#,
        square(-1.0, -1.0, 1.0, 1.0)
    );
    let mut w = world(&[around_a], OnHorizon);
    w.send_order(&order(0, "A", true, true)).unwrap();
    assert!(requests(&w).is_empty());

    // a zone that is not RELEASE
    let slow = format!(
        r#"{{"zoneId":"slow","zoneType":"SPEED_LIMIT","maximumSpeed":0.5,{}}}"#,
        square(4.0, -1.0, 7.0, 1.0)
    );
    let mut w = world(&[slow], OnHorizon);
    w.send_order(&order(0, "A", true, true)).unwrap();
    assert!(requests(&w).is_empty());
}

#[test]
fn two_zones_on_one_edge_give_two_requests_with_unique_ids() {
    let second = format!(
        r#"{{"zoneId":"gate2","zoneType":"RELEASE",{}}}"#,
        square(8.0, -1.0, 9.0, 1.0)
    );
    let mut w = world(&[gate(), second], OnRelease);
    w.send_order(&order(0, "A", true, true)).unwrap();
    assert_eq!(
        requests(&w),
        vec![
            ("req-1".into(), "gate".into()),
            ("req-2".into(), "gate2".into())
        ]
    );
}

#[test]
fn every_robot_numbers_its_own_requests_and_the_state_is_schema_valid() {
    let g = graph();
    let (a, b) = (g.node_idx("A").unwrap(), g.node_idx("B").unwrap());
    let mut w = World::new(g, DT, 0).unwrap();
    w.set_zones(zones(&[gate()]));
    for (n, node) in [("R-1", a), ("R-2", b)] {
        w.add_robot(id(n), spec(), ActionTimes::new(1.0), node, 0.0)
            .unwrap(); // added after the zones: they get them too
    }
    w.send_order(&order(0, "A", true, true)).unwrap();
    assert_eq!(requests(&w).len(), 1);
    assert!(
        w.zone_requests(&id("R-2")).unwrap().is_empty(),
        "R-2 has no order"
    );

    let schema: serde_json::Value =
        serde_json::from_str(include_str!("../../vda/tests/data/state.schema")).unwrap();
    let validator = jsonschema::options()
        .should_validate_formats(true)
        .build(&schema)
        .unwrap();
    let state = w.state(&id("R-1")).unwrap();
    assert_eq!(state.zone_requests.as_ref().unwrap().len(), 1);
    let instance = serde_json::to_value(&state).unwrap();
    let errors: Vec<String> = validator
        .iter_errors(&instance)
        .map(|e| e.to_string())
        .collect();
    assert!(errors.is_empty(), "{errors:?}");
}

// ---- waiting for the grant ----

fn run_ticks(w: &mut World, n: u32) {
    for _ in 0..n {
        w.step();
    }
}

fn status(w: &mut World) -> RequestStatus {
    w.state(&id("R-1")).unwrap().zone_requests.unwrap()[0].request_status
}

#[test]
fn without_a_grant_the_robot_waits_before_the_zone_for_ever() {
    let mut w = world(&[gate()], OnRelease);
    w.send_order(&order(0, "A", true, true)).unwrap();
    run_ticks(&mut w, 2000); // 100 s
    let p = w.pose(&id("R-1")).unwrap();
    assert_eq!((p.x, p.y), (0.0, 0.0), "still on A");
    let s = w.state(&id("R-1")).unwrap();
    assert!(!s.driving);
    assert_eq!(s.last_node_id, "A");
    assert_eq!(s.node_states.len(), 2, "its order is not touched");
    assert_eq!(status(&mut w), RequestStatus::Requested);
    assert_eq!(
        w.snapshot().robots[0].status,
        botsim_core::observer::RobotStatus::WaitingForZone
    );
}

#[test]
fn queued_and_rejected_keep_the_robot_waiting_and_granted_lets_it_go() {
    let mut w = world(&[gate()], OnRelease);
    w.send_order(&order(0, "A", true, true)).unwrap();

    respond(&mut w, "req-1", GrantType::Queued);
    run_ticks(&mut w, 400);
    assert_eq!(status(&mut w), RequestStatus::Queued);
    assert_eq!(w.pose(&id("R-1")).unwrap().x, 0.0);

    // REJECTED: the standard says the robot removes the request; it still wants
    // the zone, so it states a new one (new id) at its next update
    respond(&mut w, "req-1", GrantType::Rejected);
    assert_eq!(requests(&w), vec![("req-2".into(), "gate".into())]);
    run_ticks(&mut w, 400);
    assert_eq!(w.pose(&id("R-1")).unwrap().x, 0.0, "rejected: stays out");
    assert_eq!(status(&mut w), RequestStatus::Requested, "a fresh request");

    // an answer to the removed request is ignored
    respond(&mut w, "req-1", GrantType::Granted);
    run_ticks(&mut w, 100);
    assert_eq!(w.pose(&id("R-1")).unwrap().x, 0.0);

    respond(&mut w, "req-2", GrantType::Granted);
    assert_eq!(status(&mut w), RequestStatus::Granted);
    run_ticks(&mut w, 3000);
    let p = w.pose(&id("R-1")).unwrap();
    assert_eq!((p.x, p.y), (10.0, 5.0), "went all the way to C");
    assert!(
        requests(&w).is_empty(),
        "request gone after leaving the zone"
    );
}

#[test]
fn a_granted_request_stays_granted_when_a_queued_answer_arrives_late() {
    let mut w = world(&[gate()], OnRelease);
    w.send_order(&order(0, "A", true, true)).unwrap();
    respond(&mut w, "req-1", GrantType::Granted);
    respond(&mut w, "req-1", GrantType::Queued);
    assert_eq!(status(&mut w), RequestStatus::Granted);
    respond(&mut w, "req-1", GrantType::Rejected);
    run_ticks(&mut w, 200);
    assert!(w.pose(&id("R-1")).unwrap().x > 0.0, "still allowed to go");
}

#[test]
fn answers_to_unknown_requests_are_ignored_and_unknown_robots_are_errors() {
    let mut w = world(&[gate()], OnRelease);
    w.send_order(&order(0, "A", true, true)).unwrap();
    respond(&mut w, "req-99", GrantType::Granted);
    run_ticks(&mut w, 400);
    assert_eq!(w.pose(&id("R-1")).unwrap().x, 0.0, "nothing was granted");

    let msg: Responses = serde_json::from_value(serde_json::json!({
        "headerId": 1, "timestamp": "2026-11-20T10:00:00.000Z", "version": "3.0.1",
        "manufacturer": "ACME", "serialNumber": "R-9",
        "responses": [{"requestId": "req-1", "grantType": "GRANTED"}]
    }))
    .unwrap();
    assert!(w.send_responses(&msg).is_err());
}

#[test]
fn a_robot_may_turn_towards_the_zone_edge_but_waits_to_drive_it() {
    let g = graph();
    let a = g.node_idx("A").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.set_zones(zones(&[gate()]));
    // standing on A facing backwards (π)
    w.add_robot(
        id("R-1"),
        spec(),
        ActionTimes::new(1.0),
        a,
        std::f64::consts::PI,
    )
    .unwrap();
    w.send_order(&order(0, "A", true, true)).unwrap();
    run_ticks(&mut w, 200);
    let p = w.pose(&id("R-1")).unwrap();
    assert!(p.heading.abs() < 1e-9, "turned to face B: {}", p.heading);
    assert_eq!(p.x, 0.0, "but did not drive");
}

#[test]
fn a_released_update_makes_the_robot_request_and_wait() {
    let mut w = world(&[gate()], OnRelease);
    w.send_order(&order(0, "A", false, false)).unwrap();
    run_ticks(&mut w, 100);
    assert!(requests(&w).is_empty());
    w.send_order(&order(1, "U", true, false)).unwrap();
    run_ticks(&mut w, 400);
    assert_eq!(
        w.pose(&id("R-1")).unwrap().x,
        0.0,
        "released, requested, waiting"
    );
    respond(&mut w, "req-1", GrantType::Granted);
    run_ticks(&mut w, 1500);
    assert_eq!(w.pose(&id("R-1")).unwrap().x, 10.0, "arrived at B");
}

#[test]
fn edges_that_do_not_enter_a_zone_need_no_grant() {
    // the zone is on A-B; B-C does not touch it
    let g = graph();
    let b = g.node_idx("B").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.set_zones(zones(&[gate()]));
    w.add_robot(id("R-1"), spec(), ActionTimes::new(1.0), b, 0.0)
        .unwrap();
    let o: Order = serde_json::from_str(
        r#"{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"B","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"C","sequenceId":2,"released":true,"actions":[]}],
        "edges":[{"edgeId":"B-C","sequenceId":1,"released":true,"actions":[]}]}"#,
    )
    .unwrap();
    w.send_order(&o).unwrap();
    run_ticks(&mut w, 1500);
    assert_eq!(
        w.pose(&id("R-1")).unwrap().y,
        5.0,
        "drove to C without any grant"
    );
}

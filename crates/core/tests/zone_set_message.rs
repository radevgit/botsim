//! The `zoneSet` message and the `enableZoneSet` / `deleteZoneSet` instant actions
//! (S62): a robot has zones only after the controller sent them and enabled one.

use botsim_core::actions::ActionTimes;
use botsim_core::drive::Phase;
use botsim_core::robot::Reception;
use botsim_core::world::{RobotId, World};
use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::instant_actions::InstantActions;
use vda5050::order::Order;
use vda5050::state::{ActionStatus, EnabledStatus, State};
use vda5050::zone_set::ZoneSetMessage;
use vdma_lif::Lif;

const DT: f64 = 0.05;

fn id() -> RobotId {
    RobotId::new("ACME", "R-1")
}

/// A (0,0) → B (10,0) → C (10,5); R-1 on A; **no zones**.
fn world() -> World {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    let g = Graph::from_lif(&lif).unwrap();
    let a = g.node_idx("A").unwrap();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(
        id(),
        RobotSpec::from_factsheet(&f).unwrap(),
        ActionTimes::new(1.0),
        a,
        0.0,
    )
    .unwrap();
    w
}

fn zone_set(zone_set_id: &str, map_id: &str, zones: &str) -> ZoneSetMessage {
    serde_json::from_str(&format!(
        r#"{{"headerId":1,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1",
        "zoneSet":{{"mapId":"{map_id}","zoneSetId":"{zone_set_id}","zones":[{zones}]}}}}"#
    ))
    .unwrap()
}

/// A RELEASE zone on the way from A to B (x 4 … 7).
const GATE: &str = r#"{"zoneId":"gate","zoneType":"RELEASE","vertices":[
    {"x":4.0,"y":-1.0},{"x":7.0,"y":-1.0},{"x":7.0,"y":1.0},{"x":4.0,"y":1.0}]}"#;

fn gate_set(zone_set_id: &str) -> ZoneSetMessage {
    zone_set(zone_set_id, "M", GATE)
}

fn actions(actions: &str) -> InstantActions {
    serde_json::from_str(&format!(
        r#"{{"headerId":2,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","actions":[{actions}]}}"#
    ))
    .unwrap()
}

fn on_zone_set(kind: &str, action_id: &str, zone_set_id: &str) -> InstantActions {
    actions(&format!(
        r#"{{"actionType":"{kind}","actionId":"{action_id}","blockingType":"NONE",
        "actionParameters":[{{"key":"zoneSetId","value":"{zone_set_id}"}}]}}"#
    ))
}

fn enable(w: &mut World, zone_set_id: &str) {
    w.send_instant_actions(&on_zone_set("enableZoneSet", "en", zone_set_id))
        .unwrap();
}

/// A → B → C, everything released.
fn order() -> Order {
    serde_json::from_str(
        r#"{"headerId":1,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"A","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"B","sequenceId":2,"released":true,"actions":[]},
                 {"nodeId":"C","sequenceId":4,"released":true,"actions":[]}],
        "edges":[{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]},
                 {"edgeId":"B-C","sequenceId":3,"released":true,"actions":[]}]}"#,
    )
    .unwrap()
}

fn state(w: &mut World) -> State {
    w.state(&id()).unwrap()
}

fn sets(s: &State) -> Vec<(String, String, EnabledStatus)> {
    s.zone_sets
        .iter()
        .flatten()
        .map(|z| (z.zone_set_id.clone(), z.map_id.clone(), z.zone_set_status))
        .collect()
}

fn action(s: &State, action_id: &str) -> Option<ActionStatus> {
    s.instant_action_states
        .iter()
        .find(|a| a.action_id == action_id)
        .map(|a| a.action_status)
}

fn error_types(s: &State) -> Vec<String> {
    s.errors.iter().map(|e| e.error_type.clone()).collect()
}

#[test]
fn a_robot_has_no_zone_sets_and_no_zone_requests_until_it_is_sent_one() {
    let mut w = world();
    let s = state(&mut w);
    assert!(s.zone_sets.is_none() && s.zone_requests.is_none());

    // sent: stored, disabled, on the map it names, and still no requests
    w.send_zone_set(&gate_set("zs1")).unwrap();
    let s = state(&mut w);
    assert_eq!(
        sets(&s),
        vec![("zs1".into(), "M".into(), EnabledStatus::Disabled)]
    );
    assert!(s.zone_requests.is_none());
    assert!(s.errors.is_empty());
}

#[test]
fn a_disabled_zone_set_changes_nothing_for_the_robot() {
    let mut w = world();
    w.send_zone_set(&gate_set("zs1")).unwrap();
    assert_eq!(w.send_order(&order()), Ok(Reception::Accepted));
    for _ in 0..2500 {
        w.step();
    }
    // it drove through the zone without asking
    assert_eq!(w.phase(&id()), Ok(Phase::Finished));
    assert!(state(&mut w).zone_requests.is_none());
}

#[test]
fn an_enabled_zone_set_makes_the_robot_ask_and_wait() {
    let mut w = world();
    w.send_zone_set(&gate_set("zs1")).unwrap();
    enable(&mut w, "zs1");
    let s = state(&mut w);
    assert_eq!(
        sets(&s),
        vec![("zs1".into(), "M".into(), EnabledStatus::Enabled)]
    );
    assert_eq!(action(&s, "en"), Some(ActionStatus::Finished));
    assert_eq!(s.zone_requests, Some(vec![]), "zones but no request yet");

    assert_eq!(w.send_order(&order()), Ok(Reception::Accepted));
    for _ in 0..300 {
        w.step();
    }
    // it asked for the gate and waits on A for the grant
    let s = state(&mut w);
    assert_eq!(s.zone_requests.as_ref().map(Vec::len), Some(1));
    assert_eq!(w.phase(&id()), Ok(Phase::WaitingForClearance));
    assert_eq!(w.pose(&id()).unwrap().x, 0.0);
}

#[test]
fn a_zone_set_enabled_while_an_order_runs_counts_for_the_rest_of_it() {
    let mut w = world();
    w.send_zone_set(&gate_set("zs1")).unwrap();
    assert_eq!(w.send_order(&order()), Ok(Reception::Accepted));
    for _ in 0..20 {
        w.step();
    }
    assert_ne!(w.phase(&id()), Ok(Phase::WaitingForClearance));
    enable(&mut w, "zs1");
    for _ in 0..400 {
        w.step();
    }
    // it was on the edge into the gate already: it brakes and stands there, before
    // the zone, asking for it
    let x = w.pose(&id()).unwrap().x;
    assert!(x > 0.0 && x < 4.0, "x = {x}");
    let s = state(&mut w);
    assert!(!s.driving);
    assert_eq!(s.zone_requests.as_ref().map(Vec::len), Some(1));
}

#[test]
fn enabling_a_zone_set_disables_the_other_ones() {
    let mut w = world();
    w.send_zone_set(&gate_set("zs1")).unwrap();
    w.send_zone_set(&gate_set("zs2")).unwrap();
    enable(&mut w, "zs1");
    enable(&mut w, "zs2");
    let status: Vec<_> = sets(&state(&mut w))
        .into_iter()
        .map(|s| (s.0, s.2))
        .collect();
    assert_eq!(
        status,
        vec![
            ("zs1".to_string(), EnabledStatus::Disabled),
            ("zs2".to_string(), EnabledStatus::Enabled)
        ]
    );
}

#[test]
fn a_zone_set_with_a_known_id_is_not_taken_over_and_the_error_goes_away_later() {
    let mut w = world();
    w.send_zone_set(&gate_set("zs1")).unwrap();
    w.send_zone_set(&gate_set("zs1")).unwrap();
    let s = state(&mut w);
    assert_eq!(sets(&s).len(), 1, "stored once");
    assert_eq!(error_types(&s), vec!["DUPLICATE_ZONE_SET"]);
    let e = &s.errors[0];
    assert_eq!(e.error_level, vda5050::state::ErrorLevel::Warning);
    let refs: Vec<_> = e
        .error_references
        .iter()
        .flatten()
        .map(|r| (r.reference_key.as_str(), r.reference_value.as_str()))
        .collect();
    assert_eq!(refs, vec![("zoneSetId", "zs1")]);
    // "a reasonable amount of time" (10 s): gone after it
    for _ in 0..(10.5 / DT) as u32 {
        w.step();
    }
    assert!(state(&mut w).errors.is_empty());
}

#[test]
fn a_zone_set_that_does_not_fit_the_layout_is_refused() {
    let mut w = world();
    // a map the layout does not have
    w.send_zone_set(&zone_set("zs1", "Nowhere", GATE)).unwrap();
    // a bow tie
    let bow = r#"{"zoneId":"z","zoneType":"RELEASE","vertices":[
        {"x":0.0,"y":0.0},{"x":1.0,"y":1.0},{"x":1.0,"y":0.0},{"x":0.0,"y":1.0}]}"#;
    w.send_zone_set(&zone_set("zs2", "M", bow)).unwrap();
    let s = state(&mut w);
    assert!(s.zone_sets.is_none(), "neither is stored");
    assert_eq!(
        error_types(&s),
        vec!["UNKNOWN_MAP_ID", "VALIDATION_FAILURE"]
    );
}

#[test]
fn enable_and_delete_fail_for_zone_sets_that_are_not_there_or_in_use() {
    let mut w = world();
    w.send_zone_set(&gate_set("zs1")).unwrap();
    w.send_zone_set(&gate_set("zs2")).unwrap();
    enable(&mut w, "zs1");

    let status = |w: &mut World, a: &str| action(&state(w), a);
    w.send_instant_actions(&on_zone_set("enableZoneSet", "e9", "nope"))
        .unwrap();
    assert_eq!(status(&mut w, "e9"), Some(ActionStatus::Failed));
    // delete: not there, enabled (in use), disabled (fine)
    w.send_instant_actions(&on_zone_set("deleteZoneSet", "d1", "nope"))
        .unwrap();
    w.send_instant_actions(&on_zone_set("deleteZoneSet", "d2", "zs1"))
        .unwrap();
    w.send_instant_actions(&on_zone_set("deleteZoneSet", "d3", "zs2"))
        .unwrap();
    let s = state(&mut w);
    assert_eq!(action(&s, "d1"), Some(ActionStatus::Failed));
    assert_eq!(action(&s, "d2"), Some(ActionStatus::Failed));
    assert_eq!(action(&s, "d3"), Some(ActionStatus::Finished));
    assert_eq!(
        sets(&s),
        vec![("zs1".into(), "M".into(), EnabledStatus::Enabled)]
    );
    assert_eq!(
        error_types(&s),
        vec!["INVALID_INSTANT_ACTION"; 3],
        "one error for each of the three that failed"
    );
}

#[test]
fn a_zone_set_for_an_unknown_robot_is_an_error() {
    let mut w = world();
    let mut m = gate_set("zs1");
    m.serial_number = "R-9".into();
    assert!(w.send_zone_set(&m).is_err());
}

#[test]
fn with_a_message_delay_the_zone_set_arrives_late() {
    let mut w = world();
    w.set_message_delay(10);
    w.send_zone_set(&gate_set("zs1")).unwrap();
    for _ in 0..10 {
        w.step();
    }
    // sent at step 0: it arrives in the 11th step; the state shows it 10 steps after
    assert!(state(&mut w).zone_sets.is_none());
    for _ in 0..30 {
        w.step();
        let _ = state(&mut w);
    }
    assert_eq!(sets(&state(&mut w)).len(), 1);
}

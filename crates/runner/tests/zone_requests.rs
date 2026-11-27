use std::path::{Path, PathBuf};

use botsim::scenario::Scenario;
use botsim_model::robot::RobotId;
use vda5050::order::Order;

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

/// Two robot groups on the ring (N0 and N2), zones from zones.json: the RELEASE
/// zone "crossing" is around N1 (x 15 … 25), so N0 → N1 enters it.
fn scenario(timing_a: &str, timing_b: &str) -> String {
    format!(
        r#"
name = "zone-requests"
seed = 1
duration = 10.0
time_step = 0.05
start_time = 2026-11-20T10:00:00Z
lif = "ring.lif.json"
zone_set = "zones.json"

[[robot]]
manufacturer = "ACME"
serial_prefix = "A"
factsheet = "diff_robot.factsheet.json"
start_nodes = ["N0"]
action_time = 1.0
{timing_a}

[[robot]]
manufacturer = "ACME"
serial_prefix = "B"
factsheet = "diff_robot.factsheet.json"
start_nodes = ["N0"]
action_time = 1.0
{timing_b}
"#
    )
}

/// An order for `serial`: N0 → N1 with N0 released and N1 not (a horizon edge).
fn horizon_order(serial: &str) -> Order {
    serde_json::from_str(&format!(
        r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"{serial}","orderId":"o1","orderUpdateId":0,
        "nodes":[{{"nodeId":"N0","sequenceId":0,"released":true,"actions":[]}},
                 {{"nodeId":"N1","sequenceId":2,"released":false,"actions":[]}}],
        "edges":[{{"edgeId":"N0-N1","sequenceId":1,"released":false,"actions":[]}}]}}"#
    ))
    .unwrap()
}

/// What the controller does at the start (S62): send the site's zone set to the
/// robot and enable it.
fn give_the_zone_set(w: &mut botsim_core::world::World, serial: &str) {
    let z = w.zones().expect("the scenario has a zone set");
    let set = serde_json::json!({
        "mapId": w.graph().maps[z.map],
        "zoneSetId": z.zone_set_id,
        "zones": z.zones,
    });
    let header = |extra: serde_json::Value| {
        let mut m = serde_json::json!({
            "headerId": 0, "timestamp": "2026-11-20T10:00:00.000Z", "version": "3.0.1",
            "manufacturer": "ACME", "serialNumber": serial,
        });
        m.as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        m
    };
    let id = z.zone_set_id.clone();
    let zone_set = header(serde_json::json!({ "zoneSet": set }));
    w.send_zone_set(&serde_json::from_value(zone_set).unwrap())
        .unwrap();
    let enable = header(serde_json::json!({ "actions": [{
        "actionType": "enableZoneSet", "actionId": "enable", "blockingType": "NONE",
        "actionParameters": [{"key": "zoneSetId", "value": id}] }] }));
    w.send_instant_actions(&serde_json::from_value(enable).unwrap())
        .unwrap();
}

fn requests_after_horizon_order(text: &str) -> (usize, usize) {
    let s = Scenario::from_toml(text).unwrap();
    let mut w = s.build_world(&data()).unwrap();
    for serial in ["A-001", "B-001"] {
        give_the_zone_set(&mut w, serial);
        w.send_order(&horizon_order(serial)).unwrap();
    }
    let count = |n: &str| w.zone_requests(&RobotId::new("ACME", n)).unwrap().len();
    (count("A-001"), count("B-001"))
}

#[test]
fn the_default_is_to_request_when_the_edge_is_released() {
    // no setting: nobody asks for a horizon edge
    assert_eq!(requests_after_horizon_order(&scenario("", "")), (0, 0));
}

#[test]
fn the_timing_is_a_setting_per_robot_group() {
    let on_horizon = r#"zone_requests = "on_horizon""#;
    let on_release = r#"zone_requests = "on_release""#;
    // a mixed fleet: group A asks early, group B when released
    assert_eq!(
        requests_after_horizon_order(&scenario(on_horizon, on_release)),
        (1, 0)
    );
    assert_eq!(
        requests_after_horizon_order(&scenario(on_release, on_horizon)),
        (0, 1)
    );
    assert_eq!(
        requests_after_horizon_order(&scenario(on_horizon, on_horizon)),
        (1, 1)
    );
}

#[test]
fn a_wrong_value_is_refused() {
    let e = Scenario::from_toml(&scenario(r#"zone_requests = "sometimes""#, ""))
        .unwrap_err()
        .to_string();
    assert!(
        e.contains("zone_requests") || e.contains("sometimes"),
        "{e}"
    );
}

#[test]
fn without_a_zone_set_the_setting_does_nothing() {
    let text =
        scenario(r#"zone_requests = "on_horizon""#, "").replace("zone_set = \"zones.json\"", "");
    let s = Scenario::from_toml(&text).unwrap();
    let mut w = s.build_world(&data()).unwrap();
    w.send_order(&horizon_order("A-001")).unwrap();
    assert!(
        w.zone_requests(&RobotId::new("ACME", "A-001"))
            .unwrap()
            .is_empty()
    );
    assert!(
        w.state(&RobotId::new("ACME", "A-001"))
            .unwrap()
            .zone_requests
            .is_none()
    );
}

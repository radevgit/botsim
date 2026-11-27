//! Chargers (charging brief, step 1): a robot with a battery that stands on the
//! interaction node of a `charger` station gains charge after `startCharging` until
//! `stopCharging`, until it is full, or until it moves.

use botsim_core::actions::ActionTimes;
use botsim_core::battery::{Battery, BatterySpec};
use botsim_core::world::{World, WorldError};
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::instant_actions::InstantActions;
use vda5050::state::ActionStatus;
use vdma_lif::Lif;

const DT: f64 = 0.05;

/// The three-node layout; the station C1 of type `charger` has node A, the
/// station P of type `parking` node B.
fn graph() -> Graph {
    let mut v: serde_json::Value =
        serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    v["origins"][0]["layouts"][0]["stations"] = serde_json::json!([
        {"stationId": "C1", "stationType": "charger", "interactionNodeIds": ["A"]},
        {"stationId": "P", "stationType": "parking", "interactionNodeIds": ["B"]}
    ]);
    let lif: Lif = serde_json::from_value(v).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn id() -> RobotId {
    RobotId::new("ACME", "R-001")
}

/// 100 Wh battery, no drain at all (so only charging changes it), 50 % at start; the
/// robot stands on `node`.
fn world(node: &str, power_w: Option<f64>) -> World {
    let g = graph();
    let start = g.node_idx(node).unwrap();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(
        id(),
        RobotSpec::from_factsheet(&f).unwrap(),
        ActionTimes::new(1.0),
        start,
        0.0,
    )
    .unwrap();
    let spec = BatterySpec {
        capacity_wh: 100.0,
        drive_wh_per_m: 0.0,
        idle_w: 0.0,
        work_w: 0.0,
    };
    w.set_battery(&id(), Battery::new(spec, 50.0).unwrap())
        .unwrap();
    if let Some(p) = power_w {
        assert_eq!(w.set_chargers(p).unwrap(), 1);
    }
    w
}

fn action(kind: &str, action_id: &str) -> InstantActions {
    let json = format!(
        r#"{{"headerId":2,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001",
        "actions":[{{"actionType":"{kind}","actionId":"{action_id}","blockingType":"NONE"}}]}}"#
    );
    serde_json::from_str(&json).unwrap()
}

fn run(w: &mut World, seconds: f64) {
    for _ in 0..(seconds / DT).round() as usize {
        w.step();
    }
}

fn percent(w: &World) -> f64 {
    w.battery_percent(&id()).unwrap().unwrap()
}

fn charging(w: &mut World) -> bool {
    w.state(&id()).unwrap().power_supply.charging
}

fn status(w: &mut World, action_id: &str) -> ActionStatus {
    w.state(&id())
        .unwrap()
        .instant_action_states
        .iter()
        .find(|a| a.action_id == action_id)
        .unwrap()
        .action_status
}

#[test]
fn a_robot_on_a_charger_charges_after_start_and_stops_after_stop() {
    // 3600 W = 1 Wh per second = 1 % per second of 100 Wh
    let mut w = world("A", Some(3600.0));
    run(&mut w, 5.0);
    assert!(
        (percent(&w) - 50.0).abs() < 1e-6,
        "no charging before start"
    );
    assert!(!charging(&mut w));

    w.send_instant_actions(&action("startCharging", "s1"))
        .unwrap();
    run(&mut w, 10.0);
    assert!((percent(&w) - 60.0).abs() < 0.1, "{}", percent(&w));
    assert!(charging(&mut w));
    assert_eq!(status(&mut w, "s1"), ActionStatus::Finished);

    w.send_instant_actions(&action("stopCharging", "s2"))
        .unwrap();
    let held = percent(&w);
    run(&mut w, 10.0);
    assert!((percent(&w) - held).abs() < 1e-6);
    assert!(!charging(&mut w));
    assert_eq!(status(&mut w, "s2"), ActionStatus::Finished);
}

#[test]
fn charging_ends_when_the_battery_is_full() {
    let mut w = world("A", Some(3600.0));
    w.send_instant_actions(&action("startCharging", "s1"))
        .unwrap();
    run(&mut w, 60.0); // 50 % more takes 50 s
    assert_eq!(percent(&w), 100.0);
    assert!(!charging(&mut w), "full: no longer charging");
}

#[test]
fn start_charging_fails_off_a_charger_and_without_chargers() {
    // on node B, a parking station
    let mut w = world("B", Some(3600.0));
    w.send_instant_actions(&action("startCharging", "s1"))
        .unwrap();
    run(&mut w, 5.0);
    assert_eq!(status(&mut w, "s1"), ActionStatus::Failed);
    assert!((percent(&w) - 50.0).abs() < 1e-6);
    assert!(!charging(&mut w));

    // chargers are not switched on in the world
    let mut w = world("A", None);
    w.send_instant_actions(&action("startCharging", "s1"))
        .unwrap();
    run(&mut w, 5.0);
    assert_eq!(status(&mut w, "s1"), ActionStatus::Failed);
}

#[test]
fn the_charger_power_must_be_positive() {
    let mut w = world("A", None);
    assert_eq!(
        w.set_chargers(0.0).unwrap_err().to_string(),
        WorldError::BadChargerPower(0.0).to_string()
    );
}

//! The battery of a robot (charging brief, step 1): the charge falls with the metres
//! driven and the time standing, and the state reports it; a robot without a battery
//! model reports 100 % as before.

use botsim_core::actions::ActionTimes;
use botsim_core::battery::{Battery, BatterySpec};
use botsim_core::robot::Robot;
use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
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
    Robot::new(
        &g,
        RobotSpec::from_factsheet(&f).unwrap(),
        ActionTimes::new(2.0),
        g.node_idx("A").unwrap(),
        0.0,
    )
    .unwrap()
}

/// A→B→C (10 m east, then 5 m north), fully released; no actions.
fn order() -> Order {
    serde_json::from_str(
        r#"{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"A","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"B","sequenceId":2,"released":true,"actions":[]},
                 {"nodeId":"C","sequenceId":4,"released":true,"actions":[]}],
        "edges":[{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]},
                 {"edgeId":"B-C","sequenceId":3,"released":true,"actions":[]}]}"#,
    )
    .unwrap()
}

fn state(r: &Robot) -> State {
    r.state(
        &graph(),
        &Header {
            header_id: 1,
            timestamp: "2026-11-20T10:00:00.000Z".into(),
            version: "3.0.1".into(),
            manufacturer: "ACME".into(),
            serial_number: "R-001".into(),
        },
    )
}

fn spec() -> BatterySpec {
    BatterySpec {
        capacity_wh: 100.0,
        drive_wh_per_m: 1.0,
        idle_w: 36.0,
        work_w: 72.0,
    }
}

fn run(r: &mut Robot, seconds: f64) {
    for _ in 0..(seconds / DT) as usize {
        r.step(DT);
    }
}

#[test]
fn without_a_battery_model_the_state_says_100_percent() {
    let mut r = robot();
    r.receive_order(&graph(), &order());
    run(&mut r, 60.0);
    assert_eq!(state(&r).power_supply.state_of_charge, 100.0);
    assert!(r.battery().is_none());
}

#[test]
fn the_charge_falls_by_the_energy_per_metre_while_driving() {
    let g = graph();
    let mut r = robot();
    r.set_battery(Battery::new(spec(), 100.0).unwrap());
    r.receive_order(&g, &order());
    run(&mut r, 120.0); // the 15 m are driven well before, then it stands
    let drive = 15.0 * 1.0; // Wh
    let stand = 36.0 / 3600.0 * 120.0; // at most the whole time at idle power
    let used = 100.0 - r.battery().unwrap().charge_wh();
    assert!(
        used >= drive - 0.5 && used <= drive + stand + 0.5,
        "used {used} Wh"
    );
    let soc = state(&r).power_supply.state_of_charge;
    assert!((soc - r.battery().unwrap().percent()).abs() < 1e-9);
    assert!(soc < 100.0 && soc > 80.0, "soc {soc}");
}

#[test]
fn an_empty_battery_stops_the_robot_where_it_is_and_the_state_says_so() {
    let g = graph();
    let mut r = robot();
    // 10 Wh per metre, 30 Wh: empty after 3 m of the 15 m
    let mut s = spec();
    s.drive_wh_per_m = 10.0;
    r.set_battery(Battery::new(s, 30.0).unwrap());
    r.receive_order(&g, &order());
    run(&mut r, 20.0);
    assert!(r.is_out_of_charge() && r.ran_empty());
    let x = r.pose().x;
    assert!(
        x > 2.5 && x < 6.0,
        "stopped after about 3 m (braking), x = {x}"
    );
    run(&mut r, 30.0);
    assert_eq!(r.pose().x, x, "it does not move on");
    let st = state(&r);
    assert_eq!(st.power_supply.state_of_charge, 0.0);
    assert!(
        st.errors.iter().any(|e| e.error_type == "BATTERY_EMPTY"),
        "{:?}",
        st.errors
    );
}

#[test]
fn a_robot_with_charge_left_has_no_battery_error() {
    let mut r = robot();
    r.set_battery(Battery::new(spec(), 100.0).unwrap());
    run(&mut r, 10.0);
    assert!(!r.is_out_of_charge() && !r.ran_empty());
    assert!(
        state(&r)
            .errors
            .iter()
            .all(|e| e.error_type != "BATTERY_EMPTY")
    );
}

#[test]
fn standing_costs_the_idle_power() {
    let mut r = robot();
    r.set_battery(Battery::new(spec(), 100.0).unwrap());
    run(&mut r, 100.0); // 100 s at 36 W = 1 Wh
    let used = 100.0 - r.battery().unwrap().charge_wh();
    assert!((used - 1.0).abs() < 0.01, "used {used} Wh");
}

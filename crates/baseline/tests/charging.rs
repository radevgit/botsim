//! The baseline sends a free robot with a low battery to a free charger and starts
//! the charging there (charging brief, step 2, first part).

use botsim_baseline::controller::{ChargePolicy, Controller};
use botsim_core::actions::ActionTimes;
use botsim_core::battery::{Battery, BatterySpec};
use botsim_core::world::World;
use botsim_model::graph::GraphStation;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;

mod common;
use common::graph;

const DT: f64 = 0.05;

fn id(n: &str) -> RobotId {
    RobotId::new("ACME", n)
}

/// A → B → C, the charger (a station of type `charger`) on C. Robots R1 on A and
/// (if `second` is a node name) R2 on that node, each with `percent` charge and no
/// drain, 3600 W chargers (1 % per second of 100 Wh).
fn setup(percents: &[f64], second: Option<&str>) -> (World, Controller) {
    let mut g = graph(
        &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("C", 20.0, 0.0)],
        &[("A", "B"), ("B", "C")],
    );
    let c = g.node_idx("C").unwrap();
    g.stations.push(GraphStation {
        id: "CH".into(),
        name: None,
        station_type: Some("charger".into()),
        interaction_nodes: vec![c],
    });
    let a = g.node_idx("A").unwrap();
    let second = second.map(|n| g.node_idx(n).unwrap());
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let times = ActionTimes::new(1.0);
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(id("R1"), spec.clone(), times.clone(), a, 0.0)
        .unwrap();
    if let Some(n) = second {
        w.add_robot(id("R2"), spec, times, n, 0.0).unwrap();
    }
    let bs = BatterySpec {
        capacity_wh: 100.0,
        drive_wh_per_m: 0.0,
        idle_w: 0.0,
        work_w: 0.0,
    };
    for (n, p) in ["R1", "R2"].iter().zip(percents) {
        w.set_battery(&id(n), Battery::new(bs, *p).unwrap())
            .unwrap();
    }
    assert_eq!(w.set_chargers(3600.0).unwrap(), 1);
    w.set_safety_stop(false);
    let mut ctl = Controller::new(&w);
    assert_eq!(
        ctl.set_charging(ChargePolicy {
            low: 25.0,
            high: 80.0,
            critical: 10.0
        }),
        1
    );
    (w, ctl)
}

fn run(w: &mut World, ctl: &mut Controller, seconds: f64) {
    for _ in 0..(seconds / DT) as usize {
        ctl.step(w, []);
        w.step();
    }
}

#[test]
fn a_free_robot_below_the_low_limit_drives_to_the_charger_and_charges() {
    let (mut w, mut ctl) = setup(&[10.0], None);
    run(&mut w, &mut ctl, 40.0);
    let p = w.pose(&id("R1")).unwrap();
    assert!((p.x - 20.0).abs() < 0.2, "R1 at x = {}", p.x);
    assert!(w.state(&id("R1")).unwrap().power_supply.charging);
    assert!(w.battery_percent(&id("R1")).unwrap().unwrap() > 10.0);
    assert_eq!(ctl.charge_trips(), 1);
    // a charging trip is not a job
    assert!(ctl.completed().is_empty() && ctl.rejected().is_empty());
}

#[test]
fn at_the_upper_limit_charging_stops_and_the_robot_takes_jobs_again() {
    use botsim_core::jobs::Job;
    let (mut w, mut ctl) = setup(&[10.0], None);
    // 1 % per second: 80 % after 70 s
    run(&mut w, &mut ctl, 60.0);
    assert!(w.state(&id("R1")).unwrap().power_supply.charging);
    run(&mut w, &mut ctl, 30.0);
    let soc = w.battery_percent(&id("R1")).unwrap().unwrap();
    assert!((80.0..82.0).contains(&soc), "stopped at {soc}");
    assert!(!w.state(&id("R1")).unwrap().power_supply.charging);
    // it is free again: a job on the charger node is taken and done
    let c = w.graph().node_idx("C").unwrap();
    let job = Job {
        id: 1,
        pickup: c,
        drop: c,
        created_tick: w.tick(),
    };
    ctl.step(&mut w, [job]);
    run(&mut w, &mut ctl, 30.0);
    assert_eq!(ctl.completed().len(), 1);
    // it has more than the low limit, so it does not go to charge again
    assert_eq!(ctl.charge_trips(), 1);
}

/// R2 occupies the only charger (it starts on it); R1 on A needs a charger and gets
/// a job A → B at once.
fn job_with_the_charger_taken(r1_percent: f64) -> (World, Controller) {
    use botsim_core::jobs::Job;
    let (mut w, mut ctl) = setup(&[r1_percent, 10.0], Some("C"));
    let (a, b) = (
        w.graph().node_idx("A").unwrap(),
        w.graph().node_idx("B").unwrap(),
    );
    let job = Job {
        id: 1,
        pickup: a,
        drop: b,
        created_tick: 0,
    };
    ctl.step(&mut w, [job]);
    run(&mut w, &mut ctl, 30.0);
    (w, ctl)
}

#[test]
fn between_the_critical_and_the_low_limit_a_robot_without_a_free_charger_keeps_working() {
    let (_w, ctl) = job_with_the_charger_taken(15.0);
    assert_eq!(ctl.completed().len(), 1, "it takes the job");
    assert_eq!(
        ctl.charge_trips(),
        1,
        "only R2 charged (it was on the charger)"
    );
}

#[test]
fn below_the_critical_limit_a_robot_takes_no_job_and_the_waiting_is_counted() {
    let (w, ctl) = job_with_the_charger_taken(5.0);
    assert_eq!(ctl.completed().len(), 0, "no job below 10 %");
    assert_eq!(w.pose(&id("R1")).unwrap().x, 0.0);
    // R1 waited about the 30 s for a charger (R2 charges 70 s to 80 %)
    assert!(
        (25.0..=31.0).contains(&ctl.charge_wait_seconds()),
        "{}",
        ctl.charge_wait_seconds()
    );
}

#[test]
fn a_robot_above_the_low_limit_stays_where_it_is() {
    let (mut w, mut ctl) = setup(&[60.0], None);
    run(&mut w, &mut ctl, 30.0);
    assert_eq!(w.pose(&id("R1")).unwrap().x, 0.0);
    assert_eq!(ctl.charge_trips(), 0);
}

#[test]
fn one_charger_takes_one_robot_the_other_waits_where_it_is() {
    // R1 on A and R2 on B are both low; R2 is nearer
    let (mut w, mut ctl) = setup(&[10.0, 10.0], Some("B"));
    run(&mut w, &mut ctl, 60.0);
    assert!((w.pose(&id("R2")).unwrap().x - 20.0).abs() < 0.2);
    assert!(w.state(&id("R2")).unwrap().power_supply.charging);
    assert_eq!(w.pose(&id("R1")).unwrap().x, 0.0, "no free charger for R1");
    assert_eq!(w.battery_percent(&id("R1")).unwrap(), Some(10.0));
    assert_eq!(ctl.charge_trips(), 1);
}

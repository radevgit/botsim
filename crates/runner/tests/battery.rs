//! The battery settings of a `[[robot]]` group (charging brief, step 1): no battery
//! without `battery_wh`, a start charge from a number or a seeded range, checks.

use std::path::{Path, PathBuf};

use botsim::scenario::Scenario;
use botsim_core::world::World;
use botsim_model::robot::RobotId;

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

fn text(seed: u64, battery: &str) -> String {
    format!(
        r#"
name = "t"
seed = {seed}
duration = 10.0
time_step = 0.05
start_time = 2026-11-20T10:00:00Z
lif = "three_nodes.lif.json"

[[robot]]
manufacturer = "ACME"
serial_prefix = "R"
factsheet = "diff_robot.factsheet.json"
start_nodes = ["A", "B", "C", "A", "B", "C"]
action_time = 2.0
{battery}
"#
    )
}

fn world(seed: u64, battery: &str) -> World {
    Scenario::from_toml(&text(seed, battery))
        .unwrap()
        .build_world(&data())
        .unwrap()
}

fn charges(w: &World) -> Vec<Option<f64>> {
    let ids: Vec<RobotId> = w.robot_ids().cloned().collect();
    ids.iter()
        .map(|id| w.battery_percent(id).unwrap())
        .collect()
}

fn invalid(battery: &str) -> String {
    match Scenario::from_toml(&text(1, battery)) {
        Err(e) => e.to_string(),
        Ok(_) => panic!("accepted: {battery}"),
    }
}

#[test]
fn without_battery_wh_the_robots_have_no_battery() {
    let w = world(1, "");
    assert!(charges(&w).iter().all(|c| c.is_none()));
}

#[test]
fn a_battery_starts_full_unless_the_scenario_says_otherwise() {
    let w = world(1, "battery_wh = 500.0\ndrive_wh_per_m = 0.5");
    assert!(charges(&w).iter().all(|c| *c == Some(100.0)));
}

#[test]
fn a_single_start_charge_is_the_same_for_all_robots() {
    let w = world(1, "battery_wh = 500.0\nstart_charge = 40.0");
    assert!(charges(&w).iter().all(|c| *c == Some(40.0)));
}

#[test]
fn a_range_gives_each_robot_its_own_charge_from_the_seed() {
    let b = "battery_wh = 500.0\nstart_charge = [50.0, 100.0]";
    let a = charges(&world(7, b));
    assert!(a.iter().all(|c| (50.0..=100.0).contains(&c.unwrap())));
    // not all the same
    assert!(a.iter().any(|c| *c != a[0]), "{a:?}");
    // the same seed gives the same charges, another seed other ones
    assert_eq!(a, charges(&world(7, b)));
    assert_ne!(a, charges(&world(8, b)));
}

#[test]
fn the_charger_power_is_a_scenario_setting_and_must_be_positive() {
    let with_power = |p: &str| {
        text(1, "battery_wh = 100.0").replacen(
            "seed = 1",
            &format!("seed = 1\ncharger_power_w = {p}"),
            1,
        )
    };
    let good = Scenario::from_toml(&with_power("500.0")).unwrap();
    assert_eq!(good.charger_power_w, Some(500.0));
    good.build_world(&data()).unwrap(); // a layout without chargers is fine
    let bad = Scenario::from_toml(&with_power("0.0")).unwrap();
    let e = bad.build_world(&data()).unwrap_err().to_string();
    assert!(e.contains("charger power"), "{e}");
}

#[test]
fn the_summary_counts_robots_that_ran_empty_and_is_unchanged_without_batteries() {
    use botsim::summary;
    let run = |battery: &str| {
        let s = Scenario::from_toml(&text(1, battery)).unwrap();
        summary::run(&s, s.build_world(&data()).unwrap()).unwrap()
    };
    // no battery model: no battery section
    assert!(run("").battery.is_none());
    // 3600 W standing = 1 Wh/s; 5 Wh is empty after 5 s of the 10 s run
    let r = run("battery_wh = 5.0\nidle_w = 3600.0");
    let b = r.battery.unwrap();
    assert_eq!((b.robots, b.ran_empty, b.empty_at_end), (6, 6, 6));
    assert_eq!(b.charging_seconds, 0.0);
    // 100 Wh lasts
    let b = run("battery_wh = 100.0\nidle_w = 3600.0").battery.unwrap();
    assert_eq!((b.robots, b.ran_empty, b.empty_at_end), (6, 0, 0));
}

#[test]
fn bad_battery_settings_are_refused_with_the_reason() {
    assert!(invalid("idle_w = 5.0").contains("need battery_wh"));
    assert!(invalid("start_charge = 50.0").contains("need battery_wh"));
    assert!(invalid("battery_wh = 0.0").contains("capacity_wh"));
    assert!(invalid("battery_wh = 100.0\nidle_w = -1.0").contains("idle_w"));
    assert!(invalid("battery_wh = 100.0\nstart_charge = [80.0, 50.0]").contains("start_charge"));
    assert!(invalid("battery_wh = 100.0\nstart_charge = 120.0").contains("start_charge"));
}

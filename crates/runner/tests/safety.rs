use std::path::{Path, PathBuf};

use botsim_runner::scenario::Scenario;

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

fn scenario(extra: &str) -> String {
    format!(
        r#"
name = "safety"
seed = 1
duration = 10.0
time_step = 0.05
start_time = 2026-11-21T10:00:00Z
lif = "ring.lif.json"
{extra}

[[robot]]
manufacturer = "ACME"
serial_prefix = "R"
factsheet = "diff_robot.factsheet.json"
start_nodes = ["N0"]
action_time = 1.0
"#
    )
}

fn safety_stop(extra: &str) -> bool {
    Scenario::from_toml(&scenario(extra))
        .unwrap()
        .build_world(&data())
        .unwrap()
        .safety_stop()
}

#[test]
fn the_safety_stop_is_on_unless_the_scenario_switches_it_off() {
    assert!(safety_stop(""));
    assert!(safety_stop("safety_stop = true"));
    assert!(!safety_stop("safety_stop = false"));
}

#[test]
fn the_safety_stop_must_be_a_boolean() {
    assert!(Scenario::from_toml(&scenario("safety_stop = \"yes\"")).is_err());
}

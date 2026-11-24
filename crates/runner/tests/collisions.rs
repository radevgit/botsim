use std::path::{Path, PathBuf};
use std::process::Command;

use botsim::scenario::Scenario;
use botsim::summary;

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

#[test]
fn a_run_without_collisions_reports_zero() {
    let s = Scenario::load(&data().join("two_robots.toml")).unwrap();
    let r = summary::run(&s, s.build_world(&data()).unwrap()).unwrap();
    assert_eq!(r.collision_count, 0);
    assert!(r.collisions.is_empty());
}

#[test]
fn collisions_are_counted_and_listed() {
    let s = Scenario::load(&data().join("overlap.toml")).unwrap();
    let r = summary::run(&s, s.build_world(&data()).unwrap()).unwrap();
    assert_eq!(r.collision_count, 1, "one contact, not one per step");
    let c = &r.collisions[0];
    assert_eq!(c.tick, 1);
    assert_eq!(c.time, 0.05);
    assert_eq!(
        c.robots,
        ["ACME/R-001".to_string(), "ACME/R-002".to_string()]
    );
}

#[test]
fn the_cli_json_has_the_collision_fields() {
    let file = data().join("overlap.toml");
    let out = Command::new(env!("CARGO_BIN_EXE_botsim"))
        .arg(file)
        .output()
        .unwrap();
    assert!(out.status.success());
    let json: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert_eq!(json["collision_count"], 1);
    assert_eq!(json["collisions"][0]["tick"], 1);
    assert_eq!(json["collisions"][0]["robots"][1], "ACME/R-002");
}

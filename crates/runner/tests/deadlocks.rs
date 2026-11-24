use std::path::{Path, PathBuf};
use std::process::Command;

use botsim::scenario::Scenario;
use botsim::summary::{self, Summary};

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

fn run_text(text: &str) -> Summary {
    let s = Scenario::from_toml(text).unwrap();
    summary::run(&s, s.build_world(&data()).unwrap()).unwrap()
}

fn base() -> String {
    std::fs::read_to_string(data().join("deadlock.toml")).unwrap()
}

#[test]
fn a_corridor_scenario_deadlocks_and_the_summary_says_who_and_for_how_long() {
    let r = run_text(&base());
    eprintln!("{:?}", r.deadlocks);
    assert!(r.deadlock_count >= 1, "{:?}", r.jobs);
    assert_eq!(r.deadlock_count, r.deadlocks.len());
    let d = &r.deadlocks[0];
    assert_eq!(d.robots, vec!["ACME/R-001", "ACME/R-002"]);
    // reported after the 20 s timeout from the scenario
    assert!((d.detected_time - d.first_seen_time - 20.0).abs() < 1e-6);
    assert!(d.duration_seconds >= 20.0);
    // the robots block each other, they do not collide
    assert_eq!(r.collision_count, 0);
}

fn with_stop(text: &str) -> String {
    text.replacen(
        "deadlock_timeout = 20.0",
        "deadlock_timeout = 20.0\nstop_at_first_deadlock = true",
        1,
    )
}

#[test]
fn the_run_can_stop_at_the_first_deadlock_for_debugging() {
    let full = run_text(&base());
    assert!(!full.stopped_at_first_deadlock);
    assert_eq!(full.ticks, 24000, "1200 s at 0.05 s");

    let stopped = run_text(&with_stop(&base()));
    assert!(stopped.stopped_at_first_deadlock);
    assert_eq!(stopped.deadlock_count, 1);
    assert!(stopped.ticks < full.ticks, "{} steps", stopped.ticks);
    // it ends in the step in which the deadlock was reported, and the deadlock is
    // the first of the full run
    let d = &stopped.deadlocks[0];
    assert!((stopped.ticks as f64 * stopped.time_step - d.detected_time).abs() < 1e-6);
    assert_eq!(d.robots, full.deadlocks[0].robots);
    assert_eq!(d.first_seen_time, full.deadlocks[0].first_seen_time);
    assert_eq!(d.ended_time, None, "it still lasts when the run ends");
}

#[test]
fn without_a_deadlock_the_run_goes_on_to_its_end() {
    let base = base();
    let text = with_stop(base.split("[jobs]").next().unwrap());
    let r = run_text(&text);
    assert!(!r.stopped_at_first_deadlock);
    assert_eq!(r.ticks, 24000);
}

#[test]
fn without_jobs_there_is_no_deadlock() {
    // no [jobs] section: the robots stand still
    let text = base().split("[jobs]").next().unwrap().to_string();
    assert_eq!(run_text(&text).deadlock_count, 0);
}

#[test]
fn the_timeout_must_be_positive_and_the_cli_json_has_the_fields() {
    let bad = base().replace("deadlock_timeout = 20.0", "deadlock_timeout = 0.0");
    assert!(
        Scenario::from_toml(&bad)
            .unwrap_err()
            .to_string()
            .contains("deadlock_timeout")
    );

    let out = Command::new(env!("CARGO_BIN_EXE_botsim"))
        .arg(data().join("deadlock.toml"))
        .output()
        .unwrap();
    assert!(out.status.success());
    let json: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert!(json["deadlock_count"].as_u64().unwrap() >= 1);
    assert_eq!(json["deadlocks"][0]["robots"][0], "ACME/R-001");
    assert!(json["deadlocks"][0]["duration_seconds"].is_number());
}

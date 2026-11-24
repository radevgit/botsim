use std::path::{Path, PathBuf};
use std::process::Command;

use botsim::scenario::Scenario;
use botsim::summary::{self, ticks_for};

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

fn cli(args: &[&str]) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_botsim"))
        .args(args)
        .output()
        .unwrap()
}

#[test]
fn ticks_cover_the_duration() {
    assert_eq!(ticks_for(60.0, 0.05), 1200);
    // 0.1 is not exact in binary; rounding keeps it at 3 and not 2
    assert_eq!(ticks_for(0.3, 0.1), 3);
}

#[test]
fn the_summary_describes_where_the_robots_are() {
    let s = Scenario::load(&data().join("two_robots.toml")).unwrap();
    let world = s.build_world(&data()).unwrap();
    let r = summary::run(&s, world).unwrap();
    assert_eq!(
        (r.scenario.as_str(), r.seed, r.ticks),
        ("two-robots", 1, 1200)
    );
    assert_eq!(r.robots.len(), 2);
    let (a, b) = (&r.robots[0], &r.robots[1]);
    assert_eq!(
        (a.serial_number.as_str(), a.x, a.y, a.map_id.as_str()),
        ("R-001", 0.0, 0.0, "M")
    );
    assert_eq!((b.serial_number.as_str(), b.x, b.y), ("R-002", 10.0, 0.0));
    // nobody sent an order, so no robot has a last node
    assert!(r.robots.iter().all(|r| r.last_node_id.is_empty()));
}

#[test]
fn the_cli_prints_json_and_is_repeatable() {
    let file = data().join("two_robots.toml");
    let file = file.to_str().unwrap();
    let first = cli(&[file]);
    assert!(
        first.status.success(),
        "{}",
        String::from_utf8_lossy(&first.stderr)
    );
    assert!(first.stderr.is_empty());

    let json: serde_json::Value = serde_json::from_slice(&first.stdout).unwrap();
    assert_eq!(json["scenario"], "two-robots");
    assert_eq!(json["ticks"], 1200);
    assert_eq!(json["robots"].as_array().unwrap().len(), 2);

    // same scenario → identical bytes
    assert_eq!(first.stdout, cli(&[file]).stdout);
}

#[test]
fn the_cli_reports_errors_with_exit_codes() {
    // no argument, or too many: usage, exit 2
    let o = cli(&[]);
    assert_eq!(o.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&o.stderr).contains("usage"));
    assert_eq!(cli(&["a", "b"]).status.code(), Some(2));

    // a scenario that cannot be read: message on stderr, exit 1, nothing on stdout
    let o = cli(&["does-not-exist.toml"]);
    assert_eq!(o.status.code(), Some(1));
    assert!(o.stdout.is_empty());
    assert!(String::from_utf8_lossy(&o.stderr).starts_with("error: cannot read"));
}

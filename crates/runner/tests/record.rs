//! `botsim-runner --record`: the recording file and the unchanged summary.

use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};
use std::process::Command;

use botsim_record::read_recording;

fn scenario() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/corridor_deadlock.toml")
}

fn tmp(name: &str) -> PathBuf {
    Path::new(env!("CARGO_TARGET_TMPDIR")).join(name)
}

fn cli(args: &[&str]) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_botsim-runner"))
        .args(args)
        .output()
        .unwrap()
}

#[test]
fn record_writes_a_recording_and_leaves_the_summary_unchanged() {
    let s = scenario();
    let s = s.to_str().unwrap();
    let file = tmp("deadlock.jsonl");
    let file = file.to_str().unwrap();

    let plain = cli(&[s]);
    let recorded = cli(&["--record", file, s]);
    assert!(
        recorded.status.success(),
        "{}",
        String::from_utf8_lossy(&recorded.stderr)
    );
    assert_eq!(recorded.stdout, plain.stdout, "observing changes nothing");

    let rec = read_recording(BufReader::new(File::open(file).unwrap())).unwrap();
    assert_eq!(rec.layout.nodes.len(), 3);
    // 1200 s, one snapshot per 0.1 s by default
    assert_eq!(rec.snapshots.len(), 12000);
    assert_eq!(rec.snapshots[0].robots.len(), 2);
    // the scenario is a deadlock: a recording must show it
    assert!(rec.snapshots.iter().any(|s| !s.deadlocks.is_empty()));
}

#[test]
fn the_layout_line_lists_the_type_of_every_robot() {
    // the mixed fleet has two robot types
    let s = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/mixed_fleet.toml");
    let file = tmp("types.jsonl");
    let o = cli(&[
        "--record-interval",
        "10",
        "--record",
        file.to_str().unwrap(),
        s.to_str().unwrap(),
    ]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let rec = read_recording(BufReader::new(File::open(&file).unwrap())).unwrap();
    // one entry per robot, with the id the snapshots use
    let mut listed: Vec<&str> = rec.layout.robots.iter().map(|r| r.id.as_str()).collect();
    let mut seen: Vec<&str> = rec.snapshots[0]
        .robots
        .iter()
        .map(|r| r.id.as_str())
        .collect();
    listed.sort_unstable();
    seen.sort_unstable();
    assert_eq!(listed, seen);
    assert!(!listed.is_empty());
    // "manufacturer/seriesName"; two different types
    let mut types: Vec<&str> = rec
        .layout
        .robots
        .iter()
        .map(|r| r.robot_type.as_str())
        .collect();
    types.sort_unstable();
    types.dedup();
    assert_eq!(types.len(), 2, "{types:?}");
    assert!(types.iter().all(|t| t.starts_with("ACME/")), "{types:?}");
}

#[test]
fn record_interval_sets_the_snapshot_rate_and_the_file_is_repeatable() {
    let s = scenario();
    let s = s.to_str().unwrap();
    let (a, b) = (tmp("rate_a.jsonl"), tmp("rate_b.jsonl"));
    for f in [&a, &b] {
        let o = cli(&["--record-interval", "2", "--record", f.to_str().unwrap(), s]);
        assert!(o.status.success());
    }
    let rec = read_recording(BufReader::new(File::open(&a).unwrap())).unwrap();
    assert_eq!(rec.snapshots.len(), 600);
    assert_eq!(std::fs::read(&a).unwrap(), std::fs::read(&b).unwrap());
}

#[test]
fn geometry_goes_into_the_layout_line() {
    let s = scenario();
    let s = s.to_str().unwrap();
    let geometry = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios/corridor_deadlock.geometry.json");
    let file = tmp("with_geometry.jsonl");
    let o = cli(&[
        "--record-interval",
        "5",
        "--record",
        file.to_str().unwrap(),
        "--geometry",
        geometry.to_str().unwrap(),
        s,
    ]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let rec = read_recording(BufReader::new(File::open(&file).unwrap())).unwrap();
    assert_eq!(rec.layout.geometry.len(), 9);
    assert_eq!(rec.snapshots.len(), 240);
}

#[test]
fn bad_geometry_fails_before_anything_is_written() {
    let s = scenario();
    let s = s.to_str().unwrap();
    let out = tmp("never.jsonl");
    let _ = std::fs::remove_file(&out);

    // a file that does not exist
    let o = cli(&[
        "--record",
        out.to_str().unwrap(),
        "--geometry",
        "missing.json",
        s,
    ]);
    assert_eq!(o.status.code(), Some(1));
    assert!(o.stdout.is_empty());
    assert!(String::from_utf8_lossy(&o.stderr).starts_with("error: cannot read missing.json"));

    // a shape on a map the scenario does not have
    let bad = tmp("bad_map.geometry.json");
    std::fs::write(
        &bad,
        r#"{"version":1,"shapes":[{"mapId":"Nope","kind":"wall","shape":"polyline",
            "vertices":[{"x":0,"y":0},{"x":1,"y":0}]}]}"#,
    )
    .unwrap();
    let o = cli(&[
        "--record",
        out.to_str().unwrap(),
        "--geometry",
        bad.to_str().unwrap(),
        s,
    ]);
    assert_eq!(o.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&o.stderr).contains("no map 'Nope'"));
    assert!(!out.exists(), "no recording file was created");
}

#[test]
fn bad_record_options_are_usage_errors() {
    let s = scenario();
    let s = s.to_str().unwrap();
    for args in [
        vec!["--record", s],                       // no file left for the scenario
        vec!["--record"],                          // missing value
        vec!["--record-interval", "1", s],         // needs --record
        vec!["--geometry", "g.json", s],           // needs --record
        vec!["--record", "x.jsonl", "--geometry"], // missing value
        vec!["--record", "x.jsonl", "--record-interval", "0", s],
        vec!["--record", "x.jsonl", "--record-interval", "abc", s],
    ] {
        let o = cli(&args);
        assert_eq!(o.status.code(), Some(2), "{args:?}");
        assert!(String::from_utf8_lossy(&o.stderr).contains("usage"));
    }
}

#[test]
fn an_unwritable_recording_file_is_an_error() {
    let s = scenario();
    let o = cli(&["--record", "/nonexistent-dir/x.jsonl", s.to_str().unwrap()]);
    assert_eq!(o.status.code(), Some(1));
    assert!(o.stdout.is_empty());
    assert!(String::from_utf8_lossy(&o.stderr).starts_with("error: cannot write"));
}

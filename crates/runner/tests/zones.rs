use std::path::{Path, PathBuf};

use botsim_runner::scenario::{Scenario, ScenarioError};
use botsim_runner::summary::{self, Summary};

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

/// jobs.toml (2 robots on a ring, random jobs) with a `zone_set` line added at the top level.
fn with_zone_set(file: &str) -> String {
    let base = std::fs::read_to_string(data().join("jobs.toml")).unwrap();
    base.replace(
        "lif = \"ring.lif.json\"",
        &format!("lif = \"ring.lif.json\"\nzone_set = \"{file}\""),
    )
}

fn run_text(text: &str) -> Summary {
    let s = Scenario::from_toml(text).unwrap();
    summary::run(&s, s.build_world(&data()).unwrap()).unwrap()
}

fn build_error(text: &str) -> ScenarioError {
    Scenario::from_toml(text)
        .unwrap()
        .build_world(&data())
        .unwrap_err()
}

#[test]
fn a_zone_set_is_loaded_and_listed_in_the_summary() {
    let r = run_text(&with_zone_set("zones.json"));
    let z = r.zone_set.expect("zone set in the summary");
    assert_eq!(
        (z.zone_set_id.as_str(), z.map_id.as_str()),
        ("ring-zones", "M")
    );
    let zones: Vec<_> = z
        .zones
        .iter()
        .map(|z| (z.zone_id.as_str(), z.zone_type.as_str()))
        .collect();
    assert_eq!(
        zones,
        vec![("crossing", "RELEASE"), ("slow-corner", "SPEED_LIMIT")]
    );
}

#[test]
fn without_a_zone_set_the_summary_has_none() {
    let base = std::fs::read_to_string(data().join("jobs.toml")).unwrap();
    assert!(run_text(&base).zone_set.is_none());
    // and the JSON has no such key
    let json = serde_json::to_value(run_text(&base)).unwrap();
    assert!(json.get("zone_set").is_none());
}

#[test]
fn a_release_zone_costs_the_baseline_only_the_time_of_the_answers() {
    // robots now ask for the RELEASE zone and the baseline grants it, one robot
    // at a time; on this ring the robots never need it at the same time, so the
    // run is the same except that every request/answer costs a control cycle
    let base = std::fs::read_to_string(data().join("jobs.toml")).unwrap();
    let plain = run_text(&base);
    let mut zoned = run_text(&with_zone_set("zones.json"));
    assert!(zoned.zone_set.take().is_some());
    assert_eq!(
        (
            plain.jobs.clone(),
            plain.collision_count,
            plain.robots.clone()
        ),
        (
            zoned.jobs.clone(),
            zoned.collision_count,
            zoned.robots.clone()
        )
    );
    let (p, z) = (
        plain.metrics.waiting_avg_seconds.unwrap(),
        zoned.metrics.waiting_avg_seconds.unwrap(),
    );
    assert!(z >= p && z - p < 0.5, "waiting {p} → {z}");
}

#[test]
fn a_whole_zone_set_message_is_accepted_too() {
    let dir = std::env::temp_dir().join(format!("botsim-zones-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let zone_set: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(data().join("zones.json")).unwrap()).unwrap();
    let message = serde_json::json!({
        "headerId": 1, "timestamp": "2026-11-20T10:00:00.000Z", "version": "3.0.1",
        "manufacturer": "ACME", "serialNumber": "R-001", "zoneSet": zone_set
    });
    std::fs::write(dir.join("message.json"), message.to_string()).unwrap();
    // the layout and factsheet files are needed next to the scenario
    for f in ["ring.lif.json", "diff_robot.factsheet.json"] {
        std::fs::copy(data().join(f), dir.join(f)).unwrap();
    }
    let text = with_zone_set("message.json");
    let s = Scenario::from_toml(&text).unwrap();
    let r = summary::run(&s, s.build_world(&dir).unwrap()).unwrap();
    assert_eq!(r.zone_set.unwrap().zones.len(), 2);
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn bad_zone_set_files_are_refused_with_the_file_named() {
    // missing file
    assert!(matches!(
        build_error(&with_zone_set("nope.json")),
        ScenarioError::Read { .. }
    ));
    // not a zone set at all (the layout file)
    let e = build_error(&with_zone_set("ring.lif.json"));
    assert!(matches!(&e, ScenarioError::Input { path, .. } if path.ends_with("ring.lif.json")));

    // a zone set for another map, and a broken polygon: checked against the layout
    let dir = std::env::temp_dir().join(format!("botsim-zones-bad-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for f in ["ring.lif.json", "diff_robot.factsheet.json"] {
        std::fs::copy(data().join(f), dir.join(f)).unwrap();
    }
    let good = std::fs::read_to_string(data().join("zones.json")).unwrap();
    std::fs::write(
        dir.join("other_map.json"),
        good.replace("\"mapId\": \"M\"", "\"mapId\": \"Z\""),
    )
    .unwrap();
    std::fs::write(
        dir.join("bow_tie.json"),
        r#"{"mapId":"M","zoneSetId":"x","zones":[{"zoneId":"a","zoneType":"BLOCKED",
            "vertices":[{"x":0,"y":0},{"x":2,"y":2},{"x":2,"y":0},{"x":0,"y":2}]}]}"#,
    )
    .unwrap();
    for (file, expect) in [
        ("other_map.json", "\"Z\""),
        ("bow_tie.json", "crosses itself"),
    ] {
        let s = Scenario::from_toml(&with_zone_set(file)).unwrap();
        let e = s.build_world(&dir).unwrap_err().to_string();
        assert!(e.contains(file) && e.contains(expect), "{e}");
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

//! A run translates nothing between the world and the controller, so there is
//! nothing in the summary about it, also for a zone set that a connection through
//! VDA 5050 would refuse (`botsim-bridge`, `tests/untranslated.rs`).

use std::path::{Path, PathBuf};

use botsim::scenario::Scenario;
use botsim::summary::{self, Summary};

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

/// jobs.toml (2 robots on a ring, random jobs) with the zone set of `file`, for
/// 5 minutes.
fn run(file: &str) -> Summary {
    let text = std::fs::read_to_string(data().join("jobs.toml"))
        .unwrap()
        .replace(
            "lif = \"ring.lif.json\"",
            &format!("lif = \"ring.lif.json\"\nzone_set = \"{file}\""),
        )
        .replace("duration = 2400.0", "duration = 300.0");
    let s = Scenario::from_toml(&text).unwrap();
    summary::run(&s, s.build_world(&data()).unwrap()).unwrap()
}

#[test]
fn a_zone_set_that_cannot_be_translated_is_in_the_summary() {
    // a zone with an empty id: the scenario takes it, and as nothing is
    // translated the robots get their tasks, as in botsim 0.1
    let r = run("zones_empty_id.json");
    assert!(r.jobs.done > 0, "{:?}", r.jobs);
    let json = serde_json::to_value(&r).unwrap();
    assert!(json.get("untranslated").is_none());
}

#[test]
fn a_run_that_translates_everything_says_nothing() {
    let r = run("zones.json");
    assert!(r.jobs.done > 0, "{:?}", r.jobs);
    let json = serde_json::to_value(&r).unwrap();
    assert!(json.get("untranslated").is_none());
}

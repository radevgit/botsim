//! What cannot be translated between the world and the controller shows in the
//! summary, and the key is not there when everything can.

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
    // a zone with an empty id: the scenario takes it, the translator does not, and
    // no robot gets a task
    let r = run("zones_empty_id.json");
    assert_eq!((r.jobs.done, r.jobs.in_progress), (0, 0), "{:?}", r.jobs);
    assert!(r.jobs.waiting_for_robot > 0);
    let untranslated = r.untranslated.expect("in the summary");
    assert_eq!(untranslated.total, 1);
    let [case] = &untranslated.cases[..] else {
        panic!("one case: {untranslated:?}");
    };
    assert_eq!(
        (case.what.as_str(), case.of.as_str(), case.refused),
        ("site zone set", "ring-zones", true)
    );
    assert_eq!((case.first_time, case.count), (0.0, 1));
    assert!(case.reason.contains("zoneId"), "{}", case.reason);
}

#[test]
fn a_run_that_translates_everything_says_nothing() {
    let r = run("zones.json");
    assert!(r.jobs.done > 0, "{:?}", r.jobs);
    assert_eq!(r.untranslated, None);
    let json = serde_json::to_value(&r).unwrap();
    assert!(json.get("untranslated").is_none());
}

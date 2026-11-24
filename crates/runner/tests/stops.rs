use std::path::{Path, PathBuf};

use botsim::scenario::Scenario;
use botsim::summary::{self, Summary};

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

fn run_text(text: &str) -> Summary {
    let s = Scenario::from_toml(text).unwrap();
    summary::run(&s, s.build_world(&data()).unwrap()).unwrap()
}

/// jobs.toml (2 robots on a ring, random jobs) with a `[disturbances]` section appended.
fn with(section: &str) -> String {
    let base = std::fs::read_to_string(data().join("jobs.toml")).unwrap();
    format!("{base}\n{section}\n")
}

#[test]
fn no_stops_without_the_setting_when_switched_off_or_at_zero() {
    assert_eq!(run_text(&with("")).random_stops, 0);
    let off = "[disturbances]\nenabled = false\nstops_per_hour = 120";
    assert_eq!(run_text(&with(off)).random_stops, 0);
    let zero = "[disturbances]\nstops_per_hour = 0\nspeed_variation = 0.05";
    assert_eq!(run_text(&with(zero)).random_stops, 0);
}

#[test]
fn stops_happen_are_repeatable_and_keep_the_robots_apart() {
    let on = "[disturbances]\nstops_per_hour = 120";
    let a = run_text(&with(on));
    let plain = run_text(&with(""));
    eprintln!(
        "{} stops, {} → {} jobs done, {} collisions",
        a.random_stops, plain.jobs.done, a.jobs.done, a.collision_count
    );
    assert!(a.random_stops >= 5, "{} stops", a.random_stops);
    assert_eq!(a, run_text(&with(on)), "the same scenario, the same result");

    // the jobs themselves are the same; the stops only slow the robots down
    assert_eq!(a.jobs.created, plain.jobs.created);
    assert!(a.jobs.done <= plain.jobs.done);
    // reservations keep robots from meeting even when one is stopped
    assert_eq!(a.collision_count, 0);
}

#[test]
fn stop_lengths_come_from_the_given_range_so_longer_stops_cost_more() {
    let short = "[disturbances]\nstops_per_hour = 120\nstop_seconds_min = 1\nstop_seconds_max = 2";
    let long = "[disturbances]\nstops_per_hour = 120\nstop_seconds_min = 20\nstop_seconds_max = 30";
    let (s, l) = (run_text(&with(short)), run_text(&with(long)));
    assert!(
        l.jobs.done < s.jobs.done,
        "{} vs {}",
        l.jobs.done,
        s.jobs.done
    );
}

#[test]
fn bad_stop_values_are_refused_at_load() {
    for (what, key) in [
        ("stops_per_hour = -1", "stops_per_hour"),
        (
            "stops_per_hour = 1\nstop_seconds_min = -1",
            "stop_seconds_min",
        ),
        (
            "stops_per_hour = 1\nstop_seconds_min = 9\nstop_seconds_max = 3",
            "stop_seconds_min",
        ),
        ("stops_per_hour = 100000000", "stops_per_hour"),
    ] {
        let e = Scenario::from_toml(&with(&format!("[disturbances]\n{what}")))
            .unwrap_err()
            .to_string();
        assert!(e.contains(key), "{what}: {e}");
    }
    // a typo is an error
    assert!(Scenario::from_toml(&with("[disturbances]\nstop_per_hour = 1")).is_err());
}

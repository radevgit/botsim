use std::path::{Path, PathBuf};

use botsim_runner::scenario::Scenario;
use botsim_runner::summary::{self, Summary};

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

fn run_text(text: &str) -> Summary {
    let s = Scenario::from_toml(text).unwrap();
    summary::run(&s, s.build_world(&data()).unwrap()).unwrap()
}

/// jobs.toml (2 robots on a ring, random jobs), with a `[disturbances]` section appended.
fn with(section: &str) -> String {
    let base = std::fs::read_to_string(data().join("jobs.toml")).unwrap();
    format!("{base}\n{section}\n")
}

fn speeds(r: &Summary) -> Vec<f64> {
    r.robots.iter().map(|r| r.max_speed).collect()
}

#[test]
fn without_a_section_or_when_switched_off_speeds_are_unchanged() {
    // the factsheet's top speed is 1.5 m/s
    assert_eq!(speeds(&run_text(&with(""))), vec![1.5, 1.5]);
    let off = "[disturbances]\nenabled = false\nspeed_variation = 0.05";
    assert_eq!(speeds(&run_text(&with(off))), vec![1.5, 1.5]);
}

#[test]
fn speed_variation_lowers_each_top_speed_by_at_most_the_given_share() {
    let on = "[disturbances]\nspeed_variation = 0.05"; // enabled by default
    let s = speeds(&run_text(&with(on)));
    assert_eq!(s.len(), 2);
    for v in &s {
        assert!(*v > 1.5 * 0.95 - 1e-6 && *v <= 1.5, "{v}");
    }
    assert_ne!(s[0], s[1], "robots differ");
    assert!(s.iter().any(|v| *v < 1.5));
    // the same scenario gives the same speeds
    assert_eq!(s, speeds(&run_text(&with(on))));
}

#[test]
fn disturbances_do_not_change_the_jobs_only_how_they_are_done() {
    let plain = run_text(&with(""));
    let on = run_text(&with("[disturbances]\nspeed_variation = 0.3"));
    // the jobs come from their own random stream: the same jobs are created
    assert_eq!(plain.jobs.created, on.jobs.created);
    // slower robots finish no more jobs than at full speed in the same time
    assert!(
        on.jobs.done <= plain.jobs.done,
        "{:?} vs {:?}",
        on.jobs,
        plain.jobs
    );
}

#[test]
fn another_seed_gives_other_speeds() {
    let on = "[disturbances]\nspeed_variation = 0.05";
    let a = speeds(&run_text(&with(on)));
    let b = speeds(&run_text(&with(on).replace("seed = 7", "seed = 8")));
    assert_ne!(a, b);
}

#[test]
fn bad_values_are_refused_at_load() {
    for v in ["-0.1", "1.0", "2.0"] {
        let e = Scenario::from_toml(&with(&format!("[disturbances]\nspeed_variation = {v}")))
            .unwrap_err()
            .to_string();
        assert!(e.contains("speed_variation"), "{v}: {e}");
    }
    // also when it is switched off: a wrong value is still a mistake
    let e = Scenario::from_toml(&with(
        "[disturbances]\nenabled = false\nspeed_variation = 3.0",
    ));
    assert!(e.is_err());
    // a typo is an error
    assert!(Scenario::from_toml(&with("[disturbances]\nspeed_variaton = 0.05")).is_err());
}

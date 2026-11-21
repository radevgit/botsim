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

/// jobs.toml (2 robots on a ring, random jobs; pick and drop 5 s) with a section appended.
fn with(section: &str) -> String {
    let base = std::fs::read_to_string(data().join("jobs.toml")).unwrap();
    format!("{base}\n{section}\n")
}

#[test]
fn action_variation_slows_the_work_but_creates_the_same_jobs_and_is_repeatable() {
    let plain = run_text(&with(""));
    let on = "[disturbances]\naction_variation = 1.0"; // up to 100 % longer
    let a = run_text(&with(on));
    eprintln!("jobs done {} → {}", plain.jobs.done, a.jobs.done);
    assert_eq!(a.jobs.created, plain.jobs.created);
    assert!(a.jobs.done < plain.jobs.done);
    assert_eq!(a, run_text(&with(on)), "same scenario, same result");
}

#[test]
fn it_can_be_switched_off_or_left_at_zero() {
    let plain = run_text(&with(""));
    let off = run_text(&with(
        "[disturbances]\nenabled = false\naction_variation = 1.0",
    ));
    let zero = run_text(&with("[disturbances]\naction_variation = 0"));
    assert_eq!(off, plain);
    assert_eq!(zero, plain);
}

#[test]
fn a_negative_value_and_a_typo_are_refused() {
    let e = Scenario::from_toml(&with("[disturbances]\naction_variation = -0.1"))
        .unwrap_err()
        .to_string();
    assert!(e.contains("action_variation"), "{e}");
    // wrong also when switched off
    assert!(
        Scenario::from_toml(&with(
            "[disturbances]\nenabled = false\naction_variation = -1"
        ))
        .is_err()
    );
    assert!(Scenario::from_toml(&with("[disturbances]\naction_varation = 0.2")).is_err());
}

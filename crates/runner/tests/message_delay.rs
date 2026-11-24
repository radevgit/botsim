use std::path::{Path, PathBuf};

use botsim::scenario::Scenario;
use botsim::summary::{self, Summary};

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

/// The ring scenario of the jobs test, with `extra` added before its first section.
fn text(extra: &str) -> String {
    let base = std::fs::read_to_string(data().join("jobs.toml")).unwrap();
    base.replacen("[[robot]]", &format!("{extra}\n\n[[robot]]"), 1)
}

fn run(extra: &str) -> Summary {
    let s = Scenario::from_toml(&text(extra)).unwrap();
    summary::run(&s, s.build_world(&data()).unwrap()).unwrap()
}

#[test]
fn the_delay_is_in_seconds_and_rounded_to_whole_steps() {
    let steps = |extra: &str| {
        Scenario::from_toml(&text(extra))
            .unwrap()
            .build_world(&data())
            .unwrap()
            .message_delay()
    };
    assert_eq!(steps(""), 0);
    assert_eq!(steps("message_delay = 0.0"), 0);
    assert_eq!(steps("message_delay = 0.5"), 10); // time_step is 0.05
    assert_eq!(steps("message_delay = 0.52"), 10);
    assert_eq!(steps("message_delay = 0.53"), 11);
}

#[test]
fn a_wrong_delay_is_refused() {
    for bad in [
        "message_delay = -1.0",
        "message_delay = nan",
        "message_delay = inf",
        "message_delay = \"slow\"",
    ] {
        assert!(Scenario::from_toml(&text(bad)).is_err(), "{bad}");
    }
}

#[test]
fn a_delayed_run_finishes_no_more_jobs_than_an_instant_one_and_is_repeatable() {
    let instant = run("");
    let slow = run("message_delay = 3.0");
    assert_eq!(slow.collision_count, 0);
    assert!(slow.jobs.done > 0);
    assert!(
        slow.jobs.done <= instant.jobs.done,
        "{} against {}",
        slow.jobs.done,
        instant.jobs.done
    );
    assert_eq!(slow, run("message_delay = 3.0"));
}

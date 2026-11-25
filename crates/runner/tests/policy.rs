//! `summary::run_with_policy`: a run with a policy given from outside. With the
//! baseline rules passed in, the summary and the recording are those of `run`.

use std::fs::File;
use std::path::{Path, PathBuf};

use botsim::scenario::Scenario;
use botsim::summary::{self, Summary};
use botsim_baseline::baseline_policy::BaselinePolicy;
use botsim_record::Recorder;

fn scenarios() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

/// Run the shipped scenario `name`, recorded every 0.5 s: with `outside`, through
/// `run_with_policy` and a baseline policy built here; else through `run`.
fn recorded(name: &str, outside: bool) -> (Summary, Vec<u8>) {
    let s = Scenario::load(&scenarios().join(name)).unwrap();
    let mut world = s.build_world(&scenarios()).unwrap();
    let file = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{name}-{outside}.jsonl"));
    let (recorder, handle) = Recorder::new(File::create(&file).unwrap());
    world.set_observer_at(Box::new(recorder), 0.5).unwrap();
    let result = if outside {
        let policy = BaselinePolicy::new(world.graph());
        summary::run_with_policy(&s, world, Box::new(policy))
    } else {
        summary::run(&s, world)
    }
    .unwrap();
    handle.finish().unwrap();
    (result, std::fs::read(file).unwrap())
}

#[test]
fn the_baseline_passed_from_outside_gives_the_same_summary_and_recording() {
    // RELEASE zones and disturbances
    let (a, recording_a) = recorded("ring_jobs.toml", false);
    let (b, recording_b) = recorded("ring_jobs.toml", true);
    assert!(a.jobs.done > 0);
    assert_eq!(a, b);
    assert!(recording_a == recording_b, "the recordings differ");
}

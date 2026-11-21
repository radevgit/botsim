use std::path::{Path, PathBuf};
use std::process::Command;

use botsim_runner::scenario::Scenario;
use botsim_runner::summary::{self, Summary};

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

fn run_file(name: &str) -> Summary {
    let s = Scenario::load(&data().join(name)).unwrap();
    summary::run(&s, s.build_world(&data()).unwrap()).unwrap()
}

fn run_text(text: &str) -> Summary {
    let s = Scenario::from_toml(text).unwrap();
    summary::run(&s, s.build_world(&data()).unwrap()).unwrap()
}

fn cli(file: &Path) -> Vec<u8> {
    let out = Command::new(env!("CARGO_BIN_EXE_botsim-runner"))
        .arg(file)
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    out.stdout
}

#[test]
fn without_a_jobs_section_nothing_happens() {
    let r = run_file("two_robots.toml");
    assert_eq!(
        (
            r.jobs.created,
            r.jobs.done,
            r.jobs.in_progress,
            r.jobs.rejected
        ),
        (0, 0, 0, 0)
    );
    assert_eq!(r.metrics.throughput_per_hour, 0.0);
    assert_eq!(r.metrics.waiting_avg_seconds, None);
}

#[test]
fn a_scenario_with_jobs_runs_the_baseline_and_reports_metrics() {
    let r = run_file("jobs.toml");
    let j = &r.jobs;
    eprintln!("{j:?} {:?} collisions {}", r.metrics, r.collision_count);
    assert!(j.created >= 10, "{j:?}");
    assert!(j.done >= 5, "{j:?}");
    // every job is in exactly one state
    assert_eq!(
        j.done + j.waiting_for_robot + j.in_progress + j.rejected,
        j.created
    );
    assert_eq!(j.rejected, 0);

    // throughput = done jobs per hour of the 40 minute run
    assert!((r.metrics.throughput_per_hour - j.done as f64 * 1.5).abs() < 1e-9);
    let (avg, max) = (
        r.metrics.waiting_avg_seconds.unwrap(),
        r.metrics.waiting_max_seconds.unwrap(),
    );
    assert!(0.0 <= avg && avg <= max);
}

#[test]
fn the_same_scenario_gives_identical_bytes_and_another_seed_other_jobs() {
    let file = data().join("jobs.toml");
    assert_eq!(cli(&file), cli(&file));

    let text = std::fs::read_to_string(&file).unwrap();
    let other = run_text(&text.replace("seed = 7", "seed = 8"));
    assert_ne!(other.jobs, run_file("jobs.toml").jobs);
}

#[test]
fn the_cli_json_carries_the_job_fields() {
    let json: serde_json::Value = serde_json::from_slice(&cli(&data().join("jobs.toml"))).unwrap();
    assert!(json["jobs"]["created"].as_u64().unwrap() >= 10);
    assert!(json["metrics"]["throughput_per_hour"].as_f64().unwrap() > 0.0);
    assert!(json["metrics"]["waiting_max_seconds"].is_number());
}

#[test]
fn bad_job_settings_are_refused_clearly() {
    let base = std::fs::read_to_string(data().join("jobs.toml")).unwrap();
    let bad = |from: &str, to: &str| {
        let text = base.replace(from, to);
        match Scenario::from_toml(&text) {
            Err(e) => e.to_string(),
            Ok(s) => match s.build_world(&data()) {
                Err(e) => e.to_string(),
                Ok(w) => summary::run(&s, w).unwrap_err().to_string(),
            },
        }
    };
    assert!(bad("rate_per_hour = 120.0", "rate_per_hour = -1.0").contains("rate_per_hour"));
    assert!(bad(r#"["N0", "N1", "N2", "N3"]"#, r#"["N0"]"#).contains("at least two"));
    assert!(bad(r#""N3"]"#, r#""Z9"]"#).contains("Z9"));
    // a typo in the section is an error, not silently ignored
    assert!(bad("rate_per_hour", "rate_per_hr").contains("rate_per_hr"));
    // too many jobs for the step: more than one per step on average
    assert!(bad("rate_per_hour = 120.0", "rate_per_hour = 1000000.0").contains("too high"));
}

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

fn cli(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_botsim-runner"))
        .args(args)
        .output()
        .unwrap()
}

fn json(out: &Output) -> serde_json::Value {
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    serde_json::from_slice(&out.stdout).unwrap()
}

#[test]
fn without_the_flag_there_is_no_timing_and_the_output_is_identical() {
    let file = data().join("jobs.toml");
    let file = file.to_str().unwrap();
    let a = cli(&[file]);
    assert!(json(&a).get("timing").is_none());
    assert_eq!(a.stdout, cli(&[file]).stdout);
}

#[test]
fn the_flag_adds_the_controller_time_and_changes_nothing_else() {
    let file = data().join("jobs.toml");
    let file = file.to_str().unwrap();
    let plain = json(&cli(&[file]));
    let mut timed = json(&cli(&["--timing", file]));
    // the flag can stand after the file name too
    assert!(json(&cli(&[file, "--timing"])).get("timing").is_some());

    let t = timed.as_object_mut().unwrap().remove("timing").unwrap();
    assert_eq!(timed, plain, "apart from `timing` the result is the same");

    // jobs.toml runs 2400 s at 50 ms: 48000 cycles and one last look
    assert_eq!(t["controller_steps"], 48_001);
    let (mean, max, total) = (
        t["controller_mean_us"].as_f64().unwrap(),
        t["controller_max_us"].as_f64().unwrap(),
        t["controller_total_ms"].as_f64().unwrap(),
    );
    assert!(mean > 0.0 && max >= mean && total > 0.0, "{t}");
    // mean × steps is the total (µs → ms), up to rounding
    assert!((mean * 48_001.0 / 1000.0 - total).abs() < 0.05 * total + 1.0);
}

#[test]
fn without_a_controller_the_timing_is_empty() {
    let file = data().join("two_robots.toml"); // no [jobs]
    let j = json(&cli(&["--timing", file.to_str().unwrap()]));
    assert_eq!(j["timing"]["controller_steps"], 0);
    assert!(j["timing"]["controller_mean_us"].is_null());
    assert!(j["timing"]["controller_max_us"].is_null());
}

#[test]
fn unknown_options_and_missing_files_are_usage_errors() {
    let o = cli(&["--fast", "x.toml"]);
    assert_eq!(o.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&o.stderr).contains("--fast"));
    assert_eq!(
        cli(&["--timing"]).status.code(),
        Some(2),
        "no scenario given"
    );
    let o = cli(&["--timing", "missing.toml"]);
    assert_eq!(o.status.code(), Some(1));
}

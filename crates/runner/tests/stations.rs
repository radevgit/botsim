//! Jobs between LIF stations in the scenario (S60), on the layout of the mixed
//! fleet example: stations Goods-In (P0), Rack-A (P1 and P4), Rack-B (P2),
//! Goods-Out (P3); P5 is on no station.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use botsim::scenario::Scenario;

fn dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

/// The mixed fleet scenario with the given text in place of its `[jobs]` section.
fn text(jobs: &str) -> String {
    let base = std::fs::read_to_string(dir().join("mixed_fleet.toml")).unwrap();
    format!("{}[jobs]\n{jobs}\n", base.split("[jobs]").next().unwrap())
}

/// The nodes (by id) jobs start and end at, over a long run of the generator.
fn used_nodes(jobs: &str) -> BTreeSet<String> {
    let s = Scenario::from_toml(&text(jobs)).unwrap();
    let w = s.build_world(&dir()).unwrap();
    let mut g = s.job_generator(w.graph()).unwrap().unwrap();
    let mut used = BTreeSet::new();
    for t in 0..400_000 {
        if let Some(j) = g.step(t) {
            used.insert(w.graph().nodes[j.pickup].id.clone());
            used.insert(w.graph().nodes[j.drop].id.clone());
        }
    }
    used
}

fn set(ids: &[&str]) -> BTreeSet<String> {
    ids.iter().map(|s| s.to_string()).collect()
}

#[test]
fn without_nodes_or_stations_jobs_go_between_all_stations_of_the_layout() {
    // P5 is on no station; Rack-A has two interaction nodes and both are used
    assert_eq!(
        used_nodes("rate_per_hour = 600.0"),
        set(&["P0", "P1", "P2", "P3", "P4"])
    );
}

#[test]
fn listed_stations_limit_where_jobs_go() {
    assert_eq!(
        used_nodes("rate_per_hour = 600.0\nstations = [\"Goods-In\", \"Rack-B\"]"),
        set(&["P0", "P2"])
    );
    assert_eq!(
        used_nodes("rate_per_hour = 600.0\nstations = [\"Goods-In\", \"Rack-A\"]"),
        set(&["P0", "P1", "P4"])
    );
}

#[test]
fn nodes_still_work_as_before() {
    assert_eq!(
        used_nodes("rate_per_hour = 600.0\nnodes = [\"P5\", \"P1\"]"),
        set(&["P5", "P1"])
    );
}

#[test]
fn wrong_job_places_are_refused() {
    let bad = |jobs: &str| -> String {
        match Scenario::from_toml(&text(jobs)) {
            Err(e) => e.to_string(),
            Ok(s) => s
                .build_world(&dir())
                .and_then(|w| s.job_generator(w.graph()).map(|_| ()))
                .unwrap_err()
                .to_string(),
        }
    };
    let e = bad("rate_per_hour = 60.0\nnodes = [\"P0\", \"P1\"]\nstations = [\"Rack-A\"]");
    assert!(e.contains("not both"), "{e}");
    let e = bad("rate_per_hour = 60.0\nstations = [\"Nowhere\", \"Rack-A\"]");
    assert!(e.contains("Nowhere"), "{e}");
    let e = bad("rate_per_hour = 60.0\nstations = [\"Rack-A\"]");
    assert!(e.contains("two stations"), "{e}");
}

#[test]
fn a_layout_without_stations_needs_nodes_for_its_jobs() {
    // the ring of the jobs test has no stations
    let data = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
    let base = std::fs::read_to_string(data.join("jobs.toml")).unwrap();
    let text = format!(
        "{}[jobs]\nrate_per_hour = 60.0\n",
        base.split("[jobs]").next().unwrap()
    );
    let s = Scenario::from_toml(&text).unwrap();
    let w = s.build_world(&data).unwrap();
    let e = s.job_generator(w.graph()).unwrap_err().to_string();
    assert!(e.contains("two stations"), "{e}");
}

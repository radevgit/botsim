use std::path::{Path, PathBuf};

use botsim_core::lift::DoorState;
use botsim_runner::scenario::Scenario;
use botsim_runner::summary;

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

/// The two-level layout of official LIF example 14: N1 –(N1-N2)– N2 on floor 1.
fn scenario(doors: &str) -> String {
    format!(
        r#"
name = "doors"
seed = 1
duration = 30.0
time_step = 0.05
start_time = 2026-11-21T10:00:00Z
lif = "two_levels.lif.json"

[[robot]]
manufacturer = "ACME"
serial_prefix = "R"
factsheet = "diff_robot.factsheet.json"
start_nodes = ["N1"]
action_time = 1.0

{doors}
"#
    )
}

const DOOR: &str = r#"
[[door]]
id = "D1"
edges = ["N1-N2"]
passage_width = 1.0
open_seconds = 3.0
close_seconds = 3.0
"#;

fn build(text: &str) -> Result<botsim_core::world::World, botsim_runner::scenario::ScenarioError> {
    Scenario::from_toml(text)?.build_world(&data())
}

#[test]
fn a_door_in_the_scenario_is_built_and_can_be_commanded() {
    let mut w = build(&scenario(DOOR)).unwrap();
    let st = w.door_status("D1").unwrap();
    assert_eq!((st.state, st.wanted_open), (DoorState::Closed, false));
    assert_eq!(st.edges, vec![w.graph().edge_idx("N1-N2").unwrap()]);
    w.open_door("D1").unwrap();
    for _ in 0..80 {
        w.step();
    }
    assert_eq!(w.door_status("D1").unwrap().state, DoorState::Open);
    w.release_door("D1").unwrap();
    for _ in 0..80 {
        w.step();
    }
    assert_eq!(w.door_status("D1").unwrap().state, DoorState::Closed);
}

#[test]
fn wrong_doors_are_refused_with_the_door_named() {
    let bad = |from: &str, to: &str| {
        build(&scenario(&DOOR.replace(from, to)))
            .unwrap_err()
            .to_string()
    };
    let e = bad("\"N1-N2\"", "\"Nowhere\"");
    assert!(e.contains("D1") && e.contains("Nowhere"), "{e}");
    assert!(bad("edges = [\"N1-N2\"]", "edges = []").contains("D1"));
    assert!(bad("1.0", "-1.0").contains("passage_width"));
    assert!(bad("3.0\nclose", "-3.0\nclose").contains("open_seconds"));
    // two doors with the same id, and two doors on one edge
    let e = build(&scenario(&format!("{DOOR}{DOOR}")))
        .unwrap_err()
        .to_string();
    assert!(e.contains("exists already"), "{e}");
    let second = DOOR.replace("D1", "D2");
    let e = build(&scenario(&format!("{DOOR}{second}")))
        .unwrap_err()
        .to_string();
    assert!(e.contains("D2") && e.contains("N1-N2"), "{e}");
    // a typo in a key
    assert!(Scenario::from_toml(&scenario(&DOOR.replace("open_seconds", "open_secs"))).is_err());
}

#[test]
fn the_summary_lists_the_doors_and_a_run_without_doors_has_none() {
    let s = Scenario::from_toml(&scenario(DOOR)).unwrap();
    let r = summary::run(&s, s.build_world(&data()).unwrap()).unwrap();
    assert_eq!(r.doors.len(), 1);
    assert_eq!(
        (r.doors[0].id.as_str(), r.doors[0].state.as_str()),
        ("D1", "closed")
    );

    let s = Scenario::from_toml(&scenario("")).unwrap();
    let r = summary::run(&s, s.build_world(&data()).unwrap()).unwrap();
    assert!(r.doors.is_empty());
    let json = serde_json::to_value(&r).unwrap();
    assert!(json.get("doors").is_none(), "no `doors` key without doors");
}

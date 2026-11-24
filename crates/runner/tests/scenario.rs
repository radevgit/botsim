use std::path::{Path, PathBuf};

use botsim_core::robot::Reception::Accepted;
use botsim_core::world::RobotId;
use botsim_runner::scenario::{Scenario, ScenarioError};
use vda5050::order::Order;

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

const GOOD: &str = r#"
name = "t"
seed = 7
duration = 10.0
time_step = 0.05
start_time = 2026-11-20T10:00:00Z
lif = "three_nodes.lif.json"

[[robot]]
manufacturer = "ACME"
serial_prefix = "R"
factsheet = "diff_robot.factsheet.json"
start_nodes = ["A", "B"]
action_time = 2.0
"#;

/// The good scenario with one line replaced.
fn with(from: &str, to: &str) -> String {
    assert!(GOOD.contains(from), "test bug: {from}");
    GOOD.replace(from, to)
}

fn invalid(text: &str) -> String {
    match Scenario::from_toml(text) {
        Err(e) => e.to_string(),
        Ok(_) => panic!("accepted:\n{text}"),
    }
}

#[test]
fn the_example_file_loads_and_builds_a_world() {
    let s = Scenario::load(&data().join("two_robots.toml")).unwrap();
    assert_eq!(
        (s.name.as_str(), s.seed, s.duration),
        ("two-robots", 1, 60.0)
    );
    assert_eq!(s.start_unix_ms().unwrap(), 1_795_168_800_000);

    let mut w = s.build_world(&data()).unwrap();
    assert_eq!(w.dt(), 0.05);
    assert_eq!(w.timestamp(), "2026-11-20T10:00:00.000Z");
    // two robots, serials R-001 and R-002, on A and on B
    let ids: Vec<_> = w.robot_ids().cloned().collect();
    assert_eq!(
        ids,
        vec![RobotId::new("ACME", "R-001"), RobotId::new("ACME", "R-002")]
    );
    assert_eq!(w.pose(&ids[0]).unwrap().x, 0.0);
    assert_eq!(w.pose(&ids[1]).unwrap().x, 10.0);

    // and it runs: R-002 drives B→C
    let order: Order = serde_json::from_str(
        r#"{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-002","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"B","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"C","sequenceId":2,"released":true,"actions":[]}],
        "edges":[{"edgeId":"B-C","sequenceId":1,"released":true,"actions":[]}]}"#,
    )
    .unwrap();
    assert_eq!(w.send_order(&order), Ok(Accepted));
    for _ in 0..2000 {
        w.step();
    }
    assert_eq!(w.state(&ids[1]).unwrap().last_node_id, "C");
}

#[test]
fn per_type_action_times_are_parsed_and_accepted() {
    // only parsing is checked here; that ActionTimes works is tested in core
    let s = Scenario::from_toml(&GOOD.replace(
        "action_time = 2.0",
        "action_time = 2.0\n[robot.action_times]\npick = 3.0",
    ))
    .unwrap();
    assert_eq!(s.robots[0].action_times["pick"], 3.0);
    assert!(s.build_world(&data()).is_ok());
}

#[test]
fn bad_values_are_refused_with_a_clear_message() {
    assert!(invalid(&with("duration = 10.0", "duration = 0.0")).contains("duration"));
    assert!(invalid(&with("time_step = 0.05", "time_step = -1.0")).contains("time_step"));
    assert!(invalid(&with("action_time = 2.0", "action_time = -2.0")).contains("action_time"));
    assert!(invalid(&with(r#"["A", "B"]"#, "[]")).contains("start_nodes"));
    // a missing key and a misspelled key
    assert!(invalid(&GOOD.replace("seed = 7\n", "")).contains("seed"));
    assert!(invalid(&with("seed = 7", "sead = 7")).contains("sead"));
    // no robots at all
    let no_robots = GOOD.split("[[robot]]").next().unwrap();
    assert!(invalid(no_robots).contains("robot"));
}

#[test]
fn the_start_time_must_be_a_full_utc_time() {
    let ok = Scenario::from_toml(&with("2026-11-20T10:00:00Z", "2026-11-20T10:00:00Z"));
    assert!(ok.is_ok());
    for bad in [
        "2026-11-20",
        "10:00:00",
        "2026-11-20T10:00:00",
        "2026-11-20T10:00:00+02:00",
    ] {
        let m = invalid(&with("2026-11-20T10:00:00Z", bad));
        assert!(m.contains("start_time"), "{bad}: {m}");
    }
}

#[test]
fn start_heading_road_faces_along_the_road_that_leaves_the_node() {
    // ring: N0 → N1 east, N1 → N2 north, N2 → N3 west, N3 → N0 south
    let text = GOOD
        .replace("three_nodes.lif.json", "ring.lif.json")
        .replace(r#"["A", "B"]"#, r#"["N0", "N1", "N3"]"#)
        .replace("action_time", "start_heading = \"road\"\naction_time");
    let s = Scenario::from_toml(&text).unwrap();
    let w = s.build_world(&data()).unwrap();
    let h: Vec<f64> = w.robot_ids().map(|id| w.pose(id).unwrap().heading).collect();
    let q = std::f64::consts::FRAC_PI_2;
    for (got, want) in h.iter().zip([0.0, q, -q]) {
        assert!((got - want).abs() < 1e-9, "{h:?}");
    }
}

#[test]
fn start_headings_give_one_heading_for_each_start_node() {
    let text = GOOD
        .replace("three_nodes.lif.json", "ring.lif.json")
        .replace(r#"["A", "B"]"#, r#"["N0", "N1"]"#)
        .replace("action_time", "start_headings = [3.0, -1.0]\naction_time");
    let w = Scenario::from_toml(&text).unwrap().build_world(&data()).unwrap();
    let h: Vec<f64> = w.robot_ids().map(|id| w.pose(id).unwrap().heading).collect();
    assert_eq!(h, [3.0, -1.0]);
    // the wrong number of entries, and a word other than "road", are refused
    let few = GOOD.replace("action_time", "start_headings = [3.0]\naction_time");
    assert!(invalid(&few).contains("start_headings"));
    let word = GOOD.replace("action_time", "start_heading = \"north\"\naction_time");
    assert!(Scenario::from_toml(&word).is_err());
}

#[test]
fn building_fails_clearly_when_inputs_are_wrong() {
    let dir = data();
    // start node not in the layout
    let s = Scenario::from_toml(&with(r#"["A", "B"]"#, r#"["A", "Z"]"#)).unwrap();
    let e = s.build_world(&dir).unwrap_err().to_string();
    assert!(e.contains("\"Z\"") && e.contains("group 1"), "{e}");

    // missing layout file
    let s = Scenario::from_toml(&with("three_nodes.lif.json", "nope.lif.json")).unwrap();
    assert!(matches!(
        s.build_world(&dir),
        Err(ScenarioError::Read { .. })
    ));

    // a layout that is not a layout: the factsheet file given as lif
    let s =
        Scenario::from_toml(&with("three_nodes.lif.json", "diff_robot.factsheet.json")).unwrap();
    assert!(matches!(
        s.build_world(&dir),
        Err(ScenarioError::Input { .. })
    ));

    // two groups with the same manufacturer and prefix → same serial twice
    let twice = format!("{GOOD}{}", &GOOD[GOOD.find("[[robot]]").unwrap()..]);
    let s = Scenario::from_toml(&twice).unwrap();
    let e = s.build_world(&dir).unwrap_err().to_string();
    assert!(e.contains("exists already"), "{e}");

    // a missing scenario file
    assert!(matches!(
        Scenario::load(&dir.join("missing.toml")),
        Err(ScenarioError::Read { .. })
    ));
}

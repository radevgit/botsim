use std::path::{Path, PathBuf};

use botsim_core::lift::DoorState;
use botsim_runner::scenario::{Scenario, ScenarioError};
use botsim_runner::summary;

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

/// The two-level layout of official LIF example 14: maps "Map_Z-Level_1" and
/// "Map_Z-Level_2".
fn scenario(lift: &str) -> String {
    format!(
        r#"
name = "lifts"
seed = 1
duration = 60.0
time_step = 0.05
start_time = 2026-11-20T10:00:00Z
lif = "two_levels.lif.json"

[[robot]]
manufacturer = "ACME"
serial_prefix = "R"
factsheet = "diff_robot.factsheet.json"
start_nodes = ["N1"]
action_time = 1.0

{lift}
"#
    )
}

const LIFT: &str = r#"
[[lift]]
id = "L1"
floors = ["Map_Z-Level_1", "Map_Z-Level_2"]
travel_seconds_per_floor = 5.0
door_open_seconds = 3.0
door_close_seconds = 3.0
"#;

fn build(text: &str) -> Result<botsim_core::world::World, ScenarioError> {
    Scenario::from_toml(text)?.build_world(&data())
}

#[test]
fn a_lift_in_the_scenario_is_built_and_can_be_called() {
    let mut w = build(&scenario(LIFT)).unwrap();
    let st = w.lift_status("L1").unwrap();
    assert_eq!((st.floor, st.door), (0, DoorState::Closed));
    assert_eq!(w.lift_floor_map("L1", 1).unwrap(), "Map_Z-Level_2");

    w.call_lift("L1", "Map_Z-Level_2").unwrap();
    for _ in 0..(5.0_f64 / 0.05).round() as u32 + 4 {
        w.step();
    }
    // 5 s up, then the door opens (3 s)
    assert_eq!(w.lift_status("L1").unwrap().door, DoorState::Opening);
    for _ in 0..80 {
        w.step();
    }
    let st = w.lift_status("L1").unwrap();
    assert_eq!((st.floor, st.door), (1, DoorState::Open));
    w.release_lift("L1").unwrap();
    assert_eq!(w.lifts()[0].door, DoorState::Closing);
}

#[test]
fn doors_default_to_one_front_door_and_can_be_named() {
    let w = build(&scenario(LIFT)).unwrap();
    assert_eq!(w.lift_status("L1").unwrap().door_id, "front");
    let mut w = build(&scenario(
        &LIFT.replace("floors =", "doors = [\"north\", \"south\"]\nfloors ="),
    ))
    .unwrap();
    assert_eq!(w.lift_status("L1").unwrap().door_id, "north");

    w.call_lift_door("L1", "Map_Z-Level_2", "south").unwrap();
    for _ in 0..(10.0_f64 / 0.05).round() as u32 + 40 {
        w.step();
    }
    let st = w.lift_status("L1").unwrap();
    assert_eq!(
        (st.floor, st.door, st.door_id.as_str()),
        (1, DoorState::Open, "south")
    );
    // a door the lift does not have; and `call_lift` still means the first door
    assert!(w.call_lift_door("L1", "Map_Z-Level_2", "east").is_err());
    assert!(w.call_lift("L1", "Map_Z-Level_1").is_ok());
}

/// `LIFT` with a stop on each floor: N2 on floor 1, N102 on floor 2.
fn lift_with_stops(stops: &str) -> String {
    LIFT.replace("floors =", &format!("stops = {stops}\nfloors ="))
}

#[test]
fn stops_make_a_ride_possible_and_a_robot_rides_to_the_other_map() {
    use botsim_core::drive::Phase;
    use botsim_core::world::RobotId;

    let text = scenario(&lift_with_stops(r#"[{ node = "N2" }, { node = "N102" }]"#));
    let mut w = build(&text).unwrap();
    let r = RobotId::new("ACME", "R-001");
    let order: vda5050::order::Order = serde_json::from_str(
        r#"{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"N1","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"N2","sequenceId":2,"released":true,"actions":[]},
                 {"nodeId":"N102","sequenceId":4,"released":true,"actions":[]}],
        "edges":[{"edgeId":"N1-N2","sequenceId":1,"released":true,"actions":[]},
                 {"edgeId":"N2-N102","sequenceId":3,"released":true,"actions":[]}]}"#,
    )
    .unwrap();
    assert_eq!(
        w.send_order(&order),
        Ok(botsim_core::robot::Reception::Accepted)
    );
    w.call_lift("L1", "Map_Z-Level_1").unwrap();
    // the robot drives 11 m, gets in, and waits for the cabin to be sent
    for _ in 0..600 {
        w.step();
    }
    assert_eq!(w.phase(&r), Ok(Phase::Riding));
    assert_eq!(w.lift_rider("L1").unwrap(), Some(&r));
    w.release_lift("L1").unwrap();
    w.call_lift("L1", "Map_Z-Level_2").unwrap();
    for _ in 0..600 {
        w.step();
    }
    assert_eq!(w.phase(&r), Ok(Phase::Finished));
    let p = w.pose(&r).unwrap();
    assert_eq!(w.graph().maps[p.map], "Map_Z-Level_2");
}

/// The shipped example `scenarios/lift_jobs.toml`: two floors, one lift, two
/// robots, random jobs between floors (S48). It must stay working.
#[test]
fn the_lift_example_scenario_runs_jobs_through_the_lift() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/lift_jobs.toml");
    let s = Scenario::from_toml(&std::fs::read_to_string(&path).unwrap()).unwrap();
    let world = s.build_world(path.parent().unwrap()).unwrap();
    let r = summary::run(&s, world).unwrap();
    assert_eq!(r.collision_count, 0);
    assert_eq!(r.deadlock_count, 0);
    assert_eq!(r.jobs.rejected, 0);
    assert!(r.jobs.done >= 30, "{:?}", r.jobs);
    assert_eq!(r.lifts.len(), 1);
}

#[test]
fn wrong_stops_are_refused_with_the_lift_and_the_stop_named() {
    let with = |stops: &str| build(&scenario(&lift_with_stops(stops))).map(|_| ());
    let e = with(r#"[{ node = "Nowhere" }]"#).unwrap_err().to_string();
    assert!(e.contains("L1") && e.contains("Nowhere"), "{e}");
    // N1 is on floor 1, a floor of the lift; a lift that only has floor 2 refuses it
    let only_top = scenario(
        &lift_with_stops(r#"[{ node = "N1" }]"#)
            .replace("floors = [\"Map_Z-Level_1\", ", "floors = [\"Nowhere\", "),
    );
    assert!(build(&only_top).is_err());
    let e = with(r#"[{ node = "N2", door = "rear" }]"#)
        .unwrap_err()
        .to_string();
    assert!(e.contains("L1") && e.contains("rear"), "{e}");
    // a waiting node must exist, be on the stop's map and not be the stop itself
    let e = with(r#"[{ node = "N2", waiting = "Nowhere" }]"#)
        .unwrap_err()
        .to_string();
    assert!(e.contains("L1") && e.contains("Nowhere"), "{e}");
    assert!(with(r#"[{ node = "N2", waiting = "N102" }]"#).is_err());
    assert!(with(r#"[{ node = "N2", waiting = "N2" }]"#).is_err());
    assert!(with(r#"[{ node = "N2", waiting = "N1" }]"#).is_ok());
    let e = with(r#"[{ node = "N2" }, { node = "N2" }]"#)
        .unwrap_err()
        .to_string();
    assert!(e.contains("L1"), "{e}");
    // a typo in a key
    assert!(Scenario::from_toml(&scenario(&lift_with_stops(r#"[{ nodes = "N2" }]"#))).is_err());
}

#[test]
fn wrong_doors_are_refused_with_the_lift_named() {
    let with = |doors: &str| {
        build(&scenario(
            &LIFT.replace("floors =", &format!("doors = {doors}\nfloors =")),
        ))
        .map(|_| ())
    };
    for doors in ["[]", "[\"a\", \"a\"]"] {
        let e = with(doors).unwrap_err().to_string();
        assert!(e.contains("L1") && e.contains("door"), "{e}");
    }
}

#[test]
fn the_start_floor_is_a_map_and_defaults_to_the_first() {
    let up = LIFT.replace(
        "travel_seconds_per_floor",
        "start_floor = \"Map_Z-Level_2\"\ntravel_seconds_per_floor",
    );
    let w = build(&scenario(&up)).unwrap();
    assert_eq!(w.lift_status("L1").unwrap().floor, 1);
}

#[test]
fn wrong_lifts_are_refused_with_the_lift_named() {
    let bad = |from: &str, to: &str| {
        build(&scenario(&LIFT.replace(from, to)))
            .unwrap_err()
            .to_string()
    };
    assert!(bad("\"Map_Z-Level_2\"", "\"Nowhere\"").contains("Nowhere"));
    assert!(bad("\"Map_Z-Level_2\"", "\"Map_Z-Level_1\"").contains("L1")); // twice the same floor
    assert!(bad("5.0", "-1.0").contains("travel_seconds_per_floor"));
    let one_floor = LIFT.replace(", \"Map_Z-Level_2\"", "");
    let e = build(&scenario(&one_floor)).unwrap_err().to_string();
    assert!(e.contains("two"), "{e}");
    // start floor must be one of its floors
    let e = build(&scenario(&LIFT.replace(
        "travel_seconds_per_floor",
        "start_floor = \"Nowhere\"\ntravel_seconds_per_floor",
    )))
    .unwrap_err()
    .to_string();
    assert!(e.contains("Nowhere"), "{e}");
    // two lifts with the same id
    let e = build(&scenario(&format!("{LIFT}{LIFT}")))
        .unwrap_err()
        .to_string();
    assert!(e.contains("exists already"), "{e}");
    // a typo in a key
    assert!(
        Scenario::from_toml(&scenario(
            &LIFT.replace("door_open_seconds", "door_open_secs")
        ))
        .is_err()
    );
}

#[test]
fn calls_with_unknown_lifts_or_maps_are_errors() {
    let mut w = build(&scenario(LIFT)).unwrap();
    assert!(w.call_lift("L9", "Map_Z-Level_2").is_err());
    assert!(w.call_lift("L1", "Nowhere").is_err());
    assert!(w.release_lift("L9").is_err());
    assert!(w.lift_status("L9").is_err());
}

#[test]
fn the_summary_lists_the_lifts_and_a_run_without_lifts_has_none() {
    let s = Scenario::from_toml(&scenario(LIFT)).unwrap();
    let r = summary::run(&s, s.build_world(&data()).unwrap()).unwrap();
    assert_eq!(r.lifts.len(), 1);
    let l = &r.lifts[0];
    assert_eq!(
        (
            l.id.as_str(),
            l.floor_map_id.as_str(),
            l.door.as_str(),
            l.queued_calls
        ),
        ("L1", "Map_Z-Level_1", "closed", 0)
    );

    let s = Scenario::from_toml(&scenario("")).unwrap();
    let r = summary::run(&s, s.build_world(&data()).unwrap()).unwrap();
    assert!(r.lifts.is_empty());
    let json = serde_json::to_value(&r).unwrap();
    assert!(json.get("lifts").is_none(), "no `lifts` key without lifts");
}

#[test]
fn lifts_appear_in_the_observer_snapshots() {
    use std::cell::RefCell;
    use std::rc::Rc;

    use botsim_core::graph::Graph;
    use botsim_core::observer::{Observer, Snapshot};

    #[derive(Debug)]
    struct Recorder(Rc<RefCell<Vec<Snapshot>>>);
    impl Observer for Recorder {
        fn on_layout(&mut self, _: &Graph) {}
        fn on_step(&mut self, s: &Snapshot) {
            self.0.borrow_mut().push(s.clone());
        }
    }

    let mut w = build(&scenario(LIFT)).unwrap();
    let log = Rc::new(RefCell::new(Vec::new()));
    w.set_observer(Box::new(Recorder(log.clone())));
    w.call_lift("L1", "Map_Z-Level_2").unwrap();
    for _ in 0..400 {
        w.step();
    }
    let l = log.borrow();
    assert!(l.iter().all(|s| s.lifts.len() == 1));
    assert!(l.iter().any(|s| s.lifts[0].moving_to == Some(1)));
    assert_eq!(l.last().unwrap().lifts[0].door, DoorState::Open);
}

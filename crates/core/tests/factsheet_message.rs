//! The robot's `factsheet` message (S61).

use botsim_core::actions::ActionTimes;
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::protocol::PROTOCOL_VERSION;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_model::time::unix_ms_from_utc;
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

const DT: f64 = 0.05;

fn world() -> World {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    let g = Graph::from_lif(&lif).unwrap();
    let (a, b) = (g.node_idx("A").unwrap(), g.node_idx("B").unwrap());
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let start = unix_ms_from_utc(2026, 11, 21, 10, 0, 0);
    let mut w = World::new(g, DT, start).unwrap();
    w.add_robot(
        RobotId::new("ACME", "R-007"),
        spec.clone(),
        ActionTimes::new(1.0),
        a,
        0.0,
    )
    .unwrap();
    w.add_robot(
        RobotId::new("ACME", "R-008"),
        spec,
        ActionTimes::new(1.0),
        b,
        0.0,
    )
    .unwrap();
    w
}

#[test]
fn the_robot_sends_its_factsheet_as_its_own() {
    let w = world();
    let f = w.factsheet(&RobotId::new("ACME", "R-007")).unwrap();
    // the header is the robot's, not that of the file the factsheet came from
    assert_eq!(
        (f.manufacturer.as_str(), f.serial_number.as_str()),
        ("ACME", "R-007")
    );
    assert_eq!(f.version, PROTOCOL_VERSION);
    assert_eq!(f.header_id, 0);
    assert_eq!(f.timestamp, "2026-11-21T10:00:00.000Z");
    // the content is the file's: type, speeds, size
    assert_eq!(f.type_specification.series_name, "X1");
    assert_eq!(f.type_specification.mobile_robot_kinematics, "DIFFERENTIAL");
    assert_eq!(f.physical_parameters.maximum_speed, 1.5);
    assert_eq!(
        (f.physical_parameters.width, f.physical_parameters.length),
        (0.8, 1.2)
    );
    // each robot gets its own serial number
    let g = w.factsheet(&RobotId::new("ACME", "R-008")).unwrap();
    assert_eq!(g.serial_number, "R-008");
    assert_eq!(g.type_specification, f.type_specification);
}

#[test]
fn the_factsheet_message_can_be_turned_into_the_same_spec() {
    // what the controller (or any reader) gets describes the robot
    let w = world();
    let f = w.factsheet(&RobotId::new("ACME", "R-007")).unwrap();
    let again = RobotSpec::from_factsheet(&f).unwrap();
    let original = w.spec(&RobotId::new("ACME", "R-007")).unwrap();
    assert_eq!(again.series_name, original.series_name);
    assert_eq!(again.limits, original.limits);
    assert_eq!(again.shape, original.shape);
    // and it is JSON that reads back
    let json = serde_json::to_string(&f).unwrap();
    let back: Factsheet = serde_json::from_str(&json).unwrap();
    assert_eq!(back, f);
}

#[test]
fn an_unknown_robot_has_no_factsheet() {
    let w = world();
    assert!(w.factsheet(&RobotId::new("ACME", "R-999")).is_err());
}

#[test]
fn the_time_in_the_message_is_the_time_of_the_world() {
    let mut w = world();
    let id = RobotId::new("ACME", "R-007");
    let t0 = w.factsheet(&id).unwrap().timestamp;
    for _ in 0..40 {
        w.step();
    }
    let t1 = w.factsheet(&id).unwrap().timestamp;
    assert_ne!(t0, t1);
    assert_eq!(t1, w.timestamp());
}

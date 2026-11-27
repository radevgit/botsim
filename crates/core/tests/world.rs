use botsim_core::actions::ActionTimes;
use botsim_core::robot::Reception::Accepted;
use botsim_core::world::{World, WorldError};
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::instant_actions::InstantActions;
use vda5050::order::Order;
use vda5050::state::ActionStatus::Finished;
use vdma_lif::Lif;

const DT: f64 = 0.05;
/// 2026-11-20T10:00:00Z
const START_MS: i64 = 1_795_168_800_000;

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn id(serial: &str) -> RobotId {
    RobotId::new("ACME", serial)
}

/// Robot R-1 on A, robot R-2 on B.
fn world() -> World {
    let g = graph();
    let (a, b) = (g.node_idx("A").unwrap(), g.node_idx("B").unwrap());
    let mut w = World::new(g, DT, START_MS).unwrap();
    w.add_robot(id("R-1"), spec(), ActionTimes::new(1.0), a, 0.0)
        .unwrap();
    w.add_robot(id("R-2"), spec(), ActionTimes::new(1.0), b, 0.0)
        .unwrap();
    w
}

/// R-1: A→B. R-2: B→C.
fn order(serial: &str, from: &str, edge: &str, to: &str) -> Order {
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"{serial}","orderId":"o-{serial}","orderUpdateId":0,
        "nodes":[{{"nodeId":"{from}","sequenceId":0,"released":true,"actions":[]}},
                 {{"nodeId":"{to}","sequenceId":2,"released":true,"actions":[]}}],
        "edges":[{{"edgeId":"{edge}","sequenceId":1,"released":true,"actions":[]}}]}}"#
    );
    serde_json::from_str(&json).unwrap()
}

#[test]
fn time_is_ticks_times_the_step() {
    let mut w = world();
    assert_eq!((w.tick(), w.time()), (0, 0.0));
    assert_eq!(w.timestamp(), "2026-11-20T10:00:00.000Z");
    for _ in 0..30 {
        w.step();
    }
    assert_eq!(w.tick(), 30);
    assert!((w.time() - 1.5).abs() < 1e-9);
    assert_eq!(w.timestamp(), "2026-11-20T10:00:01.500Z");
}

#[test]
fn robots_move_independently() {
    let mut w = world();
    assert_eq!(w.send_order(&order("R-1", "A", "A-B", "B")), Ok(Accepted));
    // R-2 gets no order
    for _ in 0..40 {
        w.step();
    }
    let p1 = w.pose(&id("R-1")).unwrap();
    let p2 = w.pose(&id("R-2")).unwrap();
    assert!(p1.x > 0.0 && p1.x < 10.0, "R-1 is on its way: {}", p1.x);
    assert_eq!((p2.x, p2.y), (10.0, 0.0), "R-2 did not move");

    // now R-2 as well; both finish
    assert_eq!(w.send_order(&order("R-2", "B", "B-C", "C")), Ok(Accepted));
    for _ in 0..2000 {
        w.step();
    }
    let p1 = w.pose(&id("R-1")).unwrap();
    let p2 = w.pose(&id("R-2")).unwrap();
    assert!((p1.x - 10.0).abs() < 1e-9 && p1.y == 0.0);
    assert!((p2.x - 10.0).abs() < 1e-9 && (p2.y - 5.0).abs() < 1e-9);
}

#[test]
fn messages_go_to_the_robot_named_in_them() {
    let mut w = world();
    // R-1 is on A, so an order starting at B must be rejected there …
    let r = w.send_order(&order("R-1", "B", "B-C", "C")).unwrap();
    assert!(matches!(r, botsim_core::robot::Reception::Rejected(_)));
    // … but it is right for R-2, which stands on B
    assert_eq!(w.send_order(&order("R-2", "B", "B-C", "C")), Ok(Accepted));

    let unknown = order("R-9", "A", "A-B", "B");
    assert_eq!(
        w.send_order(&unknown),
        Err(WorldError::UnknownRobot(id("R-9")))
    );
}

#[test]
fn instant_actions_reach_the_right_robot() {
    let mut w = world();
    w.send_order(&order("R-1", "A", "A-B", "B")).unwrap();
    w.send_order(&order("R-2", "B", "B-C", "C")).unwrap();
    for _ in 0..40 {
        w.step();
    }
    let cancel: InstantActions = serde_json::from_str(
        r#"{"headerId":2,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1",
        "actions":[{"actionType":"cancelOrder","actionId":"c1","blockingType":"NONE"}]}"#,
    )
    .unwrap();
    w.send_instant_actions(&cancel).unwrap();
    for _ in 0..2000 {
        w.step();
    }
    let s1 = w.state(&id("R-1")).unwrap();
    assert_eq!(s1.instant_action_states[0].action_status, Finished);
    assert_eq!(s1.last_node_id, "B", "R-1 was cancelled and stopped at B");
    // R-2 was not cancelled and reached C
    let s2 = w.state(&id("R-2")).unwrap();
    assert!(s2.instant_action_states.is_empty());
    assert_eq!(s2.last_node_id, "C");
}

#[test]
fn state_messages_have_header_id_timestamp_and_identity() {
    let mut w = world();
    let s = w.state(&id("R-1")).unwrap();
    assert_eq!(
        (
            s.header_id,
            s.manufacturer.as_str(),
            s.serial_number.as_str()
        ),
        (0, "ACME", "R-1")
    );
    assert_eq!(
        (s.version.as_str(), s.timestamp.as_str()),
        ("3.0.1", "2026-11-20T10:00:00.000Z")
    );

    for _ in 0..20 {
        w.step();
    }
    let s = w.state(&id("R-1")).unwrap();
    assert_eq!(s.header_id, 1, "grows by one per state message");
    assert_eq!(s.timestamp, "2026-11-20T10:00:01.000Z");
    // the counter is per robot
    assert_eq!(w.state(&id("R-2")).unwrap().header_id, 0);
}

#[test]
fn robot_registration_errors() {
    let mut w = world();
    let a = w.graph().node_idx("A").unwrap();
    assert_eq!(
        w.add_robot(id("R-1"), spec(), ActionTimes::new(1.0), a, 0.0),
        Err(WorldError::DuplicateRobot(id("R-1")))
    );
    // same serial, other manufacturer is another robot
    assert!(
        w.add_robot(
            RobotId::new("OTHER", "R-1"),
            spec(),
            ActionTimes::new(1.0),
            a,
            0.0
        )
        .is_ok()
    );
    assert_eq!(w.robot_ids().count(), 3);
    assert_eq!(
        w.pose(&id("nope")),
        Err(WorldError::UnknownRobot(id("nope")))
    );
}

#[test]
fn the_time_step_must_be_positive_and_finite() {
    for dt in [0.0, -0.05, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            World::new(graph(), dt, START_MS),
            Err(WorldError::BadTimeStep)
        ));
    }
}

#[test]
fn two_runs_give_the_same_result() {
    let run = || {
        let mut w = world();
        w.send_order(&order("R-1", "A", "A-B", "B")).unwrap();
        w.send_order(&order("R-2", "B", "B-C", "C")).unwrap();
        for _ in 0..300 {
            w.step();
        }
        serde_json::to_string(&w.state(&id("R-1")).unwrap()).unwrap()
            + &serde_json::to_string(&w.state(&id("R-2")).unwrap()).unwrap()
    };
    assert_eq!(run(), run());
}

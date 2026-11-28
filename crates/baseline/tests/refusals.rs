//! The controller learns of a refused order or update from the robot's state, one
//! step later at the earliest, with and without a message delay (S58): a refused
//! new order (a door too narrow), a refused update (a curve a car-like robot cannot
//! drive, S70) and `ORIENTATION_UNREACHABLE`, which names no order.

use botsim_bridge as bridge;
use botsim_bridge::Bridged as Controller;
use botsim_core::actions::ActionTimes;
use botsim_core::door::DoorSpec;
use botsim_core::world::World;
use botsim_model::errors::ErrorType;
use botsim_model::graph::Graph;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::{Kinematics, RobotSpec};
use botsim_server::controller::NODE_TOLERANCE;
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

#[path = "../../core/tests/vda_port/mod.rs"]
mod vda_port;
use vda_port::VdaWorld;

const DT: f64 = 0.05;

/// Message delays (steps) every case is run with.
const DELAYS: [u64; 2] = [0, 10];

/// Nodes (id, x, y) and edges (from, to, orientation type, extra edge JSON) of
/// robot type ACME/X1; edge ids are "from-to".
fn layout(nodes: &[(&str, f64, f64)], edges: &[(&str, &str, &str, &str)]) -> Graph {
    let ns: Vec<String> = nodes
        .iter()
        .map(|(id, x, y)| {
            format!(
                r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
                "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]}}]}}"#
            )
        })
        .collect();
    let es: Vec<String> = edges
        .iter()
        .map(|(a, b, orientation_type, extra)| {
            format!(
                r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}",
                "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}],
                "orientationType":"{orientation_type}","mobileRobotOrientations":[0.0]{extra}}}]}}"#
            )
        })
        .collect();
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-20T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{}],"edges":[{}]}}]}}]}}"#,
        ns.join(","),
        es.join(",")
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif).unwrap()
}

const LINE: [(&str, f64, f64); 6] = [
    ("N0", 0.0, 0.0),
    ("N1", 5.0, 0.0),
    ("N2", 10.0, 0.0),
    ("N3", 15.0, 0.0),
    ("N4", 20.0, 0.0),
    ("N5", 25.0, 0.0),
];

/// The line N0 → … → N5; the edge number `global` (from 0) asks for a GLOBAL
/// orientation, which a differential robot cannot take.
fn line(global: Option<usize>) -> Graph {
    let edges: Vec<(&str, &str, &str, &str)> = (0..5)
        .map(|i| {
            let kind = if global == Some(i) {
                "GLOBAL"
            } else {
                "TANGENTIAL"
            };
            (LINE[i].0, LINE[i + 1].0, kind, "")
        })
        .collect();
    layout(&LINE, &edges)
}

fn id() -> RobotId {
    RobotId::new("ACME", "R-1")
}

/// A world with one robot of the given kinematics on N0 and the message delay.
fn world(g: Graph, kinematics: &str, tune: fn(&mut RobotSpec), delay: u64) -> World {
    let text = include_str!("data/diff_robot.factsheet.json").replace("DIFFERENTIAL", kinematics);
    let f: Factsheet = serde_json::from_str(&text).unwrap();
    let mut spec = RobotSpec::from_factsheet(&f).unwrap();
    tune(&mut spec);
    let start = g.node_idx("N0").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(
        id(),
        spec,
        ActionTimes::new(1.0).with("pick", 2.0).with("drop", 2.0),
        start,
        0.0,
    )
    .unwrap();
    w.set_message_delay(delay);
    w
}

fn job(w: &World, id: u64, pickup: &str, drop: &str) -> Job {
    let node = |n| w.graph().node_idx(n).unwrap();
    Job {
        id,
        pickup: node(pickup),
        drop: node(drop),
        created_tick: 0,
    }
}

/// Give the job and run until a job more is rejected: the number of cycles after
/// the one that took it. Fails if that takes more than `limit`.
fn until_rejected(w: &mut World, c: &mut Controller, job: Job, limit: u64) -> u64 {
    let before = c.rejected().len();
    bridge::step(c, w, [job]);
    assert_eq!(c.busy(), 1, "the job is not given out");
    assert_eq!(c.rejected().len(), before, "nobody can know yet");
    for cycle in 1..=limit {
        w.step();
        bridge::step(c, w, []);
        if c.rejected().len() > before {
            return cycle;
        }
    }
    panic!("no refusal noticed in {limit} cycles");
}

fn run(w: &mut World, c: &mut Controller, ticks: u64) {
    for _ in 0..ticks {
        w.step();
        bridge::step(c, w, []);
    }
}

/// The node the robot stands on.
fn stands_on(w: &World) -> &str {
    let p = w.pose(&id()).unwrap();
    w.graph()
        .nodes
        .iter()
        .find(|n| (n.x - p.x).hypot(n.y - p.y) <= NODE_TOLERANCE)
        .map_or("", |n| n.id.as_str())
}

#[test]
fn a_refused_new_order_is_given_up_in_the_cycle_the_state_shows_it() {
    for delay in DELAYS {
        // the robot is 0.8 m wide, the door on the first edge 0.7 m
        let g = line(None);
        let door = g.edge_idx("N0-N1").unwrap();
        let mut w = world(g, "DIFFERENTIAL", |_| {}, delay);
        w.add_door(DoorSpec {
            id: "D".into(),
            edges: vec![door],
            passage_width: Some(0.7),
            open_seconds: 2.0,
            close_seconds: 2.0,
            hold_seconds: None,
        })
        .unwrap();
        let mut c = bridge::controller(&w);
        let first = job(&w, 1, "N0", "N5");
        let cycles = until_rejected(&mut w, &mut c, first, 100);
        // the order takes `delay` steps to the robot, which refuses it in the
        // step after, and the state that shows it takes the same back
        assert_eq!(cycles, 2 * delay + 1, "delay {delay}");
        assert_eq!(c.rejected()[0].job.id, 1);
        assert_eq!(c.rejected()[0].reason, ErrorType::NoRouteToTarget);
        assert_eq!((c.busy(), c.completed().len()), (0, 0), "delay {delay}");
        // the node the order had reserved is free again
        let n1 = w.graph().node_idx("N1").unwrap();
        assert_eq!(c.policy().table().node_holder(n1), None, "delay {delay}");
        // the next order is refused too, and noticed too
        let second = job(&w, 2, "N0", "N1");
        until_rejected(&mut w, &mut c, second, 100);
        assert_eq!(c.rejected()[1].job.id, 2);
        assert_eq!(c.busy(), 0);
    }
}

#[test]
fn a_refused_update_is_cancelled_and_given_up_once_the_robot_has_stopped() {
    // a line, then a curve of radius 0.5 m (N3 → N4, east to north) that the
    // robot's lowest speed does not allow; it is released in an update
    let arc = r#","trajectory":{"degree":2,"knotVector":[0,0,0,1,1,1],"controlPoints":[{"x":15.0,"y":0.0},{"x":15.5,"y":0.0,"weight":0.7071067811865476},{"x":15.5,"y":0.5}]}"#;
    for delay in DELAYS {
        let g = layout(
            &[
                ("N0", 0.0, 0.0),
                ("N1", 5.0, 0.0),
                ("N2", 10.0, 0.0),
                ("N3", 15.0, 0.0),
                ("N4", 15.5, 0.5),
                ("N5", 15.5, 5.5),
            ],
            &[
                ("N0", "N1", "TANGENTIAL", ""),
                ("N1", "N2", "TANGENTIAL", ""),
                ("N2", "N3", "TANGENTIAL", ""),
                ("N3", "N4", "TANGENTIAL", arc),
                ("N4", "N5", "TANGENTIAL", ""),
            ],
        );
        let tune = |s: &mut RobotSpec| {
            s.min_speed = 0.5;
            s.limits.max_angular_speed = 0.5;
        };
        let mut w = world(g, "THREE_WHEEL", tune, delay);
        let mut c = bridge::controller(&w);
        let first = job(&w, 1, "N0", "N5");
        until_rejected(&mut w, &mut c, first, 2000);
        assert_eq!(c.rejected().len(), 1);
        assert_eq!(c.rejected()[0].reason, ErrorType::NoRouteToTarget);
        assert_eq!((c.busy(), c.completed().len()), (0, 0), "delay {delay}");
        // the robot has had the cancel and stands on a node short of the curve
        run(&mut w, &mut c, 100);
        let state = w.state(&id()).unwrap();
        assert!(!state.driving);
        assert!(
            state
                .instant_action_states
                .iter()
                .any(|a| a.action_id == "cancel-job-1"),
            "delay {delay}"
        );
        assert!(
            ["N2", "N3"].contains(&stands_on(&w)),
            "delay {delay}: {}",
            stands_on(&w)
        );
        assert_eq!(c.rejected().len(), 1);
        assert!(w.collisions().is_empty());
    }
}

/// A controller that takes the differential robot of `w` for an omnidirectional
/// one, so that its orders carry the layout's orientations.
fn mistaken(w: &World) -> Controller {
    bridge::with_controller(w, |mut config| {
        config.robots[0].kinematics = Kinematics::Omnidirectional;
        botsim_baseline::controller(&config)
    })
}

#[test]
fn an_unreachable_orientation_in_a_new_order_is_noticed_every_time() {
    for delay in DELAYS {
        let mut w = world(line(Some(0)), "DIFFERENTIAL", |_| {}, delay);
        let mut c = mistaken(&w);
        // twice on the same edge: the second refusal is told apart from the error
        // of the first, which the robot still reports when the order is sent
        for n in 1..=2 {
            let job = job(&w, n, "N0", "N2");
            let cycles = until_rejected(&mut w, &mut c, job, 100);
            assert_eq!(cycles, 2 * delay + 1, "delay {delay}, job {n}");
            let last = c.rejected().last().unwrap();
            assert_eq!(
                (last.job.id, last.reason),
                (n, ErrorType::OrientationUnreachable)
            );
            assert_eq!(c.busy(), 0);
        }
        let state = w.state(&id()).unwrap();
        assert!(
            state
                .errors
                .iter()
                .all(|e| e.error_references.iter().flatten().all(|r| {
                    // (the error names no order)
                    r.reference_key == "edgeId"
                })),
            "{:?}",
            state.errors
        );
    }
}

#[test]
fn an_unreachable_orientation_in_an_update_cancels_the_order() {
    for delay in DELAYS {
        // the fourth edge is released in an update
        let mut w = world(line(Some(3)), "DIFFERENTIAL", |_| {}, delay);
        let mut c = mistaken(&w);
        let first = job(&w, 1, "N0", "N5");
        until_rejected(&mut w, &mut c, first, 2000);
        assert_eq!(c.rejected()[0].reason, ErrorType::OrientationUnreachable);
        assert_eq!((c.busy(), c.completed().len()), (0, 0), "delay {delay}");
        run(&mut w, &mut c, 100);
        assert!(!w.state(&id()).unwrap().driving);
        assert!(
            ["N2", "N3"].contains(&stands_on(&w)),
            "delay {delay}: {}",
            stands_on(&w)
        );
        assert_eq!(c.rejected().len(), 1);
    }
}

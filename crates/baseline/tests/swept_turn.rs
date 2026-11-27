//! Swept-area reservation, node-node conflicts: two robots turn on two
//! neighbouring nodes at the same time. Standing along their lanes they are clear
//! of each other; turning they are not. With the node-node conflict in the table
//! the second robot waits.

use botsim_baseline::conflicts::{ConflictInput, NodeNodeConflictInput};
use botsim_baseline::controller::Controller;
use botsim_core::actions::ActionTimes;
use botsim_core::jobs::Job;
use botsim_core::spec::RobotSpec;
use botsim_core::world::{RobotId, World};
use vda5050::factsheet::Factsheet;

mod common;
use common::graph;

const DT: f64 = 0.05;

/// Two lanes 1.0 m apart, robots 1.2 m x 0.8 m. R1 drives east to P (0, 0) and
/// turns south; R2 drives west to Q (0, 1) and turns north, both over 5 m, so they
/// turn together. Returns (jobs done, collisions) after 120 s.
fn run(node_node_conflicts: bool) -> (usize, usize) {
    let g = graph(
        &[
            ("A", -5.0, 0.0),
            ("P", 0.0, 0.0),
            ("B", 0.0, -5.0),
            ("C", 5.0, 1.0),
            ("Q", 0.0, 1.0),
            ("D", 0.0, 6.0),
        ],
        &[("A", "P"), ("P", "B"), ("C", "Q"), ("Q", "D")],
    );
    let n = |id: &str| g.node_idx(id).unwrap();
    let (a, b, c, d) = (n("A"), n("B"), n("C"), n("D"));
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let times = ActionTimes::new(1.0).with("pick", 1.0).with("drop", 1.0);
    w.add_robot(
        RobotId::new("ACME", "R1"),
        spec.clone(),
        times.clone(),
        a,
        0.0,
    )
    .unwrap();
    w.add_robot(
        RobotId::new("ACME", "R2"),
        spec,
        times,
        c,
        std::f64::consts::PI,
    )
    .unwrap();
    // only the rules keep the robots apart
    w.set_safety_stop(false);
    let mut ctl = Controller::new(&w);
    // the table needs a (here empty) edge-edge input for the type
    ctl.set_conflicts(&[ConflictInput {
        type_a: "ACME/X1".into(),
        type_b: "ACME/X1".into(),
        edges: vec![],
    }]);
    if node_node_conflicts {
        let read = ctl.set_node_node_conflicts(&[NodeNodeConflictInput {
            type_a: "ACME/X1".into(),
            type_b: "ACME/X1".into(),
            pairs: vec![("P".into(), "Q".into())],
        }]);
        assert_eq!(read, 1);
    }
    let job = |id, pickup, drop| Job {
        id,
        pickup,
        drop,
        created_tick: 0,
    };
    let mut jobs = Some(vec![job(1, a, b), job(2, c, d)]);
    for _ in 0..(120.0 / DT) as usize {
        ctl.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    (ctl.completed().len(), w.collisions().len())
}

#[test]
fn without_the_node_node_conflict_the_turning_robots_collide() {
    let (_, collisions) = run(false);
    assert!(collisions > 0, "the robots should meet");
}

#[test]
fn with_the_node_node_conflict_they_turn_one_after_the_other() {
    let (done, collisions) = run(true);
    assert_eq!(collisions, 0);
    assert_eq!(done, 2);
}

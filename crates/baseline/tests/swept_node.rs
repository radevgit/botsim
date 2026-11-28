//! Swept-area reservation, node-edge conflicts: a robot stands on a node beside an
//! edge that does not touch the node; a second robot drives the edge and meets the
//! first. With the node-edge conflict in the table the second robot waits.

use botsim_bridge as bridge;
use botsim_core::actions::ActionTimes;
use botsim_core::world::World;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_server::conflicts::{ConflictInput, NodeConflictInput};
use vda5050::factsheet::Factsheet;

mod common;
use common::graph;

const DT: f64 = 0.05;

/// R1 stands on S (0, 0) for good; the edge C → D runs along y = 0.5, 0.5 m from S
/// and does not touch it. Returns (jobs done, collisions) after 60 s.
fn run(node_conflicts: bool) -> (usize, usize) {
    let g = graph(
        &[
            ("S", 0.0, 0.0),
            ("E", 0.0, -10.0),
            ("C", -10.0, 0.5),
            ("D", 10.0, 0.5),
        ],
        &[("S", "E"), ("C", "D")],
    );
    let n = |id: &str| g.node_idx(id).unwrap();
    let (s, c, d) = (n("S"), n("C"), n("D"));
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let times = ActionTimes::new(1.0).with("pick", 1.0).with("drop", 1.0);
    w.add_robot(
        RobotId::new("ACME", "R1"),
        spec.clone(),
        times.clone(),
        s,
        -std::f64::consts::FRAC_PI_2,
    )
    .unwrap();
    w.add_robot(RobotId::new("ACME", "R2"), spec, times, c, 0.0)
        .unwrap();
    w.set_safety_stop(false);
    let mut ctl = bridge::controller(&w);
    // the table needs a (here empty) edge-edge input for the type
    assert_eq!(
        ctl.policy_mut().set_conflicts(&[ConflictInput {
            type_a: "ACME/X1".into(),
            type_b: "ACME/X1".into(),
            edges: vec![],
        }]),
        0
    );
    if node_conflicts {
        let read = ctl.policy_mut().set_node_conflicts(&[NodeConflictInput {
            type_node: "ACME/X1".into(),
            type_edge: "ACME/X1".into(),
            pairs: vec![("S".into(), "C-D".into())],
        }]);
        assert_eq!(read, 1);
    }
    let mut jobs = Some(vec![Job {
        id: 1,
        pickup: c,
        drop: d,
        created_tick: 0,
    }]);
    for _ in 0..(60.0 / DT) as usize {
        bridge::step(&mut ctl, &mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    (ctl.completed().len(), w.collisions().len())
}

#[test]
fn without_the_node_conflict_the_passing_robot_hits_the_standing_one() {
    let (done, collisions) = run(false);
    assert_eq!(done, 1);
    assert!(collisions > 0, "the robots should meet");
}

#[test]
fn with_the_node_conflict_the_passing_robot_waits_for_the_standing_one_to_leave() {
    let (done, collisions) = run(true);
    assert_eq!(collisions, 0);
    assert_eq!(done, 0, "R1 never leaves S, so R2 never drives");
}

//! Swept-area reservation: two edges cross without a common node, so the node and
//! edge reservations let two robots use them at the same time and the robots meet at
//! the crossing. With the edge pair in the conflict table the second robot waits.

use botsim_baseline::conflicts::ConflictInput;
use botsim_baseline::controller::Controller;
use botsim_core::actions::ActionTimes;
use botsim_core::jobs::Job;
use botsim_core::spec::RobotSpec;
use botsim_core::world::{RobotId, World};
use vda5050::factsheet::Factsheet;

mod common;
use common::graph;

const DT: f64 = 0.05;

/// A → B east and C → D north cross at (0, 0); the two edges share no node.
/// Returns (jobs done, collisions) after 60 s.
fn run(conflicts: bool) -> (usize, usize) {
    let g = graph(
        &[
            ("A", -10.0, 0.0),
            ("B", 10.0, 0.0),
            ("C", 0.0, -10.0),
            ("D", 0.0, 10.0),
        ],
        &[("A", "B"), ("C", "D")],
    );
    let n = |id: &str| g.node_idx(id).unwrap();
    let (a, b, c, d) = (n("A"), n("B"), n("C"), n("D"));
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let times = ActionTimes::new(1.0).with("pick", 1.0).with("drop", 1.0);
    // the second robot faces north, as its edge does
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
        std::f64::consts::FRAC_PI_2,
    )
    .unwrap();
    w.set_safety_stop(false);
    let mut ctl = Controller::new(&w);
    if conflicts {
        let read = ctl.set_conflicts(&[ConflictInput {
            type_a: "ACME/X1".into(),
            type_b: "ACME/X1".into(),
            edges: vec![("A-B".into(), "C-D".into())],
        }]);
        assert_eq!(read, 1);
    }
    let jobs = vec![
        Job {
            id: 1,
            pickup: a,
            drop: b,
            created_tick: 0,
        },
        Job {
            id: 2,
            pickup: c,
            drop: d,
            created_tick: 0,
        },
    ];
    let mut jobs = Some(jobs);
    for _ in 0..(60.0 / DT) as usize {
        ctl.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    (ctl.completed().len(), w.collisions().len())
}

#[test]
fn without_the_conflict_table_the_robots_meet_at_the_crossing() {
    let (done, collisions) = run(false);
    assert_eq!(done, 2);
    assert!(collisions > 0, "the robots should meet");
}

#[test]
fn with_the_conflict_table_the_second_robot_waits_and_both_jobs_are_done() {
    let (done, collisions) = run(true);
    assert_eq!(done, 2);
    assert_eq!(collisions, 0);
}

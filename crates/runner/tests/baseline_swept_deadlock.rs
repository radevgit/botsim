//! A deadlock through a swept-area wait: robot A waits for the node robot B stands
//! on, and B waits because its next edge passes too close to the node A stands on
//! (M1, a station siding: nobody holds the node B wants).

use botsim::connect;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_model::time::Millis;
use botsim_model::waiting::WaitReason;
use botsim_server::conflicts::{ConflictInput, NodeConflictInput};
use botsim_sim::actions::ActionTimes;
use botsim_sim::waiting::Wait;
use botsim_sim::world::World;
use vda5050::factsheet::Factsheet;

#[path = "../../baseline/tests/common/mod.rs"]
mod common;
use common::graph;

const DT: f64 = 0.05;

/// A stands on X and wants S; B stands on S and wants to leave to Y along S → Y,
/// which (by the table) passes too close to X.
#[test]
fn a_cycle_through_a_node_edge_wait_is_reported_as_a_deadlock() {
    let g = graph(
        &[("X", 0.0, 0.0), ("S", 0.0, 5.0), ("Y", 5.0, 0.0)],
        &[("X", "S"), ("S", "Y")],
    );
    let n = |id: &str| g.node_idx(id).unwrap();
    let (x, s, y) = (n("X"), n("S"), n("Y"));
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let times = ActionTimes::new(1.0);
    let up = std::f64::consts::FRAC_PI_2;
    let (a, b) = (RobotId::new("ACME", "A"), RobotId::new("ACME", "B"));
    w.add_robot(a.clone(), spec.clone(), times.clone(), x, up)
        .unwrap();
    w.add_robot(b.clone(), spec, times, s, -up / 2.0).unwrap();
    w.set_deadlock_timeout(10.0);
    let mut ctl = connect::controller(&w);
    ctl.policy_mut().set_conflicts(&[ConflictInput {
        type_a: "ACME/X1".into(),
        type_b: "ACME/X1".into(),
        edges: vec![],
    }]);
    let read = ctl.policy_mut().set_node_conflicts(&[NodeConflictInput {
        type_node: "ACME/X1".into(),
        type_edge: "ACME/X1".into(),
        pairs: vec![("X".into(), "S-Y".into())],
    }]);
    assert_eq!(read, 1);
    // each job starts where one robot stands: A is given the job to S, B the one to Y
    let job = |id, pickup, drop| Job {
        id,
        pickup,
        drop,
        created: Millis(0),
    };
    let mut jobs = Some(vec![job(1, x, s), job(2, s, y)]);
    for _ in 0..(40.0 / DT) as usize {
        connect::step(&mut ctl, &mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    assert!(ctl.completed().is_empty(), "both are stuck");
    assert_eq!(ctl.policy().table().node_holder(y), None, "nobody holds Y");
    // both are release waits, each naming the other (B's for the swept area)
    let release = |blocker| {
        Some(Wait {
            reason: WaitReason::Release,
            blocker: Some(blocker),
        })
    };
    assert_eq!(w.waits(), [release(1), release(0)]);
    let deadlocks = w.deadlocks();
    assert_eq!(deadlocks.len(), 1, "{deadlocks:?}");
    assert_eq!(deadlocks[0].robots, vec![a, b]);
}

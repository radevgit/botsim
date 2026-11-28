//! The waiting metric's head-on case (brief U12): two robots come to a corridor
//! from its two ends; the second one's wait is credited to the first.

use botsim::connect;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_model::time::Millis;
use botsim_model::waiting::WaitReason;
use botsim_sim::actions::ActionTimes;
use botsim_sim::waiting::Place;
use botsim_sim::world::World;
use vda5050::factsheet::Factsheet;

#[path = "../../baseline/tests/common/mod.rs"]
mod common;
use common::graph;

const DT: f64 = 0.05;

/// A corridor B ↔ C, 20 m. The first robot drives A → B → C → E, the second
/// D → C → B → F; each leaves the corridor to a node of its own.
#[test]
fn the_second_robot_at_a_corridor_waits_for_the_first() {
    let g = graph(
        &[
            ("A", 0.0, 0.0),
            ("B", 5.0, 0.0),
            ("C", 25.0, 0.0),
            ("D", 30.0, 0.0),
            ("E", 25.0, 5.0),
            ("F", 5.0, 5.0),
        ],
        &[
            ("A", "B"),
            ("B", "C"),
            ("C", "B"),
            ("D", "C"),
            ("C", "E"),
            ("B", "F"),
        ],
    );
    let n = |id: &str| g.node_idx(id).unwrap();
    let (a, d, e, f_node) = (n("A"), n("D"), n("E"), n("F"));
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let times = ActionTimes::new(1.0);
    for (serial, node, heading) in [("R1", a, 0.0), ("R2", d, std::f64::consts::PI)] {
        let id = RobotId::new("ACME", serial);
        w.add_robot(id, spec.clone(), times.clone(), node, heading)
            .unwrap();
    }
    let job = |id, pickup, drop| Job {
        id,
        pickup,
        drop,
        created: Millis(0),
    };
    let mut jobs = Some(vec![job(1, a, e), job(2, d, f_node)]);
    let mut c = connect::controller(&w);
    for _ in 0..(120.0 / DT) as usize {
        connect::step(&mut c, &mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    assert_eq!(c.completed().len(), 2, "{:?}", c.rejected());
    let totals = w.wait_totals();
    // only the second robot waited for another one
    let named: Vec<_> = totals.iter().filter(|t| t.blocker.is_some()).collect();
    assert!(
        named
            .iter()
            .all(|t| t.waiter == 1 && t.blocker == Some(0) && t.reason == WaitReason::Release),
        "{totals:?}"
    );
    // most of it at D, where it started: the first one holds the corridor
    let at_d: u64 = named
        .iter()
        .filter(|t| t.place == Place::Node(d))
        .map(|t| t.ticks)
        .sum();
    assert!(at_d as f64 * DT > 5.0, "{totals:?}");
}

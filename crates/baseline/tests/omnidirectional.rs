//! The baseline controller with an omnidirectional robot (S63): its orders name no
//! orientation, so the robot keeps its heading all the way round.

use botsim_baseline::controller::Controller;
use botsim_core::actions::ActionTimes;
use botsim_core::graph::Graph;
use botsim_core::jobs::Job;
use botsim_core::spec::RobotSpec;
use botsim_core::world::{RobotId, World};
use vda5050::factsheet::Factsheet;

mod common;
use common::graph;

const DT: f64 = 0.05;

/// A one-way ring N0 → N1 → N2 → N3 → N0, 20 m sides: every node is a 90° turn.
fn ring() -> Graph {
    graph(
        &[
            ("N0", 0.0, 0.0),
            ("N1", 20.0, 0.0),
            ("N2", 20.0, 20.0),
            ("N3", 0.0, 20.0),
        ],
        &[("N0", "N1"), ("N1", "N2"), ("N2", "N3"), ("N3", "N0")],
    )
}

/// One robot of the given kinematics on N0, one job N0 → N3 (two corners);
/// returns (finish tick, the headings the robot had during the run).
fn run(kinematics: &str) -> (u64, Vec<f64>) {
    let g = ring();
    let text = include_str!("data/diff_robot.factsheet.json").replace("DIFFERENTIAL", kinematics);
    let f: Factsheet = serde_json::from_str(&text).unwrap();
    let n0 = g.node_idx("N0").unwrap();
    let job = Job {
        id: 1,
        pickup: n0,
        drop: g.node_idx("N3").unwrap(),
        created_tick: 0,
    };
    let mut w = World::new(g, DT, 0).unwrap();
    let id = RobotId::new("ACME", "R-1");
    w.add_robot(
        id.clone(),
        RobotSpec::from_factsheet(&f).unwrap(),
        ActionTimes::new(1.0).with("pick", 2.0).with("drop", 2.0),
        n0,
        0.0,
    )
    .unwrap();
    let mut c = Controller::new(&w);
    let mut jobs = Some(vec![job]);
    let mut headings = Vec::new();
    for _ in 0..20000 {
        c.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
        headings.push(w.pose(&id).unwrap().heading);
    }
    assert_eq!(c.completed().len(), 1, "{kinematics}: {:?}", c.rejected());
    assert!(w.collisions().is_empty());
    (c.completed()[0].finished_tick, headings)
}

#[test]
fn an_omnidirectional_robot_does_the_job_without_ever_turning() {
    let (omni_tick, headings) = run("OMNIDIRECTIONAL");
    assert!(headings.iter().all(|&h| h == 0.0), "it kept facing east");
    let (diff_tick, diff_headings) = run("DIFFERENTIAL");
    assert!(
        diff_headings.iter().any(|&h| h != 0.0),
        "the differential robot turned"
    );
    // two 90° turns at 1 rad/s (the route N0 → N1 → N2 → N3 has two corners) are
    // about 3 s (60 steps) that it does not need
    assert!(
        diff_tick > omni_tick + 55,
        "omni {omni_tick}, differential {diff_tick}"
    );
}

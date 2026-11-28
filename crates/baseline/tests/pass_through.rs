//! The baseline controller with robots that drive through nodes (S66): its window
//! of released nodes is wide enough for them to keep going.

use botsim_bridge as bridge;
use botsim_core::actions::ActionTimes;
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;

mod common;
use common::graph;

const DT: f64 = 0.05;

/// N0 → N1 → … → N7 on a line, 6 m apart, one way.
fn chain() -> Graph {
    let nodes: Vec<(String, f64, f64)> = (0..8)
        .map(|i| (format!("N{i}"), 6.0 * i as f64, 0.0))
        .collect();
    let nodes: Vec<(&str, f64, f64)> = nodes.iter().map(|(n, x, y)| (n.as_str(), *x, *y)).collect();
    let edges: Vec<(String, String)> = (0..7)
        .map(|i| (format!("N{i}"), format!("N{}", i + 1)))
        .collect();
    let edges: Vec<(&str, &str)> = edges
        .iter()
        .map(|(a, b)| (a.as_str(), b.as_str()))
        .collect();
    graph(&nodes, &edges)
}

/// A robot on N0 and a job N0 → N7; returns (finish tick, collisions).
fn run(pass_through: bool, delay: u64) -> (u64, usize, Vec<f64>) {
    let g = chain();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let n0 = g.node_idx("N0").unwrap();
    let job = Job {
        id: 1,
        pickup: n0,
        drop: g.node_idx("N7").unwrap(),
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
    w.set_pass_through(pass_through);
    w.set_message_delay(delay);
    let mut c = bridge::controller(&w);
    let mut jobs = Some(vec![job]);
    let mut top_between = Vec::new();
    for _ in 0..20000 {
        bridge::step(&mut c, &mut w, jobs.take().unwrap_or_default());
        w.step();
        // the speed while the robot is in the middle of the line
        let x = w.pose(&id).unwrap().x;
        if (12.0..30.0).contains(&x) {
            top_between.push(w.snapshot().robots[0].speed.abs());
        }
    }
    assert_eq!(c.completed().len(), 1, "{:?}", c.rejected());
    (
        c.completed()[0].finished_tick,
        w.collisions().len(),
        top_between,
    )
}

#[test]
fn a_job_on_a_straight_line_is_much_faster_when_robots_drive_through() {
    let (off, c_off, _) = run(false, 0);
    let (on, c_on, speeds) = run(true, 0);
    assert_eq!((c_off, c_on), (0, 0));
    // it keeps its speed through the middle nodes (no dips to a standstill)
    let slowest = speeds.iter().cloned().fold(f64::MAX, f64::min);
    assert!(slowest > 1.0, "slowest in the middle: {slowest}");
    // six nodes without a stop: several seconds each
    assert!(on + 100 < off, "on {on}, off {off}");
}

#[test]
fn with_late_orders_the_robot_stops_at_the_end_of_what_it_has_and_goes_on() {
    // updates arrive 1 s late: the robot sometimes has no next edge when it gets
    // to a node and must stop there; it still does the job
    let (instant, ..) = run(true, 0);
    let (late, collisions, _) = run(true, 20);
    assert_eq!(collisions, 0);
    assert!(late > instant, "late {late}, instant {instant}");
}

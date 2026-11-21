use botsim_baseline::controller::Controller;
use botsim_core::actions::ActionTimes;
use botsim_core::graph::Graph;
use botsim_core::jobs::{Job, JobGenerator};
use botsim_core::spec::RobotSpec;
use botsim_core::world::{RobotId, World};
use vda5050::factsheet::Factsheet;

mod common;
use common::graph;

const DT: f64 = 0.05;

/// N0 → N1 → … → N6, 5 m apart, one way.
fn chain() -> Graph {
    let nodes: Vec<(String, f64, f64)> = (0..7)
        .map(|i| (format!("N{i}"), 5.0 * i as f64, 0.0))
        .collect();
    let nodes: Vec<(&str, f64, f64)> = nodes.iter().map(|(n, x, y)| (n.as_str(), *x, *y)).collect();
    let edges: Vec<(String, String)> = (0..6)
        .map(|i| (format!("N{i}"), format!("N{}", i + 1)))
        .collect();
    let edges: Vec<(&str, &str)> = edges
        .iter()
        .map(|(a, b)| (a.as_str(), b.as_str()))
        .collect();
    graph(&nodes, &edges)
}

/// A one-way ring N0 → N1 → N2 → N3 → N0, 20 m sides.
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

/// A world with robots R-1, R-2, … on the given nodes.
fn world(g: Graph, starts: &[&str]) -> World {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let nodes: Vec<_> = starts.iter().map(|n| g.node_idx(n).unwrap()).collect();
    let mut w = World::new(g, DT, 0).unwrap();
    for (i, n) in nodes.into_iter().enumerate() {
        let times = ActionTimes::new(1.0).with("pick", 5.0).with("drop", 5.0);
        let id = RobotId::new("ACME", &format!("R-{}", i + 1));
        w.add_robot(id, spec.clone(), times, n, 0.0).unwrap();
    }
    w
}

fn job(g: &Graph, id: u64, pickup: &str, drop: &str) -> Job {
    Job {
        id,
        pickup: g.node_idx(pickup).unwrap(),
        drop: g.node_idx(drop).unwrap(),
        created_tick: 0,
    }
}

/// Run `ticks` cycles; `jobs` are given to the controller in the first one.
fn run(w: &mut World, c: &mut Controller, jobs: Vec<Job>, ticks: u64) {
    let mut jobs = Some(jobs);
    for _ in 0..ticks {
        c.step(w, jobs.take().unwrap_or_default());
        w.step();
    }
}

fn completed_ids(c: &Controller) -> Vec<u64> {
    c.completed().iter().map(|d| d.job.id).collect()
}

#[test]
fn one_robot_does_one_job() {
    let g = chain();
    let j = job(&g, 1, "N2", "N5");
    let mut w = world(g, &["N0"]);
    let mut c = Controller::new(&w);
    run(&mut w, &mut c, vec![j], 6000);

    assert_eq!(completed_ids(&c), vec![1]);
    let d = &c.completed()[0];
    assert_eq!(d.robot, RobotId::new("ACME", "R-1"));
    assert!(d.finished_tick > d.assigned_tick);
    assert_eq!(
        (c.busy(), c.pending().count(), c.rejected().len()),
        (0, 0, 0)
    );
    // the robot ended at the drop node and stays there (S31)
    let p = w.pose(&d.robot).unwrap();
    assert!((p.x - 25.0).abs() < 1e-9);
    // 25 m of driving takes time and both actions took 5 s: more than 10 s
    assert!(d.finished_tick as f64 * DT > 10.0 + 25.0 / 1.5);
}

#[test]
fn the_nearest_free_robot_gets_the_job() {
    let g = chain();
    let j = job(&g, 1, "N5", "N6");
    // R-1 on N0 is 25 m from the pickup, R-2 on N4 only 5 m
    let mut w = world(g, &["N0", "N4"]);
    let mut c = Controller::new(&w);
    run(&mut w, &mut c, vec![j], 6000);
    assert_eq!(c.completed().len(), 1);
    assert_eq!(c.completed()[0].robot, RobotId::new("ACME", "R-2"));
}

#[test]
fn equal_distance_goes_to_the_robot_added_first() {
    let g = chain();
    let j = job(&g, 1, "N2", "N3");
    // two robots on the same node: the same distance
    let mut w = world(g, &["N0", "N0"]);
    let mut c = Controller::new(&w);
    run(&mut w, &mut c, vec![j], 6000);
    assert_eq!(c.completed()[0].robot, RobotId::new("ACME", "R-1"));
}

#[test]
fn jobs_are_served_in_order_and_an_unreachable_job_does_not_block_others() {
    let g = chain();
    // J1 first; afterwards the robot stands on N2, so J2 (N0 → N1, backwards in a
    // one-way chain) cannot be done; J3 can
    let jobs = vec![
        job(&g, 1, "N1", "N2"),
        job(&g, 2, "N0", "N1"),
        job(&g, 3, "N3", "N4"),
    ];
    let mut w = world(g, &["N0"]);
    let mut c = Controller::new(&w);
    run(&mut w, &mut c, jobs, 8000);
    assert_eq!(completed_ids(&c), vec![1, 3]);
    assert_eq!(c.pending().map(|j| j.id).collect::<Vec<_>>(), vec![2]);
}

#[test]
fn two_robots_work_in_parallel_and_a_busy_robot_gets_no_second_job() {
    let g = ring();
    let jobs = vec![
        job(&g, 1, "N1", "N2"),
        job(&g, 2, "N3", "N0"),
        job(&g, 3, "N2", "N3"),
    ];
    let mut w = world(g, &["N0", "N2"]);
    let mut c = Controller::new(&w);
    run(&mut w, &mut c, jobs, 1);
    // after the first cycle both robots have a job and the third job waits
    assert_eq!(c.busy(), 2);
    assert_eq!(c.pending().map(|j| j.id).collect::<Vec<_>>(), vec![3]);
    run(&mut w, &mut c, vec![], 20_000);
    assert_eq!(completed_ids(&c).len(), 3);
    assert_eq!(c.rejected().len(), 0);
}

#[test]
fn many_generated_jobs_all_get_done_and_the_run_is_repeatable() {
    let once = || {
        let g = ring();
        let nodes: Vec<_> = (0..4)
            .map(|i| g.node_idx(&format!("N{i}")).unwrap())
            .collect();
        let mut gen_ = JobGenerator::new(7, &nodes, 120.0, DT).unwrap();
        let mut w = world(g, &["N0", "N2"]);
        let mut c = Controller::new(&w);
        // jobs for 10 minutes (about 20), then 30 more minutes to finish
        for t in 0..48_000u64 {
            let new: Vec<Job> = if t < 12_000 {
                gen_.step(t).into_iter().collect()
            } else {
                vec![]
            };
            c.step(&mut w, new);
            w.step();
        }
        (
            completed_ids(&c),
            c.pending().count(),
            c.busy(),
            c.rejected().len(),
            w.collisions().len(),
        )
    };
    let (done, pending, busy, rejected, collisions) = once();
    eprintln!("{} jobs done, {collisions} collisions", done.len());
    assert!(done.len() >= 10, "{done:?}");
    assert_eq!(
        (pending, busy, rejected),
        (0, 0, 0),
        "everything is finished"
    );
    // same seed, same result (jobs and collisions)
    assert_eq!(once(), (done, pending, busy, rejected, collisions));
}

/// One job N0 → N5 with pick at N2; the controller runs every `cycle` ticks.
fn job_with_cycle(cycle: u64) -> (Controller, f64) {
    let g = chain();
    let j = job(&g, 1, "N2", "N5");
    let mut w = world(g, &["N0"]);
    let mut c = Controller::new(&w);
    let mut jobs = Some(vec![j]);
    for t in 0..12_000u64 {
        if t % cycle == 0 {
            c.step(&mut w, jobs.take().unwrap_or_default());
        }
        w.step();
    }
    c.step(&mut w, vec![]);
    (c, w.time())
}

#[test]
fn a_prompt_controller_makes_the_robot_wait_for_nothing_and_a_slow_one_does() {
    // every tick: the next nodes are released before the robot needs them
    let (c, _) = job_with_cycle(1);
    assert_eq!(c.completed().len(), 1);
    assert_eq!(c.completed()[0].waiting_seconds, 0.0);

    // a cycle of 10 s: the robot reaches the end of its base and waits for the update
    let (c, _) = job_with_cycle(200);
    assert_eq!(c.completed().len(), 1);
    assert!(
        c.completed()[0].waiting_seconds > 1.0,
        "waited {} s",
        c.completed()[0].waiting_seconds
    );
}

#[test]
fn the_controller_reports_throughput_and_waiting() {
    let (c, elapsed) = job_with_cycle(1);
    let m = c.metrics(elapsed);
    assert_eq!(m.jobs_done, 1);
    // one job in 600 s is 6 jobs per hour
    assert!((m.throughput_per_hour - 6.0).abs() < 1e-9, "{m:?}");
    assert_eq!(m.waiting_avg_seconds, Some(0.0));
    assert_eq!(m.waiting_max_seconds, Some(0.0));
}

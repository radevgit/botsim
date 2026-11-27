use botsim_baseline::controller::Controller;
use botsim_baseline::reservations::Reservations;
use botsim_core::actions::ActionTimes;
use botsim_core::jobs::{Job, JobGenerator};
use botsim_core::world::{RobotId, World};
use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;

mod common;
use common::graph;

const DT: f64 = 0.05;

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

/// A two-way corridor A – B – C.
fn corridor() -> Graph {
    graph(
        &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("C", 20.0, 0.0)],
        &[("A", "B"), ("B", "A"), ("B", "C"), ("C", "B")],
    )
}

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

fn run(w: &mut World, c: &mut Controller, jobs: Vec<Job>, ticks: u64) {
    let mut jobs = Some(jobs);
    for _ in 0..ticks {
        c.step(w, jobs.take().unwrap_or_default());
        w.step();
    }
}

#[test]
fn reservations_have_one_holder_and_step_in_pairs() {
    let g = corridor();
    let (a, b) = (g.node_idx("A").unwrap(), g.node_idx("B").unwrap());
    let ab = g.edge_idx("A-B").unwrap();
    let mut r = Reservations::new(&g);

    assert!(r.reserve_node(a, 0));
    assert!(r.reserve_node(a, 0), "again by the holder is fine");
    assert!(!r.reserve_node(a, 1), "another robot cannot take it");
    assert_eq!(r.node_holder(a), Some(0));

    // an edge and its end node are taken together or not at all
    assert!(r.reserve_node(b, 1));
    assert!(!r.reserve_step(ab, b, 0));
    assert_eq!(r.edge_holder(ab), None, "nothing was taken");
    assert!(r.reserve_step(ab, b, 1));
    assert_eq!((r.held_by(1), r.held_by(0)), ((1, 1), (1, 0)));

    // only the holder can give back
    r.release_node(b, 0);
    assert_eq!(r.node_holder(b), Some(1));
    r.release_node(b, 1);
    r.release_edge(ab, 1);
    assert_eq!((r.node_holder(b), r.edge_holder(ab)), (None, None));
}

#[test]
fn a_robot_holds_its_node_and_a_job_moves_the_reservations_along() {
    let g = ring();
    let j = job(&g, 1, "N1", "N2");
    let n = |id: &str| g.node_idx(id).unwrap();
    let (n0, n1, n2) = (n("N0"), n("N1"), n("N2"));
    let mut w = world(g, &["N0"]);
    let mut c = Controller::new(&w);

    run(&mut w, &mut c, vec![], 1);
    assert_eq!(
        c.reservations().node_holder(n0),
        Some(0),
        "idle robot holds its node"
    );

    run(&mut w, &mut c, vec![j], 6000);
    assert_eq!(c.completed().len(), 1);
    // done: it holds only the node it stands on
    assert_eq!(c.reservations().held_by(0), (1, 0));
    assert_eq!(c.reservations().node_holder(n2), Some(0));
    assert_eq!(c.reservations().node_holder(n0), None);
    assert_eq!(c.reservations().node_holder(n1), None);
}

#[test]
fn a_robot_waits_for_a_node_held_by_an_idle_robot_until_it_moves_away() {
    // R-1 on A: job pick at A, drop at B. R-2 stands on B (idle) and holds it.
    let g = corridor();
    let j = job(&g, 1, "A", "B");
    let j2 = job(&g, 2, "B", "C");
    let mut w = world(g, &["A", "B"]);
    let mut c = Controller::new(&w);
    // R-1 is the nearest to pickup A (distance 0)
    run(&mut w, &mut c, vec![j], 2000);
    // it picks for 5 s, then needs B, which R-2 holds → it waits at A
    assert_eq!((c.completed().len(), c.busy()), (0, 1));
    let p = w.pose(&RobotId::new("ACME", "R-1")).unwrap();
    assert_eq!(p.x, 0.0, "R-1 did not enter B");
    assert!(w.collisions().is_empty());

    // a job for R-2 makes it leave B; then R-1 can go on and finish
    run(&mut w, &mut c, vec![j2], 20_000);
    assert_eq!(c.completed().len(), 2);
    assert!(w.collisions().is_empty());
}

#[test]
fn head_on_robots_in_a_corridor_do_not_collide_but_block_each_other() {
    // R-1 goes A → C, R-2 goes C → A: they meet at B, nobody gives way
    let g = corridor();
    let jobs = vec![job(&g, 1, "A", "C"), job(&g, 2, "C", "A")];
    let mut w = world(g, &["A", "C"]);
    let mut c = Controller::new(&w);
    run(&mut w, &mut c, jobs, 30_000); // 25 minutes
    assert!(w.collisions().is_empty(), "{:?}", w.collisions());
    // a deadlock: both are still working on their job
    assert_eq!((c.completed().len(), c.busy()), (0, 2));
    // and the world has seen it: one deadlock, both robots, still lasting
    let d = w.deadlocks();
    assert_eq!(d.len(), 1, "{d:?}");
    assert_eq!(
        d[0].robots,
        vec![RobotId::new("ACME", "R-1"), RobotId::new("ACME", "R-2")]
    );
    assert_eq!(d[0].ended_tick, None);
    // reported 30 s after the cycle formed
    assert_eq!(d[0].detected_tick - d[0].first_seen_tick, 600);
}

#[test]
fn robots_that_wait_for_a_moving_robot_or_do_a_job_are_no_deadlock() {
    // the busy ring: robots wait for each other a lot, but it always resolves
    let g = ring();
    let nodes: Vec<_> = (0..4)
        .map(|i| g.node_idx(&format!("N{i}")).unwrap())
        .collect();
    let mut gen_ = JobGenerator::new(11, &nodes, 240.0, DT).unwrap();
    let mut w = world(g, &["N0", "N1", "N2"]);
    let mut c = Controller::new(&w);
    for t in 0..60_000u64 {
        let new: Vec<Job> = gen_.step(t).into_iter().collect();
        c.step(&mut w, new);
        w.step();
    }
    assert!(w.deadlocks().is_empty(), "{:?}", w.deadlocks());
}

#[test]
fn many_generated_jobs_on_a_ring_with_three_robots_never_collide() {
    let once = || {
        let g = ring();
        let nodes: Vec<_> = (0..4)
            .map(|i| g.node_idx(&format!("N{i}")).unwrap())
            .collect();
        let mut gen_ = JobGenerator::new(11, &nodes, 240.0, DT).unwrap();
        let mut w = world(g, &["N0", "N1", "N2"]);
        let mut c = Controller::new(&w);
        for t in 0..60_000u64 {
            let new: Vec<Job> = gen_.step(t).into_iter().collect();
            c.step(&mut w, new);
            w.step();
        }
        let m = c.metrics(w.time());
        (
            w.collisions().len(),
            c.completed().len(),
            m.waiting_max_seconds,
        )
    };
    let (collisions, done, waiting_max) = once();
    eprintln!("{collisions} collisions, {done} done, waiting max {waiting_max:?}");
    assert_eq!(collisions, 0);
    assert!(done > 20, "{done} jobs done");
    // busy ring: robots do have to wait for each other now
    assert!(waiting_max.unwrap() > 0.0);
    assert_eq!(once(), (collisions, done, waiting_max), "repeatable");
}

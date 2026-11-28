//! The baseline with a message delay (S58): the controller reads old states and
//! its orders arrive late.

use botsim::connect;
use botsim::connect::Connected as Controller;
use botsim_model::graph::Graph;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_sim::actions::ActionTimes;
use botsim_sim::world::World;
use vda5050::factsheet::Factsheet;

#[path = "../../baseline/tests/common/mod.rs"]
mod common;
use common::graph;

const DT: f64 = 0.05;

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

fn world(starts: &[&str], delay: u64) -> World {
    let g = ring();
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
    w.set_message_delay(delay);
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

/// Run the jobs for `ticks` cycles; the tick each job was finished at.
fn run(w: &mut World, jobs: Vec<Job>, ticks: u64) -> (Controller, Vec<(u64, u64)>) {
    let mut c = connect::controller(w);
    let mut jobs = Some(jobs);
    for _ in 0..ticks {
        connect::step(&mut c, w, jobs.take().unwrap_or_default());
        w.step();
    }
    let done = c
        .completed()
        .iter()
        .map(|d| (d.job.id, d.finished_tick))
        .collect();
    (c, done)
}

#[test]
fn a_delay_slows_the_jobs_down_but_they_are_done_without_collisions() {
    let g = ring();
    let jobs = || vec![job(&g, 1, "N0", "N2"), job(&g, 2, "N1", "N3")];
    let finish = |delay: u64| {
        let mut w = world(&["N0", "N1"], delay);
        let (c, done) = run(&mut w, jobs(), 20000);
        assert_eq!(done.len(), 2, "delay {delay}: {:?}", c.rejected());
        assert!(w.collisions().is_empty(), "delay {delay}");
        assert_eq!(w.deadlocks().len(), 0, "delay {delay}");
        done.iter().map(|d| d.1).max().unwrap()
    };
    let (none, short, long) = (finish(0), finish(10), finish(60));
    assert!(short > none, "{short} against {none}");
    assert!(long > short, "{long} against {short}");
}

#[test]
fn an_order_that_has_not_arrived_is_not_mistaken_for_a_finished_job() {
    // with a long delay the robot's state still shows the old (finished) order for a
    // while after a new job was assigned; the job must not be counted as done then
    let g = ring();
    let mut w = world(&["N0"], 40);
    let mut c = connect::controller(&w);
    connect::step(&mut c, &mut w, vec![job(&g, 1, "N0", "N1")]);
    for _ in 0..30 {
        w.step();
        connect::step(&mut c, &mut w, vec![]);
    }
    assert!(c.completed().is_empty());
    assert_eq!(c.busy(), 1);
    // and it gets done in the end
    for _ in 0..6000 {
        w.step();
        connect::step(&mut c, &mut w, vec![]);
    }
    assert_eq!(c.completed().len(), 1);
}

#[test]
fn a_delayed_run_is_repeatable() {
    let once = || {
        let g = ring();
        let mut w = world(&["N0", "N2"], 20);
        let (_, done) = run(
            &mut w,
            vec![job(&g, 1, "N0", "N2"), job(&g, 2, "N2", "N0")],
            20000,
        );
        done
    };
    assert_eq!(once(), once());
}

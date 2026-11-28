//! Timeout re-routing of the baseline (S53): a robot that another robot has held
//! up at the end of its released route for too long gets a route around it.

use botsim_baseline::controller::{Controller, REROUTE_AFTER_SECONDS};
use botsim_core::actions::ActionTimes;
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::state::ActionStatus;

mod common;
use common::graph;

const DT: f64 = 0.05;

/// One-way streets: the main line A → B → C → D (5 m apart), a dead end C → G and
/// a way round A → E → F → D; with `shortcut`, also F → C.
///
/// ```text
///  E ----→ F
///  ↑       ↓ ↘ (F → C only with `shortcut`)
///  A → B → C → D
///          ↓
///          G
/// ```
fn streets(shortcut: bool, round: bool) -> Graph {
    // G is a dead end off C, a place for a robot to end up out of the way
    let mut nodes = vec![
        ("A", 0.0, 0.0),
        ("B", 5.0, 0.0),
        ("C", 10.0, 0.0),
        ("D", 15.0, 0.0),
        ("G", 10.0, -5.0),
    ];
    let mut edges = vec![("A", "B"), ("B", "C"), ("C", "D"), ("C", "G")];
    if round {
        nodes.extend([("E", 0.0, 5.0), ("F", 10.0, 5.0)]);
        edges.extend([("A", "E"), ("E", "F"), ("F", "D")]);
    }
    if shortcut {
        edges.push(("F", "C"));
    }
    graph(&nodes, &edges)
}

fn id(n: usize) -> RobotId {
    RobotId::new("ACME", &format!("R-{n}"))
}

fn world(g: Graph, starts: &[&str]) -> World {
    world_with(g, starts, 5.0)
}

/// Robots R-1, R-2, … on the given nodes; `pick` and `drop` take `seconds`.
fn world_with(g: Graph, starts: &[&str], seconds: f64) -> World {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let nodes: Vec<_> = starts.iter().map(|n| g.node_idx(n).unwrap()).collect();
    let mut w = World::new(g, DT, 0).unwrap();
    for (i, n) in nodes.into_iter().enumerate() {
        let times = ActionTimes::new(1.0)
            .with("pick", seconds)
            .with("drop", seconds);
        w.add_robot(id(i + 1), spec.clone(), times, n, 0.0).unwrap();
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

/// Run for `seconds`; returns the time (s) each job was finished.
fn run(w: &mut World, c: &mut Controller, jobs: Vec<Job>, seconds: f64) -> Vec<(u64, f64)> {
    let mut jobs = Some(jobs);
    for _ in 0..(seconds / DT) as u64 {
        c.step(w, jobs.take().unwrap_or_default());
        w.step();
    }
    c.completed()
        .iter()
        .map(|d| (d.job.id, d.finished_tick as f64 * DT))
        .collect()
}

#[test]
fn a_robot_held_up_by_an_idle_robot_goes_round_it_after_the_timeout() {
    // R-1 on A must go to D by B and C; R-2 stands idle on B for ever
    let g = streets(false, true);
    let j = job(&g, 1, "A", "D");
    let mut w = world(g, &["A", "B"]);
    let mut c = Controller::new(&w);
    let done = run(&mut w, &mut c, vec![j], 120.0);

    assert_eq!(done.len(), 1, "the job is done");
    assert_eq!(c.reroutes(), 1);
    // it had to wait for the timeout first (the pick takes 5 s, the wait counts
    // from the end of it)
    assert!(done[0].1 > 5.0 + REROUTE_AFTER_SECONDS, "{done:?}");
    // R-1 ended on D, R-2 never moved
    let p = w.pose(&id(1)).unwrap();
    assert_eq!((p.x, p.y), (15.0, 0.0));
    let q = w.pose(&id(2)).unwrap();
    assert_eq!((q.x, q.y), (5.0, 0.0));
    assert!(w.collisions().is_empty());
}

#[test]
fn a_short_wait_is_not_re_routed() {
    // R-2 picks up on B (2 s) and drives off to G; R-1 waits for B for less than
    // the timeout
    let g = streets(false, true);
    let jobs = vec![job(&g, 1, "B", "G"), job(&g, 2, "A", "D")];
    let mut w = world_with(g, &["B", "A"], 2.0);
    let mut c = Controller::new(&w);
    let done = run(&mut w, &mut c, jobs, 120.0);
    assert_eq!(done.len(), 2);
    assert_eq!(c.reroutes(), 0);
}

#[test]
fn without_another_way_the_robot_keeps_waiting() {
    // no way round: the robot stays, nothing breaks, nothing is "re-routed"
    let g = streets(false, false);
    let j = job(&g, 1, "A", "D");
    let mut w = world(g, &["A", "B"]);
    let mut c = Controller::new(&w);
    let done = run(&mut w, &mut c, vec![j], 90.0);
    assert!(done.is_empty());
    assert_eq!(c.reroutes(), 0);
    assert_eq!(c.busy(), 1);
    assert!(w.collisions().is_empty());
}

#[test]
fn a_new_route_keeps_the_stops_and_actions_of_the_job() {
    // R-2 does a job on B (pick and drop there) and then stays on B for ever; R-1
    // on A has a job from C to D; the way to C is A → E → F → C
    let g = streets(true, true);
    let jobs = vec![job(&g, 1, "B", "B"), job(&g, 2, "C", "D")];
    let mut w = world(g, &["A", "B"]);
    let mut c = Controller::new(&w);
    let done = run(&mut w, &mut c, jobs, 120.0);
    assert_eq!(done.iter().filter(|(j, _)| *j == 2).count(), 1, "{done:?}");
    assert_eq!(c.reroutes(), 1);
    let p = w.pose(&id(1)).unwrap();
    assert_eq!((p.x, p.y), (15.0, 0.0));
    // the pick is the action of node 3 (A, E, F, C) of the new route, and it ran
    let s = w.state(&id(1)).unwrap();
    let status = |action: &str| {
        s.action_states
            .iter()
            .find(|a| a.action_id == action)
            .map(|a| a.action_status)
    };
    let ids: Vec<_> = s
        .action_states
        .iter()
        .map(|a| (&a.action_id, a.action_status))
        .collect();
    assert_eq!(status("pick-3"), Some(ActionStatus::Finished), "{ids:?}");
    assert_eq!(status("drop-4"), Some(ActionStatus::Finished), "{ids:?}");
}

#[test]
fn the_same_run_gives_the_same_result() {
    let once = || {
        let g = streets(true, true);
        let jobs = vec![job(&g, 1, "B", "B"), job(&g, 2, "C", "D")];
        let mut w = world(g, &["A", "B"]);
        let mut c = Controller::new(&w);
        run(&mut w, &mut c, jobs, 120.0)
    };
    assert_eq!(once(), once());
}

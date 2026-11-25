//! The baseline controller uses doors (S51).

use botsim_baseline::controller::Controller;
use botsim_core::actions::ActionTimes;
use botsim_core::door::DoorSpec;
use botsim_core::graph::Graph;
use botsim_core::jobs::Job;
use botsim_core::lift::DoorState;
use botsim_core::spec::RobotSpec;
use botsim_core::waiting::{Wait, WaitReason};
use botsim_core::world::{RobotId, World};
use vda5050::factsheet::Factsheet;

mod common;
use common::graph;

const DT: f64 = 0.05;

/// N0 → N1 → … → N5, 5 m apart, one way (N_i at x = 5 i).
fn chain() -> Graph {
    let nodes: Vec<(String, f64, f64)> = (0..6)
        .map(|i| (format!("N{i}"), 5.0 * i as f64, 0.0))
        .collect();
    let nodes: Vec<(&str, f64, f64)> = nodes.iter().map(|(n, x, y)| (n.as_str(), *x, *y)).collect();
    let edges: Vec<(String, String)> = (0..5)
        .map(|i| (format!("N{i}"), format!("N{}", i + 1)))
        .collect();
    let edges: Vec<(&str, &str)> = edges
        .iter()
        .map(|(a, b)| (a.as_str(), b.as_str()))
        .collect();
    graph(&nodes, &edges)
}

fn id(n: usize) -> RobotId {
    RobotId::new("ACME", &format!("R-{n}"))
}

/// Robots R-1, R-2, … on the given nodes; a door D1 on the edge N2-N3 (x from
/// 10 m to 15 m) with the given passage width.
fn world(starts: &[&str], passage_width: Option<f64>) -> World {
    world_with(starts, passage_width, 2.0)
}

/// Like [`world`], with the time the door needs to open.
fn world_with(starts: &[&str], passage_width: Option<f64>, open_seconds: f64) -> World {
    let g = chain();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let door_edge = g.edge_idx("N2-N3").unwrap();
    let nodes: Vec<_> = starts.iter().map(|n| g.node_idx(n).unwrap()).collect();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_door(DoorSpec {
        id: "D1".into(),
        edges: vec![door_edge],
        passage_width,
        open_seconds,
        close_seconds: 2.0,
        hold_seconds: None,
    })
    .unwrap();
    for (i, n) in nodes.into_iter().enumerate() {
        let times = ActionTimes::new(1.0).with("pick", 5.0).with("drop", 5.0);
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

/// Run the jobs for `ticks` cycles. Every step: a robot between N2 and N3 (on the
/// door's edge) must have the door open. Returns the number of steps a robot
/// was on the door edge.
fn run(w: &mut World, c: &mut Controller, jobs: Vec<Job>, ticks: u64) -> usize {
    let mut jobs = Some(jobs);
    let mut on_edge = 0;
    for _ in 0..ticks {
        c.step(w, jobs.take().unwrap_or_default());
        w.step();
        for n in 1..=2 {
            let x = w.pose(&id(n)).map(|p| p.x);
            if let Ok(x) = x
                && x > 10.0 + 1e-6
                && x < 15.0 - 1e-6
                && w.pose(&id(n)).unwrap().y == 0.0
            {
                on_edge += 1;
                assert_eq!(
                    w.door_status("D1").unwrap().state,
                    DoorState::Open,
                    "robot R-{n} on the door edge at x = {x}"
                );
            }
        }
    }
    on_edge
}

#[test]
fn a_robot_goes_through_a_door_for_a_job_and_the_door_is_closed_afterwards() {
    let g = chain();
    let mut w = world(&["N0"], None);
    let mut c = Controller::new(&w);
    let on_edge = run(&mut w, &mut c, vec![job(&g, 1, "N0", "N5")], 5000);
    assert_eq!(c.completed().len(), 1);
    assert!(on_edge > 0, "the robot drove through the door edge");
    assert!(w.collisions().is_empty());
    // released after the robot had passed: closed, nobody wants it
    let st = w.door_status("D1").unwrap();
    assert_eq!((st.state, st.wanted_open), (DoorState::Closed, false));
}

#[test]
fn the_robot_waits_for_the_door_and_the_job_takes_that_long() {
    // the same job without a door and with one: the door costs the opening time
    // (the controller asks for it when the robot is two nodes away, so most of
    // the 2 s overlap with the drive; a longer opening time shows in the result)
    let g = chain();
    let done = |open: f64| {
        let mut w = world_with(&["N0"], None, open);
        let mut c = Controller::new(&w);
        run_no_check(&mut w, &mut c, vec![job(&g, 1, "N0", "N5")], 6000);
        c.completed()[0].finished_tick
    };
    let (fast, slow) = (done(0.0), done(20.0));
    assert!(
        slow > fast + 200,
        "a 20 s door should cost about 20 s (400 steps) more: fast {fast}, slow {slow}"
    );
}

fn run_no_check(w: &mut World, c: &mut Controller, jobs: Vec<Job>, ticks: u64) {
    let mut jobs = Some(jobs);
    for _ in 0..ticks {
        c.step(w, jobs.take().unwrap_or_default());
        w.step();
    }
}

#[test]
fn two_robots_take_turns_at_the_door() {
    // R-2 is one node behind R-1; both go to N5 (R-2's drop is N4 so that they do
    // not need the same node at the end)
    let g = chain();
    let mut w = world(&["N0", "N1"], None);
    let mut c = Controller::new(&w);
    let jobs = vec![job(&g, 1, "N1", "N5"), job(&g, 2, "N0", "N4")];
    let on_edge = run(&mut w, &mut c, jobs, 12000);
    assert_eq!(c.completed().len(), 2, "{:?}", c.rejected());
    assert!(on_edge > 0);
    assert!(w.collisions().is_empty());
    assert!(w.deadlocks().is_empty());
    assert_eq!(w.door_status("D1").unwrap().state, DoorState::Closed);
}

#[test]
fn a_robot_wider_than_the_door_has_its_job_refused() {
    // the robot is 0.8 m wide
    let g = chain();
    let mut w = world(&["N0"], Some(0.7));
    let mut c = Controller::new(&w);
    run_no_check(&mut w, &mut c, vec![job(&g, 1, "N0", "N5")], 200);
    assert_eq!(c.rejected().len(), 1);
    assert!(c.completed().is_empty());
    assert_eq!(c.busy(), 0);

    // a job that does not use the door is fine
    let mut w = world(&["N0"], Some(0.7));
    let mut c = Controller::new(&w);
    run_no_check(&mut w, &mut c, vec![job(&g, 1, "N0", "N2")], 3000);
    assert_eq!(c.completed().len(), 1);
}

#[test]
fn with_delayed_messages_a_refused_job_is_given_up_when_the_state_shows_the_refusal() {
    // the order reaches the robot 0.5 s later and the refusal comes back 0.5 s
    // after that, in the state; the controller has to learn it from there
    let g = chain();
    let mut w = world(&["N0"], Some(0.7));
    w.set_message_delay(10);
    let mut c = Controller::new(&w);
    let mut jobs = Some(vec![job(&g, 1, "N0", "N5")]);
    c.step(&mut w, jobs.take().unwrap());
    assert_eq!(c.busy(), 1, "the controller cannot know yet");
    for _ in 0..40 {
        w.step();
        c.step(&mut w, vec![]);
    }
    assert_eq!(c.rejected().len(), 1);
    assert_eq!(c.busy(), 0);
    // what had been reserved for the job is free again: the next job goes through
    c.step(&mut w, vec![job(&g, 2, "N0", "N2")]);
    for _ in 0..4000 {
        w.step();
        c.step(&mut w, vec![]);
    }
    assert_eq!(c.completed().len(), 1);
    assert_eq!(c.rejected().len(), 1);
}

#[test]
fn a_door_works_with_delayed_messages() {
    // `run` checks at every step that the door is open while a robot is on its edge
    let g = chain();
    let mut w = world(&["N0", "N1"], None);
    w.set_message_delay(10);
    let mut c = Controller::new(&w);
    let jobs = vec![job(&g, 1, "N1", "N5"), job(&g, 2, "N0", "N4")];
    let on_edge = run(&mut w, &mut c, jobs, 15000);
    assert_eq!(c.completed().len(), 2, "{:?}", c.rejected());
    assert!(on_edge > 0);
    assert!(w.collisions().is_empty());
    assert_eq!(w.door_status("D1").unwrap().state, DoorState::Closed);
}

#[test]
fn the_same_run_gives_the_same_result() {
    let run_once = || {
        let g = chain();
        let mut w = world(&["N0", "N1"], None);
        let mut c = Controller::new(&w);
        let jobs = vec![job(&g, 1, "N1", "N5"), job(&g, 2, "N0", "N4")];
        run_no_check(&mut w, &mut c, jobs, 12000);
        c.completed()
            .iter()
            .map(|d| (d.job.id, d.finished_tick, d.waiting_seconds.to_bits()))
            .collect::<Vec<_>>()
    };
    assert_eq!(run_once(), run_once());
}

#[test]
fn robots_going_opposite_ways_through_a_door_in_a_one_lane_corridor_do_not_pass() {
    // N0 ↔ N1 ↔ N2 ↔ N3, 5 m apart, two-way; the door guards N1-N2 and N2-N1.
    // R-1 goes N0 → N3, R-2 goes N3 → N0: each stands on the node the other needs.
    // The baseline does not resolve this (the layout needs a side space, as at a lift).
    let g = graph(
        &[
            ("N0", 0.0, 0.0),
            ("N1", 5.0, 0.0),
            ("N2", 10.0, 0.0),
            ("N3", 15.0, 0.0),
        ],
        &[
            ("N0", "N1"),
            ("N1", "N0"),
            ("N1", "N2"),
            ("N2", "N1"),
            ("N2", "N3"),
            ("N3", "N2"),
        ],
    );
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let edges = vec![g.edge_idx("N1-N2").unwrap(), g.edge_idx("N2-N1").unwrap()];
    let starts = [g.node_idx("N0").unwrap(), g.node_idx("N3").unwrap()];
    let jobs = vec![job(&g, 1, "N0", "N3"), job(&g, 2, "N3", "N0")];
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_door(DoorSpec {
        id: "D1".into(),
        edges,
        passage_width: None,
        open_seconds: 2.0,
        close_seconds: 2.0,
        hold_seconds: None,
    })
    .unwrap();
    for (i, n) in starts.into_iter().enumerate() {
        let times = ActionTimes::new(1.0).with("pick", 5.0).with("drop", 5.0);
        w.add_robot(id(i + 1), spec.clone(), times, n, 0.0).unwrap();
    }
    let mut c = Controller::new(&w);
    run_no_check(&mut w, &mut c, jobs, 20000);
    assert!(c.completed().len() < 2, "{} done", c.completed().len());
    assert!(w.collisions().is_empty());
}

#[test]
fn a_robot_that_waits_for_a_door_names_its_owner() {
    // one door on two roads side by side: A → B and C → D, 20 m each
    let g = graph(
        &[
            ("A", 0.0, 0.0),
            ("B", 20.0, 0.0),
            ("C", 0.0, 5.0),
            ("D", 20.0, 5.0),
        ],
        &[("A", "B"), ("C", "D")],
    );
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let edges = vec![g.edge_idx("A-B").unwrap(), g.edge_idx("C-D").unwrap()];
    let starts = [g.node_idx("A").unwrap(), g.node_idx("C").unwrap()];
    let jobs = vec![job(&g, 1, "A", "B"), job(&g, 2, "C", "D")];
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_door(DoorSpec {
        id: "D1".into(),
        edges,
        passage_width: None,
        open_seconds: 2.0,
        close_seconds: 2.0,
        hold_seconds: None,
    })
    .unwrap();
    for (i, n) in starts.into_iter().enumerate() {
        let times = ActionTimes::new(1.0).with("pick", 1.0).with("drop", 1.0);
        w.add_robot(id(i + 1), spec.clone(), times, n, 0.0).unwrap();
    }
    let mut c = Controller::new(&w);
    let mut jobs = Some(jobs);
    let for_the_door = Wait {
        reason: WaitReason::Door,
        blocker: Some(0),
    };
    let mut named_the_owner = false;
    for _ in 0..4000 {
        c.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
        named_the_owner |= w.waits().get(1) == Some(&Some(for_the_door));
    }
    assert!(named_the_owner, "R-2 waits for the door R-1 owns");
    assert_eq!(c.completed().len(), 2, "{:?}", c.rejected());
}

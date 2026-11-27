use std::cell::RefCell;
use std::rc::Rc;

use botsim_baseline::controller::Controller;
use botsim_core::actions::ActionTimes;
use botsim_core::jobs::Job;
use botsim_core::observer::{Blocking, Observer, Snapshot};
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;

mod common;
use common::graph;

const DT: f64 = 0.05;

#[derive(Debug)]
struct Recorder(Rc<RefCell<Vec<Snapshot>>>);

impl Observer for Recorder {
    fn on_layout(&mut self, _: &Graph) {}

    fn on_step(&mut self, s: &Snapshot) {
        self.0.borrow_mut().push(s.clone());
    }
}

/// A two-way corridor A – B – C.
fn corridor() -> Graph {
    graph(
        &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("C", 20.0, 0.0)],
        &[("A", "B"), ("B", "A"), ("B", "C"), ("C", "B")],
    )
}

fn job(g: &Graph, id: u64, pickup: &str, drop: &str) -> Job {
    Job {
        id,
        pickup: g.node_idx(pickup).unwrap(),
        drop: g.node_idx(drop).unwrap(),
        created_tick: 0,
    }
}

/// R-1 on A and R-2 on C: A → C and C → A at once, so they meet head-on.
fn head_on(ticks: u64) -> Vec<Snapshot> {
    let g = corridor();
    let jobs = vec![job(&g, 1, "A", "C"), job(&g, 2, "C", "A")];
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let starts: Vec<_> = ["A", "C"].iter().map(|n| g.node_idx(n).unwrap()).collect();
    let mut w = World::new(g, DT, 0).unwrap();
    for (i, n) in starts.into_iter().enumerate() {
        let id = RobotId::new("ACME", &format!("R-{}", i + 1));
        w.add_robot(
            id,
            spec.clone(),
            ActionTimes::new(1.0).with("pick", 5.0).with("drop", 5.0),
            n,
            0.0,
        )
        .unwrap();
    }
    let log = Rc::new(RefCell::new(Vec::new()));
    w.set_observer(Box::new(Recorder(log.clone())));
    let mut c = Controller::new(&w);
    let mut jobs = Some(jobs);
    for _ in 0..ticks {
        c.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    log.take()
}

#[test]
fn the_controller_publishes_its_reservations_and_they_agree_with_the_routes() {
    let snaps = head_on(1500);
    assert_eq!(snaps.len(), 1500);
    // from the first cycle on, both robots hold at least the node they stand on
    assert!(snaps[0].reservations.iter().all(|r| !r.nodes.is_empty()));
    assert_eq!(snaps[0].reservations.len(), 2);

    for s in &snaps {
        // nothing is reserved twice
        let mut nodes: Vec<_> = s
            .reservations
            .iter()
            .flat_map(|r| r.nodes.clone())
            .collect();
        let mut edges: Vec<_> = s
            .reservations
            .iter()
            .flat_map(|r| r.edges.clone())
            .collect();
        let (n, e) = (nodes.len(), edges.len());
        nodes.sort_unstable();
        nodes.dedup();
        edges.sort_unstable();
        edges.dedup();
        assert_eq!((nodes.len(), edges.len()), (n, e), "tick {}", s.tick);

        // what is released to a robot is reserved for it
        for (i, r) in s.robots.iter().enumerate() {
            let held = s.reservations.iter().find(|x| x.robot == i);
            for node in &r.released_nodes {
                assert!(
                    held.is_some_and(|h| h.nodes.contains(node)),
                    "tick {}: robot {i} may drive to node {node} without holding it",
                    s.tick
                );
            }
            for edge in &r.released_edges {
                assert!(held.is_some_and(|h| h.edges.contains(edge)));
            }
        }
    }
}

#[test]
fn head_on_robots_show_blocking_in_both_directions_and_a_deadlock() {
    let snaps = head_on(30_000);
    let last = snaps.last().unwrap();
    // R-1 waits for R-2 and R-2 for R-1, and the cycle is a reported deadlock
    assert!(last.blocking.contains(&Blocking {
        waiter: 0,
        blocker: 1
    }));
    assert!(last.blocking.contains(&Blocking {
        waiter: 1,
        blocker: 0
    }));
    assert_eq!(last.deadlocks, vec![vec![0, 1]]);
    // they never overlap
    assert!(snaps.iter().all(|s| s.overlapping.is_empty()));
    // the deadlock is not there from the start: it forms first
    assert!(snaps[0].deadlocks.is_empty() && snaps[0].blocking.is_empty());
}

#[test]
fn the_same_run_gives_the_same_snapshots_with_a_controller() {
    assert_eq!(head_on(3000), head_on(3000));
}

//! The fast paths added for scale (S52) must give the answers of the slow ones.

use botsim::connect;
use botsim_model::graph::Graph;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_model::time::Millis;
use botsim_server::node_grid::NodeGrid;
use botsim_sim::actions::ActionTimes;
use botsim_sim::jobs::Rng;
use botsim_sim::world::World;
use vda5050::factsheet::Factsheet;

#[path = "../../baseline/tests/common/mod.rs"]
mod common;
use common::graph;

/// Nodes at random places on a 12 m × 12 m square, some very close together
/// (equal distances and ties); edges at random between them.
fn random_graph(seed: u64) -> Graph {
    let mut rng = Rng::new(seed);
    let mut nodes: Vec<(String, f64, f64)> = Vec::new();
    for i in 0..40 {
        let (x, y) = (rng.next_f64() * 12.0, rng.next_f64() * 12.0);
        nodes.push((format!("N{i}"), x, y));
        if i % 7 == 0 {
            // a second node 5 cm away, and one exactly on top
            nodes.push((format!("N{i}b"), x + 0.05, y));
            nodes.push((format!("N{i}c"), x, y));
        }
    }
    let mut edges: Vec<(String, String)> = Vec::new();
    for _ in 0..90 {
        let (a, b) = (rng.below(nodes.len()), rng.below(nodes.len()));
        let e = (nodes[a].0.clone(), nodes[b].0.clone());
        if a != b && !edges.contains(&e) {
            edges.push(e);
        }
    }
    let n: Vec<(&str, f64, f64)> = nodes.iter().map(|(s, x, y)| (s.as_str(), *x, *y)).collect();
    let e: Vec<(&str, &str)> = edges
        .iter()
        .map(|(a, b)| (a.as_str(), b.as_str()))
        .collect();
    graph(&n, &e)
}

#[test]
fn the_node_grid_gives_the_same_node_as_scanning_every_node() {
    const TOLERANCE: f64 = 0.1;
    for seed in 1..=5 {
        let g = random_graph(seed);
        let grid = NodeGrid::new(&g);
        let mut rng = Rng::new(seed + 100);
        // positions: exactly on nodes, near nodes (inside and outside the
        // tolerance, on the border of a cell), and anywhere
        let mut queries: Vec<(f64, f64)> = Vec::new();
        for n in &g.nodes {
            queries.push((n.x, n.y));
            queries.push((n.x + TOLERANCE, n.y)); // on the border of the tolerance
            queries.push((n.x - 0.07, n.y + 0.07));
            queries.push((n.x + 0.12, n.y));
        }
        for _ in 0..300 {
            queries.push((rng.next_f64() * 13.0 - 0.5, rng.next_f64() * 13.0 - 0.5));
        }
        for (x, y) in queries {
            let scan = g
                .nodes
                .iter()
                .enumerate()
                .filter(|(_, n)| n.map == 0)
                .map(|(k, n)| (k, (n.x - x).hypot(n.y - y)))
                .filter(|&(_, d)| d <= TOLERANCE)
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|(k, _)| k);
            assert_eq!(grid.node_near(0, x, y, TOLERANCE), scan, "at ({x}, {y})");
        }
        // another map has no nodes here
        assert_eq!(
            grid.node_near(1, g.nodes[0].x, g.nodes[0].y, TOLERANCE),
            None
        );
    }
}

/// N0 → N1 → … → N4, 5 m apart, one way: nothing leads back.
fn chain() -> Graph {
    graph(
        &[
            ("N0", 0.0, 0.0),
            ("N1", 5.0, 0.0),
            ("N2", 10.0, 0.0),
            ("N3", 15.0, 0.0),
            ("N4", 20.0, 0.0),
        ],
        &[("N0", "N1"), ("N1", "N2"), ("N2", "N3"), ("N3", "N4")],
    )
}

fn job(g: &Graph, id: u64, pickup: &str, drop: &str) -> Job {
    Job {
        id,
        pickup: g.node_idx(pickup).unwrap(),
        drop: g.node_idx(drop).unwrap(),
        created: Millis(0),
    }
}

#[test]
fn a_job_that_cannot_be_done_waits_and_does_not_hold_up_the_others() {
    let g = chain();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let n0 = g.node_idx("N0").unwrap();
    // job 1 needs a way back (N3 → N1): there is none; jobs 2 and 3 are fine
    let jobs = vec![
        job(&g, 1, "N3", "N1"),
        job(&g, 2, "N1", "N3"),
        job(&g, 3, "N0", "N2"),
    ];
    let mut w = World::new(g, 0.05, 0).unwrap();
    let times = ActionTimes::new(1.0).with("pick", 5.0).with("drop", 5.0);
    w.add_robot(
        RobotId::new("ACME", "R-1"),
        spec.clone(),
        times.clone(),
        n0,
        0.0,
    )
    .unwrap();
    let n1 = w.graph().node_idx("N1").unwrap();
    w.add_robot(RobotId::new("ACME", "R-2"), spec, times, n1, 0.0)
        .unwrap();
    let mut c = connect::controller(&w);
    let mut jobs = Some(jobs);
    for _ in 0..6000 {
        connect::step(&mut c, &mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    let done: Vec<u64> = c.completed().iter().map(|d| d.job.id).collect();
    assert!(done.contains(&2) && done.contains(&3), "{done:?}");
    // the impossible job is still waiting, not refused, not lost
    let waiting: Vec<u64> = c.pending().map(|j| j.id).collect();
    assert_eq!(waiting, vec![1]);
    assert!(c.rejected().is_empty());
}

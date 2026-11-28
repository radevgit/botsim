//! The search backwards from a target (S52) must give the lengths of the searches
//! from every node.

use botsim_model::graph::Graph;
use botsim_router::route::{distances_to, in_edges, shortest_path, straight_lengths};

mod common;
use common::graph;

/// SplitMix64, the generator of the simulator's scenarios, for the random layouts.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e3779b97f4a7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Uniform in 0..n (`n` > 0).
    fn below(&mut self, n: usize) -> usize {
        ((u128::from(self.next_u64()) * n as u128) >> 64) as usize
    }
}

/// Nodes at random places on a 12 m × 12 m square, some very close together
/// (equal distances and ties); edges at random between them.
fn random_graph(seed: u64) -> Graph {
    let mut rng = Rng(seed);
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
fn distances_to_a_node_are_the_lengths_of_the_shortest_routes() {
    for seed in 1..=5 {
        let g = random_graph(seed);
        let into = in_edges(&g, &vec![true; g.edges.len()]);
        for target in 0..g.nodes.len() {
            let dist = distances_to(&g, &into, &straight_lengths(&g), target);
            for (from, &d) in dist.iter().enumerate() {
                match shortest_path(&g, from, target) {
                    None => assert!(d.is_infinite(), "seed {seed}: {from} → {target}"),
                    Some(p) => {
                        let len: f64 = p.iter().map(|&e| g.edges[e].length).sum();
                        assert!(
                            (d - len).abs() < 1e-9,
                            "seed {seed}: {from} → {target}: {d} against {len}"
                        );
                    }
                }
            }
        }
    }
}

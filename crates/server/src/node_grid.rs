//! Which layout node a position is on, without scanning every node (S52).
//!
//! The nodes are put in a grid of square cells once; a query looks at the 3 × 3
//! cells around the position. The answer is the same as scanning all nodes: the
//! nearest node within the tolerance, the one with the lower index if two are
//! equally near.

use std::collections::HashMap;

use botsim_model::graph::{Graph, NodeIdx};

/// Cell size in metres; it must be at least the largest tolerance asked for.
const CELL: f64 = 1.0;

#[derive(Debug)]
pub struct NodeGrid {
    cells: HashMap<(usize, i64, i64), Vec<NodeIdx>>,
    /// Position of every node, by index.
    at: Vec<(usize, f64, f64)>,
}

fn cell(v: f64) -> i64 {
    (v / CELL).floor() as i64
}

impl NodeGrid {
    pub fn new(graph: &Graph) -> NodeGrid {
        let mut cells: HashMap<(usize, i64, i64), Vec<NodeIdx>> = HashMap::new();
        let mut at = Vec::with_capacity(graph.nodes.len());
        for (i, n) in graph.nodes.iter().enumerate() {
            cells
                .entry((n.map, cell(n.x), cell(n.y)))
                .or_default()
                .push(i);
            at.push((n.map, n.x, n.y));
        }
        NodeGrid { cells, at }
    }

    /// The node nearest to (`x`, `y`) on `map` within `tolerance` metres
    /// (at most one cell, 1 m).
    pub fn node_near(&self, map: usize, x: f64, y: f64, tolerance: f64) -> Option<NodeIdx> {
        debug_assert!(tolerance <= CELL);
        let (cx, cy) = (cell(x), cell(y));
        let mut best: Option<(f64, NodeIdx)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for &n in self
                    .cells
                    .get(&(map, cx + dx, cy + dy))
                    .into_iter()
                    .flatten()
                {
                    let (_, nx, ny) = self.at[n];
                    let d = (nx - x).hypot(ny - y);
                    if d <= tolerance && best.is_none_or(|(bd, bn)| d < bd || (d == bd && n < bn)) {
                        best = Some((d, n));
                    }
                }
            }
        }
        best.map(|(_, n)| n)
    }
}

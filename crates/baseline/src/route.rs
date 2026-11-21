//! Shortest route between two nodes (Dijkstra on edge length).
//!
//! Edges are directed. Ties are broken by node index, so the same layout always
//! gives the same route. Not yet: per-robot-type edge rules (a type may not be
//! allowed on an edge), blocked edges, and lift edges between maps (they have
//! length 0 for now).

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use botsim_core::graph::{EdgeIdx, Graph, NodeIdx};

/// For every node, the edges that lead into it (for [`distances_to`]).
pub fn in_edges(graph: &Graph) -> Vec<Vec<EdgeIdx>> {
    let mut into = vec![Vec::new(); graph.nodes.len()];
    for (i, e) in graph.edges.iter().enumerate() {
        into[e.end].push(i);
    }
    into
}

/// The route length from **every** node to `target` (infinite if it cannot be
/// reached), found with one Dijkstra search backwards from `target`. `into` comes
/// from [`in_edges`]. It gives the same lengths as [`shortest_path`] called for
/// every node, but costs one search instead of one per node.
pub fn distances_to(graph: &Graph, into: &[Vec<EdgeIdx>], target: NodeIdx) -> Vec<f64> {
    let mut dist = vec![f64::INFINITY; graph.nodes.len()];
    let mut heap = BinaryHeap::new();
    dist[target] = 0.0;
    heap.push(Reverse((0.0_f64.to_bits(), target)));
    while let Some(Reverse((bits, node))) = heap.pop() {
        if f64::from_bits(bits) > dist[node] {
            continue;
        }
        for &e in &into[node] {
            let edge = &graph.edges[e];
            let d = dist[node] + edge.length;
            if d < dist[edge.start] {
                dist[edge.start] = d;
                heap.push(Reverse((d.to_bits(), edge.start)));
            }
        }
    }
    dist
}

/// The edges to drive from `from` to `to`; empty if they are the same node;
/// `None` if `to` cannot be reached.
pub fn shortest_path(graph: &Graph, from: NodeIdx, to: NodeIdx) -> Option<Vec<EdgeIdx>> {
    let n = graph.nodes.len();
    let mut dist = vec![f64::INFINITY; n];
    let mut via: Vec<Option<EdgeIdx>> = vec![None; n];
    // The heap key is the distance as bits: for non-negative finite floats the
    // bit pattern sorts like the number, and it gives a total order.
    let mut heap = BinaryHeap::new();
    dist[from] = 0.0;
    heap.push(Reverse((0.0_f64.to_bits(), from)));

    while let Some(Reverse((bits, node))) = heap.pop() {
        if f64::from_bits(bits) > dist[node] {
            continue; // an older, longer entry
        }
        if node == to {
            break;
        }
        for &e in graph.out_edges(node) {
            let edge = &graph.edges[e];
            let d = dist[node] + edge.length;
            if d < dist[edge.end] {
                dist[edge.end] = d;
                via[edge.end] = Some(e);
                heap.push(Reverse((d.to_bits(), edge.end)));
            }
        }
    }

    if dist[to].is_infinite() {
        return None;
    }
    let mut path = Vec::new();
    let mut at = to;
    while at != from {
        let e = via[at].expect("a reached node has an incoming edge");
        path.push(e);
        at = graph.edges[e].start;
    }
    path.reverse();
    Some(path)
}

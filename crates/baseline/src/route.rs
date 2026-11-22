//! Shortest route between two nodes (Dijkstra on edge length).
//!
//! Edges are directed. Ties are broken by node index, so the same layout always
//! gives the same route. Not yet: per-robot-type edge rules (a type may not be
//! allowed on an edge), blocked edges, and lift edges between maps (they have
//! length 0 for now).

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use botsim_core::graph::{EdgeIdx, Graph, NodeIdx, Rotation};
use botsim_core::spec::Kinematics;

/// How a robot gets to the node where a route starts (S73): it matters for the first
/// edge, if the robot cannot turn there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Arrival {
    /// Not known, or it does not matter: the first edge is not checked.
    Free,
    /// It has just driven this edge.
    Edge(EdgeIdx),
    /// It stands on the node and heads this way (radians).
    Heading(f64),
}

/// The turns a robot type cannot make (S73): a three-wheel robot cannot turn on the
/// spot, and a differential robot may be barred from turning at some nodes. The
/// rules are the layout's (`Graph::can_follow`, `can_start`).
#[derive(Debug, Clone)]
pub struct Turns {
    kinematics: Kinematics,
    manufacturer: String,
    series: String,
}

impl Turns {
    /// The turn rules of a robot type in this layout; `None` if the type is not
    /// restricted anywhere (an omnidirectional robot, or a differential robot where
    /// the layout bars no rotation), so a plain search by node is enough.
    pub fn for_type(
        graph: &Graph,
        kinematics: Kinematics,
        manufacturer: &str,
        series: &str,
    ) -> Option<Turns> {
        let restricted = match kinematics {
            Kinematics::Omnidirectional => false,
            Kinematics::ThreeWheel => true,
            Kinematics::Differential => (0..graph.edges.len()).any(|e| {
                graph.edge_allows(e, manufacturer, series) && {
                    let (start, end) = graph.edge_rotation(e, manufacturer, series);
                    start != Rotation::Both || end != Rotation::Both
                }
            }),
        };
        restricted.then(|| Turns {
            kinematics,
            manufacturer: manufacturer.to_string(),
            series: series.to_string(),
        })
    }

    /// May the robot drive `next` after arriving the way `arrival` says?
    fn allows(&self, graph: &Graph, arrival: Arrival, next: EdgeIdx) -> bool {
        let (k, m, s) = (self.kinematics, &self.manufacturer, &self.series);
        match arrival {
            Arrival::Free => true,
            Arrival::Edge(prev) => graph.can_follow(prev, next, k, m, s),
            Arrival::Heading(h) => graph.can_start(h, None, next, k, m, s),
        }
    }
}

/// What a robot type can drive: the edges its type may use (S55) and, if it is
/// restricted, its turns (S73).
#[derive(Debug, Clone, Copy)]
pub struct Mover<'a> {
    pub ok: &'a [bool],
    /// What it costs the robot to drive each edge: the distance it drives on it
    /// (S74; the length of the curve for an edge with a trajectory).
    pub cost: &'a [f64],
    pub turns: Option<&'a Turns>,
}

/// The distance a robot of this type drives on each edge (S74): the length of the
/// curve where the edge has a trajectory for the type, else the straight distance.
pub fn travel_lengths(graph: &Graph, manufacturer: &str, series: &str) -> Vec<f64> {
    (0..graph.edges.len())
        .map(|e| graph.edge_travel_length(e, manufacturer, series))
        .collect()
}

/// The straight distance between the nodes of every edge (no curves).
pub fn straight_lengths(graph: &Graph) -> Vec<f64> {
    graph.edges.iter().map(|e| e.length).collect()
}

/// The shortest route for a robot of this kind from `from` to `to`, arriving at
/// `from` the way `arrival` says; it may not enter a node in `blocked` or use an edge
/// for which `forbidden` is true. Without turn rules it is
/// [`shortest_path_avoiding`] on the edges the type may drive; with them the search
/// goes over edges (the one just driven decides the next), so a route that needs a
/// turn the robot cannot make is not found. `None` if there is no route.
pub fn plan_path(
    graph: &Graph,
    mover: &Mover,
    arrival: Arrival,
    from: NodeIdx,
    to: NodeIdx,
    blocked: &std::collections::HashSet<NodeIdx>,
    forbidden: impl Fn(EdgeIdx) -> bool,
) -> Option<Vec<EdgeIdx>> {
    let Some(turns) = mover.turns else {
        return shortest_path_avoiding(graph, from, to, blocked, mover.cost, |e| {
            !mover.ok[e] || forbidden(e)
        });
    };
    if from == to {
        return Some(Vec::new());
    }
    if blocked.contains(&to) {
        return None;
    }
    let usable =
        |e: EdgeIdx| mover.ok[e] && !forbidden(e) && !blocked.contains(&graph.edges[e].end);
    // dist[e]: the shortest way to have just driven edge e; via[e]: the edge before
    let mut dist = vec![f64::INFINITY; graph.edges.len()];
    let mut via: Vec<Option<EdgeIdx>> = vec![None; graph.edges.len()];
    let mut heap = BinaryHeap::new();
    for &e in graph.out_edges(from) {
        if usable(e) && turns.allows(graph, arrival, e) {
            dist[e] = mover.cost[e];
            heap.push(Reverse((dist[e].to_bits(), e)));
        }
    }
    let mut last = None;
    while let Some(Reverse((bits, e))) = heap.pop() {
        if f64::from_bits(bits) > dist[e] {
            continue;
        }
        let end = graph.edges[e].end;
        if end == to {
            last = Some(e);
            break;
        }
        for &next in graph.out_edges(end) {
            if !usable(next) || !turns.allows(graph, Arrival::Edge(e), next) {
                continue;
            }
            let d = dist[e] + mover.cost[next];
            if d < dist[next] {
                dist[next] = d;
                via[next] = Some(e);
                heap.push(Reverse((d.to_bits(), next)));
            }
        }
    }
    let mut path = vec![last?];
    while let Some(e) = via[*path.last()?] {
        path.push(e);
    }
    path.reverse();
    Some(path)
}

/// Like [`shortest_path`], but the route may not enter a node in `blocked` (the
/// start node may be in it) and may not use an edge for which `forbidden` is
/// true; `cost[e]` is the cost of driving edge `e` (S74). `None` if there is no
/// such route, or `to` itself is blocked.
pub fn shortest_path_avoiding(
    graph: &Graph,
    from: NodeIdx,
    to: NodeIdx,
    blocked: &std::collections::HashSet<NodeIdx>,
    cost: &[f64],
    forbidden: impl Fn(EdgeIdx) -> bool,
) -> Option<Vec<EdgeIdx>> {
    if from != to && blocked.contains(&to) {
        return None;
    }
    let n = graph.nodes.len();
    let mut dist = vec![f64::INFINITY; n];
    let mut via: Vec<Option<EdgeIdx>> = vec![None; n];
    let mut heap = BinaryHeap::new();
    dist[from] = 0.0;
    heap.push(Reverse((0.0_f64.to_bits(), from)));
    while let Some(Reverse((bits, node))) = heap.pop() {
        if f64::from_bits(bits) > dist[node] {
            continue;
        }
        if node == to {
            break;
        }
        for &e in graph.out_edges(node) {
            let edge = &graph.edges[e];
            if blocked.contains(&edge.end) || forbidden(e) {
                continue;
            }
            let d = dist[node] + cost[e];
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
        let e = via[at]?; // (a reached node has an incoming edge)
        path.push(e);
        at = graph.edges[e].start;
    }
    path.reverse();
    Some(path)
}

/// For every node, the edges that lead into it (for [`distances_to`]); only the
/// edges for which `ok[e]` is true (the ones a robot type may drive, S55).
pub fn in_edges(graph: &Graph, ok: &[bool]) -> Vec<Vec<EdgeIdx>> {
    let mut into = vec![Vec::new(); graph.nodes.len()];
    for (i, e) in graph.edges.iter().enumerate() {
        if ok[i] {
            into[e.end].push(i);
        }
    }
    into
}

/// The route length from **every** node to `target` (infinite if it cannot be
/// reached), found with one Dijkstra search backwards from `target`. `into` comes
/// from [`in_edges`], `cost[e]` is the cost of edge `e` ([`travel_lengths`] or
/// [`straight_lengths`]). It gives the same lengths as [`shortest_path`] called for
/// every node (with straight lengths), but costs one search instead of one per node.
pub fn distances_to(
    graph: &Graph,
    into: &[Vec<EdgeIdx>],
    cost: &[f64],
    target: NodeIdx,
) -> Vec<f64> {
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
            let d = dist[node] + cost[e];
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
        let e = via[at]?; // (a reached node has an incoming edge)
        path.push(e);
        at = graph.edges[e].start;
    }
    path.reverse();
    Some(path)
}

//! Who holds which node and edge (S34).
//!
//! A node or an edge has at most one holder (a robot index). The baseline
//! controller reserves them when it releases them to a robot and gives them back
//! when the robot has driven on. Nothing here knows about routes or messages.

use botsim_core::graph::{EdgeIdx, Graph, NodeIdx};

#[derive(Debug, Clone)]
pub struct Reservations {
    nodes: Vec<Option<usize>>,
    edges: Vec<Option<usize>>,
}

impl Reservations {
    /// Nothing is held.
    pub fn new(graph: &Graph) -> Reservations {
        Reservations {
            nodes: vec![None; graph.nodes.len()],
            edges: vec![None; graph.edges.len()],
        }
    }

    pub fn node_holder(&self, n: NodeIdx) -> Option<usize> {
        self.nodes[n]
    }

    pub fn edge_holder(&self, e: EdgeIdx) -> Option<usize> {
        self.edges[e]
    }

    /// Is the node free, or already held by `robot`?
    pub fn node_available(&self, n: NodeIdx, robot: usize) -> bool {
        self.nodes[n].is_none_or(|h| h == robot)
    }

    pub fn edge_available(&self, e: EdgeIdx, robot: usize) -> bool {
        self.edges[e].is_none_or(|h| h == robot)
    }

    /// Hold the node for `robot`; false (and nothing changes) if another robot holds it.
    pub fn reserve_node(&mut self, n: NodeIdx, robot: usize) -> bool {
        if self.node_available(n, robot) {
            self.nodes[n] = Some(robot);
            true
        } else {
            false
        }
    }

    /// Hold an edge and the node it leads to, both or neither.
    pub fn reserve_step(&mut self, e: EdgeIdx, n: NodeIdx, robot: usize) -> bool {
        if self.edge_available(e, robot) && self.node_available(n, robot) {
            self.edges[e] = Some(robot);
            self.nodes[n] = Some(robot);
            true
        } else {
            false
        }
    }

    /// Give the node back, if `robot` holds it.
    pub fn release_node(&mut self, n: NodeIdx, robot: usize) {
        if self.nodes[n] == Some(robot) {
            self.nodes[n] = None;
        }
    }

    pub fn release_edge(&mut self, e: EdgeIdx, robot: usize) {
        if self.edges[e] == Some(robot) {
            self.edges[e] = None;
        }
    }

    /// How many nodes and edges `robot` holds.
    pub fn held_by(&self, robot: usize) -> (usize, usize) {
        let count = |v: &[Option<usize>]| v.iter().filter(|h| **h == Some(robot)).count();
        (count(&self.nodes), count(&self.edges))
    }
}

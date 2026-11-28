//! Who holds which node and edge (S34).
//!
//! A node or an edge has at most one holder (a robot index). The baseline
//! controller reserves them when it releases them to a robot and gives them back
//! when the robot has driven on. Nothing here knows about routes or messages.

use std::collections::{BTreeMap, HashMap};

use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_model::reservation::Reservation;

#[derive(Debug, Clone)]
pub struct Reservations {
    nodes: Vec<Option<usize>>,
    edges: Vec<Option<usize>>,
    /// Nodes that a robot took over from another robot that is stuck behind it
    /// (S77): the robot that held the node before. It gets the node back when the
    /// taker lets go of it.
    displaced: HashMap<NodeIdx, usize>,
    /// The same for edges.
    displaced_edges: HashMap<EdgeIdx, usize>,
}

impl Reservations {
    /// Nothing is held.
    pub fn new(graph: &Graph) -> Reservations {
        Reservations {
            nodes: vec![None; graph.nodes.len()],
            edges: vec![None; graph.edges.len()],
            displaced: HashMap::new(),
            displaced_edges: HashMap::new(),
        }
    }

    /// Like [`Reservations::take_over_node`], for an edge.
    pub fn take_over_edge(&mut self, e: EdgeIdx, robot: usize) -> Option<usize> {
        let previous = self.edges[e].filter(|&h| h != robot)?;
        self.edges[e] = Some(robot);
        self.displaced_edges.insert(e, previous);
        Some(previous)
    }

    /// `robot` takes the node over from its holder (S77): the holder is stuck
    /// behind `robot`, so `robot` has to go first. Returns the robot that held it;
    /// `None` (and nothing changes) if nobody else held it.
    pub fn take_over_node(&mut self, n: NodeIdx, robot: usize) -> Option<usize> {
        let previous = self.nodes[n].filter(|&h| h != robot)?;
        self.nodes[n] = Some(robot);
        self.displaced.insert(n, previous);
        Some(previous)
    }

    /// Did `robot` take the node over (and not yet let go of it)?
    pub fn is_taken_over_node(&self, n: NodeIdx, robot: usize) -> bool {
        self.nodes[n] == Some(robot) && self.displaced.contains_key(&n)
    }

    /// The same for an edge. An edge that was free when taken over has no
    /// displaced holder and is not marked, but nobody else wants it back.
    pub fn is_taken_over_edge(&self, e: EdgeIdx, robot: usize) -> bool {
        self.edges[e] == Some(robot) && self.displaced_edges.contains_key(&e)
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
            // a node taken over goes back to the robot it was taken from
            self.nodes[n] = self.displaced.remove(&n);
        } else if self.displaced.get(&n) == Some(&robot) {
            // the robot it was taken from has given it up itself
            self.displaced.remove(&n);
        }
    }

    pub fn release_edge(&mut self, e: EdgeIdx, robot: usize) {
        if self.edges[e] == Some(robot) {
            self.edges[e] = self.displaced_edges.remove(&e);
        } else if self.displaced_edges.get(&e) == Some(&robot) {
            self.displaced_edges.remove(&e);
        }
    }

    /// Everything held, per robot (robots that hold nothing are left out), for
    /// the observer port.
    pub fn to_observer(&self) -> Vec<Reservation> {
        let mut by_robot: BTreeMap<usize, Reservation> = BTreeMap::new();
        let new = |robot: usize| Reservation {
            robot,
            nodes: Vec::new(),
            edges: Vec::new(),
        };
        for (n, holder) in self.nodes.iter().enumerate() {
            if let Some(r) = holder {
                by_robot.entry(*r).or_insert_with(|| new(*r)).nodes.push(n);
            }
        }
        for (e, holder) in self.edges.iter().enumerate() {
            if let Some(r) = holder {
                by_robot.entry(*r).or_insert_with(|| new(*r)).edges.push(e);
            }
        }
        by_robot.into_values().collect()
    }

    /// How many nodes and edges `robot` holds.
    pub fn held_by(&self, robot: usize) -> (usize, usize) {
        let count = |v: &[Option<usize>]| v.iter().filter(|h| **h == Some(robot)).count();
        (count(&self.nodes), count(&self.edges))
    }
}

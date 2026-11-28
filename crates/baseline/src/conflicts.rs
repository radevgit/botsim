//! Swept-area reservation: edges whose swept outlines overlap.
//!
//! Two edges that cross, merge or run close have no node in common, so the node
//! and edge reservations do not stop two robots from using them at the same time.
//! A [`ConflictTable`] lists, per pair of robot types, the pairs of edges whose swept
//! outlines overlap (it is read from [`botsim_server::conflicts`]); the baseline
//! releases a robot onto an edge only if no other robot holds an edge that conflicts
//! with it.

use std::collections::HashMap;

use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_server::conflicts::{ConflictInput, NodeConflictInput, NodeNodeConflictInput};

use crate::reservations::Reservations;

/// What a robot holds that keeps another one off an edge (node-edge and node-node
/// conflicts).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Held {
    Node(NodeIdx),
    Edge(EdgeIdx),
}

/// The conflicts by edge index: `by_type[a][b][e]` are the edges a robot of type `b`
/// may not hold while a robot of type `a` is released onto `e`.
#[derive(Debug, Default)]
pub struct ConflictTable {
    by_type: Vec<Vec<HashMap<EdgeIdx, Vec<EdgeIdx>>>>,
    /// `node_edges[s][t][e]`: the nodes where a robot of type `s` may not stand
    /// while a robot of type `t` is released onto `e`.
    node_edges: Vec<Vec<HashMap<EdgeIdx, Vec<NodeIdx>>>>,
    /// `edge_nodes[s][t][n]`: the edges a robot of type `t` may not hold while a
    /// robot of type `s` is released to node `n`.
    edge_nodes: Vec<Vec<HashMap<NodeIdx, Vec<EdgeIdx>>>>,
    /// `near_nodes[t][s][n]`: the nodes a robot of type `s` may not be on while a
    /// robot of type `t` is released to node `n`.
    near_nodes: Vec<Vec<HashMap<NodeIdx, Vec<NodeIdx>>>>,
    /// End node of each edge.
    ends: Vec<NodeIdx>,
    /// Number of conflicting edge pairs, for reports.
    pairs: usize,
    /// Number of node-edge conflicts, for reports.
    node_pairs: usize,
    /// Number of node-node conflicts, for reports.
    node_node_pairs: usize,
}

impl ConflictTable {
    /// Read the input against the layout. `types` are the robot types of the
    /// controller as `manufacturer/seriesName`, in the order of their index. Edges and
    /// types the layout or the fleet do not have are left out.
    pub fn new(graph: &Graph, types: &[String], input: &[ConflictInput]) -> ConflictTable {
        let mut by_type = vec![vec![HashMap::new(); types.len()]; types.len()];
        let mut pairs = 0;
        let type_idx = |name: &str| types.iter().position(|t| t == name);
        let ids: HashMap<&str, EdgeIdx> = graph
            .edges
            .iter()
            .enumerate()
            .map(|(i, e)| (e.id.as_str(), i))
            .collect();
        for pair in input {
            let (Some(a), Some(b)) = (type_idx(&pair.type_a), type_idx(&pair.type_b)) else {
                continue;
            };
            for (ea, eb) in &pair.edges {
                let (Some(&x), Some(&y)) = (ids.get(ea.as_str()), ids.get(eb.as_str())) else {
                    continue;
                };
                // x is driven by a robot of type a, y by one of type b: a robot of
                // type a on x is blocked by type b on y, and the other way round
                by_type[a][b].entry(x).or_insert_with(Vec::new).push(y);
                by_type[b][a].entry(y).or_insert_with(Vec::new).push(x);
                pairs += 1;
            }
        }
        for row in &mut by_type {
            for map in row {
                for list in map.values_mut() {
                    list.sort_unstable();
                    list.dedup();
                }
            }
        }
        ConflictTable {
            by_type,
            node_edges: vec![vec![HashMap::new(); types.len()]; types.len()],
            edge_nodes: vec![vec![HashMap::new(); types.len()]; types.len()],
            near_nodes: vec![vec![HashMap::new(); types.len()]; types.len()],
            ends: graph.edges.iter().map(|e| e.end).collect(),
            pairs,
            node_pairs: 0,
            node_node_pairs: 0,
        }
    }

    /// Add the node-edge conflicts (a robot standing on a node against an edge
    /// another robot drives). Returns the number read; ids the layout does not have
    /// and types the fleet does not have are left out.
    pub fn add_node_conflicts(
        &mut self,
        graph: &Graph,
        types: &[String],
        input: &[NodeConflictInput],
    ) -> usize {
        let type_idx = |name: &str| types.iter().position(|t| t == name);
        let edge_ids: HashMap<&str, EdgeIdx> = graph
            .edges
            .iter()
            .enumerate()
            .map(|(i, e)| (e.id.as_str(), i))
            .collect();
        let node_ids: HashMap<&str, NodeIdx> = graph
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (n.id.as_str(), i))
            .collect();
        let mut read = 0;
        for c in input {
            let (Some(s), Some(t)) = (type_idx(&c.type_node), type_idx(&c.type_edge)) else {
                continue;
            };
            for (node, edge) in &c.pairs {
                let (Some(&n), Some(&e)) =
                    (node_ids.get(node.as_str()), edge_ids.get(edge.as_str()))
                else {
                    continue;
                };
                self.node_edges[s][t].entry(e).or_default().push(n);
                self.edge_nodes[s][t].entry(n).or_default().push(e);
                read += 1;
            }
        }
        for row in self.node_edges.iter_mut().flatten() {
            for list in row.values_mut() {
                list.sort_unstable();
                list.dedup();
            }
        }
        for row in self.edge_nodes.iter_mut().flatten() {
            for list in row.values_mut() {
                list.sort_unstable();
                list.dedup();
            }
        }
        self.node_pairs += read;
        read
    }

    /// Number of node-edge conflicts read.
    pub fn node_pairs(&self) -> usize {
        self.node_pairs
    }

    /// Add the node-node conflicts (two robots on two nodes too close for both).
    /// Returns the number read; ids the layout does not have and types the fleet does
    /// not have are left out.
    pub fn add_node_node_conflicts(
        &mut self,
        graph: &Graph,
        types: &[String],
        input: &[NodeNodeConflictInput],
    ) -> usize {
        let type_idx = |name: &str| types.iter().position(|t| t == name);
        let node_ids: HashMap<&str, NodeIdx> = graph
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (n.id.as_str(), i))
            .collect();
        let mut read = 0;
        for c in input {
            let (Some(a), Some(b)) = (type_idx(&c.type_a), type_idx(&c.type_b)) else {
                continue;
            };
            for (na, nb) in &c.pairs {
                let (Some(&x), Some(&y)) = (node_ids.get(na.as_str()), node_ids.get(nb.as_str()))
                else {
                    continue;
                };
                // a robot of type a going to x keeps clear of type b on y, and the
                // other way round
                self.near_nodes[a][b].entry(x).or_default().push(y);
                self.near_nodes[b][a].entry(y).or_default().push(x);
                read += 1;
            }
        }
        for row in self.near_nodes.iter_mut().flatten() {
            for list in row.values_mut() {
                list.sort_unstable();
                list.dedup();
            }
        }
        self.node_node_pairs += read;
        read
    }

    /// Number of node-node conflicts read.
    pub fn node_node_pairs(&self) -> usize {
        self.node_node_pairs
    }

    /// The robot that **stands** on a node too close to the node edge `e` ends at for
    /// `robot` to go there (`standing[r]`: the node robot `r` stands on, whether or
    /// not it holds it); `None` if no robot does.
    pub fn standing_blocker(
        &self,
        standing: &[Option<NodeIdx>],
        type_of: &[usize],
        robot: usize,
        e: EdgeIdx,
    ) -> Option<usize> {
        if self.node_node_pairs == 0 {
            return None;
        }
        let row = &self.near_nodes[type_of[robot]];
        let end = self.ends[e];
        standing.iter().enumerate().find_map(|(r, on)| {
            let near = row[type_of[r]].get(&end)?;
            (r != robot && near.contains(&(*on)?)).then_some(r)
        })
    }

    /// Number of conflicting edge pairs read (both orders of two different types
    /// count once each).
    pub fn pairs(&self) -> usize {
        self.pairs
    }

    /// Is a robot of type `robot_type` that is `robot` kept off edge `e` because
    /// another robot holds an edge that conflicts with it? `type_of[r]` is the type
    /// index of robot `r`.
    pub fn blocks(
        &self,
        reservations: &Reservations,
        type_of: &[usize],
        robot: usize,
        e: EdgeIdx,
    ) -> bool {
        self.blocker(reservations, type_of, robot, e).is_some()
            || self.node_blocked(reservations, type_of, robot, e)
    }

    /// Node-edge and node-node conflicts: is a robot standing on (holding) a node
    /// that edge `e` passes too close to, or does a robot hold an edge that passes too
    /// close to the node `e` ends at, or a node too close to it?
    fn node_blocked(
        &self,
        reservations: &Reservations,
        type_of: &[usize],
        robot: usize,
        e: EdgeIdx,
    ) -> bool {
        self.node_blocker(reservations, type_of, robot, e).is_some()
    }

    /// The robot that keeps `robot` off edge `e` because of a node-edge or node-node
    /// conflict, with what it holds (the node `e` passes too close to, the edge that
    /// passes too close to the node `e` ends at, or a node too close to that node);
    /// `None` if no robot does.
    pub fn node_blocker(
        &self,
        reservations: &Reservations,
        type_of: &[usize],
        robot: usize,
        e: EdgeIdx,
    ) -> Option<(usize, Held)> {
        if self.node_pairs == 0 && self.node_node_pairs == 0 {
            return None;
        }
        let t = type_of[robot];
        // others stand on a node near the edge
        let standing = self.node_edges.iter().enumerate().find_map(|(s, row)| {
            row[t].get(&e)?.iter().find_map(|&n| {
                let h = reservations.node_holder(n)?;
                (h != robot && type_of[h] == s).then_some((h, Held::Node(n)))
            })
        });
        if standing.is_some() {
            return standing;
        }
        // the robot will stand at the end node: others drive edges near it
        let end = self.ends[e];
        let driving = self.edge_nodes[t].iter().enumerate().find_map(|(s, map)| {
            map.get(&end)?.iter().find_map(|&f| {
                let h = reservations.edge_holder(f)?;
                (h != robot && type_of[h] == s).then_some((h, Held::Edge(f)))
            })
        });
        if driving.is_some() {
            return driving;
        }
        // ... or hold a node too close to it
        self.near_nodes[t].iter().enumerate().find_map(|(s, map)| {
            map.get(&end)?.iter().find_map(|&n| {
                let h = reservations.node_holder(n)?;
                (h != robot && type_of[h] == s).then_some((h, Held::Node(n)))
            })
        })
    }

    /// The robot that keeps `robot` off edge `e`, with the conflicting edge it holds;
    /// `None` if no robot does.
    pub fn blocker(
        &self,
        reservations: &Reservations,
        type_of: &[usize],
        robot: usize,
        e: EdgeIdx,
    ) -> Option<(usize, EdgeIdx)> {
        let row = self.by_type.get(type_of[robot])?;
        row.iter().enumerate().find_map(|(other_type, map)| {
            map.get(&e)?.iter().find_map(|&f| {
                let h = reservations.edge_holder(f)?;
                (h != robot && type_of[h] == other_type).then_some((h, f))
            })
        })
    }
}

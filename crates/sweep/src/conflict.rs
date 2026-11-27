//! The conflict map.
//!
//! For every pair of robot types of the fleet (a type with itself too) and every
//! pair of different edges that both types may drive on the same map: do the swept
//! outlines overlap, or come nearer than the clearance? If so two robots must not
//! use the two edges at the same time. The map says for each pair whether the two
//! edges share a node: such a conflict is covered by the node reservation. A
//! conflict **without** a common node is the uncovered kind of
//! `docs/graph_problems.md` (G5, G6, G11).
//!
//! Also: a robot on a node (standing or turning on the spot there) against an
//! edge that does not touch it, and against a robot on another node.
//!
//! Optionally ([`MapOptions::protective_field`]) also the pairs where only a
//! robot's standstill protective field meets the other body.
//!
//! Not yet: driving backwards.

use botsim_model::graph::Graph;
use serde::Serialize;
use vdma_lif::Lif;

use crate::sweep::{Contact, SAMPLE_STEP, Sweep, field_sweep, node_bodies, sweep};
use crate::types::TypeRef;

/// Default for [`MapOptions::min_overlap`] (m): shallower overlaps are noise.
pub const MIN_OVERLAP: f64 = 0.05;

/// Default for [`MapOptions::clearance`] (m): bodies nearer than this conflict.
pub const CLEARANCE: f64 = 0.10;

/// What the map counts as a conflict.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapOptions {
    /// Distance between the samples of a swept outline (m).
    pub sample_step: f64,
    /// An overlap less deep than this (m) is noise and not a conflict by itself.
    /// 0 keeps every overlap, touching too.
    pub min_overlap: f64,
    /// Two bodies **nearer** than this (m) conflict, overlap or not: a controller
    /// keeps a gap, and positions vary from run to run. Overlap of any depth
    /// counts when this is above 0 (the gap is then below it), so `min_overlap`
    /// only filters when `clearance` is 0. 0 turns the margin off.
    pub clearance: f64,
    /// Also count a conflict when the **standstill protective field** of one
    /// robot along its edge meets the other robot's body (see
    /// [`field_sweep`](crate::sweep::field_sweep)): a robot stopped by its field
    /// stands until the other one has left. Either side's field is enough; a
    /// robot standing on a node has none. Any overlap of field and body counts:
    /// `clearance` and `min_overlap` do not apply to it. Default `false`.
    ///
    /// The entries this adds are those for which
    /// [`ConflictMap::by_field`] is true.
    pub protective_field: bool,
}

impl Default for MapOptions {
    fn default() -> Self {
        MapOptions {
            sample_step: SAMPLE_STEP,
            min_overlap: MIN_OVERLAP,
            clearance: CLEARANCE,
            protective_field: false,
        }
    }
}

impl MapOptions {
    /// Overlap only: no clearance margin, `min_overlap` filters the shallow ones.
    pub fn overlap_only(sample_step: f64, min_overlap: f64) -> MapOptions {
        MapOptions {
            sample_step,
            min_overlap,
            clearance: 0.0,
            protective_field: false,
        }
    }
}

/// One pair of edges whose swept outlines overlap (for one pair of robot types).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Conflict {
    /// Edge of the first robot type of the pair.
    pub edge_a: String,
    /// Edge of the second robot type of the pair.
    pub edge_b: String,
    /// The two edges have a node in common (the node reservation covers it).
    pub shares_node: bool,
    /// For a conflict without a common node: the two nodes (one of each edge) where
    /// the edges come together, if the conflict is **near a junction** (a diverge or
    /// merge, see [`near_junction`]); `None` for a parallel conflict.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub junction_nodes: Option<[String; 2]>,
    /// The deepest overlap of any two sampled bodies (m): how far one robot would
    /// have to move to be clear. 0 if the bodies only come nearer than the
    /// clearance. A viewer or planner may cut at its own depth.
    pub depth: f64,
    /// The smallest distance between two sampled bodies (m); 0 if they overlap.
    pub distance: f64,
}

/// The conflicts of one pair of robot types (`type_a` may equal `type_b`).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct TypePair {
    pub type_a: String,
    pub type_b: String,
    pub conflicts: Vec<Conflict>,
}

/// A robot standing on a node against an edge that another robot drives.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct NodeConflict {
    pub node: String,
    pub edge: String,
    /// The node and the end node of the edge where they come together, if the
    /// conflict is near a junction (see [`near_junction`]); `None` otherwise.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub junction_nodes: Option<[String; 2]>,
    /// The deepest overlap of the standing body with the swept outline (m); 0 if
    /// they only come nearer than the clearance.
    pub depth: f64,
    /// The smallest distance between the standing body and the swept outline (m);
    /// 0 if they overlap.
    pub distance: f64,
}

/// The node-edge conflicts for one pair of robot types.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct NodeTypePair {
    /// The type of the robot standing on the node.
    pub type_node: String,
    /// The type of the robot driving the edge (may be the same type).
    pub type_edge: String,
    pub conflicts: Vec<NodeConflict>,
}

/// Two nodes too close for a robot on each.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct NodeNodeConflict {
    /// Node of the first robot type of the pair.
    pub node_a: String,
    /// Node of the second robot type of the pair.
    pub node_b: String,
    /// The deepest overlap of the two swept outlines (m); 0 if they only come
    /// nearer than the clearance.
    pub depth: f64,
    /// The smallest distance between them (m); 0 if they overlap.
    pub distance: f64,
}

/// The node-node conflicts of one pair of robot types (`type_a` may equal
/// `type_b`).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct NodeNodeTypePair {
    pub type_a: String,
    pub type_b: String,
    pub conflicts: Vec<NodeNodeConflict>,
}

/// The conflict map of a LIF file for a fleet.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ConflictMap {
    /// Distance between the samples of the swept outlines (m).
    pub sample_step: f64,
    /// Overlaps less deep than this (m) are not in the map (unless within the
    /// clearance).
    pub min_overlap: f64,
    /// Bodies nearer than this (m) are in the map, overlap or not.
    pub clearance: f64,
    /// Robot types as `manufacturer/seriesName`.
    pub types: Vec<String>,
    /// Pairs of types, `type_a` before `type_b` in `types`; pairs without any
    /// conflict are left out. For a pair of two different types an edge pair is
    /// listed once per (edge of `type_a`, edge of `type_b`); for a type with
    /// itself, once (edge ids in layout order). The same edge twice is not
    /// listed: an edge belongs to one robot at a time.
    pub pairs: Vec<TypePair>,
    /// A robot standing on a node (with a heading of an edge to or from it) against
    /// an edge that does not touch that node, driven by another robot. Edges that
    /// touch the node are left out (the node reservation covers them). Ordered pairs
    /// of types (the standing robot, the driving robot); pairs without conflict are
    /// left out. These are all without a common node.
    pub node_pairs: Vec<NodeTypePair>,
    /// Two robots on two **different nodes**, standing or turning on the spot
    /// there (the outline at a node is swept over the turns, see
    /// [`node_bodies`]). Pairs of types, `type_a` before `type_b` in `types`;
    /// for a type with itself each pair of nodes once (layout order). The same
    /// node twice is not listed: a node belongs to one robot at a time. Pairs of
    /// types without conflict are left out.
    pub node_node_pairs: Vec<NodeNodeTypePair>,
}

/// The swept outlines of one edge for one type.
struct EdgeSweep {
    edge: usize,
    map: usize,
    body: Sweep,
    /// The protective field; only with [`MapOptions::protective_field`].
    field: Option<Sweep>,
}

/// The swept outline of one node for one type.
struct NodeSweep {
    node: usize,
    map: usize,
    body: Sweep,
}

/// The swept outlines of all edges and nodes one type may use.
struct TypeSweeps {
    edges: Vec<EdgeSweep>,
    nodes: Vec<NodeSweep>,
}

/// Do two bodies conflict? Overlap deeper than `min_overlap`, or nearer than the
/// clearance.
fn body_conflict(depth: f64, distance: f64, min_overlap: f64, clearance: f64) -> bool {
    (distance == 0.0 && depth >= min_overlap) || distance < clearance
}

/// The contact of two sweeps if their bodies conflict.
fn bodies_in_conflict(a: &Sweep, b: &Sweep, o: &MapOptions) -> Option<Contact> {
    a.contact(b, o.clearance)
        .filter(|c| body_conflict(c.depth, c.distance, o.min_overlap, o.clearance))
}

impl ConflictMap {
    /// Do the bodies of two swept outlines conflict by the rule this map was made
    /// with (overlap of at least `min_overlap`, or nearer than `clearance`)?
    pub fn conflicts(&self, a: &Sweep, b: &Sweep) -> bool {
        a.contact(b, self.clearance)
            .is_some_and(|c| body_conflict(c.depth, c.distance, self.min_overlap, self.clearance))
    }

    /// Is an entry of this map, given by its `depth` and `distance`, listed only
    /// because of a protective field ([`MapOptions::protective_field`])? Its
    /// bodies alone do not conflict: they stay `clearance` or more apart, or
    /// overlap by less than `min_overlap`. Always false for a map made without
    /// the option.
    pub fn by_field(&self, depth: f64, distance: f64) -> bool {
        !body_conflict(depth, distance, self.min_overlap, self.clearance)
    }
}

/// Compute the conflict map. `None` if the layout cannot be turned into a graph.
pub fn compute(lif: &Lif, types: &[TypeRef], options: &MapOptions) -> Option<ConflictMap> {
    let graph = Graph::from_lif(lif).ok()?;
    let step = options.sample_step;
    let adj = neighbours(&graph);
    let sweeps: Vec<TypeSweeps> = types
        .iter()
        .map(|t| TypeSweeps {
            edges: (0..graph.edges.len())
                .filter(|&e| graph.edge_allows(e, &t.manufacturer, &t.series))
                .filter_map(|e| {
                    Some(EdgeSweep {
                        edge: e,
                        map: graph.nodes[graph.edges[e].start].map,
                        body: sweep(&graph, e, t, step)?,
                        field: if options.protective_field {
                            field_sweep(&graph, e, t, step)
                        } else {
                            None
                        },
                    })
                })
                .collect(),
            nodes: (0..graph.nodes.len())
                .filter_map(|n| {
                    let bodies = node_bodies(&graph, n, t);
                    (!bodies.is_empty()).then(|| NodeSweep {
                        node: n,
                        map: graph.nodes[n].map,
                        body: Sweep::from_bodies(bodies),
                    })
                })
                .collect(),
        })
        .collect();

    let mut pairs = Vec::new();
    for i in 0..types.len() {
        for j in i..types.len() {
            let mut conflicts = Vec::new();
            for s1 in &sweeps[i].edges {
                for s2 in &sweeps[j].edges {
                    let (e1, e2) = (s1.edge, s2.edge);
                    if s1.map != s2.map || (i == j && e1 >= e2) || (i != j && e1 == e2) {
                        continue;
                    }
                    let found = bodies_in_conflict(&s1.body, &s2.body, options).or_else(|| {
                        let (f1, f2) = (s1.field.as_ref()?, s2.field.as_ref()?);
                        if !f1.overlaps(&s2.body) && !f2.overlaps(&s1.body) {
                            return None;
                        }
                        // how close the bodies themselves come
                        s1.body.contact(&s2.body, f64::INFINITY)
                    });
                    if let Some(c) = found {
                        let (a, b) = (&graph.edges[e1], &graph.edges[e2]);
                        let shares_node = a.start == b.start
                            || a.start == b.end
                            || a.end == b.start
                            || a.end == b.end;
                        let junction_nodes = if shares_node {
                            None
                        } else {
                            near_junction(
                                &graph,
                                &adj,
                                [a.start, a.end],
                                [b.start, b.end],
                                reach(&types[i], &types[j]),
                            )
                        };
                        conflicts.push(Conflict {
                            edge_a: a.id.clone(),
                            edge_b: b.id.clone(),
                            shares_node,
                            junction_nodes,
                            depth: c.depth,
                            distance: c.distance,
                        });
                    }
                }
            }
            if !conflicts.is_empty() {
                pairs.push(TypePair {
                    type_a: types[i].name.clone(),
                    type_b: types[j].name.clone(),
                    conflicts,
                });
            }
        }
    }
    let node_pairs = node_conflicts(&graph, &adj, types, &sweeps, options);
    let node_node_pairs = node_node_conflicts(&graph, types, &sweeps, options);
    Some(ConflictMap {
        sample_step: step,
        min_overlap: options.min_overlap,
        clearance: options.clearance,
        types: types.iter().map(|t| t.name.clone()).collect(),
        pairs,
        node_pairs,
        node_node_pairs,
    })
}

/// A robot of each type on each node against a robot of each type on each other
/// node.
fn node_node_conflicts(
    graph: &Graph,
    types: &[TypeRef],
    sweeps: &[TypeSweeps],
    options: &MapOptions,
) -> Vec<NodeNodeTypePair> {
    let mut out = Vec::new();
    for i in 0..types.len() {
        for j in i..types.len() {
            let mut conflicts = Vec::new();
            for a in &sweeps[i].nodes {
                for b in &sweeps[j].nodes {
                    let (x, y) = (a.node, b.node);
                    if a.map != b.map || (i == j && x >= y) || x == y {
                        continue;
                    }
                    if let Some(c) = bodies_in_conflict(&a.body, &b.body, options) {
                        conflicts.push(NodeNodeConflict {
                            node_a: graph.nodes[x].id.clone(),
                            node_b: graph.nodes[y].id.clone(),
                            depth: c.depth,
                            distance: c.distance,
                        });
                    }
                }
            }
            if !conflicts.is_empty() {
                out.push(NodeNodeTypePair {
                    type_a: types[i].name.clone(),
                    type_b: types[j].name.clone(),
                    conflicts,
                });
            }
        }
    }
    out
}

/// Two nodes this many edges apart (any direction) or fewer count as neighbours.
pub const JUNCTION_HOPS: usize = 2;

/// How close two robots' ends come: the longer of the two bodies plus the longer
/// protective field. Nodes nearer than this to the other edge's nodes belong to the
/// same junction.
fn reach(a: &TypeRef, b: &TypeRef) -> f64 {
    (a.length + a.field_margin).max(b.length + b.field_margin)
}

/// The nodes of the graph next to each node (any edge direction).
fn neighbours(graph: &Graph) -> Vec<Vec<usize>> {
    let mut adj = vec![Vec::new(); graph.nodes.len()];
    for e in &graph.edges {
        adj[e.start].push(e.end);
        adj[e.end].push(e.start);
    }
    adj
}

/// Are the nodes at most [`JUNCTION_HOPS`] edges apart?
fn hops_close(adj: &[Vec<usize>], a: usize, b: usize) -> bool {
    a == b
        || adj[a].contains(&b)
        || (JUNCTION_HOPS >= 2 && adj[a].iter().any(|&x| adj[x].contains(&b)))
}

/// Is a conflict between things at the nodes `a` and the nodes `b` **near a
/// junction** (a diverge or merge: the two come together there)? It is if one node
/// of `a` and one of `b` are at most [`JUNCTION_HOPS`] edges apart, or
/// `reach` metres apart on the same map. Returns the closest such pair.
fn near_junction(
    graph: &Graph,
    adj: &[Vec<usize>],
    a: [usize; 2],
    b: [usize; 2],
    reach: f64,
) -> Option<[String; 2]> {
    let mut best: Option<(f64, usize, usize)> = None;
    for &x in &a {
        for &y in &b {
            let (nx, ny) = (&graph.nodes[x], &graph.nodes[y]);
            let dist = if nx.map == ny.map {
                (nx.x - ny.x).hypot(nx.y - ny.y)
            } else {
                f64::INFINITY
            };
            if (hops_close(adj, x, y) || dist <= reach) && best.is_none_or(|(d, _, _)| dist < d) {
                best = Some((dist, x, y));
            }
        }
    }
    best.map(|(_, x, y)| [graph.nodes[x].id.clone(), graph.nodes[y].id.clone()])
}

/// A robot of each type standing on each node against the swept outlines of the
/// edges of each type.
fn node_conflicts(
    graph: &Graph,
    adj: &[Vec<usize>],
    types: &[TypeRef],
    sweeps: &[TypeSweeps],
    options: &MapOptions,
) -> Vec<NodeTypePair> {
    let mut out = Vec::new();
    for (i, ti) in types.iter().enumerate() {
        for (j, tj) in types.iter().enumerate() {
            let mut conflicts = Vec::new();
            for standing in &sweeps[i].nodes {
                let (n, bodies) = (standing.node, &standing.body);
                for sw in &sweeps[j].edges {
                    let edge = &graph.edges[sw.edge];
                    if standing.map != sw.map || edge.start == n || edge.end == n {
                        continue;
                    }
                    let found = bodies_in_conflict(bodies, &sw.body, options).or_else(|| {
                        // only the driving robot has a field
                        if !sw.field.as_ref()?.overlaps(bodies) {
                            return None;
                        }
                        bodies.contact(&sw.body, f64::INFINITY)
                    });
                    if let Some(c) = found {
                        conflicts.push(NodeConflict {
                            node: graph.nodes[n].id.clone(),
                            edge: edge.id.clone(),
                            junction_nodes: near_junction(
                                graph,
                                adj,
                                [n, n],
                                [edge.start, edge.end],
                                reach(ti, tj),
                            ),
                            depth: c.depth,
                            distance: c.distance,
                        });
                    }
                }
            }
            if !conflicts.is_empty() {
                out.push(NodeTypePair {
                    type_node: ti.name.clone(),
                    type_edge: tj.name.clone(),
                    conflicts,
                });
            }
        }
    }
    out
}

/// The conflict map as JSON.
pub fn to_json(map: &ConflictMap) -> String {
    serde_json::to_string_pretty(map).unwrap_or_else(|_| "{}".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use botsim_model::spec::Kinematics;

    /// Edges `(id, from, to)` between nodes `(id, x, y)`, all usable by ACME/X1 and ACME/Y2.
    fn lif(nodes: &[(&str, f64, f64)], edges: &[(&str, &str, &str)]) -> Lif {
        let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"},{"manufacturer":"ACME","seriesName":"Y2"}]}"#;
        let nodes: Vec<String> = nodes
            .iter()
            .map(|(id, x, y)| {
                format!(
                    r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},"mobileRobotTypeNodeProperties":[{ty}]}}"#
                )
            })
            .collect();
        let edges: Vec<String> = edges
            .iter()
            .map(|(id, a, b)| {
                format!(
                    r#"{{"edgeId":"{id}","startNodeId":"{a}","endNodeId":"{b}","mobileRobotTypeEdgeProperties":[{ty}]}}"#
                )
            })
            .collect();
        serde_json::from_str(&format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1","nodes":[{}],"edges":[{}]}}]}}]}}"#,
            nodes.join(","),
            edges.join(",")
        ))
        .expect("test LIF parses")
    }

    fn two_types() -> Vec<TypeRef> {
        vec![
            TypeRef::test_rect("ACME/X1", Kinematics::Differential, 1.0, 0.6),
            TypeRef::test_rect("ACME/Y2", Kinematics::Differential, 2.4, 1.4),
        ]
    }

    fn overlap_only(min_overlap: f64) -> MapOptions {
        MapOptions::overlap_only(0.25, min_overlap)
    }

    #[test]
    fn shared_node_and_uncovered_conflicts() {
        // E1 and E2 meet at B (share a node). E3 runs 1.0 m beside E1 with no node in
        // common: X1 (0.6 wide) is clear of X1, but Y2 (1.4 wide) is not clear of anything near.
        let l = lif(
            &[
                ("A", 0.0, 0.0),
                ("B", 10.0, 0.0),
                ("C", 20.0, 0.0),
                ("D", 0.0, 1.0),
                ("F", 10.0, 1.0),
            ],
            &[("E1", "A", "B"), ("E2", "B", "C"), ("E3", "D", "F")],
        );
        let m = compute(&l, &two_types(), &overlap_only(0.0)).expect("map");
        assert_eq!(m.types, ["ACME/X1", "ACME/Y2"]);
        let pair = |a: &str, b: &str| m.pairs.iter().find(|p| p.type_a == a && p.type_b == b);
        // X1 with X1: E1-E2 overlap at B (shared node); E1-E3 are 1.0 m apart, bodies 0.6 wide: clear
        let xx = pair("ACME/X1", "ACME/X1").expect("X1/X1");
        assert_eq!(xx.conflicts.len(), 1);
        assert_eq!(xx.conflicts[0].edge_a, "E1");
        assert_eq!(xx.conflicts[0].edge_b, "E2");
        assert!(xx.conflicts[0].shares_node);
        // Y2 with Y2: 1.0 m apart, bodies 1.4 wide: E1-E3 overlap with no common node,
        // and so do E2-E3 (E3 ends at (10,1), above where E2 starts at (10,0))
        let yy = pair("ACME/Y2", "ACME/Y2").expect("Y2/Y2");
        let open: Vec<_> = yy.conflicts.iter().filter(|c| !c.shares_node).collect();
        let names: Vec<_> = open
            .iter()
            .map(|c| (c.edge_a.as_str(), c.edge_b.as_str()))
            .collect();
        assert_eq!(names, [("E1", "E3"), ("E2", "E3")]);
        // mixed pair X1 (E1) with Y2 (E3): 1.0 m apart, 0.3 + 0.7 = 1.0: touching counts
        assert!(pair("ACME/X1", "ACME/Y2").is_some());
    }

    #[test]
    fn json_has_the_documented_keys() {
        let l = lif(
            &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("C", 20.0, 0.0)],
            &[("E1", "A", "B"), ("E2", "B", "C")],
        );
        let m = compute(&l, &two_types(), &overlap_only(0.0)).expect("map");
        let v: serde_json::Value = serde_json::from_str(&to_json(&m)).expect("json");
        assert_eq!(v["sample_step"], 0.25);
        assert_eq!(v["min_overlap"], 0.0);
        assert_eq!(v["clearance"], 0.0);
        assert!(v["pairs"][0]["conflicts"][0]["shares_node"].is_boolean());
        assert!(v["pairs"][0]["conflicts"][0]["depth"].is_number());
        assert!(v["pairs"][0]["conflicts"][0]["distance"].is_number());
    }

    #[test]
    fn near_a_junction_has_junction_nodes_and_parallel_has_none() {
        let y2 = vec![TypeRef::test_rect(
            "ACME/Y2",
            Kinematics::Differential,
            2.4,
            1.4,
        )];
        // E3 starts 1 m beside the start of E1: they come together there (junction)
        let l = lif(
            &[
                ("A", 0.0, 0.0),
                ("B", 30.0, 0.0),
                ("D", 0.0, 1.0),
                ("F", 5.0, 1.0),
            ],
            &[("E1", "A", "B"), ("E3", "D", "F")],
        );
        let m = compute(&l, &y2, &overlap_only(0.0)).expect("map");
        let c = &m.pairs[0].conflicts[0];
        assert_eq!(c.junction_nodes, Some(["A".to_string(), "D".to_string()]));
        // E3 runs 1 m beside the middle of E1, its nodes 10 m or more from E1's: parallel
        let l = lif(
            &[
                ("A", 0.0, 0.0),
                ("B", 30.0, 0.0),
                ("D", 10.0, 1.0),
                ("F", 20.0, 1.0),
            ],
            &[("E1", "A", "B"), ("E3", "D", "F")],
        );
        let m = compute(&l, &y2, &overlap_only(0.0)).expect("map");
        assert!(m.pairs[0].conflicts[0].junction_nodes.is_none());
    }

    #[test]
    fn robot_standing_on_a_node_against_an_edge() {
        // E1 runs (0,0)-(10,0). Node F at (5,0.5) is the start of E3 going north:
        // a robot standing on F (heading north, 1.0 x 0.6) reaches down to y = 0, into
        // the swept outline of E1 (y -0.3..0.3). E3 touches F, so it is not listed.
        let l = lif(
            &[
                ("A", 0.0, 0.0),
                ("B", 10.0, 0.0),
                ("F", 5.0, 0.5),
                ("G", 5.0, 10.0),
            ],
            &[("E1", "A", "B"), ("E3", "F", "G")],
        );
        let types = vec![TypeRef::test_rect(
            "ACME/X1",
            Kinematics::Differential,
            1.0,
            0.6,
        )];
        let m = compute(&l, &types, &overlap_only(0.0)).expect("map");
        assert_eq!(m.node_pairs.len(), 1);
        let p = &m.node_pairs[0];
        assert_eq!(
            (p.type_node.as_str(), p.type_edge.as_str()),
            ("ACME/X1", "ACME/X1")
        );
        assert!(p.conflicts.iter().all(|c| c.edge != "E3" || c.node != "F"));
        let hit = p
            .conflicts
            .iter()
            .find(|c| c.node == "F" && c.edge == "E1")
            .expect("F against E1");
        assert!(hit.depth > 0.25 && hit.depth <= 0.3 + 1e-9, "{}", hit.depth);
    }

    #[test]
    fn min_overlap_cuts_the_shallow_conflicts() {
        // Y2 bodies are 1.4 wide: E1 and E3 (1.0 m apart) overlap by 0.4 m. X1 (0.6 wide)
        // with Y2 (1.4): 1.0 m apart = exactly touching, depth 0.
        let l = lif(
            &[
                ("A", 0.0, 0.0),
                ("B", 10.0, 0.0),
                ("D", 0.0, 1.0),
                ("F", 10.0, 1.0),
            ],
            &[("E1", "A", "B"), ("E3", "D", "F")],
        );
        let all = compute(&l, &two_types(), &overlap_only(0.0)).expect("map");
        let yy = all
            .pairs
            .iter()
            .find(|p| p.type_a == "ACME/Y2" && p.type_b == "ACME/Y2");
        assert!((yy.expect("Y2/Y2").conflicts[0].depth - 0.4).abs() < 1e-9);
        let cut = compute(&l, &two_types(), &overlap_only(0.05)).expect("map");
        // the touching X1/Y2 pair is gone, the 0.4 m one stays
        assert!(
            cut.pairs
                .iter()
                .all(|p| p.conflicts.iter().all(|c| c.depth >= 0.05))
        );
        assert!(
            cut.pairs
                .iter()
                .any(|p| p.type_a == "ACME/Y2" && p.type_b == "ACME/Y2")
        );
        assert!(
            !cut.pairs
                .iter()
                .any(|p| p.type_a == "ACME/X1" && p.type_b == "ACME/Y2")
        );
        assert!(
            all.pairs
                .iter()
                .any(|p| p.type_a == "ACME/X1" && p.type_b == "ACME/Y2")
        );
    }

    #[test]
    fn robots_on_two_nodes_too_close() {
        // E1 ends at B, E2 starts at C, 0.8 m further on: no common node. X1 is
        // 1.0 long (0.2 m overlap end to end), G is far away.
        let l = lif(
            &[
                ("A", 0.0, 0.0),
                ("B", 10.0, 0.0),
                ("C", 10.8, 0.0),
                ("G", 30.0, 0.0),
            ],
            &[("E1", "A", "B"), ("E2", "C", "G")],
        );
        let m = compute(&l, &two_types(), &overlap_only(0.05)).expect("map");
        let of = |a: &str, b: &str| {
            m.node_node_pairs
                .iter()
                .find(|p| p.type_a == a && p.type_b == b)
                .expect("type pair")
        };
        // a type with itself: each pair of nodes once, never a node with itself
        let xx = of("ACME/X1", "ACME/X1");
        assert_eq!(xx.conflicts.len(), 1);
        let c = &xx.conflicts[0];
        assert_eq!((c.node_a.as_str(), c.node_b.as_str()), ("B", "C"));
        assert!((c.depth - 0.2).abs() < 1e-9 && c.distance == 0.0, "{c:?}");
        // two types: both ways round
        let xy = of("ACME/X1", "ACME/Y2");
        let names: Vec<_> = xy
            .conflicts
            .iter()
            .map(|c| (c.node_a.as_str(), c.node_b.as_str()))
            .collect();
        assert_eq!(names, [("B", "C"), ("C", "B")]);
        let v: serde_json::Value = serde_json::from_str(&to_json(&m)).expect("json");
        assert_eq!(v["node_node_pairs"][0]["conflicts"][0]["node_a"], "B");
    }

    #[test]
    fn clearance_adds_the_near_misses() {
        // X1 bodies are 0.6 wide; E3 runs 0.65 m beside E1: a 0.05 m gap, no overlap.
        let l = lif(
            &[
                ("A", 0.0, 0.0),
                ("B", 10.0, 0.0),
                ("D", 0.0, 0.65),
                ("F", 10.0, 0.65),
            ],
            &[("E1", "A", "B"), ("E3", "D", "F")],
        );
        let x1 = vec![TypeRef::test_rect(
            "ACME/X1",
            Kinematics::Differential,
            1.0,
            0.6,
        )];
        let none = compute(&l, &x1, &overlap_only(0.05)).expect("map");
        assert!(none.pairs.is_empty() && none.node_pairs.is_empty());
        let m = compute(
            &l,
            &x1,
            &MapOptions {
                sample_step: 0.25,
                min_overlap: 0.05,
                clearance: 0.10,
                protective_field: false,
            },
        )
        .expect("map");
        let c = &m.pairs[0].conflicts[0];
        assert_eq!((c.edge_a.as_str(), c.edge_b.as_str()), ("E1", "E3"));
        assert_eq!(c.depth, 0.0);
        assert!((c.distance - 0.05).abs() < 1e-9, "{}", c.distance);
        assert_eq!(m.clearance, 0.10);
        // a clearance below the gap finds nothing
        let m = compute(
            &l,
            &x1,
            &MapOptions {
                sample_step: 0.25,
                min_overlap: 0.05,
                clearance: 0.04,
                protective_field: false,
            },
        )
        .expect("map");
        assert!(m.pairs.is_empty());
    }

    /// The part of the roadmap `W_real_L` where two robots locked up: `A-002`
    /// stands on N0019 (arrived northwards from N0013), `A-001` turns left on the
    /// 39 cm curve N0026 -> N0039. Coordinates and curve from the roadmap file,
    /// the robot 0.48 m x 0.24 m.
    fn w_real_l() -> (Lif, Vec<TypeRef>) {
        let ty = r#""mobileRobotTypes":[{"manufacturer":"Zuzek2024","seriesName":"AGV"}]"#;
        let node = |id: &str, x: f64, y: f64| {
            format!(
                r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},"mobileRobotTypeNodeProperties":[{{{ty}}}]}}"#
            )
        };
        let curve = r#","trajectory":{"degree":2,"knotVector":[0,0,0,1,1,1],"controlPoints":[{"x":5.5,"y":4.25},{"x":5.75,"y":4.25,"weight":0.7071067811865476},{"x":5.75,"y":4.5}]}"#;
        let lif: Lif = serde_json::from_str(&format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1","nodes":[{},{},{},{}],
            "edges":[{{"edgeId":"N0013-N0019","startNodeId":"N0013","endNodeId":"N0019","mobileRobotTypeEdgeProperties":[{{{ty}}}]}},
                     {{"edgeId":"N0026-N0039","startNodeId":"N0026","endNodeId":"N0039","mobileRobotTypeEdgeProperties":[{{{ty}{curve}}}]}}]}}]}}]}}"#,
            node("N0013", 6.25, 3.75),
            node("N0019", 6.25, 4.0),
            node("N0026", 5.5, 4.25),
            node("N0039", 5.75, 4.5),
        ))
        .expect("test LIF parses");
        let agv = TypeRef::test_rect("Zuzek2024/AGV", Kinematics::Differential, 0.48, 0.24);
        (lif, vec![agv])
    }

    fn with_field(o: MapOptions) -> MapOptions {
        MapOptions {
            protective_field: true,
            ..o
        }
    }

    fn node_conflict<'a>(m: &'a ConflictMap, node: &str, edge: &str) -> Option<&'a NodeConflict> {
        m.node_pairs
            .iter()
            .flat_map(|p| &p.conflicts)
            .find(|c| c.node == node && c.edge == edge)
    }

    #[test]
    fn protective_field_lists_the_w_real_l_pair() {
        let (l, types) = w_real_l();
        for options in [MapOptions::default(), overlap_only(0.05)] {
            // without the option the bodies are too far apart (about 0.2 m)
            let off = compute(&l, &types, &options).expect("map");
            assert!(node_conflict(&off, "N0019", "N0026-N0039").is_none());
            // with it: listed, although the field overlaps by less than min_overlap
            let on = compute(&l, &types, &with_field(options)).expect("map");
            let c = node_conflict(&on, "N0019", "N0026-N0039").expect("the field reaches");
            assert!(on.by_field(c.depth, c.distance));
            assert!(c.depth == 0.0 && c.distance > 0.15, "{c:?}");
            assert!(c.junction_nodes.is_some());
        }
    }

    #[test]
    fn protective_field_only_adds_entries() {
        let (l, types) = w_real_l();
        let off = compute(&l, &types, &MapOptions::default()).expect("map");
        let on = compute(&l, &types, &with_field(MapOptions::default())).expect("map");
        let flat = |m: &ConflictMap| {
            let e: Vec<Conflict> = m.pairs.iter().flat_map(|p| p.conflicts.clone()).collect();
            let n: Vec<NodeConflict> = m
                .node_pairs
                .iter()
                .flat_map(|p| p.conflicts.clone())
                .collect();
            (e, n)
        };
        let ((e_off, n_off), (e_on, n_on)) = (flat(&off), flat(&on));
        assert!(e_off.iter().all(|c| e_on.contains(c)));
        assert!(n_off.iter().all(|c| n_on.contains(c)));
        // what was added is by the field, what was there before is not
        for c in &e_on {
            assert_eq!(
                on.by_field(c.depth, c.distance),
                !e_off.contains(c),
                "{c:?}"
            );
        }
        for c in &n_on {
            assert_eq!(
                on.by_field(c.depth, c.distance),
                !n_off.contains(c),
                "{c:?}"
            );
        }
        assert!(n_on.len() > n_off.len());
    }

    #[test]
    fn protective_field_reaches_its_length_ahead() {
        // E1 runs (0,0)-(10,0); the body (1.0 long) ends at x = 10.5, its field at
        // 10.8. A robot stands on F, ahead, heading north (0.6 wide in x).
        let at = |gap: f64| {
            let x = 10.5 + gap + 0.3;
            let l = lif(
                &[
                    ("A", 0.0, 0.0),
                    ("B", 10.0, 0.0),
                    ("F", x, 0.0),
                    ("G", x, 10.0),
                ],
                &[("E1", "A", "B"), ("E3", "F", "G")],
            );
            let x1 = vec![TypeRef::test_rect(
                "ACME/X1",
                Kinematics::Differential,
                1.0,
                0.6,
            )];
            let m = compute(&l, &x1, &with_field(MapOptions::default())).expect("map");
            node_conflict(&m, "F", "E1").map(|c| c.distance)
        };
        // 0.2 m ahead: beyond the clearance, within the field
        assert!((at(0.2).expect("in the field") - 0.2).abs() < 1e-9);
        // 0.35 m ahead: out of reach
        assert!(at(0.35).is_none());
    }
}

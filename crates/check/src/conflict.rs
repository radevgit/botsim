//! Check 5: the conflict map.
//!
//! For every pair of robot types of the fleet (a type with itself too) and every
//! pair of different edges that both types may drive on the same map: do the swept
//! outlines overlap? If so two robots must not use the two edges at the same time.
//! The map says for each pair whether the two edges share a node: such a conflict is
//! covered by the node reservation. A conflict **without** a common node is the
//! uncovered kind of `docs/graph_problems.md` (G5, G6, G11) and becomes a finding.
//!
//! Not yet: robots standing at nodes against edges (edge-node pairs), the protective
//! field, driving backwards.

use std::collections::HashMap;

use botsim_core::graph::Graph;
use serde::Serialize;
use vdma_lif::{Layout, Lif};

use crate::finding::{Finding, Level};
use crate::places::LayoutIndex;
use crate::reach::TypeRef;
use crate::sweep::{Body, Sweep, node_bodies, overlap_depth, sweep};

const CHECK: &str = "conflict";

/// Default for `min_overlap` (m): shallower overlaps are noise.
pub const MIN_OVERLAP: f64 = 0.05;

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
    /// have to move to be clear. A viewer or planner may cut at its own depth.
    pub depth: f64,
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
    /// The deepest overlap of the standing body with the swept outline (m).
    pub depth: f64,
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

/// The conflict map of a LIF file for a fleet.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ConflictMap {
    /// Distance between the samples of the swept outlines (m).
    pub sample_step: f64,
    /// Overlaps less deep than this (m) are not in the map.
    pub min_overlap: f64,
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
    /// left out. These are all without a common node, so all become findings.
    pub node_pairs: Vec<NodeTypePair>,
}

/// The swept outlines of all edges one type may drive, with the map of each edge.
struct TypeSweeps {
    edges: Vec<(usize, usize, Sweep)>, // (edge, map, sweep)
}

/// Compute the conflict map. `None` if the layout cannot be turned into a graph
/// (reported by the reachability check).
///
/// `min_overlap` (m): overlaps less deep than this are not conflicts (a few
/// centimetres of sampling and rounding noise); 0 keeps every overlap, touching too.
pub fn compute(
    lif: &Lif,
    types: &[TypeRef],
    step: f64,
    min_overlap: f64,
) -> Option<ConflictMap> {
    let graph = Graph::from_lif(lif).ok()?;
    let adj = neighbours(&graph);
    let sweeps: Vec<TypeSweeps> = types
        .iter()
        .map(|t| TypeSweeps {
            edges: (0..graph.edges.len())
                .filter(|&e| graph.edge_allows(e, &t.manufacturer, &t.series))
                .filter_map(|e| {
                    let s = sweep(&graph, e, t, step)?;
                    Some((e, graph.nodes[graph.edges[e].start].map, s))
                })
                .collect(),
        })
        .collect();

    let mut pairs = Vec::new();
    for i in 0..types.len() {
        for j in i..types.len() {
            let mut conflicts = Vec::new();
            for (e1, m1, s1) in &sweeps[i].edges {
                for (e2, m2, s2) in &sweeps[j].edges {
                    if m1 != m2 || (i == j && e1 >= e2) || (i != j && e1 == e2) {
                        continue;
                    }
                    if let Some(depth) = s1.max_overlap(s2)
                        && depth >= min_overlap
                    {
                        let (a, b) = (&graph.edges[*e1], &graph.edges[*e2]);
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
                            depth,
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
    let node_pairs = node_conflicts(&graph, &adj, types, &sweeps, min_overlap);
    Some(ConflictMap {
        sample_step: step,
        min_overlap,
        types: types.iter().map(|t| t.name.clone()).collect(),
        pairs,
        node_pairs,
    })
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
    min_overlap: f64,
) -> Vec<NodeTypePair> {
    let mut out = Vec::new();
    for ti in types {
        // the standing bodies of this type: (node, map, bodies, bounds)
        let standing: Vec<(usize, usize, Vec<Body>, (f64, f64, f64, f64))> = (0..graph.nodes.len())
            .filter_map(|n| {
                let bodies = node_bodies(graph, n, ti);
                if bodies.is_empty() {
                    return None;
                }
                let b = bodies.iter().map(Body::bounds).fold(
                    (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
                    |a, c| (a.0.min(c.0), a.1.min(c.1), a.2.max(c.2), a.3.max(c.3)),
                );
                Some((n, graph.nodes[n].map, bodies, b))
            })
            .collect();
        for (j, tj) in types.iter().enumerate() {
            let mut conflicts = Vec::new();
            for (n, nm, bodies, nb) in &standing {
                for (e, em, sw) in &sweeps[j].edges {
                    let edge = &graph.edges[*e];
                    if nm != em || edge.start == *n || edge.end == *n {
                        continue;
                    }
                    let sb = sw.bounds;
                    if nb.2 < sb.0 || sb.2 < nb.0 || nb.3 < sb.1 || sb.3 < nb.1 {
                        continue;
                    }
                    let mut deepest: Option<f64> = None;
                    for x in bodies {
                        for y in &sw.bodies {
                            if let Some(d) = overlap_depth(x, y) {
                                deepest = Some(deepest.map_or(d, |m: f64| m.max(d)));
                            }
                        }
                    }
                    if let Some(depth) = deepest
                        && depth >= min_overlap
                    {
                        conflicts.push(NodeConflict {
                            node: graph.nodes[*n].id.clone(),
                            edge: edge.id.clone(),
                            junction_nodes: near_junction(
                                graph,
                                adj,
                                [*n, *n],
                                [edge.start, edge.end],
                                reach(ti, tj),
                            ),
                            depth,
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

/// Findings for the conflicts without a common node: one per pair of types and layout.
pub fn findings(lif: &Lif, map: &ConflictMap) -> Vec<Finding> {
    let mut out = Vec::new();
    for pair in &map.pairs {
        for junction in [false, true] {
            let open: Vec<&Conflict> = pair
                .conflicts
                .iter()
                .filter(|c| !c.shares_node && c.junction_nodes.is_some() == junction)
                .collect();
            if open.is_empty() {
                continue;
            }
            for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
                report_layout(layout, pair, &open, junction, &mut out);
            }
        }
    }
    for pair in &map.node_pairs {
        for junction in [false, true] {
            let sel: Vec<&NodeConflict> = pair
                .conflicts
                .iter()
                .filter(|c| c.junction_nodes.is_some() == junction)
                .collect();
            if sel.is_empty() {
                continue;
            }
            for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
                report_node_layout(layout, pair, &sel, junction, &mut out);
            }
        }
    }
    out
}

/// One finding per pair of types, layout and kind: robots standing on nodes against
/// edges. `junction`: the conflicts near a junction (info) or the others (warning).
fn report_node_layout(
    layout: &Layout,
    pair: &NodeTypePair,
    sel: &[&NodeConflict],
    junction: bool,
    out: &mut Vec<Finding>,
) {
    let nodes: HashMap<&str, ()> = layout.nodes.iter().map(|n| (n.node_id.as_str(), ())).collect();
    let mine: Vec<&NodeConflict> = sel
        .iter()
        .copied()
        .filter(|c| nodes.contains_key(c.node.as_str()))
        .collect();
    let Some(deepest) = mine.iter().max_by(|a, b| a.depth.total_cmp(&b.depth)) else {
        return;
    };
    let mut ids: Vec<&str> = Vec::new();
    for c in &mine {
        for id in [c.node.as_str(), c.edge.as_str()] {
            if !ids.contains(&id) {
                ids.push(id);
            }
        }
    }
    let idx = LayoutIndex::new(layout);
    let mut places = idx.nodes(mine.iter().map(|c| c.node.as_str()));
    places.dedup_by(|a, b| a.id == b.id);
    places.extend(idx.edges(mine.iter().map(|c| c.edge.as_str())));
    let who = if pair.type_node == pair.type_edge {
        format!("a robot of {} standing on a node and another one driving an edge", pair.type_node)
    } else {
        format!(
            "a robot of {} standing on a node and a robot of {} driving an edge",
            pair.type_node, pair.type_edge
        )
    };
    let (level, kind) = if junction {
        (Level::Info, "near a junction (diverge or merge; reserve the junction as a unit)")
    } else {
        (Level::Warning, "not near a junction")
    };
    out.push(
        Finding::new(
            level,
            CHECK,
            format!(
                "{} pair(s) of node and edge (the edge does not touch the node) overlap, {kind}: {who}; deepest: node {} against edge {} by {:.2} m",
                mine.len(),
                deepest.node,
                deepest.edge,
                deepest.depth
            ),
        )
        .layout(&layout.layout_id)
        .with_places(places)
        .with_ids(ids),
    );
}

/// One finding per pair of types, layout and kind: edges without a common node.
/// `junction`: the conflicts near a junction (info, with the junction nodes as
/// places too) or the parallel ones (warning).
fn report_layout(
    layout: &Layout,
    pair: &TypePair,
    open: &[&Conflict],
    junction: bool,
    out: &mut Vec<Finding>,
) {
    let here: HashMap<&str, ()> = layout.edges.iter().map(|e| (e.edge_id.as_str(), ())).collect();
    let mine: Vec<&&Conflict> = open
        .iter()
        .filter(|c| here.contains_key(c.edge_a.as_str()))
        .collect();
    let Some(first) = mine.first() else {
        return;
    };
    let mut ids: Vec<&str> = Vec::new();
    for c in &mine {
        for id in [c.edge_a.as_str(), c.edge_b.as_str()] {
            if !ids.contains(&id) {
                ids.push(id);
            }
        }
    }
    let idx = LayoutIndex::new(layout);
    let who = if pair.type_a == pair.type_b {
        format!("two robots of {}", pair.type_a)
    } else {
        format!("a robot of {} and one of {}", pair.type_a, pair.type_b)
    };
    let (level, kind) = if junction {
        (Level::Info, "near a junction (diverge or merge; reserve the junction as a unit)")
    } else {
        (Level::Warning, "parallel, not near a junction")
    };
    let mut places = idx.edges(ids.iter().copied());
    if junction {
        // the junction nodes where the edges come together
        let mut nodes: Vec<&str> = Vec::new();
        for c in &mine {
            for n in c.junction_nodes.iter().flatten() {
                if !nodes.contains(&n.as_str()) {
                    nodes.push(n);
                }
            }
        }
        places.extend(idx.nodes(nodes));
    }
    out.push(
        Finding::new(
            level,
            CHECK,
            format!(
                "{} pair(s) of edges without a common node have overlapping swept outlines, {kind}: {who} must not use them at the same time (no node reservation covers it); first: {} and {}",
                mine.len(),
                first.edge_a,
                first.edge_b
            ),
        )
        .layout(&layout.layout_id)
        .with_places(places)
        .with_ids(ids),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use botsim_core::spec::Kinematics;

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
        let m = compute(&l, &two_types(), 0.25, 0.0).expect("map");
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
        // findings: only the Y2 pair, X1 pair has none
        let f = findings(&l, &m);
        assert!(f.iter().any(|x| x.text.contains("two robots of ACME/Y2")));
        assert!(!f.iter().any(|x| x.text.contains("two robots of ACME/X1")));
        // mixed pair X1 (E1) with Y2 (E3): 1.0 m apart, 0.3 + 0.7 = 1.0: touching counts
        assert!(pair("ACME/X1", "ACME/Y2").is_some());
    }

    #[test]
    fn json_has_the_documented_keys() {
        let l = lif(
            &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("C", 20.0, 0.0)],
            &[("E1", "A", "B"), ("E2", "B", "C")],
        );
        let m = compute(&l, &two_types(), 0.25, 0.0).expect("map");
        let v: serde_json::Value = serde_json::from_str(&to_json(&m)).expect("json");
        assert_eq!(v["sample_step"], 0.25);
        assert_eq!(v["min_overlap"], 0.0);
        assert!(v["pairs"][0]["conflicts"][0]["shares_node"].is_boolean());
        assert!(v["pairs"][0]["conflicts"][0]["depth"].is_number());
    }

    #[test]
    fn near_a_junction_is_info_and_parallel_is_a_warning() {
        let y2 = vec![TypeRef::test_rect("ACME/Y2", Kinematics::Differential, 2.4, 1.4)];
        // E3 starts 1 m beside the start of E1: they come together there (junction)
        let l = lif(
            &[("A", 0.0, 0.0), ("B", 30.0, 0.0), ("D", 0.0, 1.0), ("F", 5.0, 1.0)],
            &[("E1", "A", "B"), ("E3", "D", "F")],
        );
        let m = compute(&l, &y2, 0.25, 0.0).expect("map");
        let c = &m.pairs[0].conflicts[0];
        assert_eq!(c.junction_nodes, Some(["A".to_string(), "D".to_string()]));
        let f = findings(&l, &m);
        let edge_f: Vec<_> = f.iter().filter(|x| x.text.contains("pair(s) of edges")).collect();
        assert_eq!(edge_f.len(), 1);
        assert_eq!(edge_f[0].level, Level::Info);
        assert!(edge_f[0].places.iter().any(|p| p.kind == crate::finding::PlaceKind::Node));
        // E3 runs 1 m beside the middle of E1, its nodes 10 m or more from E1's: parallel
        let l = lif(
            &[("A", 0.0, 0.0), ("B", 30.0, 0.0), ("D", 10.0, 1.0), ("F", 20.0, 1.0)],
            &[("E1", "A", "B"), ("E3", "D", "F")],
        );
        let m = compute(&l, &y2, 0.25, 0.0).expect("map");
        assert!(m.pairs[0].conflicts[0].junction_nodes.is_none());
        let f = findings(&l, &m);
        let edge_f: Vec<_> = f.iter().filter(|x| x.text.contains("pair(s) of edges")).collect();
        assert_eq!(edge_f.len(), 1);
        assert_eq!(edge_f[0].level, Level::Warning);
        assert!(edge_f[0].text.contains("parallel"));
    }

    #[test]
    fn robot_standing_on_a_node_against_an_edge() {
        // E1 runs (0,0)-(10,0). Node F at (5,0.5) is the start of E3 going north:
        // a robot standing on F (heading north, 1.0 x 0.6) reaches down to y = 0, into
        // the swept outline of E1 (y -0.3..0.3). E3 touches F, so it is not listed.
        let l = lif(
            &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("F", 5.0, 0.5), ("G", 5.0, 10.0)],
            &[("E1", "A", "B"), ("E3", "F", "G")],
        );
        let types = vec![TypeRef::test_rect("ACME/X1", Kinematics::Differential, 1.0, 0.6)];
        let m = compute(&l, &types, 0.25, 0.0).expect("map");
        assert_eq!(m.node_pairs.len(), 1);
        let p = &m.node_pairs[0];
        assert_eq!((p.type_node.as_str(), p.type_edge.as_str()), ("ACME/X1", "ACME/X1"));
        assert!(p.conflicts.iter().all(|c| c.edge != "E3" || c.node != "F"));
        let hit = p.conflicts.iter().find(|c| c.node == "F" && c.edge == "E1").expect("F against E1");
        assert!(hit.depth > 0.25 && hit.depth <= 0.3 + 1e-9, "{}", hit.depth);
        // and it becomes a finding with the node and the edge as places
        let f = findings(&l, &m);
        let f = f.iter().find(|x| x.text.contains("standing on a node")).expect("finding");
        assert!(f.ids.contains(&"F".to_string()) && f.ids.contains(&"E1".to_string()));
        assert!(f.places.iter().any(|p| p.kind == crate::finding::PlaceKind::Node));
        assert!(f.places.iter().any(|p| p.kind == crate::finding::PlaceKind::Edge));
    }

    #[test]
    fn min_overlap_cuts_the_shallow_conflicts() {
        // Y2 bodies are 1.4 wide: E1 and E3 (1.0 m apart) overlap by 0.4 m. X1 (0.6 wide)
        // with Y2 (1.4): 1.0 m apart = exactly touching, depth 0.
        let l = lif(
            &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("D", 0.0, 1.0), ("F", 10.0, 1.0)],
            &[("E1", "A", "B"), ("E3", "D", "F")],
        );
        let all = compute(&l, &two_types(), 0.25, 0.0).expect("map");
        let yy = all.pairs.iter().find(|p| p.type_a == "ACME/Y2" && p.type_b == "ACME/Y2");
        assert!((yy.expect("Y2/Y2").conflicts[0].depth - 0.4).abs() < 1e-9);
        let cut = compute(&l, &two_types(), 0.25, 0.05).expect("map");
        // the touching X1/Y2 pair is gone, the 0.4 m one stays
        assert!(cut.pairs.iter().all(|p| p.conflicts.iter().all(|c| c.depth >= 0.05)));
        assert!(cut.pairs.iter().any(|p| p.type_a == "ACME/Y2" && p.type_b == "ACME/Y2"));
        assert!(!cut.pairs.iter().any(|p| p.type_a == "ACME/X1" && p.type_b == "ACME/Y2"));
        assert!(all.pairs.iter().any(|p| p.type_a == "ACME/X1" && p.type_b == "ACME/Y2"));
    }
}

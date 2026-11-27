//! Check: a node close to the corner of a turn arc (`docs/graph_problems.md`, G6b).
//!
//! A turn arc (an edge with a trajectory that turns by 25 to 100 degrees) bypasses the
//! corner where the tangents at its two ends meet. A node near that corner, that
//! is not one of the arc's own nodes, can be stood on by a robot beside the arc of
//! another flow (G6): the two bodies overlap but share no node. Nodes closer than
//! [`CORNER_RADIUS`] to such a corner are reported.

use std::collections::HashSet;

use botsim_model::graph::Graph;
use vdma_lif::{Layout, Lif};

use crate::conflict::ConflictMap;

use crate::finding::{Finding, Level};
use crate::places::LayoutIndex;

const CHECK: &str = "corner";

/// A node nearer than this (m) to the corner of an arc is reported (G6b: about 3 m).
pub const CORNER_RADIUS: f64 = 3.0;

/// A node this close (m) to the corner is the corner node of the junction itself.
const CORNER_NODE: f64 = 0.05;

/// Turns between these angles (degrees) are turn arcs.
const MIN_TURN: f64 = 25.0;
const MAX_TURN: f64 = 100.0;

struct Near {
    arc: usize,
    node: usize,
    distance: f64,
    /// A robot on the node overlaps the swept outline of the arc (conflict map).
    overlaps: bool,
}

/// `map`: the conflict map. A node near the corner of an arc is a `warning` only if
/// a robot standing on the node overlaps the swept outline of that arc (a node-edge
/// conflict in the map); otherwise it is `info`. Without a map there is no evidence
/// of an overlap, so all are `info`.
pub fn check(lif: &Lif, map: Option<&ConflictMap>) -> Vec<Finding> {
    // A layout that cannot be turned into a graph is reported by the reachability check.
    let Ok(graph) = Graph::from_lif(lif) else {
        return Vec::new();
    };
    let overlapping: HashSet<(&str, &str)> = map
        .into_iter()
        .flat_map(|m| {
            m.node_pairs
                .iter()
                .flat_map(|p| &p.conflicts)
                .filter(move |c| crate::conflict::is_overlap(m, c.depth, c.distance))
        })
        .map(|c| (c.node.as_str(), c.edge.as_str()))
        .collect();
    let mut near: Vec<Near> = Vec::new();
    for (e, edge) in graph.edges.iter().enumerate() {
        if edge.map_transition {
            continue;
        }
        let Some(curve) = edge.types.iter().find_map(|p| p.trajectory.as_ref()) else {
            continue;
        };
        let (d0, d1) = curve.end_tangents();
        let turn = (d1 - d0).sin().atan2((d1 - d0).cos()).to_degrees().abs();
        if !(MIN_TURN..=MAX_TURN).contains(&turn) {
            continue;
        }
        let (a, b) = (&graph.nodes[edge.start], &graph.nodes[edge.end]);
        let Some(corner) = corner((a.x, a.y), d0, (b.x, b.y), d1) else {
            continue;
        };
        for (n, node) in graph.nodes.iter().enumerate() {
            if node.map != a.map || n == edge.start || n == edge.end {
                continue;
            }
            let distance = (node.x - corner.0).hypot(node.y - corner.1);
            // a node exactly at the corner is the junction's own corner node (G6), not G6b
            if distance > CORNER_NODE && distance < CORNER_RADIUS {
                near.push(Near {
                    arc: e,
                    node: n,
                    distance,
                    overlaps: overlapping.contains(&(node.id.as_str(), edge.id.as_str())),
                });
            }
        }
    }
    let mut out = Vec::new();
    for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
        for overlaps in [true, false] {
            report_layout(layout, &graph, &near, overlaps, &mut out);
        }
    }
    out
}

/// Where the line from `p0` in direction `d0` meets the line that reaches `p1` in
/// direction `d1`; `None` if they are parallel or meet behind either end.
fn corner(p0: (f64, f64), d0: f64, p1: (f64, f64), d1: f64) -> Option<(f64, f64)> {
    let (u, v) = ((d0.cos(), d0.sin()), (d1.cos(), d1.sin()));
    let denom = u.0 * v.1 - u.1 * v.0;
    if denom.abs() < 1e-6 {
        return None;
    }
    let w = (p1.0 - p0.0, p1.1 - p0.1);
    let t = (w.0 * v.1 - w.1 * v.0) / denom; // along u from p0
    let s = (u.0 * w.1 - u.1 * w.0) / denom; // back along v from p1
    (t > 0.0 && s > 0.0).then(|| (p0.0 + t * u.0, p0.1 + t * u.1))
}

/// One finding per layout and kind: the nodes whose body overlaps the arc (warning)
/// or not (info).
fn report_layout(
    layout: &Layout,
    g: &Graph,
    near: &[Near],
    overlaps: bool,
    out: &mut Vec<Finding>,
) {
    let nodes: HashSet<&str> = layout.nodes.iter().map(|n| n.node_id.as_str()).collect();
    let mine: Vec<&Near> = near
        .iter()
        .filter(|n| n.overlaps == overlaps && nodes.contains(g.nodes[n.node].id.as_str()))
        .collect();
    let Some(closest) = mine.iter().min_by(|a, b| a.distance.total_cmp(&b.distance)) else {
        return;
    };
    let mut node_ids: Vec<&str> = Vec::new();
    let mut arc_ids: Vec<&str> = Vec::new();
    for n in &mine {
        let (nid, aid) = (g.nodes[n.node].id.as_str(), g.edges[n.arc].id.as_str());
        if !node_ids.contains(&nid) {
            node_ids.push(nid);
        }
        if !arc_ids.contains(&aid) {
            arc_ids.push(aid);
        }
    }
    let idx = LayoutIndex::new(layout);
    let mut places = idx.nodes(node_ids.iter().copied());
    places.extend(idx.edges(arc_ids.iter().copied()));
    let mut ids = node_ids.clone();
    ids.extend(arc_ids.iter().copied());
    let (level, kind) = if overlaps {
        (
            Level::Warning,
            "a robot standing there overlaps the swept outline of the arc",
        )
    } else {
        (Level::Info, "no overlap with the arc's swept outline found")
    };
    out.push(
        Finding::new(
            level,
            CHECK,
            format!(
                "{} node(s) lie within {CORNER_RADIUS} m of the corner of a turn arc ({} arcs) without being its start or end, {kind}; closest: node {} is {:.1} m from the corner of arc {}",
                node_ids.len(),
                arc_ids.len(),
                g.nodes[closest.node].id,
                closest.distance,
                g.edges[closest.arc].id
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

    /// A 90 degree turn arc from A (0,0) heading east to B (10,10) heading north: the corner
    /// is (10, 0). Node C is at `c`.
    fn lif(c: (f64, f64)) -> Lif {
        let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}"#;
        // quadratic Bezier as a degree 2 NURBS with control points A, corner, B
        let traj = r#""trajectory":{"degree":2,"knotVector":[0,0,0,1,1,1],"controlPoints":[{"x":0,"y":0},{"x":10,"y":0},{"x":10,"y":10}]}"#;
        serde_json::from_str(&format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
            "nodes":[{{"nodeId":"A","mapId":"M","nodePosition":{{"x":0,"y":0}},"mobileRobotTypeNodeProperties":[{ty}]}},
                     {{"nodeId":"B","mapId":"M","nodePosition":{{"x":10,"y":10}},"mobileRobotTypeNodeProperties":[{ty}]}},
                     {{"nodeId":"C","mapId":"M","nodePosition":{{"x":{},"y":{}}},"mobileRobotTypeNodeProperties":[{ty}]}},
                     {{"nodeId":"D","mapId":"M","nodePosition":{{"x":{},"y":{}}},"mobileRobotTypeNodeProperties":[{ty}]}}],
            "edges":[{{"edgeId":"CD","startNodeId":"C","endNodeId":"D","mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]}}]}},
                     {{"edgeId":"ARC","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}],{traj}}}]}}]}}]}}]}}"#,
            c.0,
            c.1,
            // D: 40 m further on, so that C has an edge (a heading to stand with)
            c.0 + 40.0,
            c.1
        ))
        .expect("test LIF parses")
    }

    fn map_for(l: &Lif) -> ConflictMap {
        // a robot 1.0 x 0.6 on every node (so C at the corner reaches into the arc)
        let t = crate::reach::TypeRef::test_rect(
            "ACME/X1",
            botsim_model::spec::Kinematics::Differential,
            1.0,
            0.6,
        );
        crate::conflict::compute(
            l,
            &[t],
            &crate::conflict::MapOptions::overlap_only(0.25, 0.0),
        )
        .expect("map")
    }

    #[test]
    fn node_near_the_corner_of_an_arc() {
        // C 1.5 m from the corner (10, 0): near. Without a conflict map no overlap is known: info.
        let l = lif((10.0, 1.5));
        let f = check(&l, None);
        assert_eq!(f.len(), 1, "{f:?}");
        assert_eq!(f[0].level, Level::Info);
        assert!(f[0].ids.contains(&"C".to_string()) && f[0].ids.contains(&"ARC".to_string()));
        assert!(f[0].text.contains("1.5 m from the corner"), "{}", f[0].text);
        // C 4 m from the corner: fine; the arc's own nodes A and B (10 m away) never count
        assert!(check(&lif((10.0, 4.0)), None).is_empty());
    }

    #[test]
    fn warning_only_if_the_body_overlaps_the_arc() {
        // the arc passes about 2.9 m from the corner (10, 0) at its middle (7.5, 2.5 -> no: (7.5,2.5)),
        // a node at (10, 1.5) is 1.5 m from the corner; its body (1.0 x 0.6) is clear of the arc:
        let l = lif((10.0, 1.5));
        let m = map_for(&l);
        let f = check(&l, Some(&m));
        assert!(f.iter().all(|x| x.level == Level::Info), "{f:?}");
        // a node ON the arc's middle (7.5, 2.5) is 3.5 m from the corner: out of range, but
        // a node at (8.0, 2.0) (3.6 m... ) -> use one 2.9 m from the corner that sits on the arc
        let l = lif((8.0, 2.0));
        let m = map_for(&l);
        let f = check(&l, Some(&m));
        // (8,2) is 2.83 m from the corner and 0.35 m from the arc's middle (7.5, 2.5): overlap
        let w: Vec<_> = f.iter().filter(|x| x.level == Level::Warning).collect();
        assert_eq!(w.len(), 1, "{f:?}");
        assert!(
            w[0].text.contains("overlaps the swept outline"),
            "{}",
            w[0].text
        );
    }
}

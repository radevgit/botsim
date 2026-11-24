//! Check 5: the findings of the conflict map.
//!
//! The map itself (swept outlines, overlaps, clearance) is computed by `botsim-sweep`
//! and re-exported here. This module turns it into findings. For every pair of robot
//! types and every pair of edges without a common node whose swept outlines overlap
//! (the uncovered kind of `docs/graph_problems.md`: G5, G6, G11) there is a finding;
//! near a junction (a diverge or merge) it is `info`, otherwise `warning`.
//! Conflicts that exist only because of the clearance margin (the bodies do not
//! overlap) and overlaps shallower than `min_overlap` are in the map, not in the
//! findings.

use std::collections::HashMap;

use vdma_lif::{Layout, Lif};

pub use botsim_sweep::conflict::{
    CLEARANCE, Conflict, ConflictMap, JUNCTION_HOPS, MIN_OVERLAP, MapOptions, NodeConflict,
    NodeTypePair, TypePair, compute, to_json,
};

use crate::finding::{Finding, Level};
use crate::places::LayoutIndex;

const CHECK: &str = "conflict";

/// A map entry the checker reports: the bodies overlap, by at least `min_overlap`.
pub(crate) fn is_overlap(map: &ConflictMap, depth: f64, distance: f64) -> bool {
    distance == 0.0 && depth >= map.min_overlap
}

/// Findings for the conflicts without a common node: one per pair of types and layout.
pub fn findings(lif: &Lif, map: &ConflictMap) -> Vec<Finding> {
    let mut out = Vec::new();
    for pair in &map.pairs {
        for class in Class::ALL {
            let open: Vec<&Conflict> = pair
                .conflicts
                .iter()
                .filter(|c| {
                    !c.shares_node
                        && is_overlap(map, c.depth, c.distance)
                        && Class::of(&c.junction_nodes, c.depth) == class
                })
                .collect();
            if open.is_empty() {
                continue;
            }
            for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
                report_layout(layout, pair, &open, class, &mut out);
            }
        }
    }
    for pair in &map.node_pairs {
        for class in Class::ALL {
            let sel: Vec<&NodeConflict> = pair
                .conflicts
                .iter()
                .filter(|c| {
                    is_overlap(map, c.depth, c.distance)
                        && Class::of(&c.junction_nodes, c.depth) == class
                })
                .collect();
            if sel.is_empty() {
                continue;
            }
            for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
                report_node_layout(layout, pair, &sel, class, &mut out);
            }
        }
    }
    out
}

/// An overlap near a junction at least this deep (m) is a `warning`, not `info`: it
/// is more than the two outlines touching on the turn itself.
pub const DEEP_OVERLAP: f64 = 0.2;

/// How a conflict is reported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Class {
    /// Not near a junction: `warning`.
    Parallel,
    /// Near a junction, overlap of [`DEEP_OVERLAP`] or more: `warning`.
    JunctionDeep,
    /// Near a junction, shallower: `info`.
    JunctionShallow,
}

impl Class {
    const ALL: [Class; 3] = [Class::Parallel, Class::JunctionDeep, Class::JunctionShallow];

    fn of(junction_nodes: &Option<[String; 2]>, depth: f64) -> Class {
        match (junction_nodes, depth >= DEEP_OVERLAP) {
            (None, _) => Class::Parallel,
            (Some(_), true) => Class::JunctionDeep,
            (Some(_), false) => Class::JunctionShallow,
        }
    }

    fn level_and_text(self) -> (Level, String) {
        match self {
            Class::Parallel => (Level::Warning, "parallel, not near a junction".into()),
            Class::JunctionDeep => (
                Level::Warning,
                format!(
                    "near a junction but overlapping by {DEEP_OVERLAP} m or more (the junction reservation may not be enough)"
                ),
            ),
            Class::JunctionShallow => (
                Level::Info,
                "near a junction (diverge or merge; reserve the junction as a unit)".into(),
            ),
        }
    }
}

/// One finding per pair of types, layout and kind: robots standing on nodes against
/// edges. `junction`: the conflicts near a junction (info) or the others (warning).
fn report_node_layout(
    layout: &Layout,
    pair: &NodeTypePair,
    sel: &[&NodeConflict],
    class: Class,
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
    let (level, kind) = class.level_and_text();
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
    class: Class,
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
    let (level, kind) = class.level_and_text();
    let deepest = mine.iter().map(|c| c.depth).fold(0.0, f64::max);
    let mut places = idx.edges(ids.iter().copied());
    if class != Class::Parallel {
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
                "{} pair(s) of edges without a common node have overlapping swept outlines, {kind}: {who} must not use them at the same time (no node reservation covers it); deepest overlap {deepest:.2} m; first: {} and {}",
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
    use botsim_sweep::TypeRef;

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

    fn overlap_only() -> MapOptions {
        MapOptions::overlap_only(0.25, 0.0)
    }

    #[test]
    fn uncovered_conflicts_become_findings() {
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
        let m = compute(&l, &two_types(), &overlap_only()).expect("map");
        // findings: only the Y2 pair, X1 pair has none
        let f = findings(&l, &m);
        assert!(f.iter().any(|x| x.text.contains("two robots of ACME/Y2")));
        assert!(!f.iter().any(|x| x.text.contains("two robots of ACME/X1")));
    }

    #[test]
    fn near_a_junction_is_info_and_parallel_is_a_warning() {
        let y2 = vec![TypeRef::test_rect("ACME/Y2", Kinematics::Differential, 2.4, 1.4)];
        // E3 starts 1.3 m beside the start of E1 (overlap 0.1 m): they come together
        // there (junction), shallow
        let l = lif(
            &[("A", 0.0, 0.0), ("B", 30.0, 0.0), ("D", 0.0, 1.3), ("F", 5.0, 1.3)],
            &[("E1", "A", "B"), ("E3", "D", "F")],
        );
        let m = compute(&l, &y2, &overlap_only()).expect("map");
        let f = findings(&l, &m);
        let edge_f: Vec<_> = f.iter().filter(|x| x.text.contains("pair(s) of edges")).collect();
        assert_eq!(edge_f.len(), 1);
        assert_eq!(edge_f[0].level, Level::Info);
        assert!(edge_f[0].text.contains("deepest overlap 0.10 m"), "{}", edge_f[0].text);
        assert!(edge_f[0].places.iter().any(|p| p.kind == crate::finding::PlaceKind::Node));
        // 1.0 m beside it (overlap 0.4 m): still a junction, but deep: warning
        let l = lif(
            &[("A", 0.0, 0.0), ("B", 30.0, 0.0), ("D", 0.0, 1.0), ("F", 5.0, 1.0)],
            &[("E1", "A", "B"), ("E3", "D", "F")],
        );
        let m = compute(&l, &y2, &overlap_only()).expect("map");
        let f = findings(&l, &m);
        let edge_f: Vec<_> = f.iter().filter(|x| x.text.contains("pair(s) of edges")).collect();
        assert_eq!(edge_f.len(), 1);
        assert_eq!(edge_f[0].level, Level::Warning);
        assert!(edge_f[0].text.contains("near a junction"), "{}", edge_f[0].text);
        assert!(edge_f[0].places.iter().any(|p| p.kind == crate::finding::PlaceKind::Node));
        // E3 runs 1 m beside the middle of E1, its nodes 10 m or more from E1's: parallel
        let l = lif(
            &[("A", 0.0, 0.0), ("B", 30.0, 0.0), ("D", 10.0, 1.0), ("F", 20.0, 1.0)],
            &[("E1", "A", "B"), ("E3", "D", "F")],
        );
        let m = compute(&l, &y2, &overlap_only()).expect("map");
        let f = findings(&l, &m);
        let edge_f: Vec<_> = f.iter().filter(|x| x.text.contains("pair(s) of edges")).collect();
        assert_eq!(edge_f.len(), 1);
        assert_eq!(edge_f[0].level, Level::Warning);
        assert!(edge_f[0].text.contains("parallel"));
    }

    #[test]
    fn robot_standing_on_a_node_becomes_a_finding() {
        // E1 runs (0,0)-(10,0). Node F at (5,0.5) is the start of E3 going north:
        // a robot standing on F reaches into the swept outline of E1.
        let l = lif(
            &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("F", 5.0, 0.5), ("G", 5.0, 10.0)],
            &[("E1", "A", "B"), ("E3", "F", "G")],
        );
        let types = vec![TypeRef::test_rect("ACME/X1", Kinematics::Differential, 1.0, 0.6)];
        let m = compute(&l, &types, &overlap_only()).expect("map");
        let f = findings(&l, &m);
        let f = f.iter().find(|x| x.text.contains("standing on a node")).expect("finding");
        assert!(f.ids.contains(&"F".to_string()) && f.ids.contains(&"E1".to_string()));
        assert!(f.places.iter().any(|p| p.kind == crate::finding::PlaceKind::Node));
        assert!(f.places.iter().any(|p| p.kind == crate::finding::PlaceKind::Edge));
    }

    #[test]
    fn near_misses_from_the_clearance_are_in_the_map_but_not_findings() {
        // X1 is 0.6 wide; E3 runs 0.65 m beside E1: a 5 cm gap, no overlap.
        let l = lif(
            &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("D", 0.0, 0.65), ("F", 10.0, 0.65)],
            &[("E1", "A", "B"), ("E3", "D", "F")],
        );
        let x1 = vec![TypeRef::test_rect("ACME/X1", Kinematics::Differential, 1.0, 0.6)];
        let m = compute(&l, &x1, &MapOptions::default()).expect("map");
        assert!(!m.pairs.is_empty());
        assert!(findings(&l, &m).is_empty());
    }

    #[test]
    fn overlaps_shallower_than_min_overlap_are_no_findings() {
        // Y2 is 1.4 wide: E1 and E3, 1.38 m apart, overlap by 2 cm (below 5 cm), but
        // the 10 cm clearance puts the pair in the map.
        let l = lif(
            &[("A", 0.0, 0.0), ("B", 10.0, 0.0), ("D", 0.0, 1.38), ("F", 10.0, 1.38)],
            &[("E1", "A", "B"), ("E3", "D", "F")],
        );
        let y2 = vec![TypeRef::test_rect("ACME/Y2", Kinematics::Differential, 2.4, 1.4)];
        let m = compute(&l, &y2, &MapOptions::default()).expect("map");
        assert_eq!(m.pairs[0].conflicts.len(), 1);
        assert!(findings(&l, &m).is_empty());
    }
}

//! Check: edges that are too long (`docs/graph_problems.md`, G4).
//!
//! The baseline reserves an edge for one robot at a time, so a long road is one
//! robot long: a convoy cannot form and the robots behind queue far apart.
//! Straight edges longer than [`LIMIT_FACTOR`] times the longest fleet robot
//! allowed on them are reported.

use botsim_core::graph::Graph;
use vdma_lif::{Layout, Lif};

use crate::finding::{Finding, Level};
use crate::places::LayoutIndex;
use crate::reach::TypeRef;

const CHECK: &str = "length";

/// An edge is too long above this many robot lengths (G4: 2.5, i.e. 6 m for a forklift).
pub const LIMIT_FACTOR: f64 = 2.5;

struct Long {
    edge: usize,
    limit: f64,
    robot: String,
}

pub fn check(lif: &Lif, types: &[TypeRef]) -> Vec<Finding> {
    // A layout that cannot be turned into a graph is reported by the reachability check.
    let Ok(graph) = Graph::from_lif(lif) else {
        return Vec::new();
    };
    let mut long: Vec<Long> = Vec::new();
    // Total length of straight road a fleet robot may drive (edges, one direction each).
    let mut total = 0.0;
    for (e, edge) in graph.edges.iter().enumerate() {
        if edge.map_transition {
            continue;
        }
        let allowed: Vec<&TypeRef> = types
            .iter()
            .filter(|t| graph.edge_allows(e, &t.manufacturer, &t.series))
            .collect();
        let curved = allowed.iter().any(|t| {
            edge.props_for(&t.manufacturer, &t.series)
                .is_some_and(|p| p.trajectory.is_some())
        });
        if allowed.is_empty() || curved {
            continue;
        }
        total += edge.length;
        let Some(longest) = allowed.iter().max_by(|a, b| a.length.total_cmp(&b.length)) else {
            continue;
        };
        let limit = LIMIT_FACTOR * longest.length;
        if edge.length > limit {
            long.push(Long {
                edge: e,
                limit,
                robot: longest.name.clone(),
            });
        }
    }
    let mut out = Vec::new();
    for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
        report_layout(layout, &graph, &long, total, &mut out);
    }
    out
}

fn report_layout(
    layout: &Layout,
    graph: &Graph,
    long: &[Long],
    total: f64,
    out: &mut Vec<Finding>,
) {
    let mine: Vec<&Long> = long
        .iter()
        .filter(|l| {
            layout
                .edges
                .iter()
                .any(|e| e.edge_id == graph.edges[l.edge].id)
        })
        .collect();
    let Some(longest) = mine.iter().max_by(|a, b| {
        graph.edges[a.edge]
            .length
            .total_cmp(&graph.edges[b.edge].length)
    }) else {
        return;
    };
    let covered: f64 = mine.iter().map(|l| graph.edges[l.edge].length).sum();
    let ids: Vec<&str> = mine
        .iter()
        .map(|l| graph.edges[l.edge].id.as_str())
        .collect();
    let idx = LayoutIndex::new(layout);
    out.push(
        Finding::new(
            Level::Warning,
            CHECK,
            format!(
                "{} straight edge(s) are longer than {LIMIT_FACTOR} times the longest robot allowed on them; together {:.0} m of {:.0} m straight road ({:.0} %); longest: {} is {:.1} m, limit {:.1} m for {}",
                mine.len(),
                covered,
                total,
                if total > 0.0 { 100.0 * covered / total } else { 0.0 },
                graph.edges[longest.edge].id,
                graph.edges[longest.edge].length,
                longest.limit,
                longest.robot
            ),
        )
        .layout(&layout.layout_id)
        .with_places(idx.edges(ids.iter().copied()))
        .with_ids(ids),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use botsim_core::spec::Kinematics;

    fn lif(len: f64) -> Lif {
        let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}"#;
        serde_json::from_str(&format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
            "nodes":[{{"nodeId":"A","mapId":"M","nodePosition":{{"x":0,"y":0}},"mobileRobotTypeNodeProperties":[{ty}]}},
                     {{"nodeId":"B","mapId":"M","nodePosition":{{"x":{len},"y":0}},"mobileRobotTypeNodeProperties":[{ty}]}}],
            "edges":[{{"edgeId":"E1","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[{ty}]}}]}}]}}]}}"#
        ))
        .expect("test LIF parses")
    }

    fn x1(length: f64) -> TypeRef {
        TypeRef::test_rect("ACME/X1", Kinematics::Differential, length, 1.0)
    }

    #[test]
    fn long_edge_is_a_warning() {
        let f = check(&lif(10.0), &[x1(2.4)]);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].level, Level::Warning);
        assert_eq!(f[0].ids, ["E1"]);
        assert!(f[0].text.contains("limit 6.0 m"), "{}", f[0].text);
        assert!(f[0].text.contains("(100 %)"), "{}", f[0].text);
    }

    #[test]
    fn edge_within_the_limit_is_fine() {
        assert!(check(&lif(6.0), &[x1(2.4)]).is_empty());
        // no fleet robot may use the edge: nothing to say
        let mut other = x1(2.4);
        other.series = "Q9".into();
        assert!(check(&lif(50.0), &[other]).is_empty());
    }
}

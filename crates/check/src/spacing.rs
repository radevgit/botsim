//! Check: nodes closer together than a robot is long (`docs/graph_problems.md`, G3).
//!
//! The baseline lets a robot stand on every node it has been given, so two robots
//! can stand on neighbouring nodes. If the nodes are closer than the length of the
//! longest robot that may drive the edge between them (plus a margin for the
//! protective field), the bodies overlap.

use vdma_lif::{Layout, Lif};
use botsim_core::graph::Graph;

use crate::finding::{Finding, Level};
use crate::places::LayoutIndex;
use crate::reach::TypeRef;

const CHECK: &str = "spacing";

/// One edge whose end nodes are too close.
struct Tight<'a> {
    edge: usize,
    needed: f64,
    robot: &'a str,
    /// The factsheet has no protective field: the simulator's default margin was used.
    default_field: bool,
}

/// The distance two standing robots of a type need between their nodes: the
/// body plus the reach of the protective field of the one behind.
fn needed(t: &TypeRef) -> f64 {
    t.length + t.field_margin
}

pub fn check(lif: &Lif, types: &[TypeRef]) -> Vec<Finding> {
    // A layout that cannot be turned into a graph is reported by the reachability check.
    let Ok(graph) = Graph::from_lif(lif) else {
        return Vec::new();
    };
    let mut tight: Vec<Tight> = Vec::new();
    for (e, edge) in graph.edges.iter().enumerate() {
        if edge.map_transition {
            continue;
        }
        let longest = types
            .iter()
            .filter(|t| graph.edge_allows(e, &t.manufacturer, &t.series))
            .max_by(|a, b| needed(a).total_cmp(&needed(b)));
        if let Some(t) = longest {
            let needed = needed(t);
            if edge.length < needed {
                tight.push(Tight {
                    edge: e,
                    needed,
                    robot: &t.name,
                    default_field: !t.field_from_factsheet,
                });
            }
        }
    }
    let mut out = Vec::new();
    for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
        report_layout(layout, &graph, &tight, &mut out);
    }
    out
}

fn report_layout(layout: &Layout, graph: &Graph, tight: &[Tight], out: &mut Vec<Finding>) {
    let mine: Vec<&Tight> = tight
        .iter()
        .filter(|t| layout.edges.iter().any(|e| e.edge_id == graph.edges[t.edge].id))
        .collect();
    let Some(first) = mine.first() else {
        return;
    };
    let ids: Vec<&str> = mine.iter().map(|t| graph.edges[t.edge].id.as_str()).collect();
    let mut defaults: Vec<&str> = Vec::new();
    for t in mine.iter().filter(|t| t.default_field) {
        if !defaults.contains(&t.robot) {
            defaults.push(t.robot);
        }
    }
    let note = if defaults.is_empty() {
        String::new()
    } else {
        format!(
            "; for {} the factsheet has no protective field, so the simulator's default margin ({} m) was used",
            defaults.join(", "),
            botsim_core::safety::FIELD_MARGIN
        )
    };
    let idx = LayoutIndex::new(layout);
    out.push(
        Finding::new(
            Level::Warning,
            CHECK,
            format!(
                "{} edge(s) have end nodes closer together than the longest robot allowed on them plus its protective field; first: {} is {:.2} m long, {} needs {:.2} m{note}",
                mine.len(),
                graph.edges[first.edge].id,
                graph.edges[first.edge].length,
                first.robot,
                first.needed
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
        let mut t = TypeRef::test_rect("ACME/X1", Kinematics::Differential, length, 1.0);
        t.field_margin = 0.6;
        t.field_from_factsheet = true;
        t
    }

    #[test]
    fn short_edge_is_a_warning() {
        let f = check(&lif(2.0), &[x1(2.4)]);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].level, Level::Warning);
        assert_eq!(f[0].ids, ["E1"]);
        assert_eq!(f[0].places.len(), 1);
        assert!(f[0].text.contains("needs 3.00 m"), "{}", f[0].text);
        assert!(!f[0].text.contains("default margin"), "factsheet field: no note");
        let mut d = x1(2.4);
        d.field_from_factsheet = false;
        let f = check(&lif(2.0), &[d]);
        assert!(f[0].text.contains("ACME/X1 the factsheet has no protective field"), "{}", f[0].text);
    }

    #[test]
    fn long_enough_edge_is_fine() {
        assert!(check(&lif(4.0), &[x1(2.4)]).is_empty());
        // a type that may not use the edge does not count
        let mut other = x1(10.0);
        other.series = "Q9".into();
        assert!(check(&lif(4.0), &[x1(2.4), other]).is_empty());
    }
}

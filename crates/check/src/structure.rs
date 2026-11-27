//! Check 1: structure of the layout.

use std::collections::{BTreeMap, HashSet};

use vdma_lif::Lif;

use crate::finding::{Finding, Level};
use crate::places::LayoutIndex;

const CHECK: &str = "structure";

/// Run the structure rules on every layout of the LIF file. `fleet_types` are
/// the robot types of the scenario as `manufacturer/seriesName`.
pub fn check(lif: &Lif, fleet_types: &[String]) -> Vec<Finding> {
    let mut out = Vec::new();
    duplicate_ids(lif, &mut out);
    let nodes: HashSet<&str> = lif
        .origins
        .iter()
        .flat_map(|o| &o.layouts)
        .flat_map(|l| &l.nodes)
        .map(|n| n.node_id.as_str())
        .collect();
    for origin in &lif.origins {
        for layout in &origin.layouts {
            let idx = LayoutIndex::new(layout);
            // Rule: robot types named in edge properties exist in the fleet
            // (warning: a layout may serve more types than the scenario uses).
            let mut unknown: BTreeMap<String, Vec<&str>> = BTreeMap::new();
            for e in &layout.edges {
                for p in &e.mobile_robot_type_edge_properties {
                    for t in &p.mobile_robot_types {
                        let name = format!("{}/{}", t.manufacturer, t.series_name);
                        if !fleet_types.contains(&name) {
                            let ids = unknown.entry(name).or_default();
                            if !ids.contains(&e.edge_id.as_str()) {
                                ids.push(&e.edge_id);
                            }
                        }
                    }
                }
            }
            for (name, edges) in unknown {
                out.push(
                    Finding::new(
                        Level::Warning,
                        CHECK,
                        format!(
                            "robot type \"{name}\" is used by {} edge(s) but is not in the fleet",
                            edges.len()
                        ),
                    )
                    .layout(&layout.layout_id)
                    .robot(&name)
                    .with_places(idx.edges(edges.iter().copied()))
                    .with_ids(edges),
                );
            }

            // Rule: edges without properties for a fleet type (info: the type may
            // not use them; reachability decides whether that is a problem).
            let mut seen: HashSet<&str> = HashSet::new();
            for name in fleet_types {
                if !seen.insert(name.as_str()) {
                    continue;
                }
                let without: Vec<&str> = layout
                    .edges
                    .iter()
                    .filter(|e| {
                        !e.mobile_robot_type_edge_properties.iter().any(|p| {
                            p.mobile_robot_types
                                .iter()
                                .any(|t| format!("{}/{}", t.manufacturer, t.series_name) == *name)
                        })
                    })
                    .map(|e| e.edge_id.as_str())
                    .collect();
                if !without.is_empty() {
                    out.push(
                        Finding::new(
                            Level::Info,
                            CHECK,
                            format!(
                                "{} of {} edge(s) have no properties for this robot type, so it cannot use them",
                                without.len(),
                                layout.edges.len()
                            ),
                        )
                        .layout(&layout.layout_id)
                        .robot(name)
                        .with_places(idx.edges(without.iter().copied()))
                        .with_ids(without),
                    );
                }
            }

            // Rule: edges refer to existing nodes. The node may be in another layout
            // of the file: lift edges lead from one layout (map) to another.
            for e in &layout.edges {
                for (what, id) in [("start", &e.start_node_id), ("end", &e.end_node_id)] {
                    if !nodes.contains(id.as_str()) {
                        out.push(
                            Finding::new(
                                Level::Error,
                                CHECK,
                                format!("edge {what} node \"{id}\" does not exist in the layout"),
                            )
                            .layout(&layout.layout_id)
                            .with_places(idx.edges([e.edge_id.as_str()]))
                            .with_ids([e.edge_id.clone(), id.clone()]),
                        );
                    }
                }
            }
        }
    }
    out
}

/// Rule: node, edge and station ids are unique across all layouts of the LIF
/// file (the LIF spec says so; the vdma-lif parser does not check it).
fn duplicate_ids(lif: &Lif, out: &mut Vec<Finding>) {
    let mut nodes: HashSet<&str> = HashSet::new();
    let mut edges: HashSet<&str> = HashSet::new();
    let mut stations: HashSet<&str> = HashSet::new();
    for origin in &lif.origins {
        for layout in &origin.layouts {
            let idx = LayoutIndex::new(layout);
            let mut report = |kind: &str, id: &str| {
                let places = match kind {
                    "node" => idx.nodes([id]),
                    "edge" => idx.edges([id]),
                    _ => idx.stations([id]),
                };
                out.push(
                    Finding::new(
                        Level::Error,
                        CHECK,
                        format!("{kind} id \"{id}\" is used more than once in the LIF file"),
                    )
                    .layout(&layout.layout_id)
                    .with_places(places)
                    .with_ids([id]),
                );
            };
            for n in &layout.nodes {
                if !nodes.insert(&n.node_id) {
                    report("node", &n.node_id);
                }
            }
            for e in &layout.edges {
                if !edges.insert(&e.edge_id) {
                    report("edge", &e.edge_id);
                }
            }
            for s in layout.stations.iter().flatten() {
                if !stations.insert(&s.station_id) {
                    report("station", &s.station_id);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A layout with nodes A and B and the given edges `(edgeId, start, end)`.
    fn lif(edges: &[(&str, &str, &str)]) -> Lif {
        let node = |id: &str| {
            format!(
                r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":0,"y":0}},"mobileRobotTypeNodeProperties":[]}}"#
            )
        };
        let edges: Vec<String> = edges
            .iter()
            .map(|(id, s, e)| {
                format!(
                    r#"{{"edgeId":"{id}","startNodeId":"{s}","endNodeId":"{e}","mobileRobotTypeEdgeProperties":[]}}"#
                )
            })
            .collect();
        let text = format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1","nodes":[{},{}],"edges":[{}]}}]}}]}}"#,
            node("A"),
            node("B"),
            edges.join(",")
        );
        serde_json::from_str(&text).expect("test LIF parses")
    }

    #[test]
    fn good_edges_give_no_findings() {
        assert!(check(&lif(&[("E1", "A", "B")]), &[]).is_empty());
    }

    #[test]
    fn robot_type_not_in_fleet_is_a_warning() {
        let text = r#"{"metaInformation":{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"},
            "origins":[{"originId":"O","layouts":[{"layoutId":"L","layoutVersion":"1",
            "nodes":[{"nodeId":"A","mapId":"M","nodePosition":{"x":0,"y":0},"mobileRobotTypeNodeProperties":[]},
                     {"nodeId":"B","mapId":"M","nodePosition":{"x":1,"y":0},"mobileRobotTypeNodeProperties":[]}],
            "edges":[
              {"edgeId":"E1","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[
                 {"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"},{"manufacturer":"ACME","seriesName":"Q9"}]}]},
              {"edgeId":"E2","startNodeId":"B","endNodeId":"A","mobileRobotTypeEdgeProperties":[
                 {"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"Q9"}]}]}]}]}]}"#;
        let lif: Lif = serde_json::from_str(text).expect("test LIF parses");
        let f = check(&lif, &["ACME/X1".to_string()]);
        let warnings: Vec<_> = f.iter().filter(|x| x.level == Level::Warning).collect();
        assert_eq!(warnings.len(), 1);
        assert_eq!(warnings[0].robot_type.as_deref(), Some("ACME/Q9"));
        assert_eq!(warnings[0].ids, ["E1", "E2"]);
        assert!(warnings[0].text.contains("2 edge(s)"));
    }

    #[test]
    fn edges_without_properties_for_a_fleet_type_are_info() {
        let text = r#"{"metaInformation":{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"},
            "origins":[{"originId":"O","layouts":[{"layoutId":"L","layoutVersion":"1",
            "nodes":[{"nodeId":"A","mapId":"M","nodePosition":{"x":0,"y":0},"mobileRobotTypeNodeProperties":[]},
                     {"nodeId":"B","mapId":"M","nodePosition":{"x":1,"y":0},"mobileRobotTypeNodeProperties":[]}],
            "edges":[
              {"edgeId":"E1","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[
                 {"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"},{"manufacturer":"ACME","seriesName":"Y2"}]}]},
              {"edgeId":"E2","startNodeId":"B","endNodeId":"A","mobileRobotTypeEdgeProperties":[
                 {"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}]}]}]}]}"#;
        let lif: Lif = serde_json::from_str(text).expect("test LIF parses");
        let f = check(&lif, &["ACME/X1".to_string(), "ACME/Y2".to_string()]);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].level, Level::Info);
        assert_eq!(f[0].robot_type.as_deref(), Some("ACME/Y2"));
        assert_eq!(f[0].ids, ["E2"]);
    }

    #[test]
    fn duplicate_edge_id_is_an_error() {
        let f = check(&lif(&[("E1", "A", "B"), ("E1", "B", "A")]), &[]);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].level, Level::Error);
        assert_eq!(f[0].ids, ["E1"]);
        assert!(f[0].text.contains("edge id"));
    }

    #[test]
    fn edge_to_missing_node_is_an_error() {
        let f = check(&lif(&[("E1", "A", "Z"), ("E2", "Y", "B")]), &[]);
        assert_eq!(f.len(), 2);
        assert!(f.iter().all(|x| x.level == Level::Error));
        assert_eq!(f[0].ids, ["E1", "Z"]);
        assert_eq!(f[1].ids, ["E2", "Y"]);
    }
}

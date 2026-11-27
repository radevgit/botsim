//! Check 3 (rest): doors and lifts of the scenario against the layout and the robots.
//!
//! - A `[[door]]` edge that the layout does not have: error.
//! - A robot type wider than the door's `passage_width` on an edge it may drive:
//!   warning (the simulator refuses such orders).
//! - A `[[lift]]` floor that is not a map of the layout, or a stop node that is not
//!   in the layout or not on one of the lift's floors: error.
//!
//! The scenario has no cabin size, so the cabin is not checked against the robots.

use botsim_core::graph::Graph;

use crate::finding::{Finding, Level};
use crate::input::Inputs;
use crate::places::LayoutIndex;
use crate::reach::TypeRef;

const CHECK: &str = "door";

pub fn check(inputs: &Inputs, types: &[TypeRef]) -> Vec<Finding> {
    // A layout that cannot be turned into a graph is reported by the reachability check.
    let Ok(graph) = Graph::from_lif(&inputs.lif) else {
        return Vec::new();
    };
    let indexes: Vec<LayoutIndex> = inputs
        .lif
        .origins
        .iter()
        .flat_map(|o| &o.layouts)
        .map(LayoutIndex::new)
        .collect();
    let edge_places = |id: &str| {
        indexes
            .iter()
            .find_map(|i| i.edge(id))
            .into_iter()
            .collect::<Vec<_>>()
    };
    let node_places = |id: &str| {
        indexes
            .iter()
            .find_map(|i| i.node(id))
            .into_iter()
            .collect::<Vec<_>>()
    };
    let mut out = Vec::new();

    for door in &inputs.scenario.doors {
        for edge in &door.edges {
            let Some(e) = graph.edge_idx(edge) else {
                out.push(
                    Finding::new(
                        Level::Error,
                        CHECK,
                        format!("door \"{}\": the layout has no edge \"{edge}\"", door.id),
                    )
                    .with_ids([door.id.as_str(), edge.as_str()]),
                );
                continue;
            };
            let Some(width) = door.passage_width else {
                continue;
            };
            for t in types {
                if graph.edge_allows(e, &t.manufacturer, &t.series) && t.width > width {
                    out.push(
                        Finding::new(
                            Level::Warning,
                            CHECK,
                            format!(
                                "door \"{}\": {} is {:.2} m wide, the passage is {:.2} m: it cannot use edge {edge}",
                                door.id, t.name, t.width, width
                            ),
                        )
                        .robot(&t.name)
                        .with_places(edge_places(edge))
                        .with_ids([edge.as_str()]),
                    );
                }
            }
        }
    }

    for lift in &inputs.scenario.lifts {
        for floor in &lift.floors {
            if !graph.maps.contains(floor) {
                out.push(
                    Finding::new(
                        Level::Error,
                        CHECK,
                        format!("lift \"{}\": the layout has no map \"{floor}\"", lift.id),
                    )
                    .with_ids([lift.id.as_str()]),
                );
            }
        }
        for stop in &lift.stops {
            let problem = match graph.node_idx(&stop.node) {
                None => Some("is not a node of the layout".to_string()),
                Some(n) => {
                    let map = &graph.maps[graph.nodes[n].map];
                    (!lift.floors.contains(map))
                        .then(|| format!("is on map \"{map}\", not on a floor of the lift"))
                }
            };
            if let Some(p) = problem {
                out.push(
                    Finding::new(
                        Level::Error,
                        CHECK,
                        format!("lift \"{}\": stop node \"{}\" {p}", lift.id, stop.node),
                    )
                    .with_places(node_places(&stop.node))
                    .with_ids([lift.id.as_str(), stop.node.as_str()]),
                );
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::FleetType;
    use botsim_scenario::Scenario;
    use std::path::Path;
    use vda5050::factsheet::Factsheet;
    use vdma_lif::Lif;

    fn inputs(extra: &str) -> Inputs {
        let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}"#;
        let lif: Lif = serde_json::from_str(&format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
            "nodes":[{{"nodeId":"A","mapId":"M","nodePosition":{{"x":0,"y":0}},"mobileRobotTypeNodeProperties":[{ty}]}},
                     {{"nodeId":"B","mapId":"M","nodePosition":{{"x":10,"y":0}},"mobileRobotTypeNodeProperties":[{ty}]}}],
            "edges":[{{"edgeId":"E1","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[{ty}]}}]}}]}}]}}"#
        ))
        .expect("test LIF parses");
        let scenario = Scenario::from_toml(&format!(
            r#"name = "t"
seed = 1
duration = 10.0
time_step = 0.05
start_time = 2026-01-01T00:00:00Z
lif = "x.lif.json"
[[robot]]
manufacturer = "ACME"
serial_prefix = "R"
factsheet = "f.json"
action_time = 1.0
start_nodes = ["A"]
{extra}
"#
        ))
        .expect("test scenario parses");
        let path =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/diff_robot.factsheet.json");
        let mut factsheet: Factsheet =
            serde_json::from_str(&std::fs::read_to_string(&path).expect("factsheet file"))
                .expect("factsheet parses");
        factsheet.manufacturer = "ACME".into();
        factsheet.type_specification.series_name = "X1".into();
        Inputs {
            scenario,
            lif,
            fleet: vec![FleetType {
                group: 0,
                factsheet_path: path,
                factsheet,
            }],
            geometry: None,
        }
    }

    fn types(i: &Inputs) -> Vec<TypeRef> {
        i.fleet
            .iter()
            .map(|f| crate::type_ref(f).expect("type"))
            .collect()
    }

    #[test]
    fn door_width_and_missing_door_edge() {
        // diff_robot is 0.8 m wide
        let i = inputs(
            r#"[[door]]
id = "D1"
edges = ["E1", "NOPE"]
passage_width = 0.7
open_seconds = 1.0
close_seconds = 1.0
"#,
        );
        let f = check(&i, &types(&i));
        let warn: Vec<_> = f.iter().filter(|x| x.level == Level::Warning).collect();
        let err: Vec<_> = f.iter().filter(|x| x.level == Level::Error).collect();
        assert_eq!(warn.len(), 1, "{f:?}");
        assert_eq!(warn[0].ids, ["E1"]);
        assert_eq!(warn[0].places.len(), 1);
        assert_eq!(err.len(), 1);
        assert!(err[0].text.contains("NOPE"));
        // a wide enough door: only the missing edge
        let i = inputs(
            r#"[[door]]
id = "D1"
edges = ["E1"]
passage_width = 0.9
open_seconds = 1.0
close_seconds = 1.0
"#,
        );
        assert!(check(&i, &types(&i)).is_empty());
    }

    #[test]
    fn lift_floors_and_stops() {
        let i = inputs(
            r#"[[lift]]
id = "L1"
floors = ["M", "UP"]
stops = [{ node = "A" }, { node = "Q" }]
travel_seconds_per_floor = 5.0
door_open_seconds = 1.0
door_close_seconds = 1.0
"#,
        );
        let f = check(&i, &types(&i));
        assert_eq!(f.len(), 2, "{f:?}");
        assert!(f.iter().any(|x| x.text.contains("no map \"UP\"")));
        assert!(f.iter().any(|x| x.text.contains("stop node \"Q\"")));
    }
}

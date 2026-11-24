//! Check: where the robots of the scenario start (`docs/graph_problems.md`, G8).
//!
//! For every `start_nodes` entry of a `[[robot]]` group: the node exists, the
//! type may stand on it, the robot can drive off in its start heading, and the
//! node is in the type's biggest connected set. Start nodes of all groups must
//! be at least [`MIN_START_DISTANCE`] apart.

use std::collections::HashMap;

use vdma_lif::Lif;

use crate::finding::{Finding, FindingPlace, Level};
use crate::input::Inputs;
use crate::places::LayoutIndex;
use crate::reach::{self, Analysis, TypeRef};

const CHECK: &str = "start";

/// Start nodes closer than this (metres, same map) are reported (G8).
pub const MIN_START_DISTANCE: f64 = 6.0;

pub fn check(inputs: &Inputs) -> Vec<Finding> {
    let lif: &Lif = &inputs.lif;
    let indexes: Vec<LayoutIndex> = lif
        .origins
        .iter()
        .flat_map(|o| &o.layouts)
        .map(LayoutIndex::new)
        .collect();
    let place = |id: &str| -> Vec<FindingPlace> {
        indexes.iter().find_map(|i| i.node(id)).into_iter().collect()
    };

    let mut out = Vec::new();
    // One analysis per robot type (several groups may share a type).
    let mut analyses: HashMap<String, Option<Analysis>> = HashMap::new();
    // (node id, group) of every start, for the distance rule.
    let mut starts: Vec<(&str, usize)> = Vec::new();

    for (gi, group) in inputs.scenario.robots.iter().enumerate() {
        let Some(ft) = inputs.fleet.get(gi) else {
            continue;
        };
        // A factsheet that cannot be used is reported by the other checks.
        let Ok(t) = crate::type_ref(ft) else {
            continue;
        };
        let analysis = analyses
            .entry(t.name.clone())
            .or_insert_with(|| reach::analyze(lif, &t).ok());
        let Some(a) = analysis else {
            continue; // the graph error is reported by the reachability check
        };
        check_group(gi, group, &t, a, &place, &mut out);
        for n in &group.start_nodes {
            starts.push((n, gi));
        }
    }

    close_starts(inputs, &starts, &place, &mut out);
    out
}

/// The heading (radians) of the robot on start node number `i` of the group:
/// `start_headings` if given, else `start_heading` (an angle, or along the first
/// road that leaves the node; east if none does).
fn start_heading(
    group: &botsim_scenario::RobotGroup,
    i: usize,
    g: &botsim_core::graph::Graph,
    node: usize,
    t: &TypeRef,
) -> f64 {
    match (group.start_headings.get(i), group.start_heading) {
        (Some(&h), _) => h,
        (None, botsim_scenario::StartHeading::Angle(a)) => a,
        (None, botsim_scenario::StartHeading::Road) => {
            botsim_scenario::road_heading(g, node, &t.manufacturer, &t.series).unwrap_or(0.0)
        }
    }
}

fn check_group(
    group_index: usize,
    group: &botsim_scenario::RobotGroup,
    t: &TypeRef,
    a: &Analysis,
    place: &dyn Fn(&str) -> Vec<FindingPlace>,
    out: &mut Vec<Finding>,
) {
    let g = &a.graph;
    let start_nodes = &group.start_nodes;
    let who = format!("robot group {} (\"{}\")", group_index + 1, group.serial_prefix);
    let finding = |level, text: String, node: &str| {
        Finding::new(level, CHECK, text)
            .robot(&t.name)
            .with_places(place(node))
            .with_ids([node])
    };
    let mut cannot_leave: Vec<(&str, f64)> = Vec::new();
    let mut not_main: Vec<&str> = Vec::new();
    for (i, node) in start_nodes.iter().enumerate() {
        let Some(ni) = g.node_idx(node) else {
            out.push(finding(
                Level::Error,
                format!("{who}: start node \"{node}\" does not exist in the layout"),
                node,
            ));
            continue;
        };
        if !g.node_allows(ni, &t.manufacturer, &t.series) {
            out.push(finding(
                Level::Error,
                format!("{who}: the layout does not allow robot type {} on start node \"{node}\"", t.name),
                node,
            ));
            continue;
        }
        let outs: Vec<usize> = g
            .out_edges(ni)
            .iter()
            .copied()
            .filter(|&e| a.usable[e])
            .collect();
        let heading = start_heading(group, i, g, ni, t);
        if !outs.is_empty()
            && !outs.iter().any(|&e| {
                g.can_start(heading, None, e, t.kinematics, &t.manufacturer, &t.series)
            })
        {
            cannot_leave.push((node, heading));
        }
        if !a.node_in_main(ni) {
            not_main.push(node);
        }
    }
    for (node, heading) in cannot_leave {
        let heading_deg = heading.to_degrees();
        out.push(finding(
            Level::Warning,
            format!(
                "{who}: a robot of type {} starting on \"{node}\" with heading {heading_deg:.0} deg cannot drive off on any edge (turn rule)",
                t.name
            ),
            node,
        ));
    }
    for node in not_main {
        out.push(finding(
            Level::Warning,
            format!(
                "{who}: start node \"{node}\" is not in the biggest connected set of {}: a robot there may not get out, or may not come back",
                t.name
            ),
            node,
        ));
    }
}

/// Start nodes used twice (error) or closer than [`MIN_START_DISTANCE`] (warning).
fn close_starts(
    inputs: &Inputs,
    starts: &[(&str, usize)],
    place: &dyn Fn(&str) -> Vec<FindingPlace>,
    out: &mut Vec<Finding>,
) {
    let pos: HashMap<&str, (&str, f64, f64)> = inputs
        .lif
        .origins
        .iter()
        .flat_map(|o| &o.layouts)
        .flat_map(|l| &l.nodes)
        .map(|n| {
            (
                n.node_id.as_str(),
                (n.map_id.as_str(), n.node_position.x, n.node_position.y),
            )
        })
        .collect();
    let mut seen: HashMap<&str, usize> = HashMap::new();
    let mut close: Vec<(&str, &str)> = Vec::new();
    for (i, (a, _)) in starts.iter().enumerate() {
        if seen.insert(a, i).is_some() {
            out.push(
                Finding::new(
                    Level::Error,
                    CHECK,
                    format!("start node \"{a}\" is used by more than one robot"),
                )
                .with_places(place(a))
                .with_ids([*a]),
            );
            continue;
        }
        let Some(&(ma, xa, ya)) = pos.get(a) else {
            continue;
        };
        for (b, _) in &starts[..i] {
            if b == a {
                continue;
            }
            if let Some(&(mb, xb, yb)) = pos.get(b)
                && ma == mb
                && (xa - xb).hypot(ya - yb) < MIN_START_DISTANCE
            {
                close.push((b, a));
            }
        }
    }
    if !close.is_empty() {
        let mut ids: Vec<&str> = Vec::new();
        for (a, b) in &close {
            for n in [a, b] {
                if !ids.contains(n) {
                    ids.push(n);
                }
            }
        }
        let places: Vec<FindingPlace> = ids.iter().flat_map(|n| place(n)).collect();
        out.push(
            Finding::new(
                Level::Warning,
                CHECK,
                format!(
                    "{} pair(s) of start nodes are closer than {MIN_START_DISTANCE} m (first: \"{}\" and \"{}\")",
                    close.len(),
                    close[0].0,
                    close[0].1
                ),
            )
            .with_places(places)
            .with_ids(ids),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::FleetType;
    use botsim_scenario::Scenario;
    use std::path::Path;
    use vda5050::factsheet::Factsheet;

    fn inputs(start_nodes: &str) -> Inputs {
        let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}"#;
        let pts = [(0.0, 0.0), (5.0, 0.0), (5.0, 20.0), (0.0, 20.0)];
        let nodes: Vec<String> = pts
            .iter()
            .enumerate()
            .map(|(i, (x, y))| {
                format!(
                    r#"{{"nodeId":"N{i}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},"mobileRobotTypeNodeProperties":[{ty}]}}"#
                )
            })
            .collect();
        let edges: Vec<String> = (0..4)
            .map(|k| {
                format!(
                    r#"{{"edgeId":"E{k}","startNodeId":"N{k}","endNodeId":"N{}","mobileRobotTypeEdgeProperties":[{ty}]}}"#,
                    (k + 1) % 4
                )
            })
            .collect();
        let lif: Lif = serde_json::from_str(&format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1","nodes":[{}],"edges":[{}]}}]}}]}}"#,
            nodes.join(","),
            edges.join(",")
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
start_nodes = {start_nodes}
"#
        ))
        .expect("test scenario parses");
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/diff_robot.factsheet.json");
        let mut factsheet: Factsheet =
            serde_json::from_str(&std::fs::read_to_string(&path).expect("factsheet file")).expect("factsheet parses");
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

    #[test]
    fn start_heading_follows_the_scenario() {
        let mut i = inputs(r#"["N0", "N1"]"#);
        let t = crate::type_ref(&i.fleet[0]).expect("type");
        let g = botsim_core::graph::Graph::from_lif(&i.lif).expect("graph");
        let n1 = g.node_idx("N1").expect("N1");
        let group = &mut i.scenario.robots[0];
        // an angle for all, east by default
        assert_eq!(start_heading(group, 1, &g, n1, &t), 0.0);
        // along the road: the edge N1 -> N2 runs north
        group.start_heading = botsim_scenario::StartHeading::Road;
        let h = start_heading(group, 1, &g, n1, &t);
        assert!((h - std::f64::consts::FRAC_PI_2).abs() < 1e-9, "{h}");
        // one heading for each start node wins over both
        group.start_headings = vec![1.0, 2.0];
        assert_eq!(start_heading(group, 1, &g, n1, &t), 2.0);
    }

    #[test]
    fn good_starts_give_no_findings() {
        assert!(check(&inputs(r#"["N0", "N2"]"#)).is_empty());
    }

    #[test]
    fn missing_close_and_double_starts() {
        let f = check(&inputs(r#"["N0", "N1", "NX", "N0"]"#));
        let errors: Vec<_> = f.iter().filter(|x| x.level == Level::Error).collect();
        assert_eq!(errors.len(), 2, "{f:?}"); // NX missing, N0 used twice
        let close: Vec<_> = f.iter().filter(|x| x.text.contains("closer than")).collect();
        assert_eq!(close.len(), 1);
        assert_eq!(close[0].ids, ["N0", "N1"]);
    }
}

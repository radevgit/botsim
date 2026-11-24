//! Check: nodes with only one exit (`docs/graph_problems.md`, G12).
//!
//! In a one-way layout most nodes have a single way out. A robot that is held up
//! there cannot be re-routed around the obstacle: re-routing needs a route without
//! the node another robot holds. Counts the nodes of the biggest connected set of a
//! robot type that have exactly one edge the type may drive out of them (the turn
//! rule is not applied here).

use vdma_lif::Layout;

use crate::finding::{Finding, Level};
use crate::places::LayoutIndex;
use crate::reach::{Analysis, TypeRef};

const CHECK: &str = "exits";

pub fn check(lif: &vdma_lif::Lif, a: &Analysis, t: &TypeRef) -> Vec<Finding> {
    let mut out = Vec::new();
    for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
        report_layout(layout, a, t, &mut out);
    }
    out
}

fn report_layout(layout: &Layout, a: &Analysis, t: &TypeRef, out: &mut Vec<Finding>) {
    let g = &a.graph;
    let mut main = 0usize;
    let mut one: Vec<&str> = Vec::new();
    for n in &layout.nodes {
        let Some(i) = g.node_idx(&n.node_id) else {
            continue;
        };
        if !a.node_in_main(i) {
            continue;
        }
        main += 1;
        let exits = g.out_edges(i).iter().filter(|&&e| a.usable[e]).count();
        if exits == 1 {
            one.push(&n.node_id);
        }
    }
    if one.is_empty() || main == 0 {
        return;
    }
    let idx = LayoutIndex::new(layout);
    out.push(
        Finding::new(
            Level::Info,
            CHECK,
            format!(
                "{} of {main} nodes in the main connected set ({:.0} %) have only one exit: a robot held up there cannot be re-routed around the obstacle",
                one.len(),
                100.0 * one.len() as f64 / main as f64
            ),
        )
        .layout(&layout.layout_id)
        .robot(&t.name)
        .with_places(idx.nodes(one.iter().copied()))
        .with_ids(one),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use botsim_core::spec::Kinematics;
    use vdma_lif::Lif;

    #[test]
    fn counts_nodes_with_one_exit() {
        // square A B C D, edges A>B, B>C, C>D, D>A and a diagonal B>D: only B has two exits
        let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}"#;
        let pts = [("A", 0.0, 0.0), ("B", 10.0, 0.0), ("C", 10.0, 10.0), ("D", 0.0, 10.0)];
        let nodes: Vec<String> = pts
            .iter()
            .map(|(id, x, y)| {
                format!(
                    r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},"mobileRobotTypeNodeProperties":[{ty}]}}"#
                )
            })
            .collect();
        let edges: Vec<String> = [("AB", "A", "B"), ("BC", "B", "C"), ("CD", "C", "D"), ("DA", "D", "A"), ("BD", "B", "D")]
            .iter()
            .map(|(id, a, b)| {
                format!(
                    r#"{{"edgeId":"{id}","startNodeId":"{a}","endNodeId":"{b}","mobileRobotTypeEdgeProperties":[{ty}]}}"#
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
        let t = TypeRef::test_rect("ACME/X1", Kinematics::Differential, 1.0, 0.6);
        let a = crate::reach::analyze(&lif, &t).expect("analysis");
        let f = check(&lif, &a, &t);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].level, Level::Info);
        assert_eq!(f[0].ids, ["A", "C", "D"]);
        assert!(f[0].text.starts_with("3 of 4 nodes"), "{}", f[0].text);
        assert_eq!(f[0].places.len(), 3);
    }
}

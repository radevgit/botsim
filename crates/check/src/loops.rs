//! Check: short closed one-way loops that fill up (`docs/graph_problems.md`, G10).
//!
//! In a Manhattan grid of one-way roads every block is a directed cycle. A cycle
//! holds only so many robots (its length divided by the robot length plus a gap, the
//! reach of the protective field); if more robots of the fleet can be on it than it
//! holds, it can fill up and freeze: nobody can move because the next place is taken.
//!
//! Cycles of drivable edges (the turn rules apply) up to [`MAX_LENGTH`] metres are
//! searched; a cycle of two edges between the same two nodes (a two-way road) is not
//! a loop. The search has a step budget, so a huge layout is searched only in part
//! (the finding says so).

use botsim_core::graph::Graph;

use crate::finding::{Finding, Level};
use crate::places::LayoutIndex;
use crate::reach::{Analysis, TypeRef};

const CHECK: &str = "loop";

/// Longest cycle searched (m).
pub const MAX_LENGTH: f64 = 150.0;

/// Most edge steps the search makes per robot type.
const BUDGET: usize = 20_000_000;

/// A cycle: its edges (graph indexes, in driving order) and its length.
struct Cycle {
    edges: Vec<usize>,
    length: f64,
}

/// All simple cycles of drivable edges up to `max_length` metres, each once (it
/// starts at its lowest edge index); and whether the budget was used up.
fn cycles(a: &Analysis, t: &TypeRef, max_length: f64) -> (Vec<Cycle>, bool) {
    let g = &a.graph;
    let mut found = Vec::new();
    let mut steps = 0usize;
    for start in 0..g.edges.len() {
        if !a.usable[start] {
            continue;
        }
        let mut path = vec![start];
        // nodes on the path: a cycle that passes a node twice is two loops, not one
        let mut on_node = vec![false; g.nodes.len()];
        // (the start node of the first edge is entered once more by the last edge)
        on_node[g.edges[start].end] = true;
        // depth-first with an explicit stack: (edge, next out-edge position)
        let mut stack: Vec<usize> = vec![0];
        let mut length = g.edges[start].length;
        while let Some(pos) = stack.last_mut() {
            steps += 1;
            if steps > BUDGET {
                return (found, true);
            }
            let &e = path.last().unwrap_or(&start);
            let outs = g.out_edges(g.edges[e].end);
            if *pos >= outs.len() {
                // back up
                stack.pop();
                path.pop();
                if e != start {
                    on_node[g.edges[e].end] = false;
                }
                length -= g.edges[e].length;
                continue;
            }
            let f = outs[*pos];
            *pos += 1;
            if f < start || !a.usable[f] || !g.can_follow(e, f, t.kinematics, &t.manufacturer, &t.series) {
                continue;
            }
            if f == start {
                if path.len() > 2 {
                    found.push(Cycle {
                        edges: path.clone(),
                        length,
                    });
                }
                continue;
            }
            if on_node[g.edges[f].end] || length + g.edges[f].length > max_length {
                continue;
            }
            path.push(f);
            on_node[g.edges[f].end] = true;
            length += g.edges[f].length;
            stack.push(0);
        }
    }
    (found, false)
}

/// `robots`: how many robots of this type the scenario has (0: no check).
pub fn check(lif: &vdma_lif::Lif, a: &Analysis, t: &TypeRef, robots: usize) -> Vec<Finding> {
    if robots == 0 {
        return Vec::new();
    }
    let (all, cut) = cycles(a, t, MAX_LENGTH);
    let gap = t.length + t.field_margin;
    let capacity = |c: &Cycle| (c.length / gap).floor() as usize;
    let risky: Vec<&Cycle> = all.iter().filter(|c| capacity(c) < robots).collect();
    let mut out = Vec::new();
    for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
        report_layout(layout, &a.graph, &risky, all.len(), cut, robots, t, &capacity, &mut out);
    }
    out
}

#[allow(clippy::too_many_arguments)]
fn report_layout(
    layout: &vdma_lif::Layout,
    g: &Graph,
    risky: &[&Cycle],
    total: usize,
    cut: bool,
    robots: usize,
    t: &TypeRef,
    capacity: &dyn Fn(&Cycle) -> usize,
    out: &mut Vec<Finding>,
) {
    let here: std::collections::HashSet<&str> =
        layout.edges.iter().map(|e| e.edge_id.as_str()).collect();
    let mine: Vec<&&Cycle> = risky
        .iter()
        .filter(|c| c.edges.iter().all(|&e| here.contains(g.edges[e].id.as_str())))
        .collect();
    let Some(smallest) = mine.iter().min_by_key(|c| capacity(c)) else {
        return;
    };
    let mut ids: Vec<&str> = Vec::new();
    for c in &mine {
        for &e in &c.edges {
            let id = g.edges[e].id.as_str();
            if !ids.contains(&id) {
                ids.push(id);
            }
        }
    }
    let idx = LayoutIndex::new(layout);
    let note = if cut { "; the search was cut short, there may be more" } else { "" };
    out.push(
        Finding::new(
            Level::Warning,
            CHECK,
            format!(
                "{} of {total} closed one-way loop(s) up to {MAX_LENGTH} m hold fewer robots than the {robots} of this type in the scenario and can fill up; smallest: {} edges, {:.0} m, holds {}{note}",
                mine.len(),
                smallest.edges.len(),
                smallest.length,
                capacity(smallest)
            ),
        )
        .layout(&layout.layout_id)
        .robot(&t.name)
        .with_places(idx.edges(ids.iter().copied()))
        .with_ids(ids),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use botsim_core::spec::Kinematics;
    use vdma_lif::Lif;

    /// A square block of side `side` (m), four one-way edges around it, plus a
    /// two-way road B<>E that is not a loop.
    fn lif(side: f64) -> Lif {
        let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}"#;
        let pts = [("A", 0.0, 0.0), ("B", side, 0.0), ("C", side, side), ("D", 0.0, side), ("E", side * 2.0, 0.0)];
        let nodes: Vec<String> = pts
            .iter()
            .map(|(id, x, y)| {
                format!(
                    r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},"mobileRobotTypeNodeProperties":[{ty}]}}"#
                )
            })
            .collect();
        let edges: Vec<String> = [("AB", "A", "B"), ("BC", "B", "C"), ("CD", "C", "D"), ("DA", "D", "A"), ("BE", "B", "E"), ("EB", "E", "B")]
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

    #[test]
    fn block_loop_fills_up_but_a_two_way_road_is_no_loop() {
        // body 1.0 + field 0.3 = 1.3 m per robot; a 4 x 5 m block is 20 m: holds 15
        let t = TypeRef::test_rect("ACME/X1", Kinematics::Differential, 1.0, 0.6);
        let l = lif(5.0);
        let a = crate::reach::analyze(&l, &t).expect("analysis");
        let (all, cut) = cycles(&a, &t, MAX_LENGTH);
        assert!(!cut);
        assert_eq!(all.len(), 1, "only the block, not the two-way road");
        assert_eq!(all[0].edges.len(), 4);
        // 20 robots do not fit, 10 do
        let f = check(&l, &a, &t, 20);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].ids.len(), 4);
        assert!(f[0].text.contains("holds 15"), "{}", f[0].text);
        assert!(check(&l, &a, &t, 10).is_empty());
        assert!(check(&l, &a, &t, 0).is_empty());
        // a longer limit than the block: not found
        assert!(cycles(&a, &t, 10.0).0.is_empty());
    }
}

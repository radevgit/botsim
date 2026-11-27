//! Check 2: reachability, per robot type.
//!
//! The unit is "driving edge e", not the node: whether a robot may drive edge
//! `f` after edge `e` depends on how `e` ended (the turn rules of its
//! kinematics, `Graph::can_follow` of botsim-core, the rules the driver uses).
//! There is an arc e -> f if e ends where f starts, the type may drive both and
//! `can_follow` says yes. A robot that enters a place it cannot leave is lost, so
//! "reachable" is not enough: the check uses the biggest strongly connected set
//! of edges (`docs/graph_problems.md`, G1, G7). A node is in it if a robot can
//! arrive there on an edge of that set.

use std::collections::{HashMap, HashSet};

use botsim_core::graph::Graph;
use vdma_lif::{Layout, Lif};

use crate::finding::{Finding, FindingPlace, Level};
use crate::places::LayoutIndex;

const CHECK: &str = "reachability";

/// A robot type of the fleet, as the check needs it (defined in `botsim-sweep`).
pub use botsim_sweep::TypeRef;

/// Where a node is for one robot type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Place {
    /// No edge the type may drive touches the node.
    Unused,
    /// A robot can arrive here and still get out and back (biggest connected set).
    Main,
    /// Touched by drivable edges but not in the main set: a robot may get lost
    /// here or can never get there.
    Outside,
}

/// The result of the analysis for one robot type: what the findings and the
/// start-node check are built from.
pub struct Analysis {
    pub graph: Graph,
    /// Per graph edge: may the type drive it (edge and both end nodes allow it).
    pub usable: Vec<bool>,
    /// Per graph node.
    place: Vec<Place>,
    /// Edges where the turn rule blocks every way on.
    blocked: Vec<usize>,
}

impl Analysis {
    /// Is the node in the biggest connected set of the type (graph node index)?
    pub fn node_in_main(&self, node: usize) -> bool {
        self.place.get(node) == Some(&Place::Main)
    }
}

/// Run the check for one robot type on the whole LIF file (all layouts form one
/// graph: lift edges lead from a layout to another). Findings are reported per layout.
pub fn check_type(lif: &Lif, t: &TypeRef) -> Vec<Finding> {
    match analyze(lif, t) {
        Ok(a) => {
            let mut out = Vec::new();
            for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
                report_turns(layout, &a.graph, &a.blocked, &a.usable, t, &mut out);
                report_layout(layout, &a.graph, &a.place, t, &mut out);
            }
            out
        }
        Err(f) => vec![f],
    }
}

/// Build the edge graph of the type and find its biggest connected set. The
/// error is a finding: the layout cannot be turned into a graph.
pub fn analyze(lif: &Lif, t: &TypeRef) -> Result<Analysis, Finding> {
    let graph = Graph::from_lif(lif).map_err(|e| {
        Finding::new(
            Level::Error,
            CHECK,
            format!(
                "the layout cannot be turned into a graph, so reachability is not checked: {e}"
            ),
        )
        .robot(&t.name)
    })?;
    let ne = graph.edges.len();
    let usable: Vec<bool> = (0..ne)
        .map(|e| graph.edge_allows(e, &t.manufacturer, &t.series))
        .collect();
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); ne];
    for (e, edge) in graph.edges.iter().enumerate() {
        if !usable[e] {
            continue;
        }
        for &f in graph.out_edges(edge.end) {
            if usable[f] && graph.can_follow(e, f, t.kinematics, &t.manufacturer, &t.series) {
                adj[e].push(f);
            }
        }
    }

    // Edges where a robot arrives, the type may drive on, but none of them can
    // follow (G1): the turn rule blocks it. (No way on at all is a dead end, which
    // the connected-set findings below cover.)
    let blocked: Vec<usize> = (0..ne)
        .filter(|&e| {
            usable[e]
                && adj[e].is_empty()
                && graph
                    .out_edges(graph.edges[e].end)
                    .iter()
                    .any(|&f| usable[f])
        })
        .collect();

    let comp = strongly_connected(&adj);
    let mut size: HashMap<usize, usize> = HashMap::new();
    for e in 0..ne {
        if usable[e] {
            *size.entry(comp[e]).or_default() += 1;
        }
    }
    // Biggest set with a cycle (two edges at least); the smallest component
    // number breaks ties, so the result is stable.
    let main = size
        .iter()
        .filter(|(_, s)| **s >= 2)
        .max_by_key(|(c, s)| (**s, std::cmp::Reverse(**c)))
        .map(|(c, _)| *c);

    let nn = graph.nodes.len();
    let mut place = vec![Place::Unused; nn];
    for (e, edge) in graph.edges.iter().enumerate() {
        if !usable[e] {
            continue;
        }
        for n in [edge.start, edge.end] {
            if place[n] == Place::Unused {
                place[n] = Place::Outside;
            }
        }
        if Some(comp[e]) == main {
            place[edge.end] = Place::Main;
        }
    }

    Ok(Analysis {
        graph,
        usable,
        place,
        blocked,
    })
}

/// One finding per layout: the arrivals the turn rule blocks (check name `turn`),
/// with the first one as an example.
fn report_turns(
    layout: &Layout,
    graph: &Graph,
    blocked: &[usize],
    usable: &[bool],
    t: &TypeRef,
    out: &mut Vec<Finding>,
) {
    let here: HashSet<&str> = layout.edges.iter().map(|e| e.edge_id.as_str()).collect();
    let mine: Vec<usize> = blocked
        .iter()
        .copied()
        .filter(|&e| here.contains(graph.edges[e].id.as_str()))
        .collect();
    let Some(&first) = mine.first() else {
        return;
    };
    let deg = |a: f64| a.to_degrees().rem_euclid(360.0);
    let arrives = deg(graph.edge_directions(first, &t.manufacturer, &t.series).1);
    let leaving: Vec<String> = graph
        .out_edges(graph.edges[first].end)
        .iter()
        .filter(|&&f| usable[f])
        .map(|&f| {
            format!(
                "{} starts heading {:.0} deg",
                graph.edges[f].id,
                deg(graph.edge_directions(f, &t.manufacturer, &t.series).0)
            )
        })
        .collect();
    let ids: Vec<&str> = mine.iter().map(|&e| graph.edges[e].id.as_str()).collect();
    let idx = LayoutIndex::new(layout);
    out.push(
        Finding::new(
            Level::Warning,
            "turn",
            format!(
                "{} edge(s) end where this robot type ({:?}) cannot continue on any edge that it may drive; first: {} ends heading {:.0} deg, but {}",
                mine.len(),
                t.kinematics,
                graph.edges[first].id,
                arrives,
                leaving.join(", ")
            ),
        )
        .layout(&layout.layout_id)
        .robot(&t.name)
        .with_places(idx.edges(ids.iter().copied()))
        .with_ids(ids),
    );
}

/// The findings of one layout.
fn report_layout(
    layout: &Layout,
    graph: &Graph,
    place: &[Place],
    t: &TypeRef,
    out: &mut Vec<Finding>,
) {
    let idx = LayoutIndex::new(layout);
    let finding = |level, text: String, ids: Vec<&str>, places: Vec<FindingPlace>| {
        Finding::new(level, CHECK, text)
            .layout(&layout.layout_id)
            .robot(&t.name)
            .with_places(places)
            .with_ids(ids)
    };

    let place_of = |id: &str| graph.node_idx(id).map(|i| place[i]);
    let outside: Vec<&str> = layout
        .nodes
        .iter()
        .map(|n| n.node_id.as_str())
        .filter(|id| place_of(id) == Some(Place::Outside))
        .collect();
    if !outside.is_empty() {
        out.push(finding(
            Level::Warning,
            format!(
                "{} node(s) are on roads of this robot type but a robot cannot get there and out again (biggest connected set, with its turn rules)",
                outside.len()
            ),
            outside.clone(),
            idx.nodes(outside),
        ));
    }

    let mut not_reached: Vec<&str> = Vec::new();
    let mut not_served: Vec<&str> = Vec::new();
    for s in layout.stations.iter().flatten() {
        let places: Vec<Place> = s
            .interaction_node_ids
            .iter()
            .filter_map(|id| place_of(id))
            .collect();
        if places.contains(&Place::Main) {
            continue;
        }
        if places.contains(&Place::Outside) {
            not_reached.push(&s.station_id);
        } else {
            not_served.push(&s.station_id);
        }
    }
    if !not_reached.is_empty() {
        out.push(finding(
            Level::Warning,
            format!(
                "{} station(s) have interaction nodes only outside the biggest connected set of this robot type",
                not_reached.len()
            ),
            not_reached.clone(),
            idx.stations(not_reached),
        ));
    }
    if !not_served.is_empty() {
        out.push(finding(
            Level::Info,
            format!(
                "{} station(s) are not on any road of this robot type (it does not serve them)",
                not_served.len()
            ),
            not_served.clone(),
            idx.stations(not_served),
        ));
    }
}

/// Strongly connected components (iterative Tarjan): the component number of each vertex.
fn strongly_connected(adj: &[Vec<usize>]) -> Vec<usize> {
    const NONE: usize = usize::MAX;
    let n = adj.len();
    let mut index = vec![NONE; n];
    let mut low = vec![0; n];
    let mut on_stack = vec![false; n];
    let mut comp = vec![NONE; n];
    let mut stack: Vec<usize> = Vec::new();
    let mut next_index = 0;
    let mut next_comp = 0;
    for root in 0..n {
        if index[root] != NONE {
            continue;
        }
        // (vertex, position in its adjacency list)
        let mut work: Vec<(usize, usize)> = vec![(root, 0)];
        index[root] = next_index;
        low[root] = next_index;
        next_index += 1;
        stack.push(root);
        on_stack[root] = true;
        while let Some(&mut (v, ref mut pos)) = work.last_mut() {
            if let Some(&w) = adj[v].get(*pos) {
                *pos += 1;
                if index[w] == NONE {
                    index[w] = next_index;
                    low[w] = next_index;
                    next_index += 1;
                    stack.push(w);
                    on_stack[w] = true;
                    work.push((w, 0));
                } else if on_stack[w] {
                    low[v] = low[v].min(index[w]);
                }
            } else {
                work.pop();
                if let Some(&(parent, _)) = work.last() {
                    low[parent] = low[parent].min(low[v]);
                }
                if low[v] == index[v] {
                    while let Some(w) = stack.pop() {
                        on_stack[w] = false;
                        comp[w] = next_comp;
                        if w == v {
                            break;
                        }
                    }
                    next_comp += 1;
                }
            }
        }
    }
    comp
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::finding::PlaceKind;
    use botsim_core::spec::Kinematics;

    fn x1(kinematics: Kinematics) -> TypeRef {
        TypeRef::test_rect("ACME/X1", kinematics, 1.0, 0.6)
    }

    /// Layout with nodes N0.. at the given points, edges `(from, to)` all usable by
    /// ACME/X1 (nodes list the type too), and stations `(id, node)`.
    fn layout_xy(pts: &[(f64, f64)], edges: &[(usize, usize)], stations: &[(&str, usize)]) -> Lif {
        let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}"#;
        let nodes: Vec<String> = pts
            .iter()
            .enumerate()
            .map(|(i, (x, y))| {
                format!(
                    r#"{{"nodeId":"N{i}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},"mobileRobotTypeNodeProperties":[{ty}]}}"#
                )
            })
            .collect();
        let edges: Vec<String> = edges
            .iter()
            .enumerate()
            .map(|(k, (a, b))| {
                format!(
                    r#"{{"edgeId":"E{k}","startNodeId":"N{a}","endNodeId":"N{b}","mobileRobotTypeEdgeProperties":[{ty}]}}"#
                )
            })
            .collect();
        let stations: Vec<String> = stations
            .iter()
            .map(|(id, nd)| format!(r#"{{"stationId":"{id}","interactionNodeIds":["N{nd}"]}}"#))
            .collect();
        let text = format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1","nodes":[{}],"edges":[{}],"stations":[{}]}}]}}]}}"#,
            nodes.join(","),
            edges.join(","),
            stations.join(",")
        );
        serde_json::from_str(&text).expect("test LIF parses")
    }

    /// Nodes on the x axis.
    fn layout(n: usize, edges: &[(usize, usize)], stations: &[(&str, usize)]) -> Lif {
        let pts: Vec<(f64, f64)> = (0..n).map(|i| (i as f64, 0.0)).collect();
        layout_xy(&pts, edges, stations)
    }

    #[test]
    fn ring_with_dead_end_and_unused_station() {
        // N0..N3 on a line, so a ring N0->N1->N2->N0 has straight-back edges
        // (omnidirectional keeps its heading, so it is fine); N2->N3 is a dead end;
        // N4 is not on any road.
        let l = layout(
            5,
            &[(0, 1), (1, 2), (2, 0), (2, 3)],
            &[("S0", 0), ("S3", 3), ("S4", 4)],
        );
        let f = check_type(&l, &x1(Kinematics::Omnidirectional));
        assert_eq!(f.len(), 3, "{f:?}");
        assert_eq!(f[0].level, Level::Warning);
        assert_eq!(f[0].ids, ["N3"]);
        assert_eq!(f[1].level, Level::Warning);
        assert_eq!(f[1].ids, ["S3"]);
        assert_eq!(f[2].level, Level::Info);
        assert_eq!(f[2].ids, ["S4"]);
        // places: kind, map and position come with the finding
        assert_eq!(f[0].places.len(), 1);
        assert_eq!(f[0].places[0].kind, PlaceKind::Node);
        assert_eq!(f[0].places[0].map.as_deref(), Some("M"));
        assert_eq!(f[0].places[0].points, [[3.0, 0.0]]);
        assert_eq!(f[1].places[0].kind, PlaceKind::Station);
        assert_eq!(f[1].places[0].points, [[3.0, 0.0]]);
        assert_eq!(f[2].places[0].points, [[4.0, 0.0]]);
    }

    #[test]
    fn closed_ring_is_clean() {
        let l = layout_xy(
            &[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)],
            &[(0, 1), (1, 2), (2, 3), (3, 0)],
            &[("S0", 1)],
        );
        // corners are 90 degrees: a differential robot turns on the spot, an
        // omnidirectional one needs no turn; a three-wheel robot cannot.
        assert!(check_type(&l, &x1(Kinematics::Differential)).is_empty());
        assert!(check_type(&l, &x1(Kinematics::Omnidirectional)).is_empty());
        let f = check_type(&l, &x1(Kinematics::ThreeWheel));
        assert!(!f.is_empty(), "three-wheel robot is stuck on sharp corners");
        let turn: Vec<_> = f.iter().filter(|x| x.check == "turn").collect();
        assert_eq!(turn.len(), 1);
        assert_eq!(turn[0].ids.len(), 4);
        assert_eq!(turn[0].places.len(), 4);
        assert!(turn[0].text.contains("ends heading"), "{}", turn[0].text);
    }

    #[test]
    fn type_without_edges_is_not_a_panic() {
        let l = layout(3, &[(0, 1), (1, 2), (2, 0)], &[("S0", 1)]);
        let mut t = x1(Kinematics::Differential);
        t.name = "ACME/Other".into();
        t.series = "Other".into();
        let f = check_type(&l, &t);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].level, Level::Info);
    }

    #[test]
    fn broken_layout_is_an_error_finding_not_a_panic() {
        let mut l = layout(3, &[(0, 1), (1, 2), (2, 0)], &[]);
        l.origins[0].layouts[0].edges[0].end_node_id = "NOPE".into();
        let f = check_type(&l, &x1(Kinematics::Differential));
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].level, Level::Error);
    }
}

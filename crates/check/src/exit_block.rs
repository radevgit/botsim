//! Check: a waiting robot can block the only way out of a place another robot
//! stands on (brief U6).
//!
//! - **Place** (per type): an interaction node of a station, or a dead-end node (one
//!   neighbour), with exactly one edge the type may drive out of it.
//! - **Way out**: from the place along the only edge that may follow (turn rules,
//!   the way back not counted), up to the first node with two or more choices
//!   (included); it also stops where it ends or meets itself.
//! - A node **blocks** the way if it is on it, or a robot on it conflicts with an
//!   edge or a node of the way (conflict map).
//! - **Both can be there at once**: the node is not the place; a robot there and one
//!   on the place, each standing as it arrived, do not conflict; and some edge into
//!   the node is clear of the robot on the place.
//! - **Waiting for the place** (the condition for a finding): on a shortest drivable
//!   way from the node to the place the next edge, or its end node, conflicts with
//!   the robot on the place, or the edge ends at the place. A robot that can drive
//!   on is not stuck there.
//!
//! It tells which layouts carry the risk of that deadlock; it cannot say whether a
//! controller ever lets a robot wait there.

use std::cmp::Reverse;
use std::collections::{BTreeSet, BinaryHeap, HashMap, HashSet};

use botsim_core::graph::Graph;
use botsim_sweep::{Sweep, standing_body, sweep};
use vdma_lif::{Layout, Lif};

use crate::conflict::ConflictMap;
use crate::finding::{Finding, Level};
use crate::places::LayoutIndex;
use crate::reach::{self, Analysis, TypeRef};

const CHECK: &str = "exit-block";

/// How many blocking nodes a finding's text names for its first place.
const TEXT_NODES: usize = 5;

/// A node from which a robot can block the way out.
#[derive(Debug, Clone, PartialEq)]
pub struct Blocker {
    pub node: String,
    /// The robot there is waiting for the place (see the module text); only these
    /// are findings.
    pub waiting: bool,
}

/// A place, for a robot of one type on it and one of a type (maybe the same)
/// somewhere that can be there at once and blocks its way out.
#[derive(Debug, Clone, PartialEq)]
pub struct Case {
    /// Node id of the place.
    pub place: String,
    /// A station that has the place as an interaction node.
    pub station: Option<String>,
    /// The type on the place.
    pub type_place: String,
    /// The type on the blocking nodes.
    pub type_wait: String,
    /// Node ids of the way out, from the place on.
    pub way: Vec<String>,
    /// In layout order; never empty.
    pub blockers: Vec<Blocker>,
}

/// The conflicts of the map for one ordered pair (type that waits, type on the
/// place), by graph index.
#[derive(Default)]
struct Blocking {
    /// way edge -> nodes where a waiting robot conflicts with it
    by_edge: HashMap<usize, Vec<usize>>,
    /// way node -> nodes where a waiting robot conflicts with a robot on it
    by_node: HashMap<usize, Vec<usize>>,
}

fn blocking(g: &Graph, map: &ConflictMap, wait: &str, place: &str) -> Blocking {
    let mut b = Blocking::default();
    for p in &map.node_pairs {
        if p.type_node != wait || p.type_edge != place {
            continue;
        }
        for c in &p.conflicts {
            if let (Some(n), Some(e)) = (g.node_idx(&c.node), g.edge_idx(&c.edge)) {
                b.by_edge.entry(e).or_default().push(n);
            }
        }
    }
    for p in &map.node_node_pairs {
        for c in &p.conflicts {
            let (Some(x), Some(y)) = (g.node_idx(&c.node_a), g.node_idx(&c.node_b)) else {
                continue;
            };
            if p.type_a == wait && p.type_b == place {
                b.by_node.entry(y).or_default().push(x);
            }
            if p.type_b == wait && p.type_a == place {
                b.by_node.entry(x).or_default().push(y);
            }
        }
    }
    b
}

/// The only way out of `place`: its nodes (from the place on) and edges. `None`
/// if the type does not have exactly one edge out of it.
fn way_out(a: &Analysis, t: &TypeRef, place: usize) -> Option<(Vec<usize>, Vec<usize>)> {
    let g = &a.graph;
    let outs: Vec<usize> = g
        .out_edges(place)
        .iter()
        .copied()
        .filter(|&e| a.usable[e])
        .collect();
    let [mut e] = outs[..] else {
        return None;
    };
    let mut nodes = vec![place, g.edges[e].end];
    let mut edges = vec![e];
    loop {
        let back = g.edges[e].start;
        let next: Vec<usize> = g
            .out_edges(g.edges[e].end)
            .iter()
            .copied()
            .filter(|&f| {
                a.usable[f]
                    && g.edges[f].end != back
                    && g.can_follow(e, f, t.kinematics, &t.manufacturer, &t.series)
            })
            .collect();
        let [f] = next[..] else {
            break;
        };
        if nodes.contains(&g.edges[f].end) {
            break;
        }
        nodes.push(g.edges[f].end);
        edges.push(f);
        e = f;
    }
    Some((nodes, edges))
}

/// The places of a type: node index and a station that names it.
fn places<'a>(lif: &'a Lif, a: &Analysis) -> Vec<(usize, Option<&'a str>)> {
    let g = &a.graph;
    let mut stations: HashMap<usize, &str> = HashMap::new();
    for s in lif
        .origins
        .iter()
        .flat_map(|o| &o.layouts)
        .flat_map(|l| l.stations.iter().flatten())
    {
        for n in s.interaction_node_ids.iter().filter_map(|n| g.node_idx(n)) {
            stations.entry(n).or_insert(&s.station_id);
        }
    }
    let mut neighbours: Vec<HashSet<usize>> = vec![HashSet::new(); g.nodes.len()];
    for (e, edge) in g.edges.iter().enumerate() {
        if a.usable[e] {
            neighbours[edge.start].insert(edge.end);
            neighbours[edge.end].insert(edge.start);
        }
    }
    (0..g.nodes.len())
        .filter(|n| stations.contains_key(n) || neighbours[*n].len() == 1)
        .map(|n| (n, stations.get(&n).copied()))
        .collect()
}

/// For each edge a type may drive: how far it still is to `goal` once the edge has
/// been driven (turn rules apply); infinite if there is no way.
fn distances_to(g: &Graph, preds: &[Vec<usize>], lengths: &[f64], goal: usize) -> Vec<f64> {
    let mut dist = vec![f64::INFINITY; g.edges.len()];
    // (distance in micrometres, edge): an ordered key for the queue
    let key = |d: f64| (d * 1e6).round() as u64;
    let mut queue = BinaryHeap::new();
    for (e, edge) in g.edges.iter().enumerate() {
        if edge.end == goal {
            dist[e] = 0.0;
            queue.push(Reverse((0, e)));
        }
    }
    while let Some(Reverse((k, f))) = queue.pop() {
        if k > key(dist[f]) {
            continue;
        }
        let through = lengths[f] + dist[f];
        for &e in &preds[f] {
            if through < dist[e] {
                dist[e] = through;
                queue.push(Reverse((key(through), e)));
            }
        }
    }
    dist
}

/// What one type needs for the shortest ways and the swept outlines.
struct Typed<'a> {
    t: &'a TypeRef,
    a: Analysis,
    /// Per edge: the edges that it may follow on from.
    preds: Vec<Vec<usize>>,
    lengths: Vec<f64>,
    /// Edges into each node.
    into: Vec<Vec<usize>>,
    sweeps: HashMap<usize, Option<Sweep>>,
}

impl<'a> Typed<'a> {
    fn new(t: &'a TypeRef, a: Analysis) -> Self {
        let g = &a.graph;
        let mut preds = vec![Vec::new(); g.edges.len()];
        let mut into = vec![Vec::new(); g.nodes.len()];
        let lengths = (0..g.edges.len())
            .map(|e| {
                if a.usable[e] {
                    g.edge_travel_length(e, &t.manufacturer, &t.series)
                } else {
                    f64::INFINITY
                }
            })
            .collect();
        for (e, edge) in g.edges.iter().enumerate() {
            if !a.usable[e] {
                continue;
            }
            into[edge.end].push(e);
            for &f in g.out_edges(edge.end) {
                if a.usable[f] && g.can_follow(e, f, t.kinematics, &t.manufacturer, &t.series) {
                    preds[f].push(e);
                }
            }
        }
        Typed {
            t,
            a,
            preds,
            lengths,
            into,
            sweeps: HashMap::new(),
        }
    }

    /// The headings a robot can have when it has arrived over edge `e`: where `e`
    /// ends, or after a lift ride where the edges into the lift end.
    fn arrival_headings(&self, e: usize) -> Vec<f64> {
        let g = &self.a.graph;
        let end = |f: usize| g.edge_directions(f, &self.t.manufacturer, &self.t.series).1;
        if !g.edges[e].map_transition {
            return vec![end(e)];
        }
        self.into[g.edges[e].start]
            .iter()
            .filter(|&&f| !g.edges[f].map_transition)
            .map(|&f| end(f))
            .collect()
    }

    /// A robot standing on `node` as it arrived with `heading`.
    fn standing(&self, node: usize, heading: f64) -> Sweep {
        Sweep::from_bodies(
            standing_body(&self.a.graph, node, self.t, heading)
                .into_iter()
                .collect(),
        )
    }

    /// Does driving edge `e` conflict with `other`?
    fn edge_conflicts(&mut self, map: &ConflictMap, e: usize, other: &Sweep) -> bool {
        let (g, t) = (&self.a.graph, self.t);
        self.sweeps
            .entry(e)
            .or_insert_with(|| sweep(g, e, t, map.sample_step))
            .as_ref()
            .is_some_and(|s| map.conflicts(s, other))
    }
}

/// Can a robot of `wait` be on `node` while one of `place_type` stands on `place`
/// (`None`: no), and if so, is it waiting there for the place?
fn can_wait(
    map: &ConflictMap,
    wait: &mut Typed,
    node: usize,
    place: usize,
    on_place: &[Sweep],
    dist: &[f64],
) -> Option<bool> {
    let mut found = None;
    for e in wait.into[node].clone() {
        if wait.a.graph.edges[e].start == place {
            continue; // the way in leads through the occupied place
        }
        for heading in wait.arrival_headings(e) {
            let here = wait.standing(node, heading);
            for there in on_place {
                if map.conflicts(&here, there) || wait.edge_conflicts(map, e, there) {
                    continue;
                }
                found = Some(false);
                // the next edges on a shortest way to the place
                let g = &wait.a.graph;
                let next: Vec<usize> = g
                    .out_edges(node)
                    .iter()
                    .copied()
                    .filter(|&f| {
                        wait.preds[f].contains(&e)
                            && (wait.lengths[f] + dist[f] - dist[e]).abs() < 1e-6
                    })
                    .collect();
                for f in next {
                    let end = wait.a.graph.edges[f].end;
                    let waits = end == place
                        || wait.edge_conflicts(map, f, there)
                        || wait
                            .arrival_headings(f)
                            .into_iter()
                            .any(|h| map.conflicts(&wait.standing(end, h), there));
                    if waits {
                        return Some(true);
                    }
                }
            }
        }
    }
    found
}

/// All cases of a layout file for a fleet, with the waiting condition marked.
/// `map` must be the conflict map of the same file and types.
pub fn cases(lif: &Lif, types: &[TypeRef], map: &ConflictMap) -> Vec<Case> {
    // a layout that is no graph is reported by the reachability check
    let mut typed: Vec<Typed> = types
        .iter()
        .filter_map(|t| Some(Typed::new(t, reach::analyze(lif, t).ok()?)))
        .collect();
    let mut out = Vec::new();
    for i in 0..typed.len() {
        for j in 0..typed.len() {
            let t = typed[i].t;
            let u = typed[j].t;
            let conflicts = blocking(&typed[i].a.graph, map, &u.name, &t.name);
            for (place, station) in places(lif, &typed[i].a) {
                let Some((way, edges)) = way_out(&typed[i].a, t, place) else {
                    continue;
                };
                let nodes: BTreeSet<usize> = way
                    .iter()
                    .skip(1)
                    .chain(
                        edges
                            .iter()
                            .filter_map(|e| conflicts.by_edge.get(e))
                            .flatten(),
                    )
                    .chain(
                        way.iter()
                            .filter_map(|n| conflicts.by_node.get(n))
                            .flatten(),
                    )
                    .copied()
                    .filter(|&n| n != place)
                    .collect();
                // a robot standing on the place, as it arrived
                let on_place: Vec<Sweep> = typed[i].into[place]
                    .iter()
                    .flat_map(|&e| typed[i].arrival_headings(e))
                    .map(|h| typed[i].standing(place, h))
                    .collect();
                if on_place.is_empty() || nodes.is_empty() {
                    continue;
                }
                let wait = &mut typed[j];
                let dist = distances_to(&wait.a.graph, &wait.preds, &wait.lengths, place);
                let blockers: Vec<Blocker> = nodes
                    .into_iter()
                    .filter_map(|n| {
                        let waiting = can_wait(map, wait, n, place, &on_place, &dist)?;
                        Some(Blocker {
                            node: wait.a.graph.nodes[n].id.clone(),
                            waiting,
                        })
                    })
                    .collect();
                if blockers.is_empty() {
                    continue;
                }
                let g = &typed[i].a.graph;
                out.push(Case {
                    place: g.nodes[place].id.clone(),
                    station: station.map(str::to_string),
                    type_place: t.name.clone(),
                    type_wait: u.name.clone(),
                    way: way.iter().map(|&n| g.nodes[n].id.clone()).collect(),
                    blockers,
                });
            }
        }
    }
    out
}

/// One warning per layout and pair of types: the places a waiting robot can block.
pub fn check(lif: &Lif, types: &[TypeRef], map: &ConflictMap) -> Vec<Finding> {
    let all = cases(lif, types, map);
    let mut pairs: Vec<(&str, &str)> = Vec::new();
    for c in &all {
        let pair = (c.type_place.as_str(), c.type_wait.as_str());
        if !pairs.contains(&pair) {
            pairs.push(pair);
        }
    }
    let mut out = Vec::new();
    for pair in pairs {
        for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
            report(layout, pair, &all, &mut out);
        }
    }
    out
}

fn report(layout: &Layout, pair: (&str, &str), all: &[Case], out: &mut Vec<Finding>) {
    let here: HashSet<&str> = layout.nodes.iter().map(|n| n.node_id.as_str()).collect();
    let mine: Vec<(&Case, Vec<&str>)> = all
        .iter()
        .filter(|c| (c.type_place.as_str(), c.type_wait.as_str()) == pair)
        .filter(|c| here.contains(c.place.as_str()))
        .filter_map(|c| {
            let waiting: Vec<&str> = c
                .blockers
                .iter()
                .filter(|b| b.waiting)
                .map(|b| b.node.as_str())
                .collect();
            (!waiting.is_empty()).then_some((c, waiting))
        })
        .collect();
    let Some((first, blockers)) = mine.first() else {
        return;
    };
    let count: usize = mine.iter().map(|(_, b)| b.len()).sum();
    let mut ids: Vec<&str> = Vec::new();
    for (c, waiting) in &mine {
        for id in std::iter::once(c.place.as_str()).chain(waiting.iter().copied()) {
            if !ids.contains(&id) {
                ids.push(id);
            }
        }
    }
    let name = match &first.station {
        Some(s) => format!("{} (station {s})", first.place),
        None => first.place.clone(),
    };
    let named = blockers
        .iter()
        .take(TEXT_NODES)
        .copied()
        .collect::<Vec<_>>()
        .join(", ");
    let more = blockers.len().saturating_sub(TEXT_NODES);
    let more = if more > 0 {
        format!(" and {more} more")
    } else {
        String::new()
    };
    let idx = LayoutIndex::new(layout);
    out.push(
        Finding::new(
            Level::Warning,
            CHECK,
            format!(
                "{} place(s) whose only way out a waiting robot can block ({count} (place, node) case(s)): a robot of {} on the place and one of {} waiting for it, then neither can move; first: {name} blocked from {named}{more}, way out {} to {} ({} node(s))",
                mine.len(),
                pair.0,
                pair.1,
                first.way.first().map_or("", String::as_str),
                first.way.last().map_or("", String::as_str),
                first.way.len(),
            ),
        )
        .layout(&layout.layout_id)
        .robot(pair.0)
        .with_places(idx.nodes(ids.iter().copied()))
        .with_ids(ids),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::finding::PlaceKind;
    use botsim_core::spec::Kinematics;
    use botsim_sweep::{MapOptions, compute};

    /// The neighbourhood of the drop-off `D001` (node N0053) of the roadmap
    /// `W_real_L`: coordinates and curves from the roadmap file. The queue comes by
    /// N0048 - N0049 - N0052; the way out is N0053 - N0052 - N0050 - N0051.
    fn d001() -> (Lif, Vec<TypeRef>) {
        let ty = r#""mobileRobotTypes":[{"manufacturer":"Zuzek2024","seriesName":"AGV"}]"#;
        let node = |(id, x, y): &(&str, f64, f64)| {
            format!(
                r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},"mobileRobotTypeNodeProperties":[{{{ty}}}]}}"#
            )
        };
        // a quarter circle, by its middle control point
        let edge = |(a, b, curve): &(&str, &str, Option<[(f64, f64); 3]>)| {
            let curve = curve.map_or(String::new(), |[p, q, r]| {
                format!(
                    r#","trajectory":{{"degree":2,"knotVector":[0,0,0,1,1,1],"controlPoints":[{{"x":{},"y":{}}},{{"x":{},"y":{},"weight":0.7071067811865476}},{{"x":{},"y":{}}}]}}"#,
                    p.0, p.1, q.0, q.1, r.0, r.1
                )
            });
            format!(
                r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}","mobileRobotTypeEdgeProperties":[{{{ty}{curve}}}]}}"#
            )
        };
        let nodes: Vec<String> = [
            ("N0048", 14.0, 12.25),
            ("N0049", 14.5, 12.25),
            ("N0050", 15.0, 12.25),
            ("N0051", 19.5, 12.25),
            ("N0052", 14.75, 12.5),
            ("N0053", 14.75, 12.75),
        ]
        .iter()
        .map(node)
        .collect();
        let edges: Vec<String> = [
            ("N0048", "N0049", None),
            ("N0049", "N0050", None),
            (
                "N0049",
                "N0052",
                Some([(14.5, 12.25), (14.75, 12.25), (14.75, 12.5)]),
            ),
            ("N0050", "N0051", None),
            (
                "N0052",
                "N0050",
                Some([(14.75, 12.5), (14.75, 12.25), (15.0, 12.25)]),
            ),
            ("N0052", "N0053", None),
            ("N0053", "N0052", None),
        ]
        .iter()
        .map(edge)
        .collect();
        let lif: Lif = serde_json::from_str(&format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1","nodes":[{}],"edges":[{}],
            "stations":[{{"stationId":"D001","interactionNodeIds":["N0053"]}}]}}]}}]}}"#,
            nodes.join(","),
            edges.join(",")
        ))
        .expect("test LIF parses");
        let agv = TypeRef::test_rect("Zuzek2024/AGV", Kinematics::Differential, 0.48, 0.24);
        (lif, vec![agv])
    }

    #[test]
    fn the_queue_node_blocks_the_drop_off() {
        let (lif, types) = d001();
        let map = compute(&lif, &types, &MapOptions::default()).expect("map");
        let cases = cases(&lif, &types, &map);
        let c = cases.iter().find(|c| c.place == "N0053").expect("D001");
        assert_eq!(c.station.as_deref(), Some("D001"));
        assert_eq!(c.way, ["N0053", "N0052", "N0050", "N0051"]);
        let blocker = |n: &str| c.blockers.iter().find(|b| b.node == n);
        // a robot waiting on N0049: in the way, there at once, waiting for the place
        assert!(blocker("N0049").expect("N0049").waiting);
        // next to the place: both cannot be there at once
        assert!(blocker("N0052").is_none());
        // far down the way: in it, but a robot there is not waiting for the place
        assert!(!blocker("N0051").expect("N0051").waiting);

        let f = check(&lif, &types, &map);
        assert_eq!(f.len(), 1, "{f:?}");
        assert_eq!(f[0].level, Level::Warning);
        assert!(f[0].ids.contains(&"N0053".to_string()) && f[0].ids.contains(&"N0049".to_string()));
        assert!(!f[0].ids.contains(&"N0051".to_string()));
        assert!(f[0].places.iter().all(|p| p.kind == PlaceKind::Node));
        assert!(f[0].text.contains("station D001"), "{}", f[0].text);
    }

    #[test]
    fn a_place_with_two_ways_out_is_none() {
        let (mut lif, types) = d001();
        // a second edge out of D001: no case for it any more
        let l = &mut lif.origins[0].layouts[0];
        let mut second = l
            .edges
            .iter()
            .find(|e| e.edge_id == "N0053-N0052")
            .expect("edge")
            .clone();
        second.edge_id = "N0053-N0048".into();
        second.end_node_id = "N0048".into();
        l.edges.push(second);
        let map = compute(&lif, &types, &MapOptions::default()).expect("map");
        assert!(cases(&lif, &types, &map).iter().all(|c| c.place != "N0053"));
    }
}

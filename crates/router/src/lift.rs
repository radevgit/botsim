//! A route through the waiting nodes of lift entrances (S49).
//!
//! A **ride** is a layout edge between two stops of one lift. A robot that waits
//! for a lift has to wait outside the approach to the entrance: at the stop's
//! waiting node (lay-by), if the layout has one, so its route is planned through
//! that node. Who may use a lift, and when, is not decided here.

use std::collections::{HashMap, HashSet};

use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};

use crate::route::{Arrival, Mover, plan_path};

/// The cheapest route from `from` to `to`, planned **through the waiting node** of
/// every lift entrance it uses that has one: to the waiting node, from there to the
/// entrance, then the ride. No detour if the robot starts on the entrance, already
/// passes the waiting node before the ride, or the waiting node cannot be reached
/// (also if the graph has no such node).
///
/// `rides` holds every ride with the waiting node of its entrance (the node the
/// ride starts at), `None` if there is none. `arrival` says how the robot gets to
/// `from` (S73), `mover` what it can drive. `None` if there is no route.
///
/// `from`, `to`, `arrival` and the lists of `mover` must fit the graph, as for
/// [`plan_path`].
pub fn route_through_waiting(
    graph: &Graph,
    mover: &Mover,
    rides: &HashMap<EdgeIdx, Option<NodeIdx>>,
    arrival: Arrival,
    from: NodeIdx,
    to: NodeIdx,
) -> Option<Vec<EdgeIdx>> {
    let none = HashSet::new();
    let path_of = |a: NodeIdx, arrival: Arrival, b: NodeIdx| {
        plan_path(graph, mover, arrival, a, b, &none, |_| false)
    };
    let path = path_of(from, arrival, to)?;
    let mut nodes = vec![from];
    nodes.extend(path.iter().map(|&e| graph.edges[e].end));
    for (i, &e) in path.iter().enumerate() {
        let Some(&waiting) = rides.get(&e) else {
            continue;
        };
        let (a, b) = (graph.edges[e].start, graph.edges[e].end);
        let before = match waiting {
            Some(w) if i > 0 && w < graph.nodes.len() && !nodes[..i].contains(&w) => {
                let detour = path_of(from, arrival, w).and_then(|mut to_w| {
                    let at_w = to_w.last().map_or(arrival, |&l| Arrival::Edge(l));
                    to_w.extend(path_of(w, at_w, a)?);
                    Some(to_w)
                });
                detour.unwrap_or_else(|| path[..i].to_vec())
            }
            _ => path[..i].to_vec(),
        };
        let mut edges = before;
        edges.push(e);
        edges.extend(route_through_waiting(
            graph,
            mover,
            rides,
            Arrival::Edge(e),
            b,
            to,
        )?);
        return Some(edges);
    }
    Some(path)
}

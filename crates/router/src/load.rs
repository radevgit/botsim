//! The route cost with the load and waiting penalties (S91, S93), so that a route
//! prefers an empty lane to a busy one.

use botsim_model::graph::Graph;

/// What the penalties are computed from, for the robot whose route is planned.
/// A list may be empty or short: what it does not name is not held, and nobody
/// waits there.
#[derive(Debug, Clone, Copy)]
pub struct Load<'a> {
    /// Metres added to an edge for another robot holding it, and again for another
    /// robot holding the node it ends at.
    pub load_weight: f64,
    /// Metres added to an edge for each robot in the waiting average of the node it
    /// ends at.
    pub wait_weight: f64,
    /// `edge_held[e]`: a robot other than this one holds edge `e`.
    pub edge_held: &'a [bool],
    /// `node_held[n]`: a robot other than this one holds node `n`.
    pub node_held: &'a [bool],
    /// `node_wait[n]`: how many robots have been waiting on node `n`, as a moving
    /// average (1.0 = one robot all the time).
    pub node_wait: &'a [f64],
}

/// The route cost of every edge: `cost[e]` plus the penalties of `load`, one entry
/// for each entry of `cost`. An edge the graph does not have gets no penalty of an
/// end node. `None` if neither weight is above 0 (use `cost` as it is).
pub fn loaded_cost(graph: &Graph, cost: &[f64], load: &Load) -> Option<Vec<f64>> {
    if load.load_weight <= 0.0 && load.wait_weight <= 0.0 {
        return None;
    }
    let held = |list: &[bool], i: usize| list.get(i).copied().unwrap_or(false);
    Some(
        cost.iter()
            .enumerate()
            .map(|(e, &c)| {
                let end = graph.edges.get(e).map(|edge| edge.end);
                let n = usize::from(held(load.edge_held, e))
                    + usize::from(end.is_some_and(|end| held(load.node_held, end)));
                let waiting = end
                    .and_then(|end| load.node_wait.get(end))
                    .copied()
                    .unwrap_or(0.0);
                c + load.load_weight * n as f64 + load.wait_weight * waiting
            })
            .collect(),
    )
}

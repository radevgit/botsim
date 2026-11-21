//! Turn a route into VDA 5050 `order` messages with a release window.
//!
//! A [`Plan`] is the whole route: node `i` has `sequenceId` 2·i and edge `i`
//! (from node `i` to node `i + 1`) has 2·i + 1, so the numbering stays the same
//! in every update of the order (VDA 5050 §6.1).
//!
//! **Release window** ([`RELEASE_WINDOW`] = 2; S29 chose 3, then the user changed
//! it to 2 after seeing it): the robot's node and the next 2 nodes are released
//! (base); everything after is the
//! horizon, sent unreleased. An update, sent from the decision node (the last
//! released node), moves the window on.
//!
//! **Jobs** ([`Plan::for_job`]): `pick` at the pickup node and `drop` at the drop
//! node, both HARD; their duration is the simulator's per-type action time.
//! Action ids are `<type>-<plan node index>`, e.g. `pick-2`. An update re-sends
//! the decision node with its actions, as the standard describes; the robot
//! does not run them again.
//!
//! Not yet: edge orientation, `maximumSpeed`.

use botsim_core::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_core::jobs::Job;
use botsim_core::world::{PROTOCOL_VERSION, RobotId};
use serde_json::json;
use vda5050::order::Order;

use crate::route::shortest_path;

/// How many nodes ahead of the robot are released.
pub const RELEASE_WINDOW: usize = 2;

/// Action type at the pickup node.
pub const PICK: &str = "pick";
/// Action type at the drop node.
pub const DROP: &str = "drop";

/// A whole route: `nodes.len() == edges.len() + 1`.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    pub nodes: Vec<NodeIdx>,
    pub edges: Vec<EdgeIdx>,
    /// Action types per node (same length as `nodes`); each runs as a HARD
    /// action (the robot stands still).
    pub actions: Vec<Vec<String>>,
}

impl Plan {
    /// The plan for `edges` (a route from `start`, e.g. from `shortest_path`).
    pub fn new(graph: &Graph, start: NodeIdx, edges: Vec<EdgeIdx>) -> Plan {
        let mut nodes = vec![start];
        nodes.extend(edges.iter().map(|&e| graph.edges[e].end));
        let actions = vec![Vec::new(); nodes.len()];
        Plan {
            nodes,
            edges,
            actions,
        }
    }

    /// The plan of a transport job for a robot standing on `at`: drive to the
    /// pickup node and `pick`, drive to the drop node and `drop`. `None` if a
    /// node cannot be reached.
    pub fn for_job(graph: &Graph, at: NodeIdx, job: &Job) -> Option<Plan> {
        let mut edges = shortest_path(graph, at, job.pickup)?;
        let pick_at = edges.len();
        edges.extend(shortest_path(graph, job.pickup, job.drop)?);
        let mut plan = Plan::new(graph, at, edges);
        let drop_at = plan.last();
        plan.actions[pick_at].push(PICK.to_string());
        plan.actions[drop_at].push(DROP.to_string());
        Some(plan)
    }

    /// Index of the last node.
    pub fn last(&self) -> usize {
        self.edges.len()
    }

    /// The last released node index when the robot is at node `at`.
    pub fn released_through(&self, at: usize, window: usize) -> usize {
        (at + window).min(self.last())
    }
}

/// What identifies and numbers one message.
#[derive(Debug, Clone)]
pub struct OrderHeader<'a> {
    pub robot: &'a RobotId,
    pub header_id: u32,
    pub timestamp: &'a str,
    pub order_id: &'a str,
    pub order_update_id: u32,
}

/// The order (or update) message that starts at plan node `from` and releases
/// the plan up to node `released_through`.
///
/// `from` is 0 for a new order, or the decision node for an update; the first
/// node of an order is always released, so `from <= released_through` is
/// required (a caller bug otherwise).
pub fn build_order(
    graph: &Graph,
    h: &OrderHeader,
    plan: &Plan,
    from: usize,
    released_through: usize,
) -> Order {
    let released_through = released_through.min(plan.last());
    assert!(from <= released_through, "the first node must be released");

    let nodes: Vec<_> = (from..=plan.last())
        .map(|i| {
            json!({
                "nodeId": graph.nodes[plan.nodes[i]].id,
                "sequenceId": 2 * i,
                "released": i <= released_through,
                "actions": plan.actions[i]
                    .iter()
                    .map(|t| json!({
                        "actionId": format!("{t}-{i}"),
                        "actionType": t,
                        "blockingType": "HARD",
                    }))
                    .collect::<Vec<_>>(),
            })
        })
        .collect();
    let edges: Vec<_> = (from..plan.last())
        .map(|i| {
            json!({
                "edgeId": graph.edges[plan.edges[i]].id,
                "sequenceId": 2 * i + 1,
                // an edge is released together with the node it leads to
                "released": i < released_through,
                "actions": [],
            })
        })
        .collect();

    let message = json!({
        "headerId": h.header_id,
        "timestamp": h.timestamp,
        "version": PROTOCOL_VERSION,
        "manufacturer": h.robot.manufacturer,
        "serialNumber": h.robot.serial_number,
        "orderId": h.order_id,
        "orderUpdateId": h.order_update_id,
        "nodes": nodes,
        "edges": edges,
    });
    serde_json::from_value(message).expect("the message has all required order fields")
}

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
//! **Edge orientation** ([`build_order_with`]): for omnidirectional robots each edge
//! carries the LIF's orientation for the type. Not yet: `maximumSpeed`.

use botsim_model::graph::{EdgeIdx, EdgeOrientation, Graph, NodeIdx};
use botsim_model::job::Job;
use botsim_model::protocol::PROTOCOL_VERSION;
use botsim_model::robot::RobotId;
use botsim_router::route::shortest_path;
use vda5050::order::{Action, BlockingType, Edge, Node, Order, OrientationType};

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
        let pickup_at = edges.len();
        edges.extend(shortest_path(graph, job.pickup, job.drop)?);
        Plan::for_edges(graph, at, edges, pickup_at)
    }

    /// The plan of a transport job along a route that is already found: `edges`
    /// from `start`, the first `pickup_at` of them to the pickup node, where the
    /// robot does `pick`, the rest to the drop node, the last one, where it does
    /// `drop`. `None` if `pickup_at` is beyond the route or an edge is not in the
    /// graph.
    pub fn for_edges(
        graph: &Graph,
        start: NodeIdx,
        edges: Vec<EdgeIdx>,
        pickup_at: usize,
    ) -> Option<Plan> {
        if pickup_at > edges.len() || edges.iter().any(|&e| e >= graph.edges.len()) {
            return None;
        }
        let mut plan = Plan::new(graph, start, edges);
        let drop_at = plan.last();
        plan.actions[pickup_at].push(PICK.to_string());
        plan.actions[drop_at].push(DROP.to_string());
        Some(plan)
    }

    /// This route up to node `at`, then the route `then`, which starts there. The
    /// actions `then` has on its first node are dropped: a robot does not run the
    /// actions of the node an order update starts at.
    pub fn continued(&self, at: usize, then: Plan) -> Plan {
        let at = at.min(self.last());
        let mut plan = Plan {
            nodes: self.nodes[..=at].to_vec(),
            edges: self.edges[..at].to_vec(),
            actions: self.actions[..=at].to_vec(),
        };
        plan.nodes.extend(then.nodes.into_iter().skip(1));
        plan.edges.extend(then.edges);
        plan.actions.extend(then.actions.into_iter().skip(1));
        plan
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
/// required. `None` if it is not (a caller bug), or if the message cannot be built.
pub fn build_order(
    graph: &Graph,
    h: &OrderHeader,
    plan: &Plan,
    from: usize,
    released_through: usize,
) -> Option<Order> {
    build_order_with(graph, h, plan, from, released_through, None)
}

/// Like [`build_order`]; with `orient` (manufacturer and series of the robot) every
/// edge carries the orientation the LIF gives that type on it (`orientationType`,
/// `orientation`), so that an omnidirectional robot faces as the layout says (along
/// the road for `TANGENTIAL` 0) instead of keeping its heading. An edge for which
/// the LIF gives none is sent without.
pub fn build_order_with(
    graph: &Graph,
    h: &OrderHeader,
    plan: &Plan,
    from: usize,
    released_through: usize,
    orient: Option<(&str, &str)>,
) -> Option<Order> {
    let released_through = released_through.min(plan.last());
    if from > released_through {
        return None; // the first node must be released
    }

    // `None` if a sequence number does not fit the message's 32 bits
    let nodes = (from..=plan.last())
        .map(|i| {
            Some(Node {
                node_id: graph.nodes[plan.nodes[i]].id.clone(),
                sequence_id: u32::try_from(2 * i).ok()?,
                node_descriptor: None,
                released: i <= released_through,
                node_position: None,
                actions: plan.actions[i]
                    .iter()
                    .map(|t| Action {
                        action_type: t.clone(),
                        action_id: format!("{t}-{i}"),
                        action_descriptor: None,
                        blocking_type: BlockingType::Hard,
                        action_parameters: None,
                        retriable: None,
                    })
                    .collect(),
            })
        })
        .collect::<Option<Vec<_>>>()?;
    let edges = (from..plan.last())
        .map(|i| {
            let given = orient.and_then(|(m, s)| graph.edge_orientation(plan.edges[i], m, s));
            let (orientation_type, orientation) = match given {
                Some(EdgeOrientation::Tangential(a)) => {
                    (Some(OrientationType::Tangential), Some(a))
                }
                Some(EdgeOrientation::Global(a)) => (Some(OrientationType::Global), Some(a)),
                None => (None, None),
            };
            Some(Edge {
                edge_id: graph.edges[plan.edges[i]].id.clone(),
                sequence_id: u32::try_from(2 * i + 1).ok()?,
                edge_descriptor: None,
                // an edge is released together with the node it leads to
                released: i < released_through,
                maximum_speed: None,
                maximum_mobile_robot_height: None,
                minimum_load_handling_device_height: None,
                // (JSON has no number that is not finite: such an angle is not sent)
                orientation: orientation.filter(|a| a.is_finite()),
                orientation_type,
                direction: None,
                reach_orientation_before_entering: None,
                maximum_rotation_speed: None,
                trajectory: None,
                length: None,
                corridor: None,
                actions: Vec::new(),
            })
        })
        .collect::<Option<Vec<_>>>()?;

    Some(Order {
        header_id: h.header_id,
        timestamp: h.timestamp.to_string(),
        version: PROTOCOL_VERSION.to_string(),
        manufacturer: h.robot.manufacturer.clone(),
        serial_number: h.robot.serial_number.clone(),
        order_id: h.order_id.to_string(),
        order_update_id: h.order_update_id,
        order_description: None,
        nodes,
        edges,
    })
}

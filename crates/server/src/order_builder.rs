//! Turn a route into order messages ([`botsim_link::order`], VDA 5050 `order`)
//! with a release window.
//!
//! A [`Plan`] is the whole route: node `i` has `sequenceId` 2·i and edge `i`
//! (from node `i` to node `i + 1`) has 2·i + 1, so the numbering stays the same
//! in every update of the order (VDA 5050 §6.1). An order that starts at plan node
//! `from` therefore has the first step `from`.
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

use botsim_link::ids::{ActionId, ActionType, OrderId};
use botsim_link::message::{Header, Millis, ToRobot, ToRobotBody};
use botsim_link::order::{Action, Blocking, Leg, NodeAction, Order, OrientationType};
use botsim_model::graph::{EdgeIdx, EdgeOrientation, Graph, NodeIdx};
use botsim_model::job::Job;
use botsim_model::robot::RobotNo;
use botsim_router::route::shortest_path;

/// How many nodes ahead of the robot are released.
pub const RELEASE_WINDOW: usize = 2;

/// Action type at the pickup node.
pub const PICK: &str = "pick";
/// Action type at the drop node.
pub const DROP: &str = "drop";

/// A whole route: `nodes.len() == edges.len() + 1`.
#[derive(Debug, Clone, PartialEq, Hash)]
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
    /// The robot the order goes to.
    pub robot: RobotNo,
    /// The message's sequence number ([`Header::seq`]).
    pub seq: u32,
    pub time: Millis,
    pub order_id: &'a OrderId,
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
) -> Option<ToRobot> {
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
) -> Option<ToRobot> {
    let released_through = released_through.min(plan.last());
    if from > released_through {
        return None; // the first node must be released
    }

    let legs = (from..plan.last())
        .map(|i| {
            let edge = *plan.edges.get(i)?;
            graph.edges.get(edge)?;
            let given = orient.and_then(|(m, s)| graph.edge_orientation(edge, m, s));
            let (orientation_type, orientation) = match given {
                Some(EdgeOrientation::Tangential(a)) => {
                    (Some(OrientationType::Tangential), Some(a))
                }
                Some(EdgeOrientation::Global(a)) => (Some(OrientationType::Global), Some(a)),
                None => (None, None),
            };
            Some(Leg {
                edge,
                end: *plan.nodes.get(i + 1)?,
                // (an angle that is not finite is not sent)
                orientation: orientation.filter(|a| a.is_finite()),
                orientation_type,
            })
        })
        .collect::<Option<Vec<_>>>()?;
    let mut actions = Vec::new();
    for i in from..=plan.last() {
        for t in plan.actions.get(i)? {
            actions.push(NodeAction {
                node: u32::try_from(i - from).ok()?,
                action: Action {
                    id: ActionId::new(format!("{t}-{i}")),
                    kind: ActionType::new(t.as_str()),
                    blocking: Blocking::Hard,
                },
            });
        }
    }
    // (an order checks that every sequence number fits the message's 32 bits)
    let order = Order::new(
        h.order_id.clone(),
        h.order_update_id,
        u32::try_from(from).ok()?,
        *plan.nodes.get(from)?,
        legs,
        // (an edge is released together with the node it leads to)
        u32::try_from(released_through - from).ok()?,
        actions,
    )
    .ok()?;

    Some(ToRobot {
        header: Header {
            robot: h.robot,
            seq: h.seq,
            time: h.time,
        },
        body: ToRobotBody::Order(order),
    })
}

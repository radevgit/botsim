//! Turn an order ([`botsim_link::order::Order`], VDA 5050 meaning) into the route
//! the driver follows: the released base, checked against the layout.
//!
//! An [`Order`] has by its form what VDA 5050 §6.1 asks of the lists of a message:
//! a start node and legs (edge, end node), continuous `sequenceId`s, a released
//! base from the start node on and the horizon behind it. What is left to check
//! here: that its numbers are in the layout, that each edge connects its two nodes
//! and that a new order starts at `sequenceId` 0. Only the base is driven; the
//! horizon is ignored for now.
//!
//! Node actions of the base are carried along (see [`OrderRoute::node_actions`]).
//!
//! Not yet: edge actions, edge speed limits, node positions and `theta`; the
//! special "re-send the decision node and add a new node at the same position"
//! form of releasing new actions (it needs a zero-length edge). The graph is the
//! truth for positions.

use std::fmt;

use botsim_link::ids::OrderId;
use botsim_link::order::Order;

use crate::actions::ActionSpec;
use crate::drive::RouteEdge;
use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};

/// The part of an order the robot may drive now.
#[derive(Debug, Clone, PartialEq)]
pub struct OrderRoute {
    /// Node the robot starts at (the first node of the order).
    pub start: NodeIdx,
    /// Released edges, in driving order.
    pub edges: Vec<RouteEdge>,
    /// The order continues beyond the released base (a horizon exists).
    pub has_horizon: bool,
    /// Actions to run at the nodes of the base: entry 0 is the start node, entry
    /// i + 1 the node where edge i ends (so `edges.len() + 1` entries). The start
    /// node of an order *update* is the decision point, whose actions were
    /// already queued earlier and are not run again: its entry is empty.
    pub node_actions: Vec<Vec<ActionSpec>>,
    /// The `orderId` and `orderUpdateId` of the order (for the state report).
    pub order_id: OrderId,
    pub order_update_id: u32,
    /// All nodes and edges of the order, base and horizon, for the state report.
    /// Entry i of `plan_nodes` is the node where released edge i starts (so the
    /// base is the first `edges.len() + 1` nodes); `plan_edges[i]` is the edge
    /// after `plan_nodes[i]`.
    pub plan_nodes: Vec<PlanNode>,
    pub plan_edges: Vec<PlanEdge>,
}

/// A node of the order as the state report carries it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanNode {
    pub node: NodeIdx,
    pub sequence_id: u32,
    pub released: bool,
}

/// An edge of the order as the state report carries it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanEdge {
    pub edge: EdgeIdx,
    pub sequence_id: u32,
    pub released: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum OrderError {
    /// A new order (orderUpdateId 0) must start at sequenceId 0.
    FirstNodeNotZero { found: u32 },
    /// A node or edge number of the order is not in the layout.
    OutsideLayout { what: &'static str, number: usize },
    /// The layout edge does not run between the order's two nodes in this direction.
    EdgeDoesNotConnect { edge: String },
}

impl fmt::Display for OrderError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            OrderError::FirstNodeNotZero { found } => {
                write!(
                    f,
                    "first node of a new order has sequenceId {found}, expected 0"
                )
            }
            OrderError::OutsideLayout { what, number } => {
                write!(f, "{what} number {number} is not in the layout")
            }
            OrderError::EdgeDoesNotConnect { edge } => {
                write!(
                    f,
                    "edge '{edge}' does not connect the order's nodes in this direction"
                )
            }
        }
    }
}

impl std::error::Error for OrderError {}

/// The released base of `order` as a route on `graph`.
pub fn route_from_order(graph: &Graph, order: &Order) -> Result<OrderRoute, OrderError> {
    let first = 2 * order.first_step();
    if order.update_id() == 0 && first != 0 {
        return Err(OrderError::FirstNodeNotZero { found: first });
    }
    let released = order.released();

    // match against the graph
    let node_in_layout = |number: NodeIdx| {
        (number < graph.nodes.len())
            .then_some(number)
            .ok_or(OrderError::OutsideLayout {
                what: "node",
                number,
            })
    };
    let start = node_in_layout(order.start())?;

    let mut route = Vec::new();
    let mut plan_nodes = Vec::with_capacity(order.legs().len() + 1);
    let mut plan_edges = Vec::with_capacity(order.legs().len());
    // (`Order::new` has checked that the numbers of the order's nodes fit)
    plan_nodes.push(PlanNode {
        node: start,
        sequence_id: first,
        released: true,
    });
    let mut at = start;
    for (i, leg) in (0u32..).zip(order.legs()) {
        let end = node_in_layout(leg.end)?;
        let edge = graph.edges.get(leg.edge).ok_or(OrderError::OutsideLayout {
            what: "edge",
            number: leg.edge,
        })?;
        if edge.start != at || edge.end != end {
            return Err(OrderError::EdgeDoesNotConnect {
                edge: edge.id.clone(),
            });
        }
        let is_released = i < released;
        if is_released {
            route.push(RouteEdge {
                edge: leg.edge,
                orientation: leg.orientation,
                orientation_type: leg.orientation_type,
            });
        }
        plan_edges.push(PlanEdge {
            edge: leg.edge,
            sequence_id: first + 2 * i + 1,
            released: is_released,
        });
        plan_nodes.push(PlanNode {
            node: end,
            sequence_id: first + 2 * i + 2,
            released: is_released,
        });
        at = end;
    }

    // actions of the base nodes (the first node plus the end node of every released edge)
    let node_actions = (0..=released)
        .map(|node| {
            if node == 0 && order.update_id() > 0 {
                return Vec::new();
            }
            order
                .actions_at(node)
                .iter()
                .map(|a| ActionSpec {
                    action_id: a.action.id.clone(),
                    action_type: a.action.kind.clone(),
                    blocking: a.action.blocking,
                })
                .collect()
        })
        .collect();

    Ok(OrderRoute {
        start,
        has_horizon: route.len() < order.legs().len(),
        edges: route,
        node_actions,
        order_id: order.order_id().clone(),
        order_update_id: order.update_id(),
        plan_nodes,
        plan_edges,
    })
}

//! Turn a VDA 5050 `Order` into the route the driver follows: the released
//! base, matched against the graph by id.
//!
//! Rules from VDA 5050 §6.1: `sequenceId` is shared by nodes and edges and
//! continuous (node, edge, node, …; an edge n connects the nodes n-1 and n+1);
//! at least one node and one edge fewer than nodes; the first node is always
//! released; an edge is released only if both its nodes are; after an
//! unreleased edge nothing released follows. Released = base, unreleased =
//! horizon. Only the base is driven; the horizon is ignored for now.
//!
//! Node actions of the base are carried along (see [`OrderRoute::node_actions`]).
//!
//! Not yet: edge actions, edge speed limits, node positions and `theta`; the
//! special "re-send the decision node and add a new node at the same position"
//! form of releasing new actions (it needs a zero-length edge). The graph is the
//! truth for positions.

use std::fmt;

use vda5050::order::Order;

use crate::actions::ActionSpec;
use crate::drive::RouteEdge;
use botsim_model::graph::{Graph, NodeIdx};

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
    /// The `orderId` and `orderUpdateId` of the message (for the state message).
    pub order_id: String,
    pub order_update_id: u32,
    /// All nodes and edges of the message, base and horizon, for the state
    /// message (`nodeStates`, `edgeStates`). Entry i of `plan_nodes` is the node
    /// where released edge i starts (so the base is the first `edges.len() + 1`
    /// nodes); `plan_edges[i]` is the edge after `plan_nodes[i]`.
    pub plan_nodes: Vec<PlanNode>,
    pub plan_edges: Vec<PlanEdge>,
}

/// A node of the order as the state message reports it.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanNode {
    pub node_id: String,
    pub sequence_id: u32,
    pub released: bool,
}

/// An edge of the order as the state message reports it.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanEdge {
    pub edge_id: String,
    pub sequence_id: u32,
    pub released: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum OrderError {
    /// No nodes, or the edge count is not the node count minus one.
    WrongCounts { nodes: usize, edges: usize },
    /// A sequenceId is not what the position in the list requires.
    BadSequenceId {
        what: &'static str,
        id: String,
        expected: u32,
        found: u32,
    },
    /// A new order (orderUpdateId 0) must start at sequenceId 0.
    FirstNodeNotZero { found: u32 },
    /// The first node of an order must be released.
    FirstNodeNotReleased,
    /// An edge is released but one of its nodes is not.
    EdgeReleasedWithoutNode { edge: String },
    /// Something is released after an unreleased edge or node.
    ReleasedAfterHorizon { id: String },
    /// A nodeId or edgeId of the order is not in the layout.
    Unknown { what: &'static str, id: String },
    /// The layout edge does not run between the order's two nodes in this direction.
    EdgeDoesNotConnect { edge: String },
}

impl fmt::Display for OrderError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            OrderError::WrongCounts { nodes, edges } => write!(
                f,
                "order needs at least one node and nodes - 1 edges, got {nodes} nodes and {edges} edges"
            ),
            OrderError::BadSequenceId {
                what,
                id,
                expected,
                found,
            } => write!(
                f,
                "{what} '{id}' has sequenceId {found}, expected {expected}"
            ),
            OrderError::FirstNodeNotZero { found } => {
                write!(
                    f,
                    "first node of a new order has sequenceId {found}, expected 0"
                )
            }
            OrderError::FirstNodeNotReleased => write!(f, "the first node must be released"),
            OrderError::EdgeReleasedWithoutNode { edge } => {
                write!(f, "edge '{edge}' is released but one of its nodes is not")
            }
            OrderError::ReleasedAfterHorizon { id } => {
                write!(f, "'{id}' is released after an unreleased edge or node")
            }
            OrderError::Unknown { what, id } => write!(f, "{what} '{id}' is not in the layout"),
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
    let (nodes, edges) = (&order.nodes, &order.edges);
    if nodes.is_empty() || edges.len() + 1 != nodes.len() {
        return Err(OrderError::WrongCounts {
            nodes: nodes.len(),
            edges: edges.len(),
        });
    }

    // sequenceIds: continuous, node, edge, node, ...
    let first = nodes[0].sequence_id;
    if order.order_update_id == 0 && first != 0 {
        return Err(OrderError::FirstNodeNotZero { found: first });
    }
    for (i, n) in nodes.iter().enumerate() {
        let expected = first + 2 * i as u32;
        if n.sequence_id != expected {
            return Err(OrderError::BadSequenceId {
                what: "node",
                id: n.node_id.clone(),
                expected,
                found: n.sequence_id,
            });
        }
    }
    for (i, e) in edges.iter().enumerate() {
        let expected = first + 2 * i as u32 + 1;
        if e.sequence_id != expected {
            return Err(OrderError::BadSequenceId {
                what: "edge",
                id: e.edge_id.clone(),
                expected,
                found: e.sequence_id,
            });
        }
    }

    // release rules
    if !nodes[0].released {
        return Err(OrderError::FirstNodeNotReleased);
    }
    // (A released edge after the horizon cannot get here: it would need a
    // released node after an unreleased edge, caught below, or an unreleased node.)
    let mut horizon_started = false;
    for (i, e) in edges.iter().enumerate() {
        if e.released && !(nodes[i].released && nodes[i + 1].released) {
            return Err(OrderError::EdgeReleasedWithoutNode {
                edge: e.edge_id.clone(),
            });
        }
        if !e.released {
            horizon_started = true;
        }
        if horizon_started && nodes[i + 1].released {
            // an unreleased edge followed by a released node
            return Err(OrderError::ReleasedAfterHorizon {
                id: nodes[i + 1].node_id.clone(),
            });
        }
    }

    // match against the graph
    let node_idx = |id: &str| {
        graph.node_idx(id).ok_or_else(|| OrderError::Unknown {
            what: "node",
            id: id.to_string(),
        })
    };
    let start = node_idx(&nodes[0].node_id)?;

    let mut route = Vec::new();
    for (i, e) in edges.iter().enumerate() {
        let a = node_idx(&nodes[i].node_id)?;
        let b = node_idx(&nodes[i + 1].node_id)?;
        let ei = graph
            .edge_idx(&e.edge_id)
            .ok_or_else(|| OrderError::Unknown {
                what: "edge",
                id: e.edge_id.clone(),
            })?;
        let ge = &graph.edges[ei];
        if ge.start != a || ge.end != b {
            return Err(OrderError::EdgeDoesNotConnect {
                edge: e.edge_id.clone(),
            });
        }
        if e.released {
            route.push(RouteEdge {
                edge: ei,
                orientation: e.orientation,
                orientation_type: e.orientation_type,
            });
        }
    }

    // actions of the base nodes (the first node plus the end node of every released edge)
    let node_actions = nodes[..=route.len()]
        .iter()
        .enumerate()
        .map(|(i, n)| {
            if i == 0 && order.order_update_id > 0 {
                return Vec::new();
            }
            n.actions
                .iter()
                .map(|a| ActionSpec {
                    action_id: a.action_id.clone(),
                    action_type: a.action_type.clone(),
                    blocking: a.blocking_type,
                })
                .collect()
        })
        .collect();

    Ok(OrderRoute {
        start,
        has_horizon: route.len() < edges.len(),
        edges: route,
        node_actions,
        order_id: order.order_id.clone(),
        order_update_id: order.order_update_id,
        plan_nodes: nodes
            .iter()
            .map(|n| PlanNode {
                node_id: n.node_id.clone(),
                sequence_id: n.sequence_id,
                released: n.released,
            })
            .collect(),
        plan_edges: edges
            .iter()
            .map(|e| PlanEdge {
                edge_id: e.edge_id.clone(),
                sequence_id: e.sequence_id,
                released: e.released,
            })
            .collect(),
    })
}

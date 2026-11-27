//! The order: the way a robot is to drive and what it is to do on it.
//!
//! An order has a start node and legs; each leg is an edge and the node it ends at.
//! Node 0 is the start node, node `i + 1` the node where leg `i` ends. The VDA 5050
//! `sequenceId`s are not stored: node `i` has `2 · (first_step + i)` and leg `i` the
//! number after node `i`.
//!
//! The first [`released`](Order::released) legs and the nodes up to node `released`
//! are the base, the rest is the horizon. So the start node is always released, and
//! an edge is released together with the node it ends at.

use botsim_model::graph::{EdgeIdx, NodeIdx};

use crate::ids::{ActionId, ActionType, OrderId};
use crate::shape::ShapeError;

/// An order or an order update.
///
/// [`Order::new`] promises:
/// - no more legs are released than there are;
/// - every action is at a node of the order;
/// - the actions are sorted by node, and the order of the actions at one node is the
///   one they were given in;
/// - every `sequenceId` fits in 32 bits.
///
/// It does not look at the layout: whether the edges exist and connect the nodes is
/// for the reader to check.
#[derive(Debug, Clone, PartialEq)]
pub struct Order {
    order_id: OrderId,
    update_id: u32,
    first_step: u32,
    start: NodeIdx,
    legs: Vec<Leg>,
    released: u32,
    actions: Vec<NodeAction>,
}

/// An edge to drive and the node it ends at.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Leg {
    pub edge: EdgeIdx,
    pub end: NodeIdx,
    /// VDA 5050 edge `orientation` (rad), if the order gives one.
    pub orientation: Option<f64>,
    /// VDA 5050 edge `orientationType`, if the order gives one.
    pub orientation_type: Option<OrientationType>,
}

/// What an edge orientation is relative to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OrientationType {
    /// The map.
    Global,
    /// The direction of the edge.
    Tangential,
}

/// An action at a node of an order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeAction {
    /// The node's number in the order (0 is the start node).
    pub node: u32,
    pub action: Action,
}

/// An action of an order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Action {
    pub id: ActionId,
    pub kind: ActionType,
    pub blocking: Blocking,
}

/// VDA 5050 `blockingType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Blocking {
    None,
    Soft,
    Single,
    Hard,
}

impl Order {
    /// `first_step` is half the `sequenceId` of the start node: 0 for a new order,
    /// and for an update that of the node it starts at. `actions` may come in any
    /// order of nodes.
    pub fn new(
        order_id: OrderId,
        update_id: u32,
        first_step: u32,
        start: NodeIdx,
        legs: Vec<Leg>,
        released: u32,
        mut actions: Vec<NodeAction>,
    ) -> Result<Order, ShapeError> {
        // the last node has the highest sequence id
        let last = u32::try_from(legs.len()).map_err(|_| ShapeError::SequenceTooLarge)?;
        if first_step
            .checked_add(last)
            .and_then(|step| step.checked_mul(2))
            .is_none()
        {
            return Err(ShapeError::SequenceTooLarge);
        }
        if released > last {
            return Err(ShapeError::TooManyReleased {
                released,
                there: legs.len(),
            });
        }
        if let Some(a) = actions.iter().find(|a| a.node > last) {
            return Err(ShapeError::ActionBeyondRoute { node: a.node, last });
        }
        // (a stable sort: the order at one node is kept)
        actions.sort_by_key(|a| a.node);
        Ok(Order {
            order_id,
            update_id,
            first_step,
            start,
            legs,
            released,
            actions,
        })
    }

    pub fn order_id(&self) -> &OrderId {
        &self.order_id
    }

    /// `orderUpdateId`.
    pub fn update_id(&self) -> u32 {
        self.update_id
    }

    /// Half the `sequenceId` of the start node.
    pub fn first_step(&self) -> u32 {
        self.first_step
    }

    pub fn start(&self) -> NodeIdx {
        self.start
    }

    pub fn legs(&self) -> &[Leg] {
        &self.legs
    }

    /// How many legs, from the first, are released.
    pub fn released(&self) -> u32 {
        self.released
    }

    /// All actions, sorted by node.
    pub fn actions(&self) -> &[NodeAction] {
        &self.actions
    }

    /// The actions at node `node`, in their order; empty if there are none or there
    /// is no such node.
    pub fn actions_at(&self, node: u32) -> &[NodeAction] {
        let from = self.actions.partition_point(|a| a.node < node);
        let to = self.actions.partition_point(|a| a.node <= node);
        &self.actions[from..to]
    }

    /// The layout node that is node `node` of the order.
    pub fn node(&self, node: u32) -> Option<NodeIdx> {
        match node.checked_sub(1) {
            None => Some(self.start),
            Some(leg) => self.leg(leg).map(|l| l.end),
        }
    }

    /// The `sequenceId` of node `node`; `None` if the order has no such node.
    pub fn node_sequence_id(&self, node: u32) -> Option<u32> {
        // (`new` has checked that the numbers of the order's nodes fit)
        self.node(node).map(|_| 2 * (self.first_step + node))
    }

    /// The `sequenceId` of leg `leg`; `None` if the order has no such leg.
    pub fn leg_sequence_id(&self, leg: u32) -> Option<u32> {
        self.leg(leg).map(|_| 2 * (self.first_step + leg) + 1)
    }

    fn leg(&self, leg: u32) -> Option<&Leg> {
        self.legs.get(usize::try_from(leg).ok()?)
    }
}

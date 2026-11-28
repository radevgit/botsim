//! The state report of a simulated robot ([`botsim_link::state::State`], VDA 5050
//! `state` meaning), built from the driver's [`Progress`].
//!
//! What is reported (VDA 5050 §6.6, 7.x):
//! - `orderId` / `orderUpdateId`: kept after the order is finished ("until a new
//!   order is received"); no order id and 0 if there never was an order.
//! - The last node and its step (half its `sequenceId`): the last node reached, or
//!   the node the robot stands on. A node counts as traversed on arrival, before
//!   its actions. None without an order.
//! - The way ahead: everything still to come, base *and* horizon, with how much of
//!   it is released; the first node of an order is never ahead; a node leaves the
//!   list when reached, and so does the edge leading to it.
//! - `driving`: driving or turning on the spot.
//! - The actions of the order: every action of the base, WAITING until its node is
//!   reached.
//! - Position, velocity.
//!
//! Placeholders (not modelled yet): the battery is a permanently powered robot
//! (charge 100, as the standard prescribes for those), operating mode AUTOMATIC,
//! no emergency stop, no loads.
//! Not in a report: map list, paths, `newBaseRequest`, node positions.

use botsim_link::state::{
    ActionState, Ahead, EmergencyStop, LastNode, OperatingMode, Position, Route, State, Velocity,
};

use crate::drive::Progress;
use crate::errors::RobotError;

/// Build the state report of a robot with the given progress. What depends on the
/// robot beyond its way and its actions (battery, zones) is filled in by the
/// caller.
pub fn build_state(p: &Progress, errors: &[RobotError], instant_actions: &[ActionState]) -> State {
    State {
        order_id: p.order_id.clone(),
        update_id: p.order_update_id,
        route: route(p),
        position: Some(Position {
            map: p.pose.map,
            x: p.pose.x,
            y: p.pose.y,
            theta: p.pose.heading,
            localized: true,
        }),
        velocity: Some(Velocity {
            vx: p.vx,
            vy: p.vy,
            omega: p.omega,
        }),
        driving: p.driving,
        // a robot held by an obstacle or a disturbance reports a field violation
        field_violation: p.held,
        emergency_stop: EmergencyStop::None,
        operating_mode: OperatingMode::Automatic,
        charge: 100.0,
        charging: false,
        actions: p.actions.clone(),
        instant_actions: instant_actions.to_vec(),
        zone_requests: None,
        zone_sets: Vec::new(),
        errors: errors.iter().map(RobotError::to_report).collect(),
    }
}

/// The last node and the way ahead.
fn route(p: &Progress) -> Route {
    let last = p.last_node.map(|n| LastNode {
        node: n.node,
        step: n.sequence_id / 2,
    });
    // entry i: the edge and the node it ends at
    let ahead: Vec<Ahead> = p
        .remaining_edges
        .iter()
        .zip(&p.remaining_nodes)
        .map(|(e, n)| Ahead {
            edge: e.edge,
            node: n.node,
        })
        .collect();
    // (the released ones are the first ones)
    let released = p.remaining_nodes[..ahead.len()]
        .iter()
        .take_while(|n| n.released)
        .count();
    let released = u32::try_from(released).unwrap_or(u32::MAX);
    // (a driver's plan has nothing ahead without a last node, no more released
    // than there is and numbers that fit, so this cannot fail)
    Route::new(last, ahead, released).unwrap_or_default()
}

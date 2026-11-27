//! The VDA 5050 `state` message of a simulated robot, built from the driver's
//! [`Progress`].
//!
//! What is reported (VDA 5050 §6.6, 7.x):
//! - `orderId` / `orderUpdateId`: kept after the order is finished ("until a new
//!   order is received"); empty string and 0 if there never was an order.
//! - `lastNodeId` / `lastNodeSequenceId`: the last node reached, or the node the
//!   robot stands on. A node counts as traversed on arrival, before its actions.
//! - `nodeStates` / `edgeStates`: everything still to come, base *and* horizon,
//!   with the `released` flag; the first node of an order is never in
//!   `nodeStates`; a node leaves the list when reached, and so does the edge
//!   leading to it.
//! - `driving`: driving or turning on the spot.
//! - `actionStates`: every action of the base, WAITING until its node is reached.
//! - `mobileRobotPosition`, `velocity`.
//!
//! Placeholders (not modelled yet): the battery is a permanently powered robot
//! (`stateOfCharge` 100, as the standard prescribes for those), operating mode
//! AUTOMATIC, no emergency stop, no loads.
//! Not reported: map list, zone sets, paths, `newBaseRequest`, node positions.

use vda5050::state::{
    ActionState, ActionStatus, EdgeState, EmergencyStop, NodeState, OperatingMode, PowerSupply,
    SafetyState, State,
};
use vda5050::state::{MobileRobotPosition, Velocity};

use crate::drive::Progress;
use crate::errors::RobotError;
use botsim_model::graph::Graph;

/// The header fields of a state message. The simulator's clock and counters
/// decide these, so the caller supplies them.
#[derive(Debug, Clone, PartialEq)]
pub struct Header {
    pub header_id: u32,
    /// ISO 8601 UTC, e.g. `2026-11-20T10:00:00.000Z`.
    pub timestamp: String,
    pub version: String,
    pub manufacturer: String,
    pub serial_number: String,
}

/// An instant action as reported in `instantActionStates`.
#[derive(Debug, Clone, PartialEq)]
pub struct InstantActionState {
    pub action_id: String,
    pub action_type: String,
    pub status: ActionStatus,
}

/// Build the state message for a robot with the given progress.
pub fn build_state(
    graph: &Graph,
    header: &Header,
    p: &Progress,
    errors: &[RobotError],
    instant_actions: &[InstantActionState],
) -> State {
    let (last_node_id, last_node_sequence_id) = match &p.last_node {
        Some(n) => (n.node_id.clone(), n.sequence_id),
        None => (String::new(), 0),
    };

    State {
        header_id: header.header_id,
        timestamp: header.timestamp.clone(),
        version: header.version.clone(),
        manufacturer: header.manufacturer.clone(),
        serial_number: header.serial_number.clone(),
        maps: None,
        zone_sets: None,
        order_id: p.order_id.clone(),
        order_update_id: p.order_update_id,
        last_node_id,
        last_node_sequence_id,
        node_states: p
            .remaining_nodes
            .iter()
            .map(|n| NodeState {
                node_id: n.node_id.clone(),
                sequence_id: n.sequence_id,
                node_descriptor: None,
                released: n.released,
                node_position: None,
            })
            .collect(),
        edge_states: p
            .remaining_edges
            .iter()
            .map(|e| EdgeState {
                edge_id: e.edge_id.clone(),
                sequence_id: e.sequence_id,
                edge_descriptor: None,
                released: e.released,
                trajectory: None,
            })
            .collect(),
        planned_path: None,
        intermediate_path: None,
        mobile_robot_position: Some(MobileRobotPosition {
            x: p.pose.x,
            y: p.pose.y,
            theta: p.pose.heading,
            map_id: graph.maps[p.pose.map].clone(),
            localized: true,
            localization_score: None,
            deviation_range: None,
        }),
        velocity: Some(Velocity {
            vx: Some(p.vx),
            vy: Some(p.vy),
            omega: Some(p.omega),
        }),
        loads: None,
        driving: p.driving,
        paused: Some(false),
        new_base_request: None,
        zone_requests: None,
        edge_requests: None,
        distance_since_last_node: None,
        action_states: p
            .actions
            .iter()
            .map(|(id, t, s)| ActionState {
                action_id: id.clone(),
                action_type: Some(t.clone()),
                action_descriptor: None,
                action_status: *s,
                action_result: None,
            })
            .collect(),
        instant_action_states: instant_actions
            .iter()
            .map(|a| ActionState {
                action_id: a.action_id.clone(),
                action_type: Some(a.action_type.clone()),
                action_descriptor: None,
                action_status: a.status,
                action_result: None,
            })
            .collect(),
        zone_action_states: None,
        power_supply: PowerSupply {
            state_of_charge: 100.0,
            battery_voltage: None,
            battery_current: None,
            battery_health: None,
            charging: false,
            range: None,
        },
        operating_mode: OperatingMode::Automatic,
        errors: errors.iter().map(RobotError::to_state_error).collect(),
        information: None,
        safety_state: SafetyState {
            active_emergency_stop: EmergencyStop::None,
            // a robot held by an obstacle or a disturbance reports a field violation
            field_violation: p.held,
        },
    }
}

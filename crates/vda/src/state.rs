//! `state`. A bad header or an unusable charge refuses the whole report; any other
//! bad part is left out and reported. VDA 5050 has no error types here: a report
//! is not answered.

use std::sync::Arc;

use botsim_link::ids::{ActionId, ActionType, OrderId, RequestId, ZoneId, ZoneSetId};
use botsim_link::message::FromRobot;
use botsim_link::state as link;
use botsim_model::errors::ErrorType;
use vda5050::state as vda;

use crate::header::{Incoming, angle_to_vda, finite, usable};
use crate::report::{Issue, Problem, Translated};
use crate::translator::Translator;

const ORDER_ID: &str = "orderId";
const CHARGE: &str = "powerSupply.stateOfCharge";
const LAST_NODE_SEQUENCE_ID: &str = "lastNodeSequenceId";
const NODE_SEQUENCE_ID: &str = "nodeStates[].sequenceId";
const EDGE_SEQUENCE_ID: &str = "edgeStates[].sequenceId";
const THETA: &str = "mobileRobotPosition.theta";

/// The charge in percent, as the schema allows it.
fn charge(value: f64) -> Result<f64, Problem> {
    let value = finite(value, CHARGE)?;
    if (0.0..=100.0).contains(&value) {
        Ok(value)
    } else {
        Err(Problem::OutOfRange { field: CHARGE })
    }
}

/// The entries that can be translated; the others are reported with their places.
fn entries<A, B>(
    list: &[A],
    issues: &mut Vec<Issue>,
    translate: impl Fn(&A) -> Result<B, Problem>,
) -> Vec<B> {
    list.iter()
        .enumerate()
        .filter_map(|(i, a)| {
            translate(a)
                .map_err(|problem| issues.push(Issue::new(problem).at(i)))
                .ok()
        })
        .collect()
}

/// A part that is left out if it cannot be translated.
fn part<T>(result: Result<T, Problem>, issues: &mut Vec<Issue>) -> Option<T> {
    result
        .map_err(|problem| issues.push(Issue::new(problem)))
        .ok()
}

impl Translator {
    /// The VDA 5050 `state` message that says what the report says.
    pub fn state_to_vda(&self, message: &FromRobot) -> Result<Translated<vda::State>, Issue> {
        let head = self.outgoing(&message.header).map_err(Issue::new)?;
        let state = &message.state;
        let state_of_charge = charge(state.charge).map_err(Issue::new)?;
        let mut issues = Vec::new();

        let order_id = state
            .order_id
            .as_ref()
            .and_then(|id| part(usable(id.as_str(), ORDER_ID), &mut issues))
            .unwrap_or("")
            .to_string();
        let route = self.route_to_vda(&state.route, &mut issues);
        let position = state
            .position
            .as_ref()
            .and_then(|p| self.position_to_vda(p, &mut issues));
        let velocity = state
            .velocity
            .as_ref()
            .and_then(|v| part(velocity_to_vda(v), &mut issues));
        let action_states = actions_to_vda(&state.actions, ACTIONS, &mut issues);
        let instant_action_states =
            actions_to_vda(&state.instant_actions, INSTANT_ACTIONS, &mut issues);
        let zone_requests = state
            .zone_requests
            .as_ref()
            .map(|list| entries(list, &mut issues, zone_request_to_vda));
        let zone_sets = entries(&state.zone_sets, &mut issues, |z| {
            self.zone_set_info_to_vda(z)
        });
        let errors = entries(&state.errors, &mut issues, error_to_vda);

        Ok(Translated {
            message: vda::State {
                header_id: head.header_id,
                timestamp: head.timestamp,
                version: head.version,
                manufacturer: head.manufacturer,
                serial_number: head.serial_number,
                maps: None,
                zone_sets: (!zone_sets.is_empty()).then_some(zone_sets),
                order_id,
                order_update_id: state.update_id,
                last_node_id: route.last_node_id,
                last_node_sequence_id: route.last_node_sequence_id,
                node_states: route.node_states,
                edge_states: route.edge_states,
                planned_path: None,
                intermediate_path: None,
                mobile_robot_position: position,
                velocity,
                loads: None,
                driving: state.driving,
                paused: None,
                new_base_request: None,
                zone_requests,
                edge_requests: None,
                distance_since_last_node: None,
                action_states,
                instant_action_states,
                zone_action_states: None,
                power_supply: vda::PowerSupply {
                    state_of_charge,
                    battery_voltage: None,
                    battery_current: None,
                    battery_health: None,
                    charging: state.charging,
                    range: None,
                },
                operating_mode: operating_mode_to_vda(state.operating_mode),
                errors,
                information: None,
                safety_state: vda::SafetyState {
                    active_emergency_stop: emergency_stop_to_vda(state.emergency_stop),
                    field_violation: state.field_violation,
                },
            },
            issues,
        })
    }

    /// What the `state` message says, as the fleet server receives it. The issues
    /// name the parts that are not in the result.
    pub fn state_from_vda(&self, message: &vda::State) -> Result<Translated<FromRobot>, Issue> {
        let header = self
            .incoming(Incoming {
                header_id: message.header_id,
                timestamp: &message.timestamp,
                version: &message.version,
                manufacturer: &message.manufacturer,
                serial_number: &message.serial_number,
            })
            .map_err(Issue::new)?;
        let state_of_charge = charge(message.power_supply.state_of_charge).map_err(Issue::new)?;
        let mut issues = Vec::new();

        let route = self
            .route_from_vda(message, &mut issues)
            .map_err(Issue::new)?;
        let position = message
            .mobile_robot_position
            .as_ref()
            .and_then(|p| part(self.position_from_vda(p), &mut issues));
        let velocity = message
            .velocity
            .as_ref()
            .and_then(|v| part(velocity_from_vda(v), &mut issues));
        let actions = actions_from_vda(&message.action_states, ACTIONS, &mut issues);
        let instant_actions =
            actions_from_vda(&message.instant_action_states, INSTANT_ACTIONS, &mut issues);
        let zone_requests = message
            .zone_requests
            .as_ref()
            .map(|list| entries(list, &mut issues, zone_request_from_vda));
        let zone_sets = entries(
            message.zone_sets.as_deref().unwrap_or_default(),
            &mut issues,
            |z| self.zone_set_info_from_vda(z),
        );
        let errors = entries(&message.errors, &mut issues, error_from_vda);

        Ok(Translated {
            message: FromRobot {
                header,
                state: link::State {
                    order_id: (!message.order_id.is_empty())
                        .then(|| OrderId::new(message.order_id.as_str())),
                    update_id: message.order_update_id,
                    route,
                    position,
                    velocity,
                    driving: message.driving,
                    field_violation: message.safety_state.field_violation,
                    emergency_stop: emergency_stop_from_vda(
                        message.safety_state.active_emergency_stop,
                    ),
                    operating_mode: operating_mode_from_vda(message.operating_mode),
                    charge: state_of_charge,
                    charging: message.power_supply.charging,
                    actions,
                    instant_actions,
                    zone_requests,
                    zone_sets,
                    errors,
                },
            },
            issues,
        })
    }
}

/// The route as the fields of a state message.
struct VdaRoute {
    last_node_id: String,
    last_node_sequence_id: u32,
    node_states: Vec<vda::NodeState>,
    edge_states: Vec<vda::EdgeState>,
}

impl Translator {
    fn route_to_vda(&self, route: &link::Route, issues: &mut Vec<Issue>) -> VdaRoute {
        let mut out = VdaRoute {
            last_node_id: String::new(),
            last_node_sequence_id: 0,
            node_states: Vec::new(),
            edge_states: Vec::new(),
        };
        // without the last node nothing can be ahead
        let Some(last) = route.last_node() else {
            return out;
        };
        let Some(name) = part(self.node_name(last.node), issues) else {
            return out;
        };
        out.last_node_id = name.to_string();
        // (`Route::new` has checked that the numbers fit)
        out.last_node_sequence_id = 2 * last.step;
        for (i, ahead) in route.ahead().iter().enumerate() {
            let names = self
                .edge_name(ahead.edge)
                .and_then(|edge| Ok((edge, self.node_name(ahead.node)?)));
            let (edge, node) = match names {
                Ok(names) => names,
                // the way ahead ends before the first step that cannot be named
                Err(problem) => {
                    issues.push(Issue::new(problem).at(i));
                    break;
                }
            };
            let at = i as u32;
            let (Some(edge_sequence_id), Some(node_sequence_id)) =
                (route.leg_sequence_id(at), route.node_sequence_id(at))
            else {
                break; // (cannot happen: the entry is there)
            };
            let released = at < route.released();
            out.edge_states.push(vda::EdgeState {
                edge_id: edge.to_string(),
                sequence_id: edge_sequence_id,
                edge_descriptor: None,
                released,
                trajectory: None,
            });
            out.node_states.push(vda::NodeState {
                node_id: node.to_string(),
                sequence_id: node_sequence_id,
                node_descriptor: None,
                released,
                node_position: None,
            });
        }
        out
    }

    /// The `Err` cannot happen: what is put together here is what `Route` accepts.
    fn route_from_vda(
        &self,
        message: &vda::State,
        issues: &mut Vec<Issue>,
    ) -> Result<link::Route, Problem> {
        let (nodes, edges) = (&message.node_states, &message.edge_states);
        let last = if message.last_node_id.is_empty() {
            if !(nodes.is_empty() && edges.is_empty()) {
                issues.push(Issue::new(Problem::AheadWithoutLastNode));
            }
            None
        } else {
            part(self.last_node_from_vda(message), issues)
        };

        let mut ahead = Vec::new();
        let mut released = 0;
        if let Some(last) = last {
            for i in 0..nodes.len().max(edges.len()) {
                let (Some(node), Some(edge)) = (nodes.get(i), edges.get(i)) else {
                    let problem = Problem::UnpairedRoute {
                        nodes: nodes.len(),
                        edges: edges.len(),
                    };
                    issues.push(Issue::new(problem).at(i));
                    break;
                };
                // the released steps are the first ones
                let fits =
                    node.released == edge.released && !(node.released && (released as usize) < i);
                let step = self
                    .step_from_vda(last.step, i, node, edge)
                    .and_then(|step| {
                        if fits {
                            Ok(step)
                        } else {
                            Err(Problem::ReleaseMismatch {
                                id: node.node_id.clone(),
                            })
                        }
                    });
                match step {
                    Ok(step) => {
                        ahead.push(step);
                        released += u32::from(node.released);
                    }
                    Err(problem) => {
                        issues.push(Issue::new(problem).at(i));
                        break;
                    }
                }
            }
        }
        link::Route::new(last, ahead, released).map_err(Problem::Shape)
    }

    fn last_node_from_vda(&self, message: &vda::State) -> Result<link::LastNode, Problem> {
        let sequence_id = message.last_node_sequence_id;
        // a node's number is even
        if sequence_id % 2 == 1 {
            return Err(Problem::BadSequenceId {
                field: LAST_NODE_SEQUENCE_ID,
                id: message.last_node_id.clone(),
                expected: u64::from(sequence_id - 1),
                found: sequence_id,
            });
        }
        Ok(link::LastNode {
            node: self.node_number(&message.last_node_id)?,
            step: sequence_id / 2,
        })
    }

    /// Step `i` ahead of the last node at step `last`.
    fn step_from_vda(
        &self,
        last: u32,
        i: usize,
        node: &vda::NodeState,
        edge: &vda::EdgeState,
    ) -> Result<link::Ahead, Problem> {
        let edge_sequence_id = 2 * (u64::from(last) + i as u64) + 1;
        for (field, id, expected, found) in [
            (
                EDGE_SEQUENCE_ID,
                &edge.edge_id,
                edge_sequence_id,
                edge.sequence_id,
            ),
            (
                NODE_SEQUENCE_ID,
                &node.node_id,
                edge_sequence_id + 1,
                node.sequence_id,
            ),
        ] {
            if u64::from(found) != expected {
                return Err(Problem::BadSequenceId {
                    field,
                    id: id.clone(),
                    expected,
                    found,
                });
            }
        }
        Ok(link::Ahead {
            edge: self.edge_number(&edge.edge_id)?,
            node: self.node_number(&node.node_id)?,
        })
    }

    fn position_to_vda(
        &self,
        p: &link::Position,
        issues: &mut Vec<Issue>,
    ) -> Option<vda::MobileRobotPosition> {
        let translated = (|| {
            let (theta, changed) = angle_to_vda(p.theta, THETA)?;
            let position = vda::MobileRobotPosition {
                x: finite(p.x, "mobileRobotPosition.x")?,
                y: finite(p.y, "mobileRobotPosition.y")?,
                theta,
                map_id: self.map_name(p.map)?.to_string(),
                localized: p.localized,
                localization_score: None,
                deviation_range: None,
            };
            Ok((position, changed))
        })();
        let (position, changed) = part(translated, issues)?;
        issues.extend(changed.map(Issue::new));
        Some(position)
    }

    fn position_from_vda(&self, p: &vda::MobileRobotPosition) -> Result<link::Position, Problem> {
        Ok(link::Position {
            map: self.map_number(&p.map_id)?,
            x: finite(p.x, "mobileRobotPosition.x")?,
            y: finite(p.y, "mobileRobotPosition.y")?,
            theta: finite(p.theta, THETA)?,
            localized: p.localized,
        })
    }

    fn zone_set_info_to_vda(&self, z: &link::ZoneSetInfo) -> Result<vda::ZoneSetInfo, Problem> {
        Ok(vda::ZoneSetInfo {
            zone_set_id: usable(z.id.as_str(), "zoneSets[].zoneSetId")?.to_string(),
            map_id: self.map_name(z.map)?.to_string(),
            zone_set_status: if z.enabled {
                vda::EnabledStatus::Enabled
            } else {
                vda::EnabledStatus::Disabled
            },
        })
    }

    fn zone_set_info_from_vda(&self, z: &vda::ZoneSetInfo) -> Result<link::ZoneSetInfo, Problem> {
        Ok(link::ZoneSetInfo {
            id: ZoneSetId::new(usable(&z.zone_set_id, "zoneSets[].zoneSetId")?),
            map: self.map_number(&z.map_id)?,
            enabled: z.zone_set_status == vda::EnabledStatus::Enabled,
        })
    }
}

fn velocity_to_vda(v: &link::Velocity) -> Result<vda::Velocity, Problem> {
    Ok(vda::Velocity {
        vx: Some(finite(v.vx, "velocity.vx")?),
        vy: Some(finite(v.vy, "velocity.vy")?),
        omega: Some(finite(v.omega, "velocity.omega")?),
    })
}

/// A part that is not reported counts as 0.
fn velocity_from_vda(v: &vda::Velocity) -> Result<link::Velocity, Problem> {
    Ok(link::Velocity {
        vx: finite(v.vx.unwrap_or(0.0), "velocity.vx")?,
        vy: finite(v.vy.unwrap_or(0.0), "velocity.vy")?,
        omega: finite(v.omega.unwrap_or(0.0), "velocity.omega")?,
    })
}

/// The field names of one of the two lists of action states.
#[derive(Clone, Copy)]
struct ActionFields {
    id: &'static str,
    kind: &'static str,
}

const ACTIONS: ActionFields = ActionFields {
    id: "actionStates[].actionId",
    kind: "actionStates[].actionType",
};

const INSTANT_ACTIONS: ActionFields = ActionFields {
    id: "instantActionStates[].actionId",
    kind: "instantActionStates[].actionType",
};

fn actions_to_vda(
    actions: &[link::ActionState],
    fields: ActionFields,
    issues: &mut Vec<Issue>,
) -> Vec<vda::ActionState> {
    let mut out = Vec::with_capacity(actions.len());
    for (i, a) in actions.iter().enumerate() {
        let id = match usable(a.id.as_str(), fields.id) {
            Ok(id) => id.to_string(),
            Err(problem) => {
                issues.push(Issue::new(problem).at(i));
                continue;
            }
        };
        // the type is for information only: without a usable one the entry stays
        let action_type = a.kind.as_ref().and_then(|kind| {
            usable(kind.as_str(), fields.kind)
                .map_err(|problem| issues.push(Issue::new(problem).at(i)))
                .ok()
                .map(str::to_string)
        });
        out.push(vda::ActionState {
            action_id: id,
            action_type,
            action_descriptor: None,
            action_status: action_status_to_vda(a.status),
            action_result: None,
        });
    }
    out
}

fn actions_from_vda(
    actions: &[vda::ActionState],
    fields: ActionFields,
    issues: &mut Vec<Issue>,
) -> Vec<link::ActionState> {
    let mut out = Vec::with_capacity(actions.len());
    for (i, a) in actions.iter().enumerate() {
        let id = match usable(&a.action_id, fields.id) {
            Ok(id) => ActionId::new(id),
            Err(problem) => {
                issues.push(Issue::new(problem).at(i));
                continue;
            }
        };
        let kind = a.action_type.as_deref().and_then(|kind| {
            usable(kind, fields.kind)
                .map_err(|problem| issues.push(Issue::new(problem).at(i)))
                .ok()
                .map(ActionType::new)
        });
        out.push(link::ActionState {
            id,
            kind,
            status: action_status_from_vda(a.action_status),
        });
    }
    out
}

fn zone_request_to_vda(r: &link::ZoneRequest) -> Result<vda::ZoneRequest, Problem> {
    Ok(vda::ZoneRequest {
        request_id: usable(r.id.as_str(), "zoneRequests[].requestId")?.to_string(),
        request_type: request_type_to_vda(r.kind),
        zone_id: usable(r.zone.as_str(), "zoneRequests[].zoneId")?.to_string(),
        zone_set_id: usable(r.zone_set.as_str(), "zoneRequests[].zoneSetId")?.to_string(),
        request_status: request_status_to_vda(r.status),
        trajectory: None,
    })
}

fn zone_request_from_vda(r: &vda::ZoneRequest) -> Result<link::ZoneRequest, Problem> {
    Ok(link::ZoneRequest {
        id: RequestId::new(usable(&r.request_id, "zoneRequests[].requestId")?),
        kind: request_type_from_vda(r.request_type),
        zone: ZoneId::new(usable(&r.zone_id, "zoneRequests[].zoneId")?),
        zone_set: ZoneSetId::new(usable(&r.zone_set_id, "zoneRequests[].zoneSetId")?),
        status: request_status_from_vda(r.request_status),
    })
}

const ERROR_TYPE: &str = "errors[].errorType";

fn error_to_vda(e: &link::Error) -> Result<vda::StateError, Problem> {
    let error_type = match &e.kind {
        link::ErrorKind::Predefined(t) => t.as_str(),
        link::ErrorKind::Other(text) => {
            let text = usable(text, ERROR_TYPE)?;
            if ErrorType::from_text(text).is_some() {
                return Err(Problem::ReadsBackDifferently { field: ERROR_TYPE });
            }
            text
        }
    };
    let references = e
        .references
        .iter()
        .map(|(key, value)| vda::Reference {
            reference_key: key.to_string(),
            reference_value: value.to_string(),
        })
        .collect::<Vec<_>>();
    Ok(vda::StateError {
        error_type: error_type.to_string(),
        error_references: (!references.is_empty()).then_some(references),
        error_description: e.description.as_deref().map(str::to_string),
        error_description_translations: None,
        error_hint: e.hint.as_deref().map(str::to_string),
        error_hint_translations: None,
        error_level: error_level_to_vda(e.level),
    })
}

fn error_from_vda(e: &vda::StateError) -> Result<link::Error, Problem> {
    let text = usable(&e.error_type, ERROR_TYPE)?;
    Ok(link::Error {
        kind: ErrorType::from_text(text).map_or_else(
            || link::ErrorKind::Other(text.into()),
            link::ErrorKind::Predefined,
        ),
        level: error_level_from_vda(e.error_level),
        references: e
            .error_references
            .iter()
            .flatten()
            .map(|r| {
                (
                    Arc::from(r.reference_key.as_str()),
                    Arc::from(r.reference_value.as_str()),
                )
            })
            .collect(),
        description: e.error_description.as_deref().map(Arc::from),
        hint: e.error_hint.as_deref().map(Arc::from),
    })
}

fn error_level_to_vda(l: link::ErrorLevel) -> vda::ErrorLevel {
    match l {
        link::ErrorLevel::Warning => vda::ErrorLevel::Warning,
        link::ErrorLevel::Urgent => vda::ErrorLevel::Urgent,
        link::ErrorLevel::Critical => vda::ErrorLevel::Critical,
        link::ErrorLevel::Fatal => vda::ErrorLevel::Fatal,
    }
}

fn error_level_from_vda(l: vda::ErrorLevel) -> link::ErrorLevel {
    match l {
        vda::ErrorLevel::Warning => link::ErrorLevel::Warning,
        vda::ErrorLevel::Urgent => link::ErrorLevel::Urgent,
        vda::ErrorLevel::Critical => link::ErrorLevel::Critical,
        vda::ErrorLevel::Fatal => link::ErrorLevel::Fatal,
    }
}

fn emergency_stop_to_vda(e: link::EmergencyStop) -> vda::EmergencyStop {
    match e {
        link::EmergencyStop::Manual => vda::EmergencyStop::Manual,
        link::EmergencyStop::Remote => vda::EmergencyStop::Remote,
        link::EmergencyStop::None => vda::EmergencyStop::None,
    }
}

fn emergency_stop_from_vda(e: vda::EmergencyStop) -> link::EmergencyStop {
    match e {
        vda::EmergencyStop::Manual => link::EmergencyStop::Manual,
        vda::EmergencyStop::Remote => link::EmergencyStop::Remote,
        vda::EmergencyStop::None => link::EmergencyStop::None,
    }
}

fn operating_mode_to_vda(m: link::OperatingMode) -> vda::OperatingMode {
    match m {
        link::OperatingMode::Startup => vda::OperatingMode::Startup,
        link::OperatingMode::Automatic => vda::OperatingMode::Automatic,
        link::OperatingMode::Semiautomatic => vda::OperatingMode::Semiautomatic,
        link::OperatingMode::Intervened => vda::OperatingMode::Intervened,
        link::OperatingMode::Manual => vda::OperatingMode::Manual,
        link::OperatingMode::Service => vda::OperatingMode::Service,
        link::OperatingMode::TeachIn => vda::OperatingMode::TeachIn,
    }
}

fn operating_mode_from_vda(m: vda::OperatingMode) -> link::OperatingMode {
    match m {
        vda::OperatingMode::Startup => link::OperatingMode::Startup,
        vda::OperatingMode::Automatic => link::OperatingMode::Automatic,
        vda::OperatingMode::Semiautomatic => link::OperatingMode::Semiautomatic,
        vda::OperatingMode::Intervened => link::OperatingMode::Intervened,
        vda::OperatingMode::Manual => link::OperatingMode::Manual,
        vda::OperatingMode::Service => link::OperatingMode::Service,
        vda::OperatingMode::TeachIn => link::OperatingMode::TeachIn,
    }
}

fn action_status_to_vda(s: link::ActionStatus) -> vda::ActionStatus {
    match s {
        link::ActionStatus::Waiting => vda::ActionStatus::Waiting,
        link::ActionStatus::Initializing => vda::ActionStatus::Initializing,
        link::ActionStatus::Running => vda::ActionStatus::Running,
        link::ActionStatus::Paused => vda::ActionStatus::Paused,
        link::ActionStatus::Retriable => vda::ActionStatus::Retriable,
        link::ActionStatus::Finished => vda::ActionStatus::Finished,
        link::ActionStatus::Failed => vda::ActionStatus::Failed,
    }
}

fn action_status_from_vda(s: vda::ActionStatus) -> link::ActionStatus {
    match s {
        vda::ActionStatus::Waiting => link::ActionStatus::Waiting,
        vda::ActionStatus::Initializing => link::ActionStatus::Initializing,
        vda::ActionStatus::Running => link::ActionStatus::Running,
        vda::ActionStatus::Paused => link::ActionStatus::Paused,
        vda::ActionStatus::Retriable => link::ActionStatus::Retriable,
        vda::ActionStatus::Finished => link::ActionStatus::Finished,
        vda::ActionStatus::Failed => link::ActionStatus::Failed,
    }
}

fn request_type_to_vda(t: link::ZoneRequestType) -> vda::ZoneRequestType {
    match t {
        link::ZoneRequestType::Access => vda::ZoneRequestType::Access,
        link::ZoneRequestType::Replanning => vda::ZoneRequestType::Replanning,
    }
}

fn request_type_from_vda(t: vda::ZoneRequestType) -> link::ZoneRequestType {
    match t {
        vda::ZoneRequestType::Access => link::ZoneRequestType::Access,
        vda::ZoneRequestType::Replanning => link::ZoneRequestType::Replanning,
    }
}

fn request_status_to_vda(s: link::RequestStatus) -> vda::RequestStatus {
    match s {
        link::RequestStatus::Requested => vda::RequestStatus::Requested,
        link::RequestStatus::Queued => vda::RequestStatus::Queued,
        link::RequestStatus::Granted => vda::RequestStatus::Granted,
        link::RequestStatus::Revoked => vda::RequestStatus::Revoked,
        link::RequestStatus::Expired => vda::RequestStatus::Expired,
    }
}

fn request_status_from_vda(s: vda::RequestStatus) -> link::RequestStatus {
    match s {
        vda::RequestStatus::Requested => link::RequestStatus::Requested,
        vda::RequestStatus::Queued => link::RequestStatus::Queued,
        vda::RequestStatus::Granted => link::RequestStatus::Granted,
        vda::RequestStatus::Revoked => link::RequestStatus::Revoked,
        vda::RequestStatus::Expired => link::RequestStatus::Expired,
    }
}

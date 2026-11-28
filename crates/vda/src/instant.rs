//! `instantActions`. A bad action is left out, the others pass.

use botsim_link::ids::{ActionId, ActionType, OrderId, ZoneSetId};
use botsim_link::instant::{InstantAction, InstantKind};
use botsim_link::message::{Header, ToRobot, ToRobotBody};
use botsim_model::errors::ErrorType;
use vda5050::instant_actions as vda;

use crate::header::{Incoming, usable};
use crate::report::{Issue, Problem, Translated};
use crate::translator::Translator;

const CANCEL_ORDER: &str = "cancelOrder";
const ENABLE_ZONE_SET: &str = "enableZoneSet";
const DELETE_ZONE_SET: &str = "deleteZoneSet";
const START_CHARGING: &str = "startCharging";
const STOP_CHARGING: &str = "stopCharging";

const ORDER_ID: &str = "orderId";
const ZONE_SET_ID: &str = "zoneSetId";

/// Why an action is left out: the problem and the error type that goes with it.
type Refusal = (Problem, ErrorType);

impl Translator {
    pub(crate) fn instant_actions_to_vda(
        &self,
        header: &Header,
        actions: &[InstantAction],
    ) -> Result<Translated<vda::InstantActions>, Issue> {
        let head = self.outgoing(header).map_err(Issue::new)?;
        let mut message = vda::InstantActions {
            header_id: head.header_id,
            timestamp: head.timestamp,
            version: head.version,
            manufacturer: head.manufacturer,
            serial_number: head.serial_number,
            actions: Vec::with_capacity(actions.len()),
        };
        let mut issues = Vec::new();
        for (i, a) in actions.iter().enumerate() {
            match action_to_vda(a) {
                Ok(action) => message.actions.push(action),
                Err(r) => issues.push(refuse(r, a.id.as_str(), i)),
            }
        }
        Ok(Translated { message, issues })
    }

    /// What the `instantActions` message says, as a robot receives it. Actions
    /// that cannot be taken are in the issues.
    pub fn instant_actions_from_vda(
        &self,
        message: &vda::InstantActions,
    ) -> Result<Translated<ToRobot>, Issue> {
        let header = self
            .incoming(Incoming {
                header_id: message.header_id,
                timestamp: &message.timestamp,
                version: &message.version,
                manufacturer: &message.manufacturer,
                serial_number: &message.serial_number,
            })
            .map_err(Issue::new)?;
        let mut actions = Vec::with_capacity(message.actions.len());
        let mut issues = Vec::new();
        for (i, a) in message.actions.iter().enumerate() {
            match action_from_vda(a) {
                Ok(action) => actions.push(action),
                Err(r) => issues.push(refuse(r, &a.action_id, i)),
            }
        }
        Ok(Translated {
            message: ToRobot {
                header,
                body: ToRobotBody::InstantActions(actions),
            },
            issues,
        })
    }
}

fn refuse((problem, error): Refusal, action_id: &str, entry: usize) -> Issue {
    Issue::new(problem)
        .with_error(error, "actionId", action_id)
        .at(entry)
}

fn invalid(problem: Problem) -> Refusal {
    (problem, ErrorType::InvalidInstantAction)
}

/// A `cancelOrder` that names an empty order id: no order has it.
fn no_order() -> Refusal {
    let problem = Problem::UnusableId { field: ORDER_ID };
    (problem, ErrorType::NoOrderToCancel)
}

fn action_to_vda(a: &InstantAction) -> Result<vda::InstantAction, Refusal> {
    let id = usable(a.id.as_str(), "actions[].actionId").map_err(invalid)?;
    let (kind, parameter) = match &a.kind {
        InstantKind::CancelOrder { order_id: None } => (CANCEL_ORDER, None),
        InstantKind::CancelOrder {
            order_id: Some(order),
        } => {
            if order.as_str().is_empty() {
                return Err(no_order());
            }
            (CANCEL_ORDER, Some((ORDER_ID, order.as_str())))
        }
        InstantKind::EnableZoneSet { zone_set } => {
            let set = usable(zone_set.as_str(), ZONE_SET_ID).map_err(invalid)?;
            (ENABLE_ZONE_SET, Some((ZONE_SET_ID, set)))
        }
        InstantKind::DeleteZoneSet { zone_set } => {
            let set = usable(zone_set.as_str(), ZONE_SET_ID).map_err(invalid)?;
            (DELETE_ZONE_SET, Some((ZONE_SET_ID, set)))
        }
        InstantKind::StartCharging => (START_CHARGING, None),
        InstantKind::StopCharging => (STOP_CHARGING, None),
        InstantKind::Other(kind) => {
            let field = "actions[].actionType";
            let kind = usable(kind.as_str(), field).map_err(invalid)?;
            if known(kind) {
                return Err(invalid(Problem::ReadsBackDifferently { field }));
            }
            (kind, None)
        }
    };
    Ok(vda::InstantAction {
        action_type: kind.to_string(),
        action_id: id.to_string(),
        action_descriptor: None,
        blocking_type: vda::InstantBlockingType::None,
        action_parameters: parameter.map(|(key, value)| {
            vec![vda::ActionParameter {
                key: key.to_string(),
                value: serde_json::Value::from(value),
            }]
        }),
        retriable: None,
    })
}

/// Is it one of the action types that have a kind of their own?
fn known(action_type: &str) -> bool {
    [
        CANCEL_ORDER,
        ENABLE_ZONE_SET,
        DELETE_ZONE_SET,
        START_CHARGING,
        STOP_CHARGING,
    ]
    .contains(&action_type)
}

/// The text of the first parameter `key`; `None` if there is none or its value is
/// not a text.
fn parameter<'a>(a: &'a vda::InstantAction, key: &str) -> Option<&'a str> {
    a.action_parameters
        .iter()
        .flatten()
        .find(|p| p.key == key)
        .and_then(|p| p.value.as_str())
}

fn action_from_vda(a: &vda::InstantAction) -> Result<InstantAction, Refusal> {
    let id = usable(&a.action_id, "actions[].actionId").map_err(invalid)?;
    // the zone set that the action names (none, or no text: an unusable id)
    let zone_set = || {
        usable(parameter(a, ZONE_SET_ID).unwrap_or(""), ZONE_SET_ID)
            .map(ZoneSetId::new)
            .map_err(invalid)
    };
    let kind = match a.action_type.as_str() {
        CANCEL_ORDER => InstantKind::CancelOrder {
            order_id: match parameter(a, ORDER_ID) {
                Some("") => return Err(no_order()),
                order => order.map(OrderId::new),
            },
        },
        ENABLE_ZONE_SET => InstantKind::EnableZoneSet {
            zone_set: zone_set()?,
        },
        DELETE_ZONE_SET => InstantKind::DeleteZoneSet {
            zone_set: zone_set()?,
        },
        START_CHARGING => InstantKind::StartCharging,
        STOP_CHARGING => InstantKind::StopCharging,
        other => InstantKind::Other(ActionType::new(
            usable(other, "actions[].actionType").map_err(invalid)?,
        )),
    };
    Ok(InstantAction {
        id: ActionId::new(id),
        kind,
    })
}

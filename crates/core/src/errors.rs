//! Errors a robot reports in its VDA 5050 state message.
//!
//! A [`RobotError`] is one entry of the state's `errors` array. The predefined
//! error types of VDA 5050, with their names and levels, are in
//! [`botsim_model::errors`].
//!
//! Mapping of our own errors (see the `From`-style constructors below) is a
//! judgement where the standard does not name an error for the case; it is
//! documented at each constructor.

use botsim_model::errors::ErrorType;
use vda5050::state::{Reference, StateError};

use crate::drive::DriveError;
use crate::order_route::OrderError;

/// One entry of the state's `errors` array.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotError {
    pub error_type: ErrorType,
    /// Reference key and value, e.g. `("orderId", "o1")`; the standard says which
    /// references each error type must carry.
    pub references: Vec<(String, String)>,
    pub description: Option<String>,
}

fn reference(key: &str, value: impl Into<String>) -> (String, String) {
    (key.to_string(), value.into())
}

impl RobotError {
    /// An error about a rejected order, with the references the standard names:
    /// `orderId`, and also `orderUpdateId` for the types that carry both.
    pub fn for_order(
        error_type: ErrorType,
        order_id: &str,
        order_update_id: u32,
        description: String,
    ) -> RobotError {
        let mut references = vec![reference("orderId", order_id)];
        if error_type.references_order_update_id() {
            references.push(reference("orderUpdateId", order_update_id.to_string()));
        }
        RobotError {
            error_type,
            references,
            description: Some(description),
        }
    }

    /// An order that does not pass the checks of [`crate::order_route::route_from_order`].
    /// Structure and release rules → `VALIDATION_FAILURE`; ids that are not in the
    /// layout, or an edge that does not connect the order's nodes →
    /// `NO_ROUTE_TO_TARGET` ("at least one unreachable node").
    pub fn from_order_error(e: &OrderError, order_id: &str, order_update_id: u32) -> RobotError {
        let error_type = match e {
            OrderError::Unknown { .. } | OrderError::EdgeDoesNotConnect { .. } => {
                ErrorType::NoRouteToTarget
            }
            _ => ErrorType::ValidationFailure,
        };
        RobotError::for_order(error_type, order_id, order_update_id, e.to_string())
    }

    /// An order the driver cannot execute. An orientation the robot cannot take →
    /// `ORIENTATION_UNREACHABLE` (reference `edgeId`); an update that does not start
    /// at the decision point → `VALIDATION_FAILURE` (the standard names no error
    /// for that case); anything else (unconnected, zero-length or map-changing
    /// edge, unsupported kinematics) → `NO_ROUTE_TO_TARGET` as the nearest
    /// predefined type.
    pub fn from_drive_error(e: &DriveError, order_id: &str, order_update_id: u32) -> RobotError {
        match e {
            DriveError::Orientation { edge, .. } => RobotError {
                error_type: ErrorType::OrientationUnreachable,
                references: vec![reference("edgeId", edge.clone())],
                description: Some(e.to_string()),
            },
            DriveError::NotContinuation { .. } => RobotError::for_order(
                ErrorType::ValidationFailure,
                order_id,
                order_update_id,
                e.to_string(),
            ),
            _ => RobotError::for_order(
                ErrorType::NoRouteToTarget,
                order_id,
                order_update_id,
                e.to_string(),
            ),
        }
    }

    /// The state message entry.
    pub fn to_state_error(&self) -> StateError {
        StateError {
            error_type: self.error_type.as_str().to_string(),
            error_references: if self.references.is_empty() {
                None
            } else {
                Some(
                    self.references
                        .iter()
                        .map(|(k, v)| Reference {
                            reference_key: k.clone(),
                            reference_value: v.clone(),
                        })
                        .collect(),
                )
            },
            error_description: self.description.clone(),
            error_description_translations: None,
            error_hint: None,
            error_hint_translations: None,
            error_level: self.error_type.level(),
        }
    }
}

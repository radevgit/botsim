//! Errors a robot reports in its VDA 5050 state message.
//!
//! [`ErrorType`] lists the predefined error types of VDA 5050 (Table 9) as an
//! enum, with the error level the standard prescribes for each. The wire format
//! is a string (the standard calls it an "extensible enum"); [`ErrorType::as_str`]
//! gives it. A [`RobotError`] is one entry of the state's `errors` array.
//!
//! Mapping of our own errors (see the `From`-style constructors below) is a
//! judgement where the standard does not name an error for the case; it is
//! documented at each constructor.

use vda5050::state::{ErrorLevel, Reference, StateError};

use crate::drive::DriveError;
use crate::order_route::OrderError;

/// Predefined VDA 5050 error types (Table 9).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ErrorType {
    UnsupportedParameter,
    NoOrderToCancel,
    ValidationFailure,
    InvalidOrderAction,
    InvalidInstantAction,
    OutdatedOrderUpdate,
    SameOrderUpdateId,
    OrderUpdateFollowingCancel,
    OutsideOfCorridor,
    InsufficientMemory,
    DuplicateMap,
    BlockedZoneViolation,
    DuplicateZoneSet,
    ZoneReleaseLossHandling,
    ZoneReleaseLost,
    CorridorReleaseLossHandling,
    CorridorReleaseLost,
    ZoneActionConflict,
    NodeUnreachable,
    LocalizationError,
    NoRouteToTarget,
    OtherOrderActive,
    StartNodeOutOfRange,
    MobileRobotNotAvailable,
    UnknownMapId,
    OrientationUnreachable,
    InstantActionStatesFull,
    ZoneActionStatesFull,
}

impl ErrorType {
    /// All predefined types.
    pub const ALL: [ErrorType; 28] = [
        ErrorType::UnsupportedParameter,
        ErrorType::NoOrderToCancel,
        ErrorType::ValidationFailure,
        ErrorType::InvalidOrderAction,
        ErrorType::InvalidInstantAction,
        ErrorType::OutdatedOrderUpdate,
        ErrorType::SameOrderUpdateId,
        ErrorType::OrderUpdateFollowingCancel,
        ErrorType::OutsideOfCorridor,
        ErrorType::InsufficientMemory,
        ErrorType::DuplicateMap,
        ErrorType::BlockedZoneViolation,
        ErrorType::DuplicateZoneSet,
        ErrorType::ZoneReleaseLossHandling,
        ErrorType::ZoneReleaseLost,
        ErrorType::CorridorReleaseLossHandling,
        ErrorType::CorridorReleaseLost,
        ErrorType::ZoneActionConflict,
        ErrorType::NodeUnreachable,
        ErrorType::LocalizationError,
        ErrorType::NoRouteToTarget,
        ErrorType::OtherOrderActive,
        ErrorType::StartNodeOutOfRange,
        ErrorType::MobileRobotNotAvailable,
        ErrorType::UnknownMapId,
        ErrorType::OrientationUnreachable,
        ErrorType::InstantActionStatesFull,
        ErrorType::ZoneActionStatesFull,
    ];

    /// The `errorType` string of the message.
    pub fn as_str(self) -> &'static str {
        match self {
            ErrorType::UnsupportedParameter => "UNSUPPORTED_PARAMETER",
            ErrorType::NoOrderToCancel => "NO_ORDER_TO_CANCEL",
            ErrorType::ValidationFailure => "VALIDATION_FAILURE",
            ErrorType::InvalidOrderAction => "INVALID_ORDER_ACTION",
            ErrorType::InvalidInstantAction => "INVALID_INSTANT_ACTION",
            ErrorType::OutdatedOrderUpdate => "OUTDATED_ORDER_UPDATE",
            ErrorType::SameOrderUpdateId => "SAME_ORDER_UPDATE_ID",
            ErrorType::OrderUpdateFollowingCancel => "ORDER_UPDATE_FOLLOWING_CANCEL",
            ErrorType::OutsideOfCorridor => "OUTSIDE_OF_CORRIDOR",
            ErrorType::InsufficientMemory => "INSUFFICIENT_MEMORY",
            ErrorType::DuplicateMap => "DUPLICATE_MAP",
            ErrorType::BlockedZoneViolation => "BLOCKED_ZONE_VIOLATION",
            ErrorType::DuplicateZoneSet => "DUPLICATE_ZONE_SET",
            ErrorType::ZoneReleaseLossHandling => "ZONE_RELEASE_LOSS_HANDLING",
            ErrorType::ZoneReleaseLost => "ZONE_RELEASE_LOST",
            ErrorType::CorridorReleaseLossHandling => "CORRIDOR_RELEASE_LOSS_HANDLING",
            ErrorType::CorridorReleaseLost => "CORRIDOR_RELEASE_LOST",
            ErrorType::ZoneActionConflict => "ZONE_ACTION_CONFLICT",
            ErrorType::NodeUnreachable => "NODE_UNREACHABLE",
            ErrorType::LocalizationError => "LOCALIZATION_ERROR",
            ErrorType::NoRouteToTarget => "NO_ROUTE_TO_TARGET",
            ErrorType::OtherOrderActive => "OTHER_ORDER_ACTIVE",
            ErrorType::StartNodeOutOfRange => "START_NODE_OUT_OF_RANGE",
            ErrorType::MobileRobotNotAvailable => "MOBILE_ROBOT_NOT_AVAILABLE",
            ErrorType::UnknownMapId => "UNKNOWN_MAP_ID",
            ErrorType::OrientationUnreachable => "ORIENTATION_UNREACHABLE",
            ErrorType::InstantActionStatesFull => "INSTANT_ACTION_STATES_FULL",
            ErrorType::ZoneActionStatesFull => "ZONE_ACTION_STATES_FULL",
        }
    }

    /// The error level the standard prescribes.
    pub fn level(self) -> ErrorLevel {
        use ErrorType::*;
        match self {
            UnsupportedParameter | OutsideOfCorridor | BlockedZoneViolation | ZoneReleaseLost
            | CorridorReleaseLost | ZoneActionConflict | NodeUnreachable => ErrorLevel::Critical,
            InsufficientMemory | InstantActionStatesFull | ZoneActionStatesFull => {
                ErrorLevel::Urgent
            }
            LocalizationError => ErrorLevel::Fatal,
            NoOrderToCancel
            | ValidationFailure
            | InvalidOrderAction
            | InvalidInstantAction
            | OutdatedOrderUpdate
            | SameOrderUpdateId
            | OrderUpdateFollowingCancel
            | DuplicateMap
            | DuplicateZoneSet
            | ZoneReleaseLossHandling
            | CorridorReleaseLossHandling
            | NoRouteToTarget
            | OtherOrderActive
            | StartNodeOutOfRange
            | MobileRobotNotAvailable
            | UnknownMapId
            | OrientationUnreachable => ErrorLevel::Warning,
        }
    }
}

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
    /// An error with the references the standard names for rejected orders.
    fn for_order(
        error_type: ErrorType,
        order_id: &str,
        order_update_id: u32,
        description: String,
    ) -> RobotError {
        // Table 9: VALIDATION_FAILURE, OUTDATED_ORDER_UPDATE etc. reference both ids;
        // NO_ROUTE_TO_TARGET, OTHER_ORDER_ACTIVE, ... only `orderId`.
        let mut references = vec![reference("orderId", order_id)];
        if matches!(
            error_type,
            ErrorType::ValidationFailure
                | ErrorType::OutdatedOrderUpdate
                | ErrorType::SameOrderUpdateId
                | ErrorType::OrderUpdateFollowingCancel
                | ErrorType::InsufficientMemory
        ) {
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

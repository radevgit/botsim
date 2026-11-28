//! The predefined error types of VDA 5050 (Table 9), with their names in messages
//! and the error level the standard prescribes for each.

/// VDA 5050 `errorLevel`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorLevel {
    Warning,
    Urgent,
    Critical,
    Fatal,
}

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

    /// The predefined type with this `errorType` string; `None` if there is none
    /// (the standard allows other types).
    pub fn from_text(text: &str) -> Option<ErrorType> {
        ErrorType::ALL.into_iter().find(|t| t.as_str() == text)
    }

    /// Does an error of this type about an order reference `orderUpdateId` beside
    /// `orderId`? (Table 9: `VALIDATION_FAILURE`, `OUTDATED_ORDER_UPDATE` etc.
    /// reference both; `NO_ROUTE_TO_TARGET`, `OTHER_ORDER_ACTIVE`, ... only
    /// `orderId`.)
    pub fn references_order_update_id(self) -> bool {
        matches!(
            self,
            ErrorType::ValidationFailure
                | ErrorType::OutdatedOrderUpdate
                | ErrorType::SameOrderUpdateId
                | ErrorType::OrderUpdateFollowingCancel
                | ErrorType::InsufficientMemory
        )
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

//! Instant actions: what a robot is to do at once.

use crate::ids::{ActionId, ActionType, OrderId, ZoneSetId};

/// One instant action. Its VDA 5050 `blockingType` is always `NONE`, the only one
/// the standard allows here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InstantAction {
    pub id: ActionId,
    pub kind: InstantKind,
}

/// The instant actions botsim knows, with their parameters.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InstantKind {
    /// `cancelOrder`, with the parameter `orderId` if it is given.
    CancelOrder { order_id: Option<OrderId> },
    /// `enableZoneSet`.
    EnableZoneSet { zone_set: ZoneSetId },
    /// `deleteZoneSet`.
    DeleteZoneSet { zone_set: ZoneSetId },
    /// `startCharging`.
    StartCharging,
    /// `stopCharging`.
    StopCharging,
    /// Any other action type; its parameters are not carried.
    Other(ActionType),
}

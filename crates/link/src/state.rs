//! The state report of a robot: its own account of where it is, what it still has
//! ahead and how it is doing (VDA 5050 `state`).

use std::sync::Arc;

use botsim_model::errors::ErrorType;
use botsim_model::graph::{EdgeIdx, MapIdx, NodeIdx};

use crate::ids::{ActionId, ActionType, OrderId, RequestId, ZoneId, ZoneSetId};
use crate::shape::ShapeError;

#[derive(Debug, Clone, PartialEq)]
pub struct State {
    /// The order the robot executes or executed last; `None`: it never had one
    /// (an empty `orderId`).
    pub order_id: Option<OrderId>,
    /// `orderUpdateId`; 0 without an order.
    pub update_id: u32,
    /// The last node, and the nodes and edges still ahead.
    pub route: Route,
    /// `None`: the robot reports no position that can be used.
    pub position: Option<Position>,
    pub velocity: Option<Velocity>,
    pub driving: bool,
    /// A protective field is violated.
    pub field_violation: bool,
    pub emergency_stop: EmergencyStop,
    pub operating_mode: OperatingMode,
    /// The charge of the battery in percent.
    pub charge: f64,
    pub charging: bool,
    /// The actions of the order.
    pub actions: Vec<ActionState>,
    /// All instant actions the robot reports.
    pub instant_actions: Vec<ActionState>,
    /// `None`: the robot states no `zoneRequests`; empty: it states that it asks
    /// for nothing.
    pub zone_requests: Option<Vec<ZoneRequest>>,
    /// The zone sets stored on the robot.
    pub zone_sets: Vec<ZoneSetInfo>,
    pub errors: Vec<Error>,
}

/// Where a robot is on its order: the last node it reached and what is still ahead.
///
/// Entry `i` of [`ahead`](Route::ahead) is an edge and the node it ends at. The
/// VDA 5050 `sequenceId`s are not stored: with the last node at step `s`, the node
/// of entry `i` has `2 · (s + 1 + i)` and its edge the number before. The first
/// [`released`](Route::released) entries are released.
///
/// [`Route::new`] promises:
/// - there is nothing ahead without a last node;
/// - no more entries are released than there are;
/// - every `sequenceId` fits in 32 bits.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Route {
    last_node: Option<LastNode>,
    ahead: Vec<Ahead>,
    released: u32,
}

/// The last node reached, or the node the robot stands on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LastNode {
    pub node: NodeIdx,
    /// Half its `sequenceId`.
    pub step: u32,
}

/// An edge still ahead and the node it ends at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ahead {
    pub edge: EdgeIdx,
    pub node: NodeIdx,
}

impl Route {
    pub fn new(
        last_node: Option<LastNode>,
        ahead: Vec<Ahead>,
        released: u32,
    ) -> Result<Route, ShapeError> {
        if last_node.is_none() && !ahead.is_empty() {
            return Err(ShapeError::AheadWithoutLastNode);
        }
        let entries = u32::try_from(ahead.len()).map_err(|_| ShapeError::SequenceTooLarge)?;
        // the node of the last entry has the highest sequence id
        if let Some(last) = last_node
            && last
                .step
                .checked_add(entries)
                .and_then(|step| step.checked_mul(2))
                .is_none()
        {
            return Err(ShapeError::SequenceTooLarge);
        }
        if released > entries {
            return Err(ShapeError::TooManyReleased {
                released,
                there: ahead.len(),
            });
        }
        Ok(Route {
            last_node,
            ahead,
            released,
        })
    }

    /// `None`: the robot reports no last node.
    pub fn last_node(&self) -> Option<LastNode> {
        self.last_node
    }

    /// Everything still ahead, released or not, nearest first.
    pub fn ahead(&self) -> &[Ahead] {
        &self.ahead
    }

    /// How many entries of [`ahead`](Route::ahead), from the first, are released.
    pub fn released(&self) -> u32 {
        self.released
    }

    /// The `sequenceId` of the node of entry `entry`; `None` if there is no such
    /// entry.
    pub fn node_sequence_id(&self, entry: u32) -> Option<u32> {
        // (the leg's number is the one before, and `new` has checked that it fits)
        self.leg_sequence_id(entry).map(|n| n + 1)
    }

    /// The `sequenceId` of the edge of entry `entry`; `None` if there is no such
    /// entry.
    pub fn leg_sequence_id(&self, entry: u32) -> Option<u32> {
        self.ahead.get(usize::try_from(entry).ok()?)?;
        self.last_node.map(|last| 2 * (last.step + entry) + 1)
    }
}

/// VDA 5050 `mobileRobotPosition`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position {
    pub map: MapIdx,
    /// Metres.
    pub x: f64,
    pub y: f64,
    /// Heading, rad.
    pub theta: f64,
    /// The position can be trusted.
    pub localized: bool,
}

/// Velocity in the robot's frame. A part a robot does not report counts as 0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Velocity {
    /// Forwards, m/s.
    pub vx: f64,
    /// To the left, m/s.
    pub vy: f64,
    /// Turning, rad/s, counter-clockwise positive.
    pub omega: f64,
}

/// VDA 5050 `activeEmergencyStop`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EmergencyStop {
    Manual,
    Remote,
    None,
}

/// VDA 5050 `operatingMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperatingMode {
    Startup,
    Automatic,
    Semiautomatic,
    Intervened,
    Manual,
    Service,
    TeachIn,
}

/// An action and how far it is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ActionState {
    pub id: ActionId,
    /// `None`: the robot does not say.
    pub kind: Option<ActionType>,
    pub status: ActionStatus,
}

/// VDA 5050 `actionStatus`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActionStatus {
    Waiting,
    Initializing,
    Running,
    Paused,
    Retriable,
    Finished,
    Failed,
}

/// A request of the robot about a zone.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ZoneRequest {
    pub id: RequestId,
    pub kind: ZoneRequestType,
    pub zone: ZoneId,
    pub zone_set: ZoneSetId,
    pub status: RequestStatus,
}

/// VDA 5050 `requestType` of a zone request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZoneRequestType {
    Access,
    Replanning,
}

/// VDA 5050 `requestStatus`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RequestStatus {
    Requested,
    Queued,
    Granted,
    Revoked,
    Expired,
}

/// A zone set stored on the robot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ZoneSetInfo {
    pub id: ZoneSetId,
    pub map: MapIdx,
    pub enabled: bool,
}

/// An error the robot reports.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error {
    pub kind: ErrorKind,
    pub level: ErrorLevel,
    /// `errorReferences` as key and value, for example `("orderId", "job-5")`.
    pub references: Vec<(Arc<str>, Arc<str>)>,
    pub description: Option<Arc<str>>,
    pub hint: Option<Arc<str>>,
}

/// VDA 5050 `errorType`: a predefined type, or any other by its text.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum ErrorKind {
    Predefined(ErrorType),
    /// A type that is not predefined, for example `BATTERY_EMPTY`.
    Other(Arc<str>),
}

/// VDA 5050 `errorLevel`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorLevel {
    Warning,
    Urgent,
    Critical,
    Fatal,
}

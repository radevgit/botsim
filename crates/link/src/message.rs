//! The messages to and from robots, and what every one of them carries.

use std::sync::Arc;

use botsim_model::robot::RobotNo;

use crate::instant::InstantAction;
use crate::order::Order;
use crate::response::Response;
use crate::state::State;
use crate::zone_set::ZoneSet;

/// A time in whole milliseconds since 1970-01-01T00:00:00Z.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Millis(pub i64);

/// What every message to or from a robot carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    /// The robot the message goes to or comes from.
    pub robot: RobotNo,
    /// Counts for each robot and each kind of message on its own, as the VDA 5050
    /// `headerId` does.
    pub seq: u32,
    /// When the message was made.
    pub time: Millis,
}

/// A message to a robot.
#[derive(Debug, Clone, PartialEq)]
pub struct ToRobot {
    pub header: Header,
    pub body: ToRobotBody,
}

/// What a message to a robot says.
#[derive(Debug, Clone, PartialEq)]
pub enum ToRobotBody {
    /// An order or an order update.
    Order(Order),
    InstantActions(Vec<InstantAction>),
    /// Answers to requests of the robot.
    Responses(Vec<Response>),
    /// The whole zone set. Messages to several robots share it.
    ZoneSet(Arc<ZoneSet>),
}

/// A message from a robot: its state report.
#[derive(Debug, Clone, PartialEq)]
pub struct FromRobot {
    pub header: Header,
    pub state: State,
}

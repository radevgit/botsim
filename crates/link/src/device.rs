//! Doors and lifts: the commands to them and the states they report.
//!
//! They are not part of VDA 5050. A message names the device, not a robot.

use botsim_model::graph::MapIdx;
use botsim_model::robot::RobotNo;

use crate::message::Millis;

/// The number of a door: its place in the site's list of doors, from 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct DoorNo(pub u32);

/// The number of a lift: its place in the site's list of lifts, from 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct LiftNo(pub u32);

/// What every message to or from a door or a lift carries; `D` is [`DoorNo`] or
/// [`LiftNo`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DeviceHeader<D> {
    /// The device the message goes to or comes from.
    pub device: D,
    /// Counts for each device and each kind of message on its own.
    pub seq: u32,
    /// When the message was made.
    pub time: Millis,
}

/// A command to a door.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ToDoor {
    pub header: DeviceHeader<DoorNo>,
    pub command: DoorCommand,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DoorCommand {
    /// The server wants the door open.
    Open,
    /// The server is done with the door; it may close.
    Release,
}

/// The state of a door.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FromDoor {
    pub header: DeviceHeader<DoorNo>,
    pub door: DoorPhase,
}

/// How far a door is open.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DoorPhase {
    Closed,
    Opening,
    Open,
    Closing,
}

/// A command to a lift.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ToLift {
    pub header: DeviceHeader<LiftNo>,
    pub command: LiftCommand,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LiftCommand {
    /// Bring the cabin to the floor of map `map` and open the cabin door
    /// `car_door` (its place in the lift's list of cabin doors, from 0).
    Call { map: MapIdx, car_door: u32 },
    /// The server is done with the open door; it may close.
    Release,
}

/// The state of a lift.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FromLift {
    pub header: DeviceHeader<LiftNo>,
    /// The map of the floor the cabin is at, or of the one it left while it moves.
    pub map: MapIdx,
    /// The cabin door that is open, opening or closing, or was used last.
    pub car_door: u32,
    /// How far that door is open.
    pub door: DoorPhase,
    /// The robot in the cabin, if the lift's side knows it.
    ///
    /// A real lift cannot know which robot is in it; the simulator does, and the
    /// controller of botsim 0.1 reads it from there. The server is to derive it
    /// from what the robots report (implementation plan, step C4).
    pub occupant: Option<RobotNo>,
}

//! What the controller is given and what it gives back. It holds no robot, lift
//! or door: the caller tells it the site once ([`Config`]), then runs it in
//! cycles with what has arrived ([`Input`]) and sends what comes out
//! ([`Output`]). What passes in the cycles is [`botsim_link`].
//!
//! **Numbers of robots, lifts and doors.** [`Config`] lists them in a fixed order,
//! and they are named by their places in it: robot `i` is [`robot_no`]`(i)`, the
//! `i`-th door is `DoorNo(i)`, the `i`-th lift `LiftNo(i)`. The controller also
//! breaks ties between robots by this order (the one listed first wins).
//!
//! **Sequence numbers.** Every message sent carries one that counts from 0 for each
//! robot and kind of message (order, instant actions, responses, zone set) on its
//! own, and for each door and each lift.

use std::sync::Arc;

use botsim_link::device::{FromLift, ToDoor, ToLift};
use botsim_link::message::{FromRobot, Millis, ToRobot};
use botsim_link::zone_set::ZoneSet;
use botsim_model::graph::Graph;
use botsim_model::reservation::Reservation;
use botsim_model::robot::{RobotId, RobotNo};
use botsim_model::spec::{Kinematics, RobotSpec, Shape, SpecError};
use botsim_model::waiting::HeldBack;
use botsim_model::zones::Zones;
use vda5050::factsheet::Factsheet;

use crate::doors::Door;
use crate::lifts::Lift;

/// The number of the robot at place `index` of [`Config::robots`]: that slot, in
/// its first generation (no robot joins or leaves yet).
pub fn robot_no(index: usize) -> RobotNo {
    // (a fleet of more robots than 32 bits can number does not fit in memory)
    RobotNo::new(u32::try_from(index).unwrap_or(u32::MAX), 0)
}

/// The site and the fleet: what stays the same during a run.
///
/// A lift stop that names a node or a door of the lift that is not there is left
/// out, and so is a zone set for a map the layout does not have.
#[derive(Debug, Clone)]
pub struct Config {
    /// The layout.
    pub graph: Graph,
    pub robots: Vec<FleetRobot>,
    /// The site's zones, if it has any: where they are, for the zone rules.
    pub zones: Option<Zones>,
    /// The message with the same zones, which the controller sends to every robot.
    /// With [`Config::zones`] but without this, no robot is ready for a task: it
    /// would drive into zones that it does not know. Not looked at without
    /// [`Config::zones`].
    pub zone_set: Option<Arc<ZoneSet>>,
    pub lifts: Vec<Lift>,
    pub doors: Vec<Door>,
    /// The seconds between two ticks of [`Time::tick`].
    pub step_seconds: f64,
    /// The tick the controller starts at: the first cycle counts the time from
    /// it.
    pub start_tick: u64,
}

/// One robot of the fleet, as far as it does not change.
#[derive(Debug, Clone, PartialEq)]
pub struct FleetRobot {
    pub id: RobotId,
    /// The series name of its type (the manufacturer is in `id`).
    pub series: String,
    pub kinematics: Kinematics,
    /// The body outline.
    pub shape: Shape,
}

impl FleetRobot {
    /// The robot as its `factsheet` message describes it (S61).
    pub fn from_factsheet(factsheet: &Factsheet) -> Result<FleetRobot, SpecError> {
        let spec = RobotSpec::from_factsheet(factsheet)?;
        Ok(FleetRobot {
            id: RobotId::new(&factsheet.manufacturer, &factsheet.serial_number),
            series: spec.series_name,
            kinematics: spec.kinematics,
            shape: spec.shape,
        })
    }
}

/// When a cycle runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Time {
    /// Counts the steps of [`Config::step_seconds`]; it does not go back.
    pub tick: u64,
    /// Milliseconds since 1970-01-01T00:00:00Z, for the times of the messages.
    pub unix_ms: i64,
}

impl Time {
    /// The time as the messages carry it.
    pub(crate) fn millis(self) -> Millis {
        Millis(self.unix_ms)
    }
}

/// What has arrived for one cycle.
#[derive(Debug, Clone, Copy)]
pub struct Input<'a> {
    pub time: Time,
    /// Every robot's state report, one for each of [`Config::robots`] and in that
    /// order: report `i` is from robot [`robot_no`]`(i)`. Otherwise the cycle does
    /// nothing but take the new jobs and the time.
    pub states: &'a [FromRobot],
    /// The states of the lifts, in any order; each names its lift. Of several of
    /// one lift the last counts. A lift without one has no robot in its cabin as
    /// far as the controller knows; one of a lift the site does not have is not
    /// looked at.
    pub lifts: &'a [FromLift],
    /// Somebody watches the run: the reservations are wanted.
    pub watched: bool,
}

/// One message to send.
#[derive(Debug, Clone, PartialEq)]
pub enum Message {
    /// To the robot its header names.
    Robot(ToRobot),
    /// To the door its header names.
    Door(ToDoor),
    /// To the lift its header names.
    Lift(ToLift),
}

/// What a cycle gives back.
#[derive(Debug, Clone, Default)]
pub struct Output {
    /// To be sent in this order.
    pub messages: Vec<Message>,
    /// For each robot, why it is held back and by whom; `None` if the cycle did
    /// nothing.
    pub held_back: Option<Vec<Option<HeldBack>>>,
    /// What is reserved; only if [`Input::watched`] and the cycle ran.
    pub reservations: Option<Vec<Reservation>>,
}

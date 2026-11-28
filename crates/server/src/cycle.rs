//! What the controller is given and what it gives back. It holds no robot, lift
//! or door and reads no clock: the caller tells it the site once ([`Config`]),
//! then runs it in cycles with the time and what has arrived since the cycle
//! before ([`Inbox`]), and sends what comes out ([`Outbox`]). What passes in the
//! cycles is [`botsim_link`]. Cycles may come at any interval.
//!
//! **Numbers of robots, lifts and doors.** [`Config`] lists them in a fixed order,
//! and they are named by their places in it: robot `i` is [`robot_no`]`(i)`, the
//! `i`-th door is `DoorNo(i)`, the `i`-th lift `LiftNo(i)`. The controller also
//! breaks ties between robots by this order (the one listed first wins).
//!
//! **Sequence numbers.** Every message sent carries one that counts from 0 for each
//! robot and kind of message (order, instant actions, responses, zone set) on its
//! own, and for each door and each lift.
//!
//! # Reports
//!
//! An inbox holds the reports that have arrived, of any robots, in any order and
//! number. The controller keeps the **newest** report of each robot: the one with
//! the latest [`Header::time`](botsim_link::message::Header::time), of equal times
//! the one with the highest [`seq`](botsim_link::message::Header::seq), and of
//! equal headers the one later in the list.
//! - A robot has a **new report** in a cycle if the inbox holds one that is not
//!   older than the one kept. One with the very same header counts: it has
//!   arrived.
//! - An older one is not looked at. So **a robot whose clock goes back is held
//!   back** until its time has passed that of the kept report.
//! - A report of a robot the fleet does not have (by slot or generation) is not
//!   looked at.
//!
//! Of a lift, the newest state in the inbox counts, in the same sense; none is
//! kept for later cycles.
//!
//! # Before every robot has reported
//!
//! Until each robot of [`Config::robots`] has reported at least once, **nothing
//! new goes to anyone**: no task, release, order, action, answer or command, as
//! the server cannot know where a robot it has not heard stands. The cycle takes
//! the reports and the new jobs; the traffic rules and the dispatcher are not asked
//! (the dispatcher gets the jobs). [`Outbox::held_back`] says nothing of any robot
//! and the reservations are those of the rules before their first cycle.
//!
//! # A robot without a new report
//!
//! After that, a robot without a new report holds back only itself: it stands
//! still when in doubt, and what it holds stays held.
//! - Nothing is sent to it: no order, instant action, zone set or answer.
//! - Its report is not looked at again for refusals, an arrival, a release, a new
//!   route or waiting time.
//! - The traffic rules are told which robots have a new report ([`Known::new`]) and
//!   give nothing to the others and take nothing from them (see
//!   [`crate::policy`]).
//! - The dispatcher sees it as not ready, one with a task as busy, on the node of
//!   the kept report.
//! - A lift or door it owns stays its own; the lift is not sent on or let go, and
//!   neither is the door.
//! - Why it is held back, and the route load average, go by the kept report.
//!
//! # Time
//!
//! The time of a cycle is in whole milliseconds since 1970. The controller's
//! **clock** is the latest time it has been given: a time that does not advance, or
//! goes back, counts no time; a jump counts in full. Messages carry the time as
//! given. A robot's waiting, and what the rules and the dispatcher add up for it,
//! counts the time since the cycle that took its report before (its first report:
//! since the cycle before). A wait is seen only in cycles, each of which counts
//! all of that time.

use std::sync::Arc;

use botsim_link::device::{FromLift, ToDoor, ToLift};
use botsim_link::message::{FromRobot, Millis, ToRobot};
use botsim_link::zone_set::ZoneSet;
use botsim_model::graph::Graph;
use botsim_model::job::Job;
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
    /// The time the controller starts at: the first cycle counts the time from
    /// it.
    pub start: Millis,
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

/// What has arrived since the cycle before.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Inbox {
    /// State reports of robots, in any order, none or several of a robot (see the
    /// module text).
    pub robots: Vec<FromRobot>,
    /// States of lifts, in any order; each names its lift. A lift without one has
    /// no robot in its cabin as far as the controller knows in this cycle; one of
    /// a lift the site does not have is not looked at.
    pub lifts: Vec<FromLift>,
    /// New jobs, oldest first; all are taken.
    pub jobs: Vec<Job>,
    /// Somebody watches the run: the reservations are wanted.
    pub watched: bool,
}

/// The newest report of a robot, as the traffic rules see it.
#[derive(Debug, Clone, PartialEq)]
pub struct Known {
    pub report: FromRobot,
    /// It arrived in this cycle (see the module text).
    pub new: bool,
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
pub struct Outbox {
    /// To be sent in this order.
    pub messages: Vec<Message>,
    /// For each robot, why it is held back and by whom; `None` where it is not,
    /// or nothing can be said.
    pub held_back: Vec<Option<HeldBack>>,
    /// What is reserved; only if [`Inbox::watched`].
    pub reservations: Option<Vec<Reservation>>,
}

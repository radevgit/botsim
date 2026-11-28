//! What the controller is given and what it gives back. It holds no robot, lift
//! or door: the caller tells it the site once ([`Config`]), then runs it in
//! cycles with what has arrived ([`Input`]) and sends what comes out
//! ([`Output`]).
//!
//! **Order of robots, lifts and doors.** [`Config`] lists them in a fixed order.
//! The controller numbers them by it (ties between robots go to the one listed
//! first), and every list of an [`Input`] and an [`Output`] is in that order.

use botsim_model::graph::Graph;
use botsim_model::reservation::Reservation;
use botsim_model::robot::RobotId;
use botsim_model::spec::{Kinematics, RobotSpec, Shape, SpecError};
use botsim_model::waiting::HeldBack;
use botsim_model::zones::Zones;
use vda5050::factsheet::Factsheet;
use vda5050::instant_actions::InstantActions;
use vda5050::order::Order;
use vda5050::responses::Responses;
use vda5050::state::State;
use vda5050::zone_set::ZoneSetMessage;

use crate::doors::Door;
use crate::lifts::Lift;

/// The site and the fleet: what stays the same during a run.
///
/// A lift stop or a zone set that names a node, a door of the lift or a map that
/// is not there is left out.
#[derive(Debug, Clone)]
pub struct Config {
    /// The layout.
    pub graph: Graph,
    pub robots: Vec<FleetRobot>,
    /// The site's zone set, if it has one.
    pub zones: Option<Zones>,
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
    /// Milliseconds since 1970-01-01T00:00:00Z, for the timestamps of the
    /// messages.
    pub unix_ms: i64,
}

/// What has arrived for one cycle.
#[derive(Debug, Clone, Copy)]
pub struct Input<'a> {
    pub time: Time,
    /// Every robot's state message, one for each of [`Config::robots`]. With any
    /// other number the cycle does nothing but take the new jobs and the time.
    pub states: &'a [State],
    /// For each of [`Config::lifts`], the robot in its cabin, if the lift names
    /// one; a missing entry counts as none.
    pub lift_riders: &'a [Option<RobotId>],
    /// Somebody watches the run: the reservations are wanted.
    pub watched: bool,
}

/// One message to send.
#[derive(Debug, Clone, PartialEq)]
pub enum Message {
    /// To the robot it names, like the three that follow.
    Order(Order),
    InstantActions(InstantActions),
    Responses(Responses),
    ZoneSet(ZoneSetMessage),
    /// The door with this id is wanted open.
    OpenDoor(String),
    /// The controller is done with the door.
    ReleaseDoor(String),
    /// Bring the cabin of a lift to the floor of a map and open a door.
    CallLift {
        lift: String,
        map: String,
        door: String,
    },
    /// The controller is done with the door of the lift with this id.
    ReleaseLift(String),
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

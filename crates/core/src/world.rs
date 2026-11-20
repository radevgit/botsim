//! The simulated world: one layout, a clock with a fixed time step, and many
//! robots. This is the "plain struct" version (no ECS); it is what a controller
//! talks to.
//!
//! - **Time** is a tick counter times a fixed step `dt` (never wall-clock time),
//!   so a run is repeatable. The VDA 5050 timestamps are made from the simulated
//!   time plus a start date given by the caller.
//! - **Addressing**: a robot is `(manufacturer, serialNumber)`, as in the VDA 5050
//!   topics; `send_order` and `send_instant_actions` take the id from the message.
//! - **One step** moves every robot, in the order they were added (deterministic).
//!   Messages are applied when sent, i.e. between steps.
//! - **State messages** are produced on request ([`World::state`]); each robot
//!   has its own `headerId` counter, which grows by one per state message.
//!
//! Not yet: collisions, the spatial index, a controller port with queues, state
//! publishing rules (`minimumStateInterval`), peripherals.

use std::collections::HashSet;
use std::fmt;

use vda5050::instant_actions::InstantActions;
use vda5050::order::Order;
use vda5050::state::State;

use crate::actions::ActionTimes;
use crate::collision::{Body, overlap, place};
use crate::drive::{DriveError, Pose};
use crate::graph::{Graph, NodeIdx};
use crate::robot::{Reception, Robot};
use crate::spec::RobotSpec;
use crate::state_report::Header;

/// VDA 5050 version written in the headers.
pub const PROTOCOL_VERSION: &str = "3.0.1";

/// Identity of a robot, as in the VDA 5050 topic.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct RobotId {
    pub manufacturer: String,
    pub serial_number: String,
}

impl RobotId {
    pub fn new(manufacturer: &str, serial_number: &str) -> RobotId {
        RobotId {
            manufacturer: manufacturer.to_string(),
            serial_number: serial_number.to_string(),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum WorldError {
    DuplicateRobot(RobotId),
    UnknownRobot(RobotId),
    /// The robot cannot be placed on its start node.
    Start(DriveError),
    /// The time step must be a positive, finite number of seconds.
    BadTimeStep,
}

impl fmt::Display for WorldError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            WorldError::DuplicateRobot(id) => {
                write!(
                    f,
                    "robot {}/{} exists already",
                    id.manufacturer, id.serial_number
                )
            }
            WorldError::UnknownRobot(id) => {
                write!(f, "no robot {}/{}", id.manufacturer, id.serial_number)
            }
            WorldError::Start(e) => write!(f, "cannot place the robot: {e:?}"),
            WorldError::BadTimeStep => write!(f, "the time step must be positive and finite"),
        }
    }
}

impl std::error::Error for WorldError {}

#[derive(Debug)]
struct Entry {
    id: RobotId,
    robot: Robot,
    /// `headerId` of the next state message.
    next_header_id: u32,
}

#[derive(Debug)]
pub struct World {
    graph: Graph,
    dt: f64,
    tick: u64,
    /// Date of tick 0, milliseconds since 1970-01-01T00:00:00Z.
    start_unix_ms: i64,
    robots: Vec<Entry>,
    collisions: Vec<Collision>,
    /// Robot index pairs that overlapped after the last step.
    touching: HashSet<(usize, usize)>,
}

/// Two robots started to overlap.
#[derive(Debug, Clone, PartialEq)]
pub struct Collision {
    /// The tick after whose movement the overlap was found.
    pub tick: u64,
    /// The pair, in the order the robots were added.
    pub a: RobotId,
    pub b: RobotId,
}

impl World {
    /// `dt`: the fixed time step in seconds (e.g. 0.05); `start_unix_ms`: the
    /// calendar time of tick 0, used only for message timestamps.
    pub fn new(graph: Graph, dt: f64, start_unix_ms: i64) -> Result<World, WorldError> {
        if !(dt.is_finite() && dt > 0.0) {
            return Err(WorldError::BadTimeStep);
        }
        Ok(World {
            graph,
            dt,
            tick: 0,
            start_unix_ms,
            robots: Vec::new(),
            collisions: Vec::new(),
            touching: HashSet::new(),
        })
    }

    pub fn graph(&self) -> &Graph {
        &self.graph
    }

    pub fn dt(&self) -> f64 {
        self.dt
    }

    pub fn tick(&self) -> u64 {
        self.tick
    }

    /// Simulated seconds since the start.
    pub fn time(&self) -> f64 {
        self.tick as f64 * self.dt
    }

    /// The current simulated time as an ISO 8601 UTC string with milliseconds.
    pub fn timestamp(&self) -> String {
        let ms = self.start_unix_ms + (self.time() * 1000.0).round() as i64;
        iso8601(ms)
    }

    /// Put a new, idle robot on `start`, facing `heading`.
    pub fn add_robot(
        &mut self,
        id: RobotId,
        spec: RobotSpec,
        times: ActionTimes,
        start: NodeIdx,
        heading: f64,
    ) -> Result<(), WorldError> {
        if self.index(&id).is_some() {
            return Err(WorldError::DuplicateRobot(id));
        }
        let robot =
            Robot::new(&self.graph, spec, times, start, heading).map_err(WorldError::Start)?;
        self.robots.push(Entry {
            id,
            robot,
            next_header_id: 0,
        });
        Ok(())
    }

    /// The robots in the order they were added.
    pub fn robot_ids(&self) -> impl Iterator<Item = &RobotId> {
        self.robots.iter().map(|e| &e.id)
    }

    pub fn pose(&self, id: &RobotId) -> Result<Pose, WorldError> {
        Ok(self.robots[self.find(id)?].robot.pose())
    }

    /// Advance the whole world by one time step: move the robots, then check
    /// for collisions.
    pub fn step(&mut self) {
        for e in &mut self.robots {
            e.robot.step(self.dt);
        }
        self.tick += 1;
        self.check_collisions();
    }

    /// All collisions so far, oldest first. One entry per contact: it is added
    /// when two robots start to overlap, not again on every step they stay
    /// overlapped (they must separate first).
    pub fn collisions(&self) -> &[Collision] {
        &self.collisions
    }

    /// Every pair is tested (fine for tens of robots; a spatial index comes
    /// later). Robots on different maps never collide.
    fn check_collisions(&mut self) {
        let bodies: Vec<(usize, Body)> = self
            .robots
            .iter()
            .map(|e| {
                let p = e.robot.pose();
                (p.map, place(e.robot.shape(), p.x, p.y, p.heading))
            })
            .collect();
        let mut now = HashSet::new();
        for i in 0..bodies.len() {
            for j in i + 1..bodies.len() {
                if bodies[i].0 == bodies[j].0 && overlap(&bodies[i].1, &bodies[j].1) {
                    now.insert((i, j));
                    if !self.touching.contains(&(i, j)) {
                        self.collisions.push(Collision {
                            tick: self.tick,
                            a: self.robots[i].id.clone(),
                            b: self.robots[j].id.clone(),
                        });
                    }
                }
            }
        }
        self.touching = now;
    }

    /// Deliver an `order`; the robot is the one named in the message.
    pub fn send_order(&mut self, order: &Order) -> Result<Reception, WorldError> {
        let id = RobotId::new(&order.manufacturer, &order.serial_number);
        let i = self.find(&id)?;
        Ok(self.robots[i].robot.receive_order(&self.graph, order))
    }

    /// Deliver an `instantActions` message.
    pub fn send_instant_actions(&mut self, msg: &InstantActions) -> Result<(), WorldError> {
        let id = RobotId::new(&msg.manufacturer, &msg.serial_number);
        let i = self.find(&id)?;
        self.robots[i].robot.receive_instant_actions(msg);
        Ok(())
    }

    /// The robot's state message now. Every call uses the next `headerId`.
    pub fn state(&mut self, id: &RobotId) -> Result<State, WorldError> {
        let i = self.find(id)?;
        let header = Header {
            header_id: self.robots[i].next_header_id,
            timestamp: self.timestamp(),
            version: PROTOCOL_VERSION.to_string(),
            manufacturer: id.manufacturer.clone(),
            serial_number: id.serial_number.clone(),
        };
        self.robots[i].next_header_id += 1;
        Ok(self.robots[i].robot.state(&self.graph, &header))
    }

    fn index(&self, id: &RobotId) -> Option<usize> {
        self.robots.iter().position(|e| e.id == *id)
    }

    fn find(&self, id: &RobotId) -> Result<usize, WorldError> {
        self.index(id)
            .ok_or_else(|| WorldError::UnknownRobot(id.clone()))
    }
}

/// Milliseconds since the Unix epoch for a UTC calendar time (month 1–12, day
/// 1–31; the caller checks the ranges).
pub fn unix_ms_from_utc(year: i64, month: i64, day: i64, hour: i64, min: i64, sec: i64) -> i64 {
    // days since 1970-01-01 (H. Hinnant's algorithm, inverse of `iso8601`)
    let y = year - i64::from(month <= 2);
    let era = y.div_euclid(400);
    let yoe = y.rem_euclid(400);
    let mp = if month > 2 { month - 3 } else { month + 9 };
    let doy = (153 * mp + 2) / 5 + day - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146_097 + doe - 719_468;
    ((days * 24 + hour) * 60 + min) * 60_000 + sec * 1000
}

/// `2026-11-20T10:00:00.250Z` from milliseconds since the Unix epoch.
fn iso8601(unix_ms: i64) -> String {
    let secs = unix_ms.div_euclid(1000);
    let ms = unix_ms.rem_euclid(1000);
    let days = secs.div_euclid(86_400);
    let sod = secs.rem_euclid(86_400);
    // civil date from days since 1970-01-01 (H. Hinnant's algorithm)
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}.{ms:03}Z",
        sod / 3600,
        sod % 3600 / 60,
        sod % 60
    )
}

#[cfg(test)]
mod tests {
    use super::{iso8601, unix_ms_from_utc};

    #[test]
    fn unix_ms_from_utc_is_the_inverse() {
        assert_eq!(unix_ms_from_utc(1970, 1, 1, 0, 0, 0), 0);
        assert_eq!(unix_ms_from_utc(2024, 2, 29, 12, 34, 56), 1_709_210_096_000);
        assert_eq!(unix_ms_from_utc(2026, 11, 20, 10, 0, 0), 1_795_168_800_000);
        for ms in [0, 951_782_400_000, 1_709_210_096_000, 4_102_444_799_000] {
            let s = iso8601(ms);
            let n = |a: usize, b: usize| s[a..b].parse::<i64>().unwrap();
            assert_eq!(
                unix_ms_from_utc(n(0, 4), n(5, 7), n(8, 10), n(11, 13), n(14, 16), n(17, 19)),
                ms
            );
        }
    }

    #[test]
    fn iso8601_known_dates() {
        assert_eq!(iso8601(0), "1970-01-01T00:00:00.000Z");
        assert_eq!(iso8601(1_000_000_000_000), "2001-09-09T01:46:40.000Z");
        // leap day, and a time with milliseconds
        assert_eq!(iso8601(1_709_210_096_250), "2024-02-29T12:34:56.250Z");
        assert_eq!(iso8601(1_795_168_800_000), "2026-11-20T10:00:00.000Z");
    }
}

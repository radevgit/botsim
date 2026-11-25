//! The observer port (brief U5): a **read-only** view of a run for a viewer, a
//! recorder or a test. Core calls it and never depends on any of them; the types
//! here are plain data (no file format, no viewer code).
//!
//! - Once per run, when the observer is attached: [`Observer::on_layout`] with the
//!   layout (maps, nodes, edges).
//! - After every step, or every chosen interval of simulated time (like a frame
//!   rate, `World::set_observer_at`): [`Observer::on_step`] with a [`Snapshot`]: the time, every
//!   robot (id, map, pose, shape, state, speed, released and planned route),
//!   the reservations the controller published, who blocks whom, robots that
//!   overlap, and the deadlocks that last.
//!
//! Observing changes nothing in the run: a run with and without an observer gives
//! the same result. Without an observer no snapshot is built.
//!
//! Not in it yet, because it does not exist yet: stations, zones, RELEASE zone
//! requests, lifts and doors.

use crate::door::DoorStatus;
use crate::drive::Pose;
use crate::graph::{EdgeIdx, Graph, NodeIdx};
use crate::lift::LiftStatus;
use crate::spec::Shape;
use crate::waiting::Wait;
use crate::world::RobotId;

/// Receives a run. All methods are called from the simulation thread.
pub trait Observer: std::fmt::Debug {
    /// Once, when the observer is attached: the layout. Maps, nodes and edges are
    /// numbered by their position in the lists; the snapshots use those numbers.
    fn on_layout(&mut self, layout: &Graph);

    /// After every step, once the robots have moved and collisions and deadlocks
    /// have been checked.
    fn on_step(&mut self, snapshot: &Snapshot);
}

/// What a robot is doing, for colouring it in a viewer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotStatus {
    /// No order, or the order is done.
    Idle,
    Driving,
    /// Turning on the spot.
    Turning,
    /// Standing at a node while actions run.
    Acting,
    /// Standing at the end of its released route, waiting for an update.
    WaitingForRelease,
    /// Standing before an edge that it must not start yet: it enters a RELEASE
    /// zone that is not granted, or leads through a door that is not open.
    WaitingForZone,
    /// Standing at the entrance of a lift, waiting for the cabin and its door.
    WaitingForLift,
    /// In a lift cabin. The robot's pose is still where it got in (it appears at
    /// the other floor when it gets out).
    RidingLift,
    /// Stopped by a disturbance (random stop).
    Held,
    /// Stopped by its protective field: another robot is ahead (S54).
    Obstructed,
}

/// The whole world after one step.
#[derive(Debug, Clone, PartialEq)]
pub struct Snapshot {
    pub tick: u64,
    /// Simulated seconds since the start.
    pub time: f64,
    /// All robots, in the order they were added; the other lists refer to a
    /// robot by its index in this list.
    pub robots: Vec<RobotSnapshot>,
    /// What the controller holds, if it publishes it (the baseline does).
    pub reservations: Vec<Reservation>,
    /// Who waits for whom now, by the rule of [`crate::deadlock`].
    pub blocking: Vec<Blocking>,
    /// Pairs of robots that overlap right now.
    pub overlapping: Vec<(usize, usize)>,
    /// Deadlocks that are reported and still last; each is the list of its robots.
    pub deadlocks: Vec<Vec<usize>>,
    /// The lifts (peripherals): floor, door, queue.
    pub lifts: Vec<LiftStatus>,
    /// The doors (peripherals): state and the edges they guard.
    pub doors: Vec<DoorStatus>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RobotSnapshot {
    pub id: RobotId,
    /// Map (floor), position of the kinematic center, heading.
    pub pose: Pose,
    /// The body outline, relative to the kinematic center (turns with the heading).
    pub shape: Shape,
    pub status: RobotStatus,
    /// Speed along the robot's x axis in m/s; negative when driving backwards.
    pub speed: f64,
    /// Empty if the robot never had an order.
    pub order_id: String,
    /// The last node reached, or the node the robot stands on.
    pub last_node: Option<NodeIdx>,
    /// Still ahead and released (the robot may drive them).
    pub released_nodes: Vec<NodeIdx>,
    pub released_edges: Vec<EdgeIdx>,
    /// Still ahead and not released yet (the horizon).
    pub planned_nodes: Vec<NodeIdx>,
    pub planned_edges: Vec<EdgeIdx>,
    /// The battery charge in percent; `None` if the robot has no battery model.
    pub charge: Option<f64>,
    /// The robot is charging (always false until the chargers exist).
    pub charging: bool,
    /// Why the robot waits in this step and for whom (the blocker is an index in
    /// [`Snapshot::robots`]; `None` there means unknown), as the "waiting caused by
    /// others" metric counts it (see [`crate::waiting`]): `None` for a robot that
    /// does not wait, or that is in its own stop (a random stop, an empty battery).
    pub waiting: Option<Wait>,
}

/// The nodes and edges a controller has reserved for a robot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Reservation {
    /// Index of the robot in [`Snapshot::robots`].
    pub robot: usize,
    pub nodes: Vec<NodeIdx>,
    pub edges: Vec<EdgeIdx>,
}

/// One robot waits for another.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Blocking {
    pub waiter: usize,
    pub blocker: usize,
}

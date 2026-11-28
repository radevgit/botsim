//! The simulated world: one layout, a clock with a fixed time step, many robots,
//! and the lifts and doors of the site. This is the "plain struct" version (no
//! ECS). It talks in the messages of [`botsim_link`] (VDA 5050 meaning):
//!
//! - **To robots** [`World::send`]: orders, instant actions, responses and zone
//!   sets. [`World::send_unread`] is for what whoever reads a robot's messages
//!   could not read.
//! - **From robots** [`World::reports`]: a state report of every robot; call it
//!   once in every step (with a message delay the reports on their way move on
//!   when they are asked for). [`World::report`] gives one robot's.
//! - **Doors and lifts**: [`World::command_door`], [`World::command_lift`],
//!   [`World::door_reports`], [`World::lift_reports`].
//! - **Time** is a tick counter times a fixed step `dt` (never wall-clock time),
//!   so a run is repeatable. The times of the messages are the simulated time plus
//!   a start date given by the caller.
//! - **Addressing**: robot `i` of the world, in the order they were added, is the
//!   robot number `(i, 0)` ([`World::robot_no`]); door `i` is `DoorNo(i)` and lift
//!   `i` is `LiftNo(i)`. A robot's identity is `(manufacturer, serialNumber)`, as
//!   in the VDA 5050 topics.
//! - **One step** moves every robot, in the order they were added (deterministic).
//!   Messages are applied when sent, i.e. between steps, or with a message delay
//!   ([`World::set_message_delay`]) that many steps later.
//! - Each robot numbers its state reports (as `headerId` does), each door and lift
//!   its own.
//!
//! A message that names what is not there, whatever its numbers, is an error or is
//! refused by the robot, never a panic.
//!
//! Not yet: state publishing rules (`minimumStateInterval`).

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::fmt;

use aabb::prelude::AABB;
use togo::prelude::{circle_bounding_rect, orect_bounding_rect};

use vda5050::factsheet::Factsheet;

use crate::actions::ActionTimes;
use crate::deadlock::{DEFAULT_TIMEOUT_SECONDS, DeadlockDetector};
use crate::disturbances::{DisturbanceError, StopGenerator, StopSpec, check_duration_variation};
use crate::door::{Door, DoorError, DoorSpec, DoorStatus};
use crate::drive::Phase;
use crate::drive::{DriveError, Glance, Pose};
use crate::jobs::Rng;
use crate::lift::{DoorState, Lift, LiftError, LiftSpec, LiftStatus};
use crate::observer::{Blocking, Observer, RobotSnapshot, RobotStatus, Snapshot};
use crate::robot::{Reception, Robot, START_NODE_TOLERANCE, Unread};
use crate::waiting::{Place, Wait, WaitTotal};
use crate::zone_requests::ZoneRequestTiming;
use botsim_link::device::{
    DeviceHeader, DoorCommand, DoorNo, DoorPhase, FromDoor, FromLift, LiftCommand, LiftNo, ToDoor,
    ToLift,
};
use botsim_link::message::{FromRobot, Header, Millis, ToRobot, ToRobotBody};
use botsim_link::state::ZoneRequest;
use botsim_model::collision::{Body, gap, overlap, place};
use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_model::protocol::PROTOCOL_VERSION;
use botsim_model::reservation::Reservation;
use botsim_model::robot::{RobotId, RobotNo};
use botsim_model::spec::RobotSpec;
use botsim_model::time::iso8601;
use botsim_model::waiting::{HeldBack, WaitReason, stands_waiting};
use botsim_model::zones::Zones;

/// A robot that the protective field stops again within this time after it last held
/// it is still in the same safety stop (see [`World::safety_stops`]).
pub const SAFETY_STOP_GAP_SECONDS: f64 = 1.0;

/// What the batteries of a run show (see [`World::battery_report`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryReport {
    /// Robots with a battery model.
    pub robots: usize,
    /// Robots whose battery was empty at some time.
    pub ran_empty: usize,
    /// Robots standing empty now.
    pub empty_now: usize,
    /// Simulated seconds all robots together spent charging.
    pub charging_seconds: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum WorldError {
    DuplicateRobot(RobotId),
    UnknownRobot(RobotId),
    /// No robot with this number.
    UnknownRobotNo(RobotNo),
    /// No door with this number.
    UnknownDoorNo(DoorNo),
    /// No lift with this number.
    UnknownLiftNo(LiftNo),
    /// The robot cannot be placed on its start node.
    Start(DriveError),
    /// The time step must be a positive, finite number of seconds.
    BadTimeStep,
    /// Wrong disturbance settings.
    Disturbance(DisturbanceError),
    /// The observer interval must be a positive, finite number of seconds.
    BadObserverInterval(f64),
    /// The charger power must be a positive, finite number of watts.
    BadChargerPower(f64),
    /// A lift is wrong, or a call to it.
    Lift(LiftError),
    DuplicateLift(String),
    UnknownLift(String),
    /// A door name the lift does not have.
    UnknownDoor(String),
    /// A map id that the layout does not have (a lift floor, a lift call).
    UnknownMap(String),
    /// A door is wrong.
    Door(DoorError),
    DuplicateDoor(String),
    /// No door with this id (not to be mixed up with a door of a lift).
    NoSuchDoor(String),
    /// A door edge is not an edge of the layout, or another door guards it
    /// already.
    BadDoorEdge(String),
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
            WorldError::UnknownRobotNo(no) => write!(
                f,
                "no robot number {} (generation {})",
                no.slot(),
                no.generation()
            ),
            WorldError::UnknownDoorNo(no) => write!(f, "no door number {}", no.0),
            WorldError::UnknownLiftNo(no) => write!(f, "no lift number {}", no.0),
            WorldError::Start(e) => write!(f, "cannot place the robot: {e}"),
            WorldError::BadTimeStep => write!(f, "the time step must be positive and finite"),
            WorldError::Disturbance(e) => write!(f, "{e}"),
            WorldError::Lift(e) => write!(f, "lift: {e}"),
            WorldError::DuplicateLift(id) => write!(f, "lift \"{id}\" exists already"),
            WorldError::UnknownLift(id) => write!(f, "no lift \"{id}\""),
            WorldError::UnknownDoor(id) => write!(f, "the lift has no door \"{id}\""),
            WorldError::UnknownMap(id) => write!(f, "the layout has no map \"{id}\""),
            WorldError::Door(e) => write!(f, "door: {e}"),
            WorldError::DuplicateDoor(id) => write!(f, "door \"{id}\" exists already"),
            WorldError::NoSuchDoor(id) => write!(f, "no door \"{id}\""),
            WorldError::BadDoorEdge(id) => write!(
                f,
                "edge \"{id}\" is not in the layout, or another door guards it already"
            ),
            WorldError::BadObserverInterval(v) => {
                write!(
                    f,
                    "the observer interval must be positive and finite, got {v}"
                )
            }
            WorldError::BadChargerPower(v) => {
                write!(f, "the charger power must be positive and finite, got {v}")
            }
        }
    }
}

impl std::error::Error for WorldError {}

#[derive(Debug)]
struct Entry {
    id: RobotId,
    robot: Robot,
    /// Sequence number of the next state report.
    next_header_id: u32,
    /// Random stops of this robot, if disturbances are on.
    stops: Option<StopGenerator>,
    /// The state reports asked for, with the tick they were made at (only with a
    /// message delay, S58).
    seen: VecDeque<(u64, FromRobot)>,
    /// The last step in which the protective field stopped the robot.
    obstructed_tick: Option<u64>,
}

/// What is on its way to a robot (S58).
#[derive(Debug)]
enum Delivery {
    Message(ToRobotBody),
    Unread(Unread),
}

#[derive(Debug)]
pub struct World {
    /// The power (W) of the chargers, and the nodes where they are ([`World::set_chargers`]).
    charger_power_w: Option<f64>,
    charger_nodes: Vec<NodeIdx>,
    /// Messages to the robots that are on their way: (tick they arrive, index of
    /// the robot, message).
    inbox: VecDeque<(u64, usize, Delivery)>,
    /// How many steps a message takes, each way (S58); 0 = none.
    message_delay: u64,
    /// Seed and spec of the random stops; robots added later get them too.
    stops: Option<(u64, StopSpec)>,
    /// How many random stops started so far.
    stops_started: usize,
    /// Safety stops counted so far ([`World::safety_stops`]).
    safety_stops: usize,
    /// Seed and variation of the random action durations; robots added later get them too.
    duration_variation: Option<(u64, f64)>,
    graph: Graph,
    dt: f64,
    tick: u64,
    /// Date of tick 0, milliseconds since 1970-01-01T00:00:00Z.
    start_unix_ms: i64,
    robots: Vec<Entry>,
    collisions: Vec<Collision>,
    /// Robot index pairs that overlapped after the last step.
    touching: HashSet<(usize, usize)>,
    deadlocks: DeadlockDetector,
    /// Who waited, why and for whom after the last step.
    waits: Vec<Option<Wait>>,
    /// Steps waited so far, by (waiter, blocker, reason, place).
    wait_ticks: BTreeMap<(usize, Option<usize>, WaitReason, Place), u64>,
    /// For a robot stopped by its protective field after the last step: the robot
    /// in the field (the nearest if there are several).
    field_blocker: Vec<Option<usize>>,
    /// For each robot, why the controller holds it back and who causes it; `None`
    /// until a controller publishes.
    held_back: Option<Vec<Option<HeldBack>>>,
    /// What the controller has published for the observer.
    reservations: Vec<Reservation>,
    observer: Option<Box<dyn Observer>>,
    /// The observer gets a snapshot every this many steps.
    observer_every: u64,
    zones: Option<Zones>,
    /// Lifts: peripherals with their own command port.
    lifts: Vec<Lift>,
    /// The layout edges that are lift rides (between two stops of one lift).
    rides: HashSet<EdgeIdx>,
    /// Doors: peripherals with their own command port (S50).
    doors: Vec<Door>,
    /// The door that guards an edge → index into `doors`.
    door_of_edge: HashMap<EdgeIdx, usize>,
    /// Passage width of the doors by edge (for robots added later).
    door_widths: HashMap<EdgeIdx, f64>,
    /// For each door and each lift, the sequence number of its next report.
    door_seqs: Vec<u32>,
    lift_seqs: Vec<u32>,
    /// Broad phase of the collision check (S52): rebuilt every step.
    broad: AABB,
    /// The robots brake when another robot is in their protective field (S54).
    safety_stop: bool,
    /// The robots drive through nodes where they can (S66).
    pass_through: bool,
    /// `zones` were given to the robots already enabled (`set_zones`), not only
    /// set for the site (`set_site_zones`, S62).
    zones_on_robots: bool,
}

/// A deadlock: robots waiting for each other in a cycle (see [`crate::deadlock`]).
#[derive(Debug, Clone, PartialEq)]
pub struct Deadlock {
    /// The robots in the cycle, in the order they were added.
    pub robots: Vec<RobotId>,
    pub first_seen_tick: u64,
    /// When it had lasted for the timeout and was reported.
    pub detected_tick: u64,
    /// `None` if it still lasts.
    pub ended_tick: Option<u64>,
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
            deadlocks: DeadlockDetector::new(DEFAULT_TIMEOUT_SECONDS, dt),
            stops: None,
            stops_started: 0,
            safety_stops: 0,
            duration_variation: None,
            waits: Vec::new(),
            wait_ticks: BTreeMap::new(),
            field_blocker: Vec::new(),
            held_back: None,
            reservations: Vec::new(),
            observer: None,
            observer_every: 1,
            zones: None,
            lifts: Vec::new(),
            rides: HashSet::new(),
            doors: Vec::new(),
            door_of_edge: HashMap::new(),
            door_widths: HashMap::new(),
            door_seqs: Vec::new(),
            lift_seqs: Vec::new(),
            broad: AABB::new(),
            safety_stop: false,
            pass_through: false,
            charger_power_w: None,
            charger_nodes: Vec::new(),
            inbox: VecDeque::new(),
            message_delay: 0,
            zones_on_robots: false,
        })
    }

    /// Every message between the controller and the robots takes `steps` time
    /// steps, in each direction (S58): orders, instant actions, responses and
    /// zone sets arrive at the robot `steps` steps after they were sent, and the
    /// state reports the controller gets ([`World::reports`]) are `steps` steps
    /// old. 0 (the default) = instant. Set it before the run. The robot's answer
    /// to an order is not returned any more ([`Reception::InTransit`]); it shows
    /// in the state. Lift and door commands are not delayed.
    pub fn set_message_delay(&mut self, steps: u64) {
        self.message_delay = steps;
    }

    /// The message delay in steps.
    pub fn message_delay(&self) -> u64 {
        self.message_delay
    }

    /// Deliver the messages whose time has come, oldest first.
    fn deliver_due(&mut self) {
        while self
            .inbox
            .front()
            .is_some_and(|(due, _, _)| *due <= self.tick)
        {
            let Some((_, i, delivery)) = self.inbox.pop_front() else {
                break;
            };
            match delivery {
                Delivery::Message(body) => {
                    self.deliver(i, &body);
                }
                Delivery::Unread(unread) => self.deliver_unread(i, &unread),
            }
        }
    }

    /// Is the onboard safety stop on?
    pub fn safety_stop(&self) -> bool {
        self.safety_stop
    }

    /// Switch the onboard safety stop (S54, see [`botsim_model::safety`]) on or off for
    /// all robots. It is **off** by default in the world (the collision tests
    /// need robots that drive into each other); the scenario switches it on.
    pub fn set_safety_stop(&mut self, on: bool) {
        self.safety_stop = on;
        if !on {
            for e in &mut self.robots {
                e.robot.set_obstructed(false);
            }
        }
    }

    /// How long a cycle of waiting robots must last to be a deadlock (default
    /// 30 s). Call it before the run; it forgets deadlocks seen so far.
    pub fn set_deadlock_timeout(&mut self, seconds: f64) {
        self.deadlocks = DeadlockDetector::new(seconds, self.dt);
    }

    /// All deadlocks reported so far, oldest first.
    pub fn deadlocks(&self) -> Vec<Deadlock> {
        self.deadlocks
            .records()
            .iter()
            .map(|r| Deadlock {
                robots: r
                    .robots
                    .iter()
                    .map(|&i| self.robots[i].id.clone())
                    .collect(),
                first_seen_tick: r.first_seen_tick,
                detected_tick: r.detected_tick,
                ended_tick: r.ended_tick,
            })
            .collect()
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

    /// The current simulated time in milliseconds since 1970.
    pub fn now_ms(&self) -> i64 {
        self.start_unix_ms + (self.time() * 1000.0).round() as i64
    }

    /// The current simulated time as an ISO 8601 UTC string with milliseconds.
    pub fn timestamp(&self) -> String {
        let ms = self.start_unix_ms + (self.time() * 1000.0).round() as i64;
        iso8601(ms)
    }

    /// The robot's `factsheet` (S61): setup data, in the form of the VDA 5050
    /// message. It is the factsheet the robot's spec was made from, as the robot
    /// states it: its own `manufacturer` and `serialNumber`, the protocol version
    /// of the world, and the time now. Not delayed by a message delay (a
    /// controller asks for it once, when it starts). The controller learns the
    /// robot's type from it (`manufacturer`, `typeSpecification.seriesName`).
    pub fn factsheet(&self, id: &RobotId) -> Result<Factsheet, WorldError> {
        let mut f = self.robots[self.find(id)?].robot.spec().factsheet.clone();
        f.header_id = 0;
        f.timestamp = self.timestamp();
        f.version = PROTOCOL_VERSION.to_string();
        f.manufacturer = id.manufacturer.clone();
        f.serial_number = id.serial_number.clone();
        Ok(f)
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
        // the layout must allow the robot's type on its start node (S55)
        if !self
            .graph
            .node_allows(start, &spec.manufacturer, &spec.series_name)
        {
            return Err(WorldError::Start(DriveError::NotAllowedForType {
                what: format!(
                    "node '{}' (robot type {}/{})",
                    self.graph.nodes[start].id, spec.manufacturer, spec.series_name
                ),
            }));
        }
        let mut robot =
            Robot::new(&self.graph, spec, times, start, heading).map_err(WorldError::Start)?;
        robot.set_time(self.now_ms());
        robot.set_rides(self.rides.clone());
        robot.set_door_widths(self.door_widths.clone());
        robot.set_door_edges(self.door_edges());
        robot.set_pass_through(self.pass_through);
        if let (Some(zones), true) = (&self.zones, self.zones_on_robots) {
            robot.set_zones(zones.clone(), ZoneRequestTiming::default());
        }
        if let Some((seed, variation)) = self.duration_variation {
            robot.set_duration_noise(
                Rng::stream(seed, &format!("durations-{}", self.robots.len())),
                variation,
            );
        }
        let stops = match self.stops {
            Some((seed, spec)) => Some(
                StopGenerator::new(seed, self.robots.len(), spec, self.dt)
                    .map_err(WorldError::Disturbance)?,
            ),
            None => None,
        };
        self.robots.push(Entry {
            id,
            robot,
            next_header_id: 0,
            stops,
            seen: VecDeque::new(),
            obstructed_tick: None,
        });
        Ok(())
    }

    /// Turn on random stops (S37) for all robots, now and added later; the
    /// seed makes them repeatable. Call it before the run.
    pub fn enable_random_stops(&mut self, seed: u64, spec: StopSpec) -> Result<(), WorldError> {
        spec.check(self.dt).map_err(WorldError::Disturbance)?;
        self.stops = Some((seed, spec));
        for (i, e) in self.robots.iter_mut().enumerate() {
            e.stops =
                Some(StopGenerator::new(seed, i, spec, self.dt).map_err(WorldError::Disturbance)?);
        }
        Ok(())
    }

    /// Turn on random extra time for every action (S38): each action takes up to
    /// `variation` longer (0.2 = 20 %). For all robots, now and added later; each
    /// robot has its own random stream. Call it before the run.
    pub fn enable_duration_variation(
        &mut self,
        seed: u64,
        variation: f64,
    ) -> Result<(), WorldError> {
        check_duration_variation(variation).map_err(WorldError::Disturbance)?;
        self.duration_variation = Some((seed, variation));
        for (i, e) in self.robots.iter_mut().enumerate() {
            e.robot
                .set_duration_noise(Rng::stream(seed, &format!("durations-{i}")), variation);
        }
        Ok(())
    }

    /// How many random stops have started so far.
    pub fn random_stops(&self) -> usize {
        self.stops_started
    }

    /// How many safety stops there were: a robot driving or turning was stopped by
    /// its protective field. A new stop of the same robot less than
    /// [`SAFETY_STOP_GAP_SECONDS`] after the field last held it counts as the same
    /// one (the field gets shorter as the robot brakes, so it can let go and catch
    /// again). Random stops are not counted.
    pub fn safety_stops(&self) -> usize {
        self.safety_stops
    }

    /// The robots in the order they were added.
    pub fn robot_ids(&self) -> impl Iterator<Item = &RobotId> {
        self.robots.iter().map(|e| &e.id)
    }

    /// The robot's spec as it is in the run (with any disturbance applied).
    pub fn spec(&self, id: &RobotId) -> Result<&RobotSpec, WorldError> {
        Ok(self.robots[self.find(id)?].robot.spec())
    }

    pub fn pose(&self, id: &RobotId) -> Result<Pose, WorldError> {
        Ok(self.robots[self.find(id)?].robot.pose())
    }

    /// Advance the whole world by one time step: move the robots, then check
    /// for collisions.
    pub fn step(&mut self) {
        self.deliver_due();
        let now_ms = self.now_ms();
        self.step_doors();
        self.refresh_chargers();
        for e in &mut self.robots {
            e.robot.set_time(now_ms);
            // a robot that is moving, and not stopped already, may be stopped (S37)
            if let Some(g) = e.stops.as_mut()
                && e.robot.is_moving()
                && !e.robot.is_held()
                && let Some(seconds) = g.step()
            {
                e.robot.hold(seconds);
                self.stops_started += 1;
            }
            e.robot.step(self.dt);
        }
        for l in &mut self.lifts {
            l.step(self.dt);
        }
        self.update_rides();
        self.tick += 1;
        self.check_collisions();
        self.check_deadlocks();
        self.notify_observer();
    }

    /// Attach an observer (brief U5): it gets the layout now and a snapshot after
    /// **every** step. Observing is read-only and changes nothing in the run. A
    /// second call replaces the first observer. For big runs use
    /// [`World::set_observer_at`].
    pub fn set_observer(&mut self, observer: Box<dyn Observer>) {
        // (the time step is a valid interval, so this cannot fail; if it did, no
        // observer would be attached)
        let _ = self.set_observer_at(observer, self.dt);
    }

    /// Like [`World::set_observer`], but the observer gets a snapshot only every
    /// `interval_seconds` of simulated time, like the frame rate of a game: 0.5
    /// is two snapshots per simulated second. The interval is rounded to whole
    /// steps and is at least one step. The first snapshot comes after the first
    /// interval. What happens between two snapshots (a short overlap, a short
    /// stop) is not seen by the observer; the world still counts it.
    pub fn set_observer_at(
        &mut self,
        mut observer: Box<dyn Observer>,
        interval_seconds: f64,
    ) -> Result<(), WorldError> {
        if !(interval_seconds.is_finite() && interval_seconds > 0.0) {
            return Err(WorldError::BadObserverInterval(interval_seconds));
        }
        observer.on_layout(&self.graph);
        self.observer = Some(observer);
        self.observer_every = ((interval_seconds / self.dt).round() as u64).max(1);
        Ok(())
    }

    /// Is an observer attached? (A controller can skip preparing data for it.)
    pub fn observing(&self) -> bool {
        self.observer.is_some()
    }

    /// Add a door (a peripheral, see [`crate::door`]). Its edges are edge
    /// indices of the layout; an edge has at most one door; the id must be new.
    pub fn add_door(&mut self, spec: DoorSpec) -> Result<(), WorldError> {
        if self.doors.iter().any(|d| d.spec().id == spec.id) {
            return Err(WorldError::DuplicateDoor(spec.id));
        }
        for &e in &spec.edges {
            if e >= self.graph.edges.len() || self.door_of_edge.contains_key(&e) {
                let id = self
                    .graph
                    .edges
                    .get(e)
                    .map_or(format!("#{e}"), |x| x.id.clone());
                return Err(WorldError::BadDoorEdge(id));
            }
        }
        let door = Door::new(spec).map_err(WorldError::Door)?;
        let index = self.doors.len();
        for &e in &door.spec().edges {
            self.door_of_edge.insert(e, index);
            if let Some(w) = door.spec().passage_width {
                self.door_widths.insert(e, w);
            }
        }
        let door_edges = self.door_edges();
        for e in &mut self.robots {
            e.robot.set_door_widths(self.door_widths.clone());
            e.robot.set_door_edges(door_edges.clone());
        }
        self.doors.push(door);
        self.door_seqs.push(0);
        Ok(())
    }

    /// All edges that have a door.
    fn door_edges(&self) -> HashSet<EdgeIdx> {
        self.door_of_edge.keys().copied().collect()
    }

    /// Let the robots drive **through** nodes without stopping where they can
    /// (S66): straight on, nothing to do at the node, the next edge released and
    /// free to be driven, and not in front of a door or a lift. Off by default in
    /// the world (every robot stops at every node, as before); the scenario switches
    /// it on. See `docs/rules.md` §2.
    pub fn set_pass_through(&mut self, on: bool) {
        self.pass_through = on;
        for e in &mut self.robots {
            e.robot.set_pass_through(on);
        }
    }

    /// Do robots drive through nodes where they can?
    pub fn pass_through(&self) -> bool {
        self.pass_through
    }

    /// Move the doors one step and tell the robots which door edges they must
    /// not start: a door closes only while no robot is on one of its edges.
    fn step_doors(&mut self) {
        if self.doors.is_empty() {
            return;
        }
        let mut occupied = vec![false; self.doors.len()];
        for e in &self.robots {
            if let Some(edge) = e.robot.driving_edge()
                && let Some(&d) = self.door_of_edge.get(&edge)
            {
                occupied[d] = true;
            }
        }
        for (d, door) in self.doors.iter_mut().enumerate() {
            door.step(self.dt, occupied[d]);
        }
        let closed: HashSet<EdgeIdx> = self
            .doors
            .iter()
            .filter(|d| !d.is_open())
            .flat_map(|d| d.spec().edges.iter().copied())
            .collect();
        for e in &mut self.robots {
            e.robot.set_closed_door_edges(closed.clone());
        }
    }

    fn door_index(&self, id: &str) -> Result<usize, WorldError> {
        self.doors
            .iter()
            .position(|d| d.spec().id == id)
            .ok_or_else(|| WorldError::NoSuchDoor(id.to_string()))
    }

    /// Door command port: the controller wants door `door_id` open (see [`Door::open`]).
    pub fn open_door(&mut self, door_id: &str) -> Result<(), WorldError> {
        let i = self.door_index(door_id)?;
        self.doors[i].open();
        Ok(())
    }

    /// Door command port: the controller is done with the door; it closes once no
    /// robot is on its edges (see [`Door::release`]).
    pub fn release_door(&mut self, door_id: &str) -> Result<(), WorldError> {
        let i = self.door_index(door_id)?;
        self.doors[i].release();
        Ok(())
    }

    /// What door `door_id` looks like now.
    pub fn door_status(&self, door_id: &str) -> Result<DoorStatus, WorldError> {
        Ok(self.doors[self.door_index(door_id)?].status())
    }

    /// All doors, in the order they were added.
    pub fn doors(&self) -> Vec<DoorStatus> {
        self.doors.iter().map(Door::status).collect()
    }

    /// What the doors are like (edges, width, times), in the order they were added.
    pub fn door_specs(&self) -> Vec<DoorSpec> {
        self.doors.iter().map(|d| d.spec().clone()).collect()
    }

    /// A command to the door the header names: the controller wants it open, or is
    /// done with it (see [`Door::open`], [`Door::release`]).
    pub fn command_door(&mut self, message: &ToDoor) -> Result<(), WorldError> {
        let no = message.header.device;
        let door = usize::try_from(no.0)
            .ok()
            .and_then(|i| self.doors.get_mut(i))
            .ok_or(WorldError::UnknownDoorNo(no))?;
        match message.command {
            DoorCommand::Open => door.open(),
            DoorCommand::Release => door.release(),
        }
        Ok(())
    }

    /// The state of every door, in the order they were added.
    pub fn door_reports(&mut self) -> Vec<FromDoor> {
        let time = Millis(self.now_ms());
        (0u32..)
            .zip(self.doors.iter().zip(&mut self.door_seqs))
            .map(|(i, (door, seq))| {
                let header = DeviceHeader {
                    device: DoorNo(i),
                    seq: *seq,
                    time,
                };
                *seq = seq.wrapping_add(1);
                FromDoor {
                    header,
                    door: door_phase(door.status().state),
                }
            })
            .collect()
    }

    /// Add a lift (a peripheral, see [`crate::lift`]). Its floors are map indices
    /// of the layout; the id must be new.
    pub fn add_lift(&mut self, spec: LiftSpec) -> Result<(), WorldError> {
        if self.lifts.iter().any(|l| l.spec().id == spec.id) {
            return Err(WorldError::DuplicateLift(spec.id));
        }
        if let Some(&bad) = spec.floors.iter().find(|&&m| m >= self.graph.maps.len()) {
            return Err(WorldError::UnknownMap(format!("#{bad}")));
        }
        // a stop is a node of the layout, on the map of its floor, used by one lift only
        for (i, s) in spec.stops.iter().enumerate() {
            let ok = self
                .graph
                .nodes
                .get(s.node)
                .is_some_and(|n| spec.floors.get(s.floor) == Some(&n.map))
                && !self.lifts.iter().any(|l| l.stop_at(s.node).is_some())
                // a waiting node is another node on the same map
                && s.waiting.is_none_or(|w| {
                    w != s.node
                        && self
                            .graph
                            .nodes
                            .get(w)
                            .is_some_and(|n| spec.floors.get(s.floor) == Some(&n.map))
                });
            if !ok {
                return Err(WorldError::Lift(LiftError::BadStop(i)));
            }
        }
        let lift = Lift::new(spec).map_err(WorldError::Lift)?;
        // the layout edges between two stops of this lift on different floors are rides
        for (i, e) in self.graph.edges.iter().enumerate() {
            if let (Some(a), Some(b)) = (lift.stop_at(e.start), lift.stop_at(e.end))
                && a.floor != b.floor
                && e.map_transition
            {
                self.rides.insert(i);
            }
        }
        for e in &mut self.robots {
            e.robot.set_rides(self.rides.clone());
        }
        self.lifts.push(lift);
        self.lift_seqs.push(0);
        Ok(())
    }

    /// Robots get into and out of the cabins. A robot waiting at the start of a
    /// ride gets in when the cabin is at its floor with the door of its stop open
    /// and nobody is in the cabin; a robot in the cabin gets out when the door of
    /// the stop at the end of the ride is open at that floor. Both are instant
    /// (see `docs/rules.md` §7g). Robots are served in the order they were added.
    fn update_rides(&mut self) {
        for i in 0..self.robots.len() {
            let Some(edge) = self.robots[i].robot.ride_edge() else {
                continue;
            };
            let e = &self.graph.edges[edge];
            let Some((li, from, to)) = self
                .lifts
                .iter()
                .enumerate()
                .find_map(|(li, l)| Some((li, l.stop_at(e.start)?, l.stop_at(e.end)?)))
            else {
                continue; // cannot happen: only rides of a lift are accepted
            };
            if self.robots[i].robot.is_riding() {
                if self.lifts[li].open_at(to) {
                    self.robots[i].robot.alight();
                }
            } else if self.lifts[li].open_at(from)
                && !self.robots.iter().any(|r| {
                    r.robot.is_riding()
                        && r.robot
                            .ride_edge()
                            .is_some_and(|re| self.lift_of_ride(re) == Some(li))
                })
            {
                self.robots[i].robot.board();
            }
        }
    }

    /// The index of the lift a ride edge belongs to.
    fn lift_of_ride(&self, edge: EdgeIdx) -> Option<usize> {
        let e = &self.graph.edges[edge];
        self.lifts
            .iter()
            .position(|l| l.stop_at(e.start).is_some() && l.stop_at(e.end).is_some())
    }

    /// The robot in the cabin of lift `lift_id`, if any (one robot at a time).
    pub fn lift_rider(&self, lift_id: &str) -> Result<Option<&RobotId>, WorldError> {
        let li = self.lift_index(lift_id)?;
        Ok(self.rider(li).map(|i| &self.robots[i].id))
    }

    /// The index of the robot in the cabin of lift `li`.
    fn rider(&self, li: usize) -> Option<usize> {
        self.robots.iter().position(|r| {
            r.robot.is_riding()
                && r.robot
                    .ride_edge()
                    .is_some_and(|e| self.lift_of_ride(e) == Some(li))
        })
    }

    /// A command to the lift the header names: bring the cabin to a floor and open
    /// a door there, or close the open door (see [`Lift::call_door`],
    /// [`Lift::release`]).
    pub fn command_lift(&mut self, message: &ToLift) -> Result<(), WorldError> {
        let no = message.header.device;
        let lift = usize::try_from(no.0)
            .ok()
            .and_then(|i| self.lifts.get_mut(i))
            .ok_or(WorldError::UnknownLiftNo(no))?;
        match message.command {
            LiftCommand::Release => lift.release(),
            LiftCommand::Call { map, car_door } => {
                let floor = lift
                    .spec()
                    .floors
                    .iter()
                    .position(|&m| m == map)
                    .ok_or_else(|| WorldError::UnknownMap(format!("number {map}")))?;
                let door = usize::try_from(car_door)
                    .ok()
                    .filter(|&d| d < lift.spec().doors.len())
                    .ok_or_else(|| WorldError::UnknownDoor(format!("number {car_door}")))?;
                lift.call_door(floor, door).map_err(WorldError::Lift)?;
            }
        }
        Ok(())
    }

    /// The state of every lift, in the order they were added, with the robot in its
    /// cabin (which a real lift could not name).
    pub fn lift_reports(&mut self) -> Vec<FromLift> {
        let time = Millis(self.now_ms());
        let mut reports = Vec::with_capacity(self.lifts.len());
        for (i, li) in (0u32..).zip(0..self.lifts.len()) {
            let status = self.lifts[li].status();
            // (the cabin is at a floor of the lift and uses one of its doors)
            let (Some(&map), Ok(car_door)) = (
                self.lifts[li].spec().floors.get(status.floor),
                u32::try_from(status.door_index),
            ) else {
                continue;
            };
            let occupant = self.rider(li).and_then(number);
            let header = DeviceHeader {
                device: LiftNo(i),
                seq: self.lift_seqs[li],
                time,
            };
            self.lift_seqs[li] = self.lift_seqs[li].wrapping_add(1);
            reports.push(FromLift {
                header,
                map,
                car_door,
                door: door_phase(status.door),
                occupant,
            });
        }
        reports
    }

    fn lift_index(&self, id: &str) -> Result<usize, WorldError> {
        self.lifts
            .iter()
            .position(|l| l.spec().id == id)
            .ok_or_else(|| WorldError::UnknownLift(id.to_string()))
    }

    /// Lift command port: bring the cabin of lift `lift_id` to the floor of map
    /// `map_id` and open the door (see [`Lift::call`]).
    pub fn call_lift(&mut self, lift_id: &str, map_id: &str) -> Result<(), WorldError> {
        self.call_lift_door_index(lift_id, map_id, 0)
    }

    /// Like [`World::call_lift`], but opens the door named `door_id` (S45; for a
    /// lift with more than one door).
    pub fn call_lift_door(
        &mut self,
        lift_id: &str,
        map_id: &str,
        door_id: &str,
    ) -> Result<(), WorldError> {
        let door = self.lifts[self.lift_index(lift_id)?]
            .spec()
            .doors
            .iter()
            .position(|d| d == door_id)
            .ok_or_else(|| WorldError::UnknownDoor(door_id.to_string()))?;
        self.call_lift_door_index(lift_id, map_id, door)
    }

    fn call_lift_door_index(
        &mut self,
        lift_id: &str,
        map_id: &str,
        door: usize,
    ) -> Result<(), WorldError> {
        let i = self.lift_index(lift_id)?;
        let map = self
            .graph
            .map_idx(map_id)
            .ok_or_else(|| WorldError::UnknownMap(map_id.to_string()))?;
        let floor = self.lifts[i]
            .spec()
            .floors
            .iter()
            .position(|&m| m == map)
            .ok_or_else(|| WorldError::UnknownMap(map_id.to_string()))?;
        self.lifts[i]
            .call_door(floor, door)
            .map_err(WorldError::Lift)
    }

    /// Lift command port: the controller is done with the open door; close it.
    pub fn release_lift(&mut self, lift_id: &str) -> Result<(), WorldError> {
        let i = self.lift_index(lift_id)?;
        self.lifts[i].release();
        Ok(())
    }

    /// What lift `lift_id` looks like now.
    pub fn lift_status(&self, lift_id: &str) -> Result<LiftStatus, WorldError> {
        Ok(self.lifts[self.lift_index(lift_id)?].status())
    }

    /// What the lifts are like (floors, doors, stops), in the order they were
    /// added. A controller needs it to know which layout edges are lift rides.
    pub fn lift_specs(&self) -> Vec<LiftSpec> {
        self.lifts.iter().map(|l| l.spec().clone()).collect()
    }

    /// All lifts, in the order they were added.
    pub fn lifts(&self) -> Vec<LiftStatus> {
        self.lifts.iter().map(Lift::status).collect()
    }

    /// The map a floor of a lift stands for, for printing.
    pub fn lift_floor_map(&self, lift_id: &str, floor: usize) -> Result<&str, WorldError> {
        let l = &self.lifts[self.lift_index(lift_id)?];
        let map = l
            .spec()
            .floors
            .get(floor)
            .ok_or(WorldError::Lift(LiftError::UnknownFloor(floor)))?;
        Ok(&self.graph.maps[*map])
    }

    /// Set the zones of the run (already checked against the layout) **and give
    /// every robot, now and added later, that zone set already enabled**, as if
    /// the controller had sent and enabled it (a shortcut for tests and set-up;
    /// S62 added the real messages, see [`World::set_site_zones`]). The robots
    /// state RELEASE zone requests (S41); `set_zone_request_timing` changes when,
    /// per robot.
    pub fn set_zones(&mut self, zones: Zones) {
        for e in &mut self.robots {
            e.robot
                .set_zones(zones.clone(), ZoneRequestTiming::default());
        }
        self.zones = Some(zones);
        self.zones_on_robots = true;
    }

    /// Set the zone set of the site (already checked against the layout) **without**
    /// giving it to the robots (S62): the controller reads it (`zones()`), and
    /// must send it to each robot in a zone set message and enable it with the
    /// `enableZoneSet` instant action. Until then the robots have no zones and
    /// state no requests.
    pub fn set_site_zones(&mut self, zones: Zones) {
        self.zones = Some(zones);
        self.zones_on_robots = false;
    }

    /// When this robot requests a RELEASE zone (S41). Needs the zones to be set.
    pub fn set_zone_request_timing(
        &mut self,
        id: &RobotId,
        timing: ZoneRequestTiming,
    ) -> Result<(), WorldError> {
        let i = self.find(id)?;
        self.robots[i].robot.set_zone_request_timing(timing);
        Ok(())
    }

    /// Make the stations of the layout whose `stationType` is `charger` (any case)
    /// chargers with `power_w` watts: a robot with a battery standing on one of their
    /// interaction nodes charges after `startCharging`. Returns the number of
    /// charger nodes. Without this call there are no chargers.
    pub fn set_chargers(&mut self, power_w: f64) -> Result<usize, WorldError> {
        if !(power_w.is_finite() && power_w > 0.0) {
            return Err(WorldError::BadChargerPower(power_w));
        }
        let mut nodes: Vec<NodeIdx> = self
            .graph
            .stations
            .iter()
            .filter(|s| {
                s.station_type
                    .as_deref()
                    .is_some_and(|t| t.eq_ignore_ascii_case("charger"))
            })
            .flat_map(|s| s.interaction_nodes.iter().copied())
            .collect();
        nodes.sort_unstable();
        nodes.dedup();
        self.charger_power_w = Some(power_w);
        self.charger_nodes = nodes;
        self.refresh_chargers();
        Ok(self.charger_nodes.len())
    }

    /// A robot that stands on a charger node gets the charger's power.
    fn refresh_chargers(&mut self) {
        for e in &mut self.robots {
            e.robot.set_charger_power(self.charger_power_w.filter(|_| {
                let p = e.robot.pose();
                self.charger_nodes.iter().any(|&n| {
                    let n = &self.graph.nodes[n];
                    n.map == p.map
                        && (n.x - p.x).hypot(n.y - p.y) <= crate::robot::START_NODE_TOLERANCE
                })
            }));
        }
    }

    /// Battery figures of the run: `None` if no robot has a battery model.
    pub fn battery_report(&self) -> Option<BatteryReport> {
        let with: Vec<_> = self
            .robots
            .iter()
            .filter(|e| e.robot.battery().is_some())
            .collect();
        if with.is_empty() {
            return None;
        }
        Some(BatteryReport {
            robots: with.len(),
            ran_empty: with.iter().filter(|e| e.robot.ran_empty()).count(),
            empty_now: with.iter().filter(|e| e.robot.is_out_of_charge()).count(),
            charging_seconds: with.iter().map(|e| e.robot.charging_seconds()).sum(),
        })
    }

    /// Give a robot a battery (see [`crate::battery`]).
    pub fn set_battery(
        &mut self,
        id: &RobotId,
        battery: crate::battery::Battery,
    ) -> Result<(), WorldError> {
        let i = self.find(id)?;
        self.robots[i].robot.set_battery(battery);
        Ok(())
    }

    /// The charge of a robot in percent; `None` if it has no battery model.
    pub fn battery_percent(&self, id: &RobotId) -> Result<Option<f64>, WorldError> {
        let i = self.find(id)?;
        Ok(self.robots[i].robot.battery().map(|b| b.percent()))
    }

    /// The robot's zone requests as it states them now (its report says them too,
    /// with a message delay later).
    pub fn zone_requests_of(&self, id: &RobotId) -> Result<Vec<ZoneRequest>, WorldError> {
        Ok(self.robots[self.find(id)?].robot.zone_requests())
    }

    pub fn zones(&self) -> Option<&Zones> {
        self.zones.as_ref()
    }

    /// The controller tells the observer what it has reserved; the next
    /// snapshots carry it until it is published again. The simulator does not use it.
    pub fn publish_reservations(&mut self, reservations: Vec<Reservation>) {
        self.reservations = reservations;
    }

    /// The controller says, robot by robot (in the order they were added), why it
    /// holds the robot back and which robots cause it; `None` where it does not.
    /// It holds until published again. Once a controller has published, release,
    /// zone, lift and door waits come only from it (see [`crate::waiting`]).
    pub fn publish_held_back(&mut self, held_back: Vec<Option<HeldBack>>) {
        self.held_back = Some(held_back);
    }

    /// Who waited in the last step, why and for whom, robot by robot in the order
    /// they were added (see [`crate::waiting`]). Empty before the first step.
    pub fn waits(&self) -> &[Option<Wait>] {
        &self.waits
    }

    /// How long robots have waited so far, one entry per waiter, blocker, reason and
    /// place, in that order (see [`crate::waiting`]).
    pub fn wait_totals(&self) -> Vec<WaitTotal> {
        self.wait_ticks
            .iter()
            .map(|(&(waiter, blocker, reason, place), &ticks)| WaitTotal {
                waiter,
                blocker,
                reason,
                place,
                ticks,
            })
            .collect()
    }

    /// The wait of robot `i` as the waiting metric counts it: not while the robot
    /// is in its own stop (a random stop or an empty battery, S170).
    fn counted_wait(&self, i: usize) -> Option<Wait> {
        let robot = &self.robots.get(i)?.robot;
        let own_stop = robot.is_held() || robot.is_out_of_charge();
        self.waits.get(i).copied().flatten().filter(|_| !own_stop)
    }

    /// Where robot `i`, with the progress `now`, stands: on the node it was at or is
    /// going to if it is within [`START_NODE_TOLERANCE`] of it, else on the edge it
    /// drives; `None` if its order names neither.
    fn place_of(&self, i: usize, now: &Glance) -> Option<Place> {
        let p = self.robots[i].robot.pose();
        let last = now.last_node.map(|n| n.node);
        let next = now.remaining_nodes.first().map(|n| n.node);
        let on = [last, next].into_iter().flatten().find(|&n| {
            let node = &self.graph.nodes[n];
            node.map == p.map && (p.x - node.x).hypot(p.y - node.y) <= START_NODE_TOLERANCE
        });
        let edge = || now.remaining_edges.first().map(|e| e.edge);
        on.map(Place::Node)
            .or_else(|| edge().map(Place::Edge))
            .or_else(|| last.map(Place::Node))
    }

    /// The world as it is now.
    pub fn snapshot(&self) -> Snapshot {
        let robots = self
            .robots
            .iter()
            .enumerate()
            .map(|(i, e)| {
                let state = e.robot.report();
                let p = e.robot.pose();
                // (a robot with an empty battery stands still: it is held, also when
                // it was standing already)
                let status = match (
                    e.robot.is_held() || e.robot.is_out_of_charge(),
                    e.robot.phase(),
                ) {
                    (true, _) => RobotStatus::Held,
                    _ if e.robot.is_obstructed() => RobotStatus::Obstructed,
                    (_, Phase::Rotating) => RobotStatus::Turning,
                    (_, Phase::Driving) => RobotStatus::Driving,
                    (_, Phase::Acting) => RobotStatus::Acting,
                    (_, Phase::Waiting) => RobotStatus::WaitingForRelease,
                    (_, Phase::WaitingForClearance) => RobotStatus::WaitingForZone,
                    (_, Phase::WaitingForLift) => RobotStatus::WaitingForLift,
                    (_, Phase::Riding) => RobotStatus::RidingLift,
                    (_, Phase::Finished) => RobotStatus::Idle,
                };
                let mut snap = RobotSnapshot {
                    id: e.id.clone(),
                    pose: p,
                    shape: *e.robot.shape(),
                    status,
                    // along the heading (negative backwards); an omnidirectional robot
                    // that moves sideways: the speed over the ground
                    speed: state
                        .velocity
                        .as_ref()
                        .map_or(0.0, |v| if v.vy == 0.0 { v.vx } else { v.vx.hypot(v.vy) }),
                    order_id: state
                        .order_id
                        .as_ref()
                        .map(ToString::to_string)
                        .unwrap_or_default(),
                    // (a robot that reports no last node counts as at the node
                    // with the empty id, if a layout has one)
                    last_node: state
                        .route
                        .last_node()
                        .map(|n| n.node)
                        .or_else(|| self.graph.node_idx("")),
                    released_nodes: Vec::new(),
                    released_edges: Vec::new(),
                    planned_nodes: Vec::new(),
                    planned_edges: Vec::new(),
                    charge: e.robot.battery().map(|b| b.percent()),
                    charging: state.charging,
                    waiting: self.counted_wait(i),
                };
                for (at, ahead) in (0u32..).zip(state.route.ahead()) {
                    if at < state.route.released() {
                        snap.released_nodes.push(ahead.node);
                        snap.released_edges.push(ahead.edge);
                    } else {
                        snap.planned_nodes.push(ahead.node);
                        snap.planned_edges.push(ahead.edge);
                    }
                }
                snap
            })
            .collect();

        let mut overlapping: Vec<(usize, usize)> = self.touching.iter().copied().collect();
        overlapping.sort_unstable();
        let blocking = self
            .waits
            .iter()
            .enumerate()
            .filter_map(|(waiter, w)| {
                let blocker = w.and_then(|w| w.blocker)?;
                Some(Blocking { waiter, blocker })
            })
            .collect();
        let deadlocks = self
            .deadlocks
            .records()
            .iter()
            .filter(|r| r.ended_tick.is_none())
            .map(|r| r.robots.clone())
            .collect();

        Snapshot {
            tick: self.tick,
            time: self.time(),
            robots,
            reservations: self.reservations.clone(),
            blocking,
            overlapping,
            deadlocks,
            lifts: self.lifts.iter().map(Lift::status).collect(),
            doors: self.doors.iter().map(Door::status).collect(),
        }
    }

    /// Give the observer a snapshot if it is due (the snapshot is only built then).
    fn notify_observer(&mut self) {
        if !self.tick.is_multiple_of(self.observer_every) {
            return;
        }
        if let Some(mut o) = self.observer.take() {
            o.on_step(&self.snapshot());
            self.observer = Some(o);
        }
    }

    /// The body of robot `i` where it is now.
    fn body(&self, i: usize) -> Body {
        let e = &self.robots[i];
        let p = e.robot.pose();
        place(e.robot.shape(), p.x, p.y, p.heading)
    }

    /// The robot of `others` nearest to robot `i` (the smallest gap between the
    /// bodies, at equal gaps the one added first); `None` if there is none.
    fn nearest(&self, i: usize, others: &[usize]) -> Option<usize> {
        let mine = self.body(i);
        others
            .iter()
            .copied()
            .filter(|&j| j != i && j < self.robots.len())
            .map(|j| (gap(&mine, &self.body(j)), j))
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)))
            .map(|(_, j)| j)
    }

    /// Who waits, why and for whom (the rule is in [`crate::waiting`]), from what
    /// the robots' state reports say, read from the robots directly (no report is
    /// built); then look for deadlocks among these waits.
    fn check_deadlocks(&mut self) {
        // a robot in a lift cabin stands on no node (map = none)
        let poses: Vec<Pose> = self
            .robots
            .iter()
            .map(|e| {
                let mut p = e.robot.pose();
                if e.robot.is_riding() {
                    p.map = usize::MAX;
                }
                p
            })
            .collect();
        // Robots by 1 m grid cell (S52), so that "who stands on this node" looks at
        // a few cells and not at every robot. START_NODE_TOLERANCE is under 1 m.
        let cell = |v: f64| (v / 1.0).floor() as i64;
        let mut by_cell: HashMap<(usize, i64, i64), Vec<usize>> = HashMap::new();
        for (j, p) in poses.iter().enumerate() {
            by_cell
                .entry((p.map, cell(p.x), cell(p.y)))
                .or_default()
                .push(j);
        }
        let glances: Vec<_> = self.robots.iter().map(|e| e.robot.glance()).collect();
        // Who holds a node: the robot that has it released ahead of it (a reserved
        // node, S73). The lowest index if several do.
        let mut released_by: HashMap<NodeIdx, usize> = HashMap::new();
        for (j, g) in glances.iter().enumerate().rev() {
            for n in g.remaining_nodes.iter().filter(|n| n.released) {
                released_by.insert(n.node, j);
            }
        }
        let waits: Vec<Option<Wait>> = self
            .robots
            .iter()
            .enumerate()
            .map(|(i, e)| {
                let now = &glances[i];
                // a robot stopped by its protective field waits for the robot in the
                // field (S54, S76)
                if e.robot.is_obstructed()
                    && let Some(j) = self.field_blocker.get(i).copied().flatten()
                {
                    return Some(Wait {
                        reason: WaitReason::SafetyStop,
                        blocker: Some(j),
                    });
                }
                let next = now.remaining_nodes.first()?;
                if !stands_waiting(now.driving, now.held, now.acting) {
                    return None;
                }
                // with a controller, its word; without, the two rules below
                if let Some(held_back) = &self.held_back {
                    let held = held_back.get(i)?.as_ref()?;
                    return Some(Wait {
                        reason: held.reason,
                        blocker: self.nearest(i, &held.blockers),
                    });
                }
                let node = &self.graph.nodes[next.node];
                // the robot with the lowest index on the node (within the tolerance)
                let (cx, cy) = (cell(node.x), cell(node.y));
                let standing = (-1..=1)
                    .flat_map(|dx| (-1..=1).map(move |dy| (dx, dy)))
                    .flat_map(|(dx, dy)| {
                        by_cell
                            .get(&(node.map, cx + dx, cy + dy))
                            .into_iter()
                            .flatten()
                    })
                    .copied()
                    .filter(|&j| {
                        j != i
                            && (poses[j].x - node.x).hypot(poses[j].y - node.y)
                                <= START_NODE_TOLERANCE
                    })
                    .min();
                // else the node is reserved, though empty: its holder has it released
                let blocker =
                    standing.or_else(|| released_by.get(&next.node).copied().filter(|&j| j != i));
                blocker.map(|j| Wait {
                    reason: WaitReason::Release,
                    blocker: Some(j),
                })
            })
            .collect();
        let waits_for: Vec<Option<usize>> =
            waits.iter().map(|w| w.and_then(|w| w.blocker)).collect();
        self.deadlocks.observe(self.tick, &waits_for);
        // add the waits up, but not those of a robot in its own stop (S170)
        self.waits = waits;
        for (i, now) in glances.iter().enumerate() {
            if let Some(w) = self.counted_wait(i)
                && let Some(place) = self.place_of(i, now)
            {
                let key = (i, w.blocker, w.reason, place);
                *self.wait_ticks.entry(key).or_insert(0) += 1;
            }
        }
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
        // a robot in a lift cabin is not on any floor: it touches nobody
        let bodies: Vec<Option<(usize, Body)>> = self
            .robots
            .iter()
            .map(|e| {
                let p = e.robot.pose();
                (!e.robot.is_riding()).then(|| (p.map, place(e.robot.shape(), p.x, p.y, p.heading)))
            })
            .collect();
        // Broad phase (S52): an `aabb` index of the bounding boxes, rebuilt every
        // step; only robots whose boxes meet go to the narrow phase. The boxes are
        // padded a little so that touching bodies (an overlap) are never missed.
        const PAD: f64 = 1e-6;
        self.broad.clear();
        let mut robot_of: Vec<usize> = Vec::new(); // tree id → robot index
        let mut boxes: Vec<(f64, f64, f64, f64)> = Vec::new(); // by robot index
        for (i, b) in bodies.iter().enumerate() {
            let (x0, y0, x1, y1) = match b {
                Some((_, Body::Rect(r))) => {
                    let b = orect_bounding_rect(r);
                    (b.p1.x, b.p1.y, b.p2.x, b.p2.y)
                }
                Some((_, Body::Circle(c))) => {
                    let b = circle_bounding_rect(c);
                    (b.p1.x, b.p1.y, b.p2.x, b.p2.y)
                }
                None => {
                    boxes.push((0.0, 0.0, 0.0, 0.0));
                    continue;
                }
            };
            let bx = (x0 - PAD, y0 - PAD, x1 + PAD, y1 + PAD);
            boxes.push(bx);
            self.broad.add(bx.0, bx.1, bx.2, bx.3);
            robot_of.push(i);
        }
        self.broad.build();

        let mut now = HashSet::new();
        let mut hits = Vec::new();
        let mut partners: Vec<usize> = Vec::new();
        for &i in &robot_of {
            let (Some(a), bx) = (&bodies[i], boxes[i]) else {
                continue;
            };
            self.broad
                .query_intersecting(bx.0, bx.1, bx.2, bx.3, &mut hits);
            // only later robots, in order: the same pairs in the same order as
            // checking every pair
            partners.clear();
            partners.extend(hits.iter().map(|&h| robot_of[h]).filter(|&j| j > i));
            partners.sort_unstable();
            for &j in &partners {
                let Some(b) = &bodies[j] else {
                    continue;
                };
                if a.0 == b.0 && overlap(&a.1, &b.1) {
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

        // Protective fields (S54), with the same index and bodies: a robot whose
        // field meets another robot's body brakes in the next step.
        self.field_blocker = vec![None; self.robots.len()];
        if self.safety_stop {
            for i in 0..self.robots.len() {
                let field = self.robots[i].robot.protective_field(self.dt);
                let blocker = match (field, &bodies[i]) {
                    (Some(f), Some((map, own))) => {
                        let r = orect_bounding_rect(&f);
                        self.broad.query_intersecting(
                            r.p1.x - PAD,
                            r.p1.y - PAD,
                            r.p2.x + PAD,
                            r.p2.y + PAD,
                            &mut hits,
                        );
                        let field = Body::Rect(f);
                        // the nearest robot in the field (at equal gaps the one
                        // added first)
                        hits.iter()
                            .map(|&h| robot_of[h])
                            .filter(|&j| j != i)
                            .filter_map(|j| {
                                let (m, b) = bodies[j].as_ref()?;
                                (m == map && overlap(&field, b)).then(|| (gap(own, b), j))
                            })
                            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)))
                            .map(|(_, j)| j)
                    }
                    _ => None,
                };
                self.field_blocker[i] = blocker;
                self.robots[i].robot.set_obstructed(blocker.is_some());
            }
            // count the safety stops that begin
            let gap = (SAFETY_STOP_GAP_SECONDS / self.dt).round() as u64;
            for e in &mut self.robots {
                if e.robot.is_obstructed() {
                    if e.obstructed_tick
                        .is_none_or(|t| self.tick.saturating_sub(t) > gap)
                    {
                        self.safety_stops += 1;
                    }
                    e.obstructed_tick = Some(self.tick);
                }
            }
        }
    }

    /// Send a message to the robot its header names: an order, instant actions,
    /// responses or a zone set (S62: the robot stores it disabled, and the
    /// `enableZoneSet` instant action enables it).
    ///
    /// For an order the robot's answer is returned: without a message delay at
    /// once, with one [`Reception::InTransit`]. `None` for the other messages.
    pub fn send(&mut self, message: &ToRobot) -> Result<Option<Reception>, WorldError> {
        let i = self.numbered(message.header.robot)?;
        if self.message_delay > 0 {
            self.queue(i, Delivery::Message(message.body.clone()));
            let order = matches!(message.body, ToRobotBody::Order(_));
            return Ok(order.then_some(Reception::InTransit));
        }
        Ok(self.deliver(i, &message.body))
    }

    /// Something sent to robot `robot` could not be read (see
    /// [`Robot::take_unread`]). It takes the way of a message: with a message
    /// delay it arrives later, in its turn.
    pub fn send_unread(&mut self, robot: RobotNo, unread: Unread) -> Result<(), WorldError> {
        let i = self.numbered(robot)?;
        if self.message_delay > 0 {
            self.queue(i, Delivery::Unread(unread));
        } else {
            self.deliver_unread(i, &unread);
        }
        Ok(())
    }

    /// Put what is sent to robot `i` on its way.
    fn queue(&mut self, i: usize, delivery: Delivery) {
        let due = self.tick.saturating_add(self.message_delay);
        self.inbox.push_back((due, i, delivery));
    }

    /// A message arrives at robot `i`; the answer to an order is returned.
    fn deliver(&mut self, i: usize, body: &ToRobotBody) -> Option<Reception> {
        let now_ms = self.now_ms();
        let robot = &mut self.robots[i].robot;
        match body {
            ToRobotBody::Order(order) => return Some(robot.take_order(&self.graph, order)),
            ToRobotBody::InstantActions(actions) => {
                robot.take_instant_actions(&self.graph, actions);
            }
            ToRobotBody::Responses(responses) => {
                robot.set_time(now_ms);
                robot.take_responses(responses);
            }
            ToRobotBody::ZoneSet(zone_set) => {
                robot.set_time(now_ms);
                robot.take_zone_set(&self.graph, zone_set);
            }
        }
        None
    }

    /// Something that could not be read arrives at robot `i`.
    fn deliver_unread(&mut self, i: usize, unread: &Unread) {
        let now_ms = self.now_ms();
        let robot = &mut self.robots[i].robot;
        if matches!(unread, Unread::ZoneSet(_)) {
            // (how long its error is reported is counted from now)
            robot.set_time(now_ms);
        }
        robot.take_unread(&self.graph, unread);
    }

    /// What the robot is doing now (turning, driving, waiting for a lift, …).
    pub fn phase(&self, id: &RobotId) -> Result<Phase, WorldError> {
        Ok(self.robots[self.find(id)?].robot.phase())
    }

    /// The number of the robot `id`.
    pub fn robot_no(&self, id: &RobotId) -> Option<RobotNo> {
        self.index(id).and_then(number)
    }

    /// The state report of robot `robot` as the controller gets it: the state now,
    /// or with a message delay ([`World::set_message_delay`]) the one the robot
    /// sent that many steps ago (before the first one has arrived, the first
    /// one). Every call makes a new report with the next sequence number.
    pub fn report(&mut self, robot: RobotNo) -> Result<FromRobot, WorldError> {
        let i = self.numbered(robot)?;
        Ok(self.report_of(i, robot))
    }

    /// The report of every robot, in the order they were added (see
    /// [`World::report`]). Call it once in every step: with a message delay a
    /// report is on its way from the call that made it.
    pub fn reports(&mut self) -> Vec<FromRobot> {
        (0..self.robots.len())
            .filter_map(|i| Some((i, number(i)?)))
            .map(|(i, robot)| self.report_of(i, robot))
            .collect()
    }

    /// The report of robot `i`, whose number is `robot`.
    fn report_of(&mut self, i: usize, robot: RobotNo) -> FromRobot {
        let time = Millis(self.now_ms());
        let (tick, delay) = (self.tick, self.message_delay);
        let entry = &mut self.robots[i];
        let header = Header {
            robot,
            seq: entry.next_header_id,
            time,
        };
        entry.next_header_id = entry.next_header_id.wrapping_add(1);
        let now = FromRobot {
            header,
            state: entry.robot.report(),
        };
        if delay == 0 {
            return now;
        }
        let due = tick.saturating_sub(delay);
        let seen = &mut entry.seen;
        seen.push_back((tick, now));
        // keep the newest report that has arrived by now (made at or before `due`)
        while seen.len() >= 2 && seen[1].0 <= due {
            seen.pop_front();
        }
        seen[0].1.clone()
    }

    fn index(&self, id: &RobotId) -> Option<usize> {
        self.robots.iter().position(|e| e.id == *id)
    }

    fn find(&self, id: &RobotId) -> Result<usize, WorldError> {
        self.index(id)
            .ok_or_else(|| WorldError::UnknownRobot(id.clone()))
    }

    /// The index of the robot with the number `no`.
    fn numbered(&self, no: RobotNo) -> Result<usize, WorldError> {
        usize::try_from(no.slot())
            .ok()
            .filter(|&i| i < self.robots.len() && no.generation() == 0)
            .ok_or(WorldError::UnknownRobotNo(no))
    }
}

/// The number of robot `i`.
fn number(i: usize) -> Option<RobotNo> {
    u32::try_from(i).ok().map(|slot| RobotNo::new(slot, 0))
}

fn door_phase(state: DoorState) -> DoorPhase {
    match state {
        DoorState::Closed => DoorPhase::Closed,
        DoorState::Opening => DoorPhase::Opening,
        DoorState::Open => DoorPhase::Open,
        DoorState::Closing => DoorPhase::Closing,
    }
}

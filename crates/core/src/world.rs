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

use std::collections::{HashMap, HashSet};
use std::fmt;

use vda5050::instant_actions::InstantActions;
use vda5050::order::Order;
use vda5050::state::State;

use crate::actions::ActionTimes;
use crate::collision::{Body, overlap, place};
use crate::deadlock::{DEFAULT_TIMEOUT_SECONDS, DeadlockDetector};
use crate::disturbances::{DisturbanceError, StopGenerator, StopSpec, check_duration_variation};
use crate::door::{Door, DoorError, DoorSpec, DoorStatus};
use crate::drive::Phase;
use crate::drive::{DriveError, Pose};
use crate::graph::{EdgeIdx, Graph, NodeIdx};
use crate::jobs::Rng;
use crate::lift::{Lift, LiftError, LiftSpec, LiftStatus};
use crate::metrics::is_waiting;
use crate::observer::{Blocking, Observer, Reservation, RobotSnapshot, RobotStatus, Snapshot};
use crate::robot::{Reception, Robot, START_NODE_TOLERANCE};
use crate::spec::RobotSpec;
use crate::state_report::Header;
use crate::zone_requests::ZoneRequestTiming;
use crate::zones::Zones;

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
    /// Wrong disturbance settings.
    Disturbance(DisturbanceError),
    /// The observer interval must be a positive, finite number of seconds.
    BadObserverInterval(f64),
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
            WorldError::Start(e) => write!(f, "cannot place the robot: {e:?}"),
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
    /// Random stops of this robot, if disturbances are on.
    stops: Option<StopGenerator>,
}

#[derive(Debug)]
pub struct World {
    /// Seed and spec of the random stops; robots added later get them too.
    stops: Option<(u64, StopSpec)>,
    /// How many random stops started so far.
    stops_started: usize,
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
    /// Who waited for whom after the last step.
    waits_for: Vec<Option<usize>>,
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
    /// Passage width of the doors by edge id (for robots added later).
    door_widths: HashMap<String, f64>,
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
            duration_variation: None,
            waits_for: Vec::new(),
            reservations: Vec::new(),
            observer: None,
            observer_every: 1,
            zones: None,
            lifts: Vec::new(),
            rides: HashSet::new(),
            doors: Vec::new(),
            door_of_edge: HashMap::new(),
            door_widths: HashMap::new(),
        })
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
        let mut robot =
            Robot::new(&self.graph, spec, times, start, heading).map_err(WorldError::Start)?;
        robot.set_time(self.now_ms());
        robot.set_rides(self.rides.clone());
        robot.set_door_widths(self.door_widths.clone());
        if let Some(zones) = &self.zones {
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
                    .expect("checked when the stops were enabled"),
            ),
            None => None,
        };
        self.robots.push(Entry {
            id,
            robot,
            next_header_id: 0,
            stops,
        });
        Ok(())
    }

    /// Turn on random stops (S37) for all robots, now and added later; the
    /// seed makes them repeatable. Call it before the run.
    pub fn enable_random_stops(&mut self, seed: u64, spec: StopSpec) -> Result<(), WorldError> {
        spec.check(self.dt).map_err(WorldError::Disturbance)?;
        self.stops = Some((seed, spec));
        for (i, e) in self.robots.iter_mut().enumerate() {
            e.stops = Some(StopGenerator::new(seed, i, spec, self.dt).expect("checked just above"));
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
        let now_ms = self.now_ms();
        self.step_doors();
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
        self.set_observer_at(observer, self.dt)
            .expect("the time step is a valid interval");
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
                self.door_widths.insert(self.graph.edges[e].id.clone(), w);
            }
        }
        for e in &mut self.robots {
            e.robot.set_door_widths(self.door_widths.clone());
        }
        self.doors.push(door);
        Ok(())
    }

    /// Move the doors one step and tell the robots which door edges they must
    /// not start: a door closes only while no robot is on one of its edges.
    fn step_doors(&mut self) {
        if self.doors.is_empty() {
            return;
        }
        let mut occupied = vec![false; self.doors.len()];
        for e in &self.robots {
            if let Some(id) = e.robot.driving_edge_id()
                && let Some(edge) = self.graph.edge_idx(id)
                && let Some(&d) = self.door_of_edge.get(&edge)
            {
                occupied[d] = true;
            }
        }
        for (d, door) in self.doors.iter_mut().enumerate() {
            door.step(self.dt, occupied[d]);
        }
        let closed: HashSet<String> = self
            .doors
            .iter()
            .filter(|d| !d.is_open())
            .flat_map(|d| {
                d.spec()
                    .edges
                    .iter()
                    .map(|&e| self.graph.edges[e].id.clone())
            })
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
        Ok(self
            .robots
            .iter()
            .find(|r| {
                r.robot.is_riding()
                    && r.robot
                        .ride_edge()
                        .is_some_and(|e| self.lift_of_ride(e) == Some(li))
            })
            .map(|r| &r.id))
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

    /// Set the zones of the run (already checked against the layout). They are
    /// only stored for now; no robot behaviour uses them yet.
    pub fn set_zones(&mut self, zones: Zones) {
        // every robot, now and added later, states RELEASE zone requests (S41);
        // `set_zone_request_timing` changes when, per robot
        for e in &mut self.robots {
            e.robot
                .set_zones(zones.clone(), ZoneRequestTiming::default());
        }
        self.zones = Some(zones);
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

    /// The robot's zone requests as it states them now.
    pub fn zone_requests(
        &self,
        id: &RobotId,
    ) -> Result<Vec<vda5050::state::ZoneRequest>, WorldError> {
        Ok(self.robots[self.find(id)?].robot.zone_requests())
    }

    pub fn zones(&self) -> Option<&Zones> {
        self.zones.as_ref()
    }

    /// The controller tells the observer what it has reserved; the next
    /// snapshots carry it until it is published again. Core does not use it.
    pub fn publish_reservations(&mut self, reservations: Vec<Reservation>) {
        self.reservations = reservations;
    }

    /// The world as it is now.
    pub fn snapshot(&self) -> Snapshot {
        let header = plain_header();
        let robots = self
            .robots
            .iter()
            .map(|e| {
                let state = e.robot.state(&self.graph, &header);
                let p = e.robot.pose();
                let status = match (e.robot.is_held(), e.robot.phase()) {
                    (true, _) => RobotStatus::Held,
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
                    speed: state.velocity.and_then(|v| v.vx).unwrap_or(0.0),
                    order_id: state.order_id.clone(),
                    last_node: self.graph.node_idx(&state.last_node_id),
                    released_nodes: Vec::new(),
                    released_edges: Vec::new(),
                    planned_nodes: Vec::new(),
                    planned_edges: Vec::new(),
                };
                for n in &state.node_states {
                    if let Some(i) = self.graph.node_idx(&n.node_id) {
                        if n.released {
                            snap.released_nodes.push(i);
                        } else {
                            snap.planned_nodes.push(i);
                        }
                    }
                }
                for ed in &state.edge_states {
                    if let Some(i) = self.graph.edge_idx(&ed.edge_id) {
                        if ed.released {
                            snap.released_edges.push(i);
                        } else {
                            snap.planned_edges.push(i);
                        }
                    }
                }
                snap
            })
            .collect();

        let mut overlapping: Vec<(usize, usize)> = self.touching.iter().copied().collect();
        overlapping.sort_unstable();
        let blocking = self
            .waits_for
            .iter()
            .enumerate()
            .filter_map(|(waiter, b)| b.map(|blocker| Blocking { waiter, blocker }))
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

    /// Who waits for whom, from the robots' state messages (the rule is in
    /// [`crate::deadlock`]).
    fn check_deadlocks(&mut self) {
        let header = plain_header();
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
        let waits_for: Vec<Option<usize>> = self
            .robots
            .iter()
            .enumerate()
            .map(|(i, e)| {
                let state = e.robot.state(&self.graph, &header);
                let next = state.node_states.first()?;
                if !is_waiting(&state) {
                    return None;
                }
                let node = &self.graph.nodes[self.graph.node_idx(&next.node_id)?];
                (0..poses.len()).find(|&j| {
                    j != i
                        && poses[j].map == node.map
                        && (poses[j].x - node.x).hypot(poses[j].y - node.y) <= START_NODE_TOLERANCE
                })
            })
            .collect();
        self.deadlocks.observe(self.tick, &waits_for);
        self.waits_for = waits_for;
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
        let mut now = HashSet::new();
        for i in 0..bodies.len() {
            for j in i + 1..bodies.len() {
                let (Some(a), Some(b)) = (&bodies[i], &bodies[j]) else {
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
    }

    /// Deliver an `order`; the robot is the one named in the message.
    pub fn send_order(&mut self, order: &Order) -> Result<Reception, WorldError> {
        let id = RobotId::new(&order.manufacturer, &order.serial_number);
        let i = self.find(&id)?;
        Ok(self.robots[i].robot.receive_order(&self.graph, order))
    }

    /// Deliver a `responses` message (answers to the robot's zone requests).
    pub fn send_responses(
        &mut self,
        msg: &vda5050::responses::Responses,
    ) -> Result<(), WorldError> {
        let id = RobotId::new(&msg.manufacturer, &msg.serial_number);
        let i = self.find(&id)?;
        let now_ms = self.now_ms();
        self.robots[i].robot.set_time(now_ms);
        self.robots[i].robot.receive_responses(msg);
        Ok(())
    }

    /// Deliver an `instantActions` message.
    pub fn send_instant_actions(&mut self, msg: &InstantActions) -> Result<(), WorldError> {
        let id = RobotId::new(&msg.manufacturer, &msg.serial_number);
        let i = self.find(&id)?;
        self.robots[i].robot.receive_instant_actions(msg);
        Ok(())
    }

    /// What the robot is doing now (turning, driving, waiting for a lift, …).
    pub fn phase(&self, id: &RobotId) -> Result<Phase, WorldError> {
        Ok(self.robots[self.find(id)?].robot.phase())
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

/// Milliseconds since the Unix epoch from an ISO 8601 UTC timestamp such as
/// `2026-11-20T10:00:00.250Z` (the milliseconds are optional). `None` if the text
/// is not of that form or a field is out of range.
pub fn unix_ms_from_iso8601(s: &str) -> Option<i64> {
    let s = s.strip_suffix('Z')?;
    let (date, time) = s.split_once('T')?;
    let mut d = date.split('-');
    let (year, month, day) = (
        d.next()?.parse::<i64>().ok()?,
        d.next()?.parse::<i64>().ok()?,
        d.next()?.parse::<i64>().ok()?,
    );
    if d.next().is_some() {
        return None;
    }
    let (hms, frac) = match time.split_once('.') {
        Some((a, b)) => (a, Some(b)),
        None => (time, None),
    };
    let mut t = hms.split(':');
    let (h, m, sec) = (
        t.next()?.parse::<i64>().ok()?,
        t.next()?.parse::<i64>().ok()?,
        t.next()?.parse::<i64>().ok()?,
    );
    if t.next().is_some() {
        return None;
    }
    let ms = match frac {
        None => 0,
        Some(f) if !f.is_empty() && f.len() <= 9 && f.bytes().all(|c| c.is_ascii_digit()) => {
            // the first three digits are the milliseconds
            let padded = format!("{f:0<3}");
            padded[..3].parse::<i64>().ok()?
        }
        Some(_) => return None,
    };
    let valid = (1..=12).contains(&month)
        && (1..=31).contains(&day)
        && (0..=23).contains(&h)
        && (0..=59).contains(&m)
        && (0..=60).contains(&sec);
    valid.then(|| unix_ms_from_utc(year, month, day, h, m, sec) + ms)
}

/// A header for state messages that are only read inside the simulator.
fn plain_header() -> Header {
    Header {
        header_id: 0,
        timestamp: String::new(),
        version: PROTOCOL_VERSION.to_string(),
        manufacturer: String::new(),
        serial_number: String::new(),
    }
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

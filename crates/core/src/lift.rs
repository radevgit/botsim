//! Lifts (elevators) as **peripherals** of the simulated world. They are not
//! robots and not part of VDA 5050 (the standard leaves talking to lifts and doors
//! to the fleet control), so a lift has its **own small command port** next to the
//! VDA 5050 messages.
//!
//! Robots ride a lift through its **stops** (S46): see [`LiftStop`] and
//! `docs/rules.md` §7g. The world, not this type, moves the robots in and out.
//!
//! **Model.** A lift has a list of **floors** (the maps it stops at, in physical
//! order) and one cabin with one or more **doors** (S45: a through-cabin has a
//! front and a rear door). Every door can open at every floor; **only one door is
//! open at a time**.
//! - Cabin: moves between floors at `travel_seconds_per_floor` per floor passed.
//! - Door: closed → opening (`door_open_seconds`) → open → closing
//!   (`door_close_seconds`) → closed. The cabin only moves with the door closed.
//!   All doors have the same times.
//!
//! **Command port** ([`Lift::call`], [`Lift::call_door`], [`Lift::release`]):
//! - `call_door(floor, door)`: bring the cabin to `floor` and open `door` there.
//!   `call(floor)` is the same with the first door (the only one of most lifts).
//!   Calls are served **in the order they came** (first come first served). A call
//!   for the floor and door that are open or opening, with nothing queued, needs
//!   nothing; a call for the floor and door the cabin is going to, or the same as
//!   the last queued call, is not queued twice. Another door at the same floor is
//!   a new call: the open door has to be closed first.
//! - `release()`: "I am done here": a **door that is open** is closed. It is how
//!   the controller says that the robots have got in or out. Without it the door
//!   stays open and later calls wait.
//! - Option `door_hold_seconds`: the door closes by itself after being open that
//!   long (then the controller does not need `release`). `None`: it stays open until
//!   released.
//!
//! The time is quantised to the simulation step, like the actions of a robot: a
//! stage that has a few steps left is over when its time is used up.
//!
//! Not yet: more than one robot in the cabin (capacity), walking in and out
//! taking time, several cabins in one shaft.

use std::collections::VecDeque;
use std::fmt;

use crate::actions::TIME_EPSILON;
use crate::graph::{MapIdx, NodeIdx};

/// What a lift is like.
#[derive(Debug, Clone, PartialEq)]
pub struct LiftSpec {
    pub id: String,
    /// The maps the lift stops at, lowest floor first; floor `i` is `floors[i]`.
    pub floors: Vec<MapIdx>,
    /// The names of the cabin's doors (e.g. `front`, `rear`); at least one, all
    /// different. Door `i` is `doors[i]`.
    pub doors: Vec<String>,
    /// Where robots get in and out (S46): the node of the layout at the cabin
    /// entrance, with the floor and the door that belong to it. A layout edge
    /// between two stops of the lift on different floors is a **ride**. Without
    /// stops the lift can be called but no robot can ride it.
    pub stops: Vec<LiftStop>,
    /// The floor the cabin starts at (index into `floors`).
    pub start_floor: usize,
    /// Seconds for the cabin to pass one floor.
    pub travel_seconds_per_floor: f64,
    pub door_open_seconds: f64,
    pub door_close_seconds: f64,
    /// The door closes by itself after being open this long; `None` = only on `release`.
    pub door_hold_seconds: Option<f64>,
}

/// A place where robots get into or out of the cabin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LiftStop {
    /// The layout node at the cabin entrance (on the map of `floor`).
    pub node: NodeIdx,
    /// Index into the lift's floors.
    pub floor: usize,
    /// Index into the lift's doors: the door that opens here.
    pub door: usize,
    /// A waiting node (lay-by) for robots that wait for the lift, on the same
    /// map as `node` and off the lane that leads to it (S49). The simulator does
    /// not use it; a controller can (the baseline routes robots through it and
    /// parks them there while the lift is busy).
    pub waiting: Option<NodeIdx>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum LiftError {
    /// A stop names a floor or door the lift does not have, or two stops share a
    /// node.
    BadStop(usize),
    /// A lift needs at least two floors, all different.
    BadFloors,
    /// A lift needs at least one door; the names must be different.
    BadDoors,
    /// `start_floor` is not a floor of the lift.
    BadStartFloor(usize),
    /// A time must be a finite number >= 0 (`door_hold_seconds` > 0); the text
    /// names the setting.
    BadTime(&'static str),
    /// A call for a floor the lift does not have.
    UnknownFloor(usize),
    /// A call for a door the lift does not have.
    UnknownDoor(usize),
}

impl fmt::Display for LiftError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LiftError::BadFloors => write!(f, "a lift needs at least two different floors"),
            LiftError::BadDoors => {
                write!(f, "a lift needs at least one door, with different names")
            }
            LiftError::UnknownDoor(i) => write!(f, "the lift has no door {i}"),
            LiftError::BadStop(i) => write!(
                f,
                "stop {i} names a floor or door the lift does not have, a node used twice, or a bad waiting node"
            ),
            LiftError::BadStartFloor(i) => write!(f, "start floor {i} is not a floor of the lift"),
            LiftError::BadTime(what) => write!(f, "{what} must be a finite number >= 0"),
            LiftError::UnknownFloor(i) => write!(f, "the lift has no floor {i}"),
        }
    }
}

impl std::error::Error for LiftError {}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DoorState {
    Closed,
    Opening,
    Open,
    Closing,
}

/// A call: the cabin to `floor`, open door `door` (indices into the lift's
/// floors and doors).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Call {
    pub floor: usize,
    pub door: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    /// Doors closed, standing at `floor`.
    Idle,
    Moving {
        from: usize,
        to: usize,
        /// The door to open on arrival.
        door: usize,
        left: f64,
        total: f64,
    },
    Opening {
        left: f64,
    },
    Open {
        /// How long the door has been open.
        for_seconds: f64,
    },
    Closing {
        left: f64,
    },
}

/// What a lift looks like from outside (for the controller and the observer).
#[derive(Debug, Clone, PartialEq)]
pub struct LiftStatus {
    pub id: String,
    /// The floor the cabin is at, or the one it left while it is moving.
    pub floor: usize,
    /// The cabin's place in floors (1.5 = half way between floors 1 and 2).
    pub position: f64,
    pub door: DoorState,
    /// The door that is open, opening or closing; while the door is closed, the
    /// one that was used last (the first door at the start).
    pub door_index: usize,
    /// The name of that door.
    pub door_id: String,
    /// The floor the cabin is moving to.
    pub moving_to: Option<usize>,
    /// Calls that are waiting, oldest first.
    pub queue: Vec<Call>,
}

#[derive(Debug, Clone)]
pub struct Lift {
    spec: LiftSpec,
    floor: usize,
    /// The door that is open, opening or closing, or was used last.
    door: usize,
    phase: Phase,
    queue: VecDeque<Call>,
}

fn time_ok(v: f64) -> bool {
    v.is_finite() && v >= 0.0
}

impl Lift {
    pub fn new(spec: LiftSpec) -> Result<Lift, LiftError> {
        let n = spec.floors.len();
        let distinct = spec
            .floors
            .iter()
            .enumerate()
            .all(|(i, f)| !spec.floors[..i].contains(f));
        if n < 2 || !distinct {
            return Err(LiftError::BadFloors);
        }
        let doors_ok = !spec.doors.is_empty()
            && spec
                .doors
                .iter()
                .enumerate()
                .all(|(i, d)| !spec.doors[..i].contains(d));
        if !doors_ok {
            return Err(LiftError::BadDoors);
        }
        for (i, s) in spec.stops.iter().enumerate() {
            let twice = spec.stops[..i].iter().any(|o| o.node == s.node);
            if s.floor >= n || s.door >= spec.doors.len() || twice {
                return Err(LiftError::BadStop(i));
            }
        }
        if spec.start_floor >= n {
            return Err(LiftError::BadStartFloor(spec.start_floor));
        }
        if !time_ok(spec.travel_seconds_per_floor) {
            return Err(LiftError::BadTime("travel_seconds_per_floor"));
        }
        if !time_ok(spec.door_open_seconds) {
            return Err(LiftError::BadTime("door_open_seconds"));
        }
        if !time_ok(spec.door_close_seconds) {
            return Err(LiftError::BadTime("door_close_seconds"));
        }
        if spec
            .door_hold_seconds
            .is_some_and(|h| !(h.is_finite() && h > 0.0))
        {
            return Err(LiftError::BadTime("door_hold_seconds"));
        }
        Ok(Lift {
            floor: spec.start_floor,
            door: 0,
            spec,
            phase: Phase::Idle,
            queue: VecDeque::new(),
        })
    }

    pub fn spec(&self) -> &LiftSpec {
        &self.spec
    }

    /// The stop at layout node `node`, if the lift has one there.
    pub fn stop_at(&self, node: NodeIdx) -> Option<LiftStop> {
        self.spec.stops.iter().copied().find(|s| s.node == node)
    }

    /// Is the door open at `stop` right now: the cabin is at its floor and the
    /// door of the stop is fully open (not opening or closing)?
    pub fn open_at(&self, stop: LiftStop) -> bool {
        matches!(self.phase, Phase::Open { .. })
            && self.floor == stop.floor
            && self.door == stop.door
    }

    pub fn status(&self) -> LiftStatus {
        let (door, moving_to, position) = match self.phase {
            Phase::Idle => (DoorState::Closed, None, self.floor as f64),
            Phase::Moving {
                from,
                to,
                left,
                total,
                ..
            } => {
                let done = if total > 0.0 { 1.0 - left / total } else { 1.0 };
                (
                    DoorState::Closed,
                    Some(to),
                    from as f64 + (to as f64 - from as f64) * done,
                )
            }
            Phase::Opening { .. } => (DoorState::Opening, None, self.floor as f64),
            Phase::Open { .. } => (DoorState::Open, None, self.floor as f64),
            Phase::Closing { .. } => (DoorState::Closing, None, self.floor as f64),
        };
        LiftStatus {
            id: self.spec.id.clone(),
            floor: self.floor,
            position,
            door,
            door_index: self.door,
            door_id: self.spec.doors[self.door].clone(),
            moving_to,
            queue: self.queue.iter().copied().collect(),
        }
    }

    /// Command port: bring the cabin to `floor` and open the first door there.
    pub fn call(&mut self, floor: usize) -> Result<(), LiftError> {
        self.call_door(floor, 0)
    }

    /// Command port: bring the cabin to `floor` and open door `door` there.
    pub fn call_door(&mut self, floor: usize, door: usize) -> Result<(), LiftError> {
        if floor >= self.spec.floors.len() {
            return Err(LiftError::UnknownFloor(floor));
        }
        if door >= self.spec.doors.len() {
            return Err(LiftError::UnknownDoor(door));
        }
        let call = Call { floor, door };
        let satisfied = self.queue.is_empty()
            && match self.phase {
                Phase::Open { .. } | Phase::Opening { .. } => {
                    self.floor == floor && self.door == door
                }
                Phase::Moving { to, door: d, .. } => to == floor && d == door,
                _ => false,
            };
        if satisfied || self.queue.back() == Some(&call) {
            return Ok(());
        }
        self.queue.push_back(call);
        Ok(())
    }

    /// Command port: the controller is done with the open door; close it. Nothing
    /// happens if the door is not open.
    pub fn release(&mut self) {
        if let Phase::Open { .. } = self.phase {
            self.begin_closing();
        }
    }

    fn begin_closing(&mut self) {
        self.phase = if self.spec.door_close_seconds <= TIME_EPSILON {
            Phase::Idle
        } else {
            Phase::Closing {
                left: self.spec.door_close_seconds,
            }
        };
    }

    fn begin_opening(&mut self, door: usize) {
        self.door = door;
        self.phase = if self.spec.door_open_seconds <= TIME_EPSILON {
            Phase::Open { for_seconds: 0.0 }
        } else {
            Phase::Opening {
                left: self.spec.door_open_seconds,
            }
        };
    }

    /// The door is closed and the cabin stands: take the next call.
    fn start_next(&mut self) {
        let Some(next) = self.queue.pop_front() else {
            return;
        };
        if next.floor == self.floor {
            self.begin_opening(next.door);
        } else {
            let total =
                self.spec.travel_seconds_per_floor * (next.floor as f64 - self.floor as f64).abs();
            self.phase = Phase::Moving {
                from: self.floor,
                to: next.floor,
                door: next.door,
                left: total,
                total,
            };
        }
    }

    /// Advance by `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        match self.phase {
            Phase::Idle => self.start_next(),
            Phase::Moving {
                from,
                to,
                door,
                left,
                total,
            } => {
                let left = left - dt;
                if left <= TIME_EPSILON {
                    self.floor = to;
                    self.begin_opening(door);
                } else {
                    self.phase = Phase::Moving {
                        from,
                        to,
                        door,
                        left,
                        total,
                    };
                }
            }
            Phase::Opening { left } => {
                let left = left - dt;
                self.phase = if left <= TIME_EPSILON {
                    Phase::Open { for_seconds: 0.0 }
                } else {
                    Phase::Opening { left }
                };
            }
            Phase::Open { for_seconds } => {
                let for_seconds = for_seconds + dt;
                self.phase = Phase::Open { for_seconds };
                if self
                    .spec
                    .door_hold_seconds
                    .is_some_and(|h| for_seconds + TIME_EPSILON >= h)
                {
                    self.begin_closing();
                }
            }
            Phase::Closing { left } => {
                let left = left - dt;
                self.phase = if left <= TIME_EPSILON {
                    Phase::Idle
                } else {
                    Phase::Closing { left }
                };
            }
        }
    }
}

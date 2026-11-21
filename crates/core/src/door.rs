//! Doors as **peripherals** of the simulated world (S50), next to the lifts. A
//! door is not a robot and not part of VDA 5050 (the standard leaves talking to
//! doors to the fleet control), so it has its **own small command port**.
//!
//! **Model.** A door guards one or more **layout edges** (the passage, usually
//! the edge through it and the one back). It is closed → opening
//! (`open_seconds`) → open → closing (`close_seconds`) → closed.
//!
//! **Command port** ([`Door::open`], [`Door::release`]):
//! - `open()`: the controller wants the door open. A closed door opens; a door
//!   that is closing finishes closing first and then opens again (like a queued
//!   call of a lift).
//! - `release()`: the controller is done with the door; it closes. Without a
//!   release it stays open. Option `hold_seconds`: it releases itself after
//!   being open that long.
//! - **A door never closes while a robot is on one of its edges** (a safety
//!   sensor): it stays open until the robot has left, then closes.
//!
//! **Robots** do not open doors. A robot does not start an edge of a door unless
//! the door is **fully open** (it waits at the start node, like for a RELEASE
//! zone). A robot wider than the door's `passage_width` cannot pass: an order
//! with an edge of that door is refused (`NO_ROUTE_TO_TARGET`).
//!
//! The time is quantised to the simulation step, like the lifts.
//!
//! Not yet: doors that open by themselves for a robot, doors on nodes, the door
//! blocking the way while it is not open for other reasons (a robot standing in
//! the doorway while the door is closed cannot happen), partially open doors.

use std::fmt;

use crate::actions::TIME_EPSILON;
use crate::graph::EdgeIdx;
use crate::lift::DoorState;

/// What a door is like.
#[derive(Debug, Clone, PartialEq)]
pub struct DoorSpec {
    pub id: String,
    /// The layout edges that lead through the door; at least one.
    pub edges: Vec<EdgeIdx>,
    /// How wide the passage is (metres); robots wider than this cannot pass.
    /// `None`: any robot.
    pub passage_width: Option<f64>,
    pub open_seconds: f64,
    pub close_seconds: f64,
    /// The door releases itself after being open this long; `None` = only on
    /// [`Door::release`].
    pub hold_seconds: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum DoorError {
    /// A door needs at least one edge, all different.
    BadEdges,
    /// The width must be a finite number > 0; a time a finite number >= 0 (the
    /// hold time > 0); the text names the setting.
    BadValue(&'static str),
}

impl fmt::Display for DoorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DoorError::BadEdges => write!(f, "a door needs at least one edge, all different"),
            DoorError::BadValue(what) => write!(f, "{what} is not a usable number"),
        }
    }
}

impl std::error::Error for DoorError {}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    Closed,
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

/// What a door looks like from outside (for the controller and the observer).
#[derive(Debug, Clone, PartialEq)]
pub struct DoorStatus {
    pub id: String,
    pub state: DoorState,
    /// The controller wants the door open (it has called `open` and not released).
    pub wanted_open: bool,
    /// The edges the door guards.
    pub edges: Vec<EdgeIdx>,
}

#[derive(Debug, Clone)]
pub struct Door {
    spec: DoorSpec,
    phase: Phase,
    wanted_open: bool,
}

fn time_ok(v: f64) -> bool {
    v.is_finite() && v >= 0.0
}

impl Door {
    pub fn new(spec: DoorSpec) -> Result<Door, DoorError> {
        let distinct = spec
            .edges
            .iter()
            .enumerate()
            .all(|(i, e)| !spec.edges[..i].contains(e));
        if spec.edges.is_empty() || !distinct {
            return Err(DoorError::BadEdges);
        }
        if spec
            .passage_width
            .is_some_and(|w| !(w.is_finite() && w > 0.0))
        {
            return Err(DoorError::BadValue("passage_width"));
        }
        if !time_ok(spec.open_seconds) {
            return Err(DoorError::BadValue("open_seconds"));
        }
        if !time_ok(spec.close_seconds) {
            return Err(DoorError::BadValue("close_seconds"));
        }
        if spec
            .hold_seconds
            .is_some_and(|h| !(h.is_finite() && h > 0.0))
        {
            return Err(DoorError::BadValue("hold_seconds"));
        }
        Ok(Door {
            spec,
            phase: Phase::Closed,
            wanted_open: false,
        })
    }

    pub fn spec(&self) -> &DoorSpec {
        &self.spec
    }

    pub fn state(&self) -> DoorState {
        match self.phase {
            Phase::Closed => DoorState::Closed,
            Phase::Opening { .. } => DoorState::Opening,
            Phase::Open { .. } => DoorState::Open,
            Phase::Closing { .. } => DoorState::Closing,
        }
    }

    /// Fully open: robots may use the edges of the door.
    pub fn is_open(&self) -> bool {
        matches!(self.phase, Phase::Open { .. })
    }

    pub fn status(&self) -> DoorStatus {
        DoorStatus {
            id: self.spec.id.clone(),
            state: self.state(),
            wanted_open: self.wanted_open,
            edges: self.spec.edges.clone(),
        }
    }

    /// Command port: the controller wants the door open.
    pub fn open(&mut self) {
        self.wanted_open = true;
    }

    /// Command port: the controller is done with the door; it closes (as soon
    /// as no robot is on its edges).
    pub fn release(&mut self) {
        self.wanted_open = false;
    }

    fn begin_opening(&mut self) {
        self.phase = if self.spec.open_seconds <= TIME_EPSILON {
            Phase::Open { for_seconds: 0.0 }
        } else {
            Phase::Opening {
                left: self.spec.open_seconds,
            }
        };
    }

    fn begin_closing(&mut self) {
        self.phase = if self.spec.close_seconds <= TIME_EPSILON {
            Phase::Closed
        } else {
            Phase::Closing {
                left: self.spec.close_seconds,
            }
        };
    }

    /// Advance by `dt` seconds. `occupied`: a robot is on one of the door's edges.
    pub fn step(&mut self, dt: f64, occupied: bool) {
        match self.phase {
            Phase::Closed => {
                if self.wanted_open {
                    self.begin_opening();
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
                    .hold_seconds
                    .is_some_and(|h| for_seconds + TIME_EPSILON >= h)
                {
                    self.wanted_open = false;
                }
                if !self.wanted_open && !occupied {
                    self.begin_closing();
                }
            }
            Phase::Closing { left } => {
                let left = left - dt;
                self.phase = if left <= TIME_EPSILON {
                    Phase::Closed
                } else {
                    Phase::Closing { left }
                };
            }
        }
    }
}

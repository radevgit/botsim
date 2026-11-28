//! Lines 2 and following of a recording: one snapshot per line.

use botsim_model::reservation::Reservation;
use botsim_model::spec::Shape;
use botsim_model::waiting::WaitReason;
use botsim_sim::door::DoorStatus;
use botsim_sim::lift::{Call, DoorState, LiftStatus};
use botsim_sim::observer::{Blocking, RobotSnapshot, RobotStatus, Snapshot};
use botsim_sim::waiting::Wait;
use serde::{Deserialize, Serialize};

/// The world after one step. Robots are referred to by their position in
/// [`SnapshotLine::robots`]; nodes, edges and floors by their position in the
/// layout line.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename = "snapshot")]
pub struct SnapshotLine {
    pub tick: u64,
    /// Simulated seconds since the start.
    pub time: f64,
    pub robots: Vec<RobotRecord>,
    pub reservations: Vec<ReservationRecord>,
    /// `[waiter, blocker]`.
    pub blocking: Vec<[usize; 2]>,
    pub overlapping: Vec<[usize; 2]>,
    pub deadlocks: Vec<Vec<usize>>,
    pub lifts: Vec<LiftRecord>,
    pub doors: Vec<DoorRecord>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RobotRecord {
    /// `"manufacturer/serialNumber"`.
    pub id: String,
    pub map: usize,
    pub x: f64,
    pub y: f64,
    pub heading: f64,
    pub shape: ShapeRecord,
    pub status: StatusRecord,
    pub speed: f64,
    /// Empty if the robot never had an order.
    pub order: String,
    pub last_node: Option<usize>,
    pub released: RouteRecord,
    pub planned: RouteRecord,
    /// The state of charge in percent; `None` if the simulator has no battery model (the
    /// recordings made before it have no such field).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub charge: Option<f64>,
    /// The robot is charging.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub charging: bool,
    /// Who the robot waits for in this step and why, as the "waiting caused by others"
    /// metric counts it; `None` if it does not wait or is in its own stop (and in the
    /// recordings made before the field).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub waiting: Option<WaitRecord>,
}

/// One robot's wait in one step (`botsim_sim::waiting::Wait`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct WaitRecord {
    /// The robot it waits for, by its position in [`SnapshotLine::robots`]; `null`
    /// (`None`) if unknown.
    pub waiting_for: Option<usize>,
    pub reason: WaitReasonRecord,
}

/// Same variants as `botsim_model::waiting::WaitReason`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum WaitReasonRecord {
    Release,
    Zone,
    Lift,
    Door,
    SafetyStop,
}

/// The body outline relative to the kinematic center (turns with the heading).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ShapeRecord {
    /// `[cx, cy, half_length, half_width]`
    Rect([f64; 4]),
    /// `[cx, cy, radius]`
    Circle([f64; 3]),
}

/// Same variants as `botsim_sim::observer::RobotStatus`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StatusRecord {
    Idle,
    Driving,
    Turning,
    Acting,
    WaitingForRelease,
    WaitingForZone,
    WaitingForLift,
    RidingLift,
    Held,
    Obstructed,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct RouteRecord {
    pub nodes: Vec<usize>,
    pub edges: Vec<usize>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReservationRecord {
    pub robot: usize,
    pub nodes: Vec<usize>,
    pub edges: Vec<usize>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DoorStateRecord {
    Closed,
    Opening,
    Open,
    Closing,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiftRecord {
    pub id: String,
    pub floor: usize,
    /// In floors (1.5 = half way between floors 1 and 2).
    pub position: f64,
    pub door: DoorStateRecord,
    pub moving_to: Option<usize>,
    pub queue: Vec<CallRecord>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct CallRecord {
    pub floor: usize,
    pub door: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DoorRecord {
    pub id: String,
    pub state: DoorStateRecord,
    pub wanted_open: bool,
    pub edges: Vec<usize>,
}

impl From<Shape> for ShapeRecord {
    fn from(s: Shape) -> Self {
        match s {
            Shape::Rect {
                cx,
                cy,
                half_length,
                half_width,
            } => ShapeRecord::Rect([cx, cy, half_length, half_width]),
            Shape::Circle { cx, cy, radius } => ShapeRecord::Circle([cx, cy, radius]),
        }
    }
}

impl From<RobotStatus> for StatusRecord {
    fn from(s: RobotStatus) -> Self {
        match s {
            RobotStatus::Idle => StatusRecord::Idle,
            RobotStatus::Driving => StatusRecord::Driving,
            RobotStatus::Turning => StatusRecord::Turning,
            RobotStatus::Acting => StatusRecord::Acting,
            RobotStatus::WaitingForRelease => StatusRecord::WaitingForRelease,
            RobotStatus::WaitingForZone => StatusRecord::WaitingForZone,
            RobotStatus::WaitingForLift => StatusRecord::WaitingForLift,
            RobotStatus::RidingLift => StatusRecord::RidingLift,
            RobotStatus::Held => StatusRecord::Held,
            RobotStatus::Obstructed => StatusRecord::Obstructed,
        }
    }
}

impl From<DoorState> for DoorStateRecord {
    fn from(s: DoorState) -> Self {
        match s {
            DoorState::Closed => DoorStateRecord::Closed,
            DoorState::Opening => DoorStateRecord::Opening,
            DoorState::Open => DoorStateRecord::Open,
            DoorState::Closing => DoorStateRecord::Closing,
        }
    }
}

impl From<&RobotSnapshot> for RobotRecord {
    fn from(r: &RobotSnapshot) -> Self {
        RobotRecord {
            id: format!("{}/{}", r.id.manufacturer, r.id.serial_number),
            map: r.pose.map,
            x: r.pose.x,
            y: r.pose.y,
            heading: r.pose.heading,
            shape: r.shape.into(),
            status: r.status.into(),
            speed: r.speed,
            order: r.order_id.clone(),
            last_node: r.last_node,
            released: RouteRecord {
                nodes: r.released_nodes.clone(),
                edges: r.released_edges.clone(),
            },
            planned: RouteRecord {
                nodes: r.planned_nodes.clone(),
                edges: r.planned_edges.clone(),
            },
            charge: r.charge,
            charging: r.charging,
            waiting: r.waiting.map(WaitRecord::from),
        }
    }
}

impl From<Wait> for WaitRecord {
    fn from(w: Wait) -> Self {
        WaitRecord {
            waiting_for: w.blocker,
            reason: match w.reason {
                WaitReason::Release => WaitReasonRecord::Release,
                WaitReason::Zone => WaitReasonRecord::Zone,
                WaitReason::Lift => WaitReasonRecord::Lift,
                WaitReason::Door => WaitReasonRecord::Door,
                WaitReason::SafetyStop => WaitReasonRecord::SafetyStop,
            },
        }
    }
}

impl From<&Reservation> for ReservationRecord {
    fn from(r: &Reservation) -> Self {
        ReservationRecord {
            robot: r.robot,
            nodes: r.nodes.clone(),
            edges: r.edges.clone(),
        }
    }
}

impl From<&Call> for CallRecord {
    fn from(c: &Call) -> Self {
        CallRecord {
            floor: c.floor,
            door: c.door,
        }
    }
}

impl From<&LiftStatus> for LiftRecord {
    fn from(l: &LiftStatus) -> Self {
        LiftRecord {
            id: l.id.clone(),
            floor: l.floor,
            position: l.position,
            door: l.door.into(),
            moving_to: l.moving_to,
            queue: l.queue.iter().map(CallRecord::from).collect(),
        }
    }
}

impl From<&DoorStatus> for DoorRecord {
    fn from(d: &DoorStatus) -> Self {
        DoorRecord {
            id: d.id.clone(),
            state: d.state.into(),
            wanted_open: d.wanted_open,
            edges: d.edges.clone(),
        }
    }
}

impl From<&Snapshot> for SnapshotLine {
    fn from(s: &Snapshot) -> Self {
        SnapshotLine {
            tick: s.tick,
            time: s.time,
            robots: s.robots.iter().map(RobotRecord::from).collect(),
            reservations: s.reservations.iter().map(ReservationRecord::from).collect(),
            blocking: s
                .blocking
                .iter()
                .map(|b: &Blocking| [b.waiter, b.blocker])
                .collect(),
            overlapping: s.overlapping.iter().map(|&(a, b)| [a, b]).collect(),
            deadlocks: s.deadlocks.clone(),
            lifts: s.lifts.iter().map(LiftRecord::from).collect(),
            doors: s.doors.iter().map(DoorRecord::from).collect(),
        }
    }
}

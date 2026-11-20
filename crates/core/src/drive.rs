//! One robot driving a fixed route (a list of edges) on its own, without ECS.
//!
//! First version: differential kinematics only; the robot stops at every node,
//! turns on the spot to the heading the edge asks for (forwards, or backwards
//! if the edge has orientation π), then drives it. At the end of the released
//! base of an order with a horizon it waits; an order update extends the route.
//! Speed limits per edge and passing nodes without stopping come later.

use std::fmt;

use crate::actions::{ActionQueue, ActionSpec, ActionTimes};
use crate::graph::{EdgeIdx, Graph, MapIdx, NodeIdx};
use crate::motion::{Along, EdgeGoal, advance, angle_diff, normalize_angle, rotate};
use crate::order_route::OrderRoute;
use crate::orientation::{OrientationError, edge_heading};
use crate::spec::{Kinematics, Limits, RobotSpec};
use vda5050::order::OrientationType;
use vda5050::state::ActionStatus;

/// Map (floor), position of the kinematic center in that map's coordinates, and
/// heading. Metres, radians (CCW positive).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub map: MapIdx,
    pub x: f64,
    pub y: f64,
    pub heading: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Turning on the spot towards the start direction of the next edge.
    Rotating,
    Driving,
    /// Standing at a node while actions run: a `SOFT` or `HARD` action blocks
    /// driving, or (at the end of the base) actions are still unfinished.
    Acting,
    /// Standing at the end of the released base; the order has a horizon and the
    /// robot waits for an order update that releases more.
    Waiting,
    /// The whole order is done.
    Finished,
}

#[derive(Debug, Clone, PartialEq)]
pub enum DriveError {
    /// Only differential robots are modelled so far.
    UnsupportedKinematics(Kinematics),
    /// The edge does not start at the node where the previous one ended.
    NotConnected { edge: String },
    /// The edge has no usable length (start and end node coincide, or the length
    /// is not a finite number), so it has no direction.
    ZeroLengthEdge { edge: String },
    /// The edge leads to another map (e.g. a lift ride); not supported until the
    /// elevator peripheral exists.
    MapTransition { edge: String },
    /// The robot cannot take the orientation the order asks for on this edge
    /// (VDA 5050 error `ORIENTATION_UNREACHABLE`, level WARNING).
    Orientation {
        edge: String,
        error: OrientationError,
    },
    /// An order update does not start at the end of the base driven so far.
    NotContinuation { expected: String, found: String },
}

impl fmt::Display for DriveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DriveError::UnsupportedKinematics(k) => write!(f, "kinematics {k:?} not supported yet"),
            DriveError::NotConnected { edge } => {
                write!(
                    f,
                    "edge '{edge}' does not start where the previous edge ended"
                )
            }
            DriveError::ZeroLengthEdge { edge } => write!(f, "edge '{edge}' has zero length"),
            DriveError::MapTransition { edge } => {
                write!(f, "edge '{edge}' changes map; not supported yet")
            }
            DriveError::Orientation { edge, error } => write!(f, "edge '{edge}': {error}"),
            DriveError::NotContinuation { expected, found } => write!(
                f,
                "order update starts at node '{found}', but the base ends at node '{expected}'"
            ),
        }
    }
}

impl std::error::Error for DriveError {}

/// One edge of a route, with the orientation the order asks for on it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteEdge {
    pub edge: EdgeIdx,
    /// VDA 5050 edge `orientation`; `None` = not given (robot drives forwards).
    pub orientation: Option<f64>,
    /// VDA 5050 edge `orientationType`; `None` = default (TANGENTIAL).
    pub orientation_type: Option<OrientationType>,
}

impl RouteEdge {
    /// An edge without orientation: driven forwards.
    pub fn new(edge: EdgeIdx) -> RouteEdge {
        RouteEdge {
            edge,
            orientation: None,
            orientation_type: None,
        }
    }

    /// An edge with a tangential orientation (0.0 forwards, π backwards).
    pub fn tangential(edge: EdgeIdx, orientation: f64) -> RouteEdge {
        RouteEdge {
            edge,
            orientation: Some(orientation),
            orientation_type: Some(OrientationType::Tangential),
        }
    }
}

impl From<EdgeIdx> for RouteEdge {
    fn from(edge: EdgeIdx) -> RouteEdge {
        RouteEdge::new(edge)
    }
}

/// A straight edge copied out of the graph, so the driver needs no graph reference.
#[derive(Debug, Clone, Copy)]
struct Segment {
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    length: f64,
    /// Heading the robot must have on this edge (edge direction, or + π when
    /// driving backwards). Travel is always along the edge.
    heading: f64,
}

/// Copy the edges of `route`, which must run on from `start`, out of the graph.
/// Returns the segments and the node where the last one ends (`start` if empty).
fn build_segments(
    graph: &Graph,
    kinematics: Kinematics,
    start: NodeIdx,
    route: &[RouteEdge],
) -> Result<(Vec<Segment>, NodeIdx), DriveError> {
    let mut segments = Vec::with_capacity(route.len());
    let mut at = start;
    for re in route {
        let e = &graph.edges[re.edge];
        if e.start != at {
            return Err(DriveError::NotConnected { edge: e.id.clone() });
        }
        if e.map_transition {
            return Err(DriveError::MapTransition { edge: e.id.clone() });
        }
        // Also guards the division by the length when the pose is computed in `step`.
        if !(e.length.is_finite() && e.length > 0.0) {
            return Err(DriveError::ZeroLengthEdge { edge: e.id.clone() });
        }
        let (a, b) = (&graph.nodes[e.start], &graph.nodes[e.end]);
        let direction = (b.y - a.y).atan2(b.x - a.x);
        let heading = edge_heading(kinematics, direction, re.orientation, re.orientation_type)
            .map_err(|error| DriveError::Orientation {
                edge: e.id.clone(),
                error,
            })?;
        segments.push(Segment {
            ax: a.x,
            ay: a.y,
            bx: b.x,
            by: b.y,
            length: e.length,
            heading,
        });
        at = e.end;
    }
    Ok((segments, at))
}

#[derive(Debug, Clone)]
pub struct Driver {
    kinematics: Kinematics,
    limits: Limits,
    segments: Vec<Segment>,
    /// Actions to queue when segment i ends, i.e. at its end node (parallel to `segments`).
    arrive_actions: Vec<Vec<ActionSpec>>,
    times: ActionTimes,
    queue: ActionQueue,
    /// Node where the released base ends (the decision point).
    base_end: NodeIdx,
    /// The order goes on beyond the base: wait at its end for an update.
    has_horizon: bool,
    /// Index of the segment being driven, or turned towards.
    current: usize,
    phase: Phase,
    pose: Pose,
    /// Distance along the current segment and speed.
    along: Along,
}

impl Driver {
    /// A robot standing at `start` with `heading`, about to drive `route`
    /// (a complete order: it is finished at the end).
    pub fn new(
        graph: &Graph,
        spec: &RobotSpec,
        start: NodeIdx,
        heading: f64,
        route: &[RouteEdge],
    ) -> Result<Driver, DriveError> {
        Driver::build(
            graph,
            spec,
            start,
            heading,
            route,
            false,
            &[],
            vec![Vec::new(); route.len()],
            ActionTimes::new(0.0), // no actions, so the times do not matter
        )
    }

    /// A robot at the first node of an order, about to drive its released base.
    /// The actions of the start node (new order only) are queued at once, with
    /// durations from `times`, which also applies to all later actions. If the
    /// order has a horizon the robot waits at the end of the base.
    pub fn from_order_route(
        graph: &Graph,
        spec: &RobotSpec,
        heading: f64,
        route: &OrderRoute,
        times: ActionTimes,
    ) -> Result<Driver, DriveError> {
        Driver::build(
            graph,
            spec,
            route.start,
            heading,
            &route.edges,
            route.has_horizon,
            &route.node_actions[0],
            route.node_actions[1..].to_vec(),
            times,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn build(
        graph: &Graph,
        spec: &RobotSpec,
        start: NodeIdx,
        heading: f64,
        route: &[RouteEdge],
        has_horizon: bool,
        start_actions: &[ActionSpec],
        arrive_actions: Vec<Vec<ActionSpec>>,
        times: ActionTimes,
    ) -> Result<Driver, DriveError> {
        if spec.kinematics != Kinematics::Differential {
            return Err(DriveError::UnsupportedKinematics(spec.kinematics));
        }
        let (segments, base_end) = build_segments(graph, spec.kinematics, start, route)?;

        let s = &graph.nodes[start];
        let mut queue = ActionQueue::new();
        queue.push(start_actions.iter().map(|a| times.plan(a)));
        let mut d = Driver {
            kinematics: spec.kinematics,
            limits: spec.limits,
            segments,
            arrive_actions,
            times,
            queue,
            base_end,
            has_horizon,
            current: 0,
            phase: Phase::Finished,
            pose: Pose {
                map: s.map,
                x: s.x,
                y: s.y,
                heading: normalize_angle(heading),
            },
            along: Along { s: 0.0, v: 0.0 },
        };
        d.at_node();
        Ok(d)
    }

    /// Id, type and status of every action queued so far (for the state message).
    pub fn action_states(&self) -> impl Iterator<Item = (&str, &str, ActionStatus)> {
        self.queue.states()
    }

    /// An order update: `route` continues the order. Its first node must be the
    /// end of the base driven so far (the decision point). The new edges are
    /// appended; a robot waiting or finished at the decision point drives on.
    /// On error the driver is unchanged.
    pub fn extend(&mut self, graph: &Graph, route: &OrderRoute) -> Result<(), DriveError> {
        if route.start != self.base_end {
            return Err(DriveError::NotContinuation {
                expected: graph.nodes[self.base_end].id.clone(),
                found: graph.nodes[route.start].id.clone(),
            });
        }
        let (segments, base_end) =
            build_segments(graph, self.kinematics, route.start, &route.edges)?;
        self.segments.extend(segments);
        // entry 0 is the decision point, whose actions were queued when it was reached
        self.arrive_actions
            .extend(route.node_actions[1..].iter().cloned());
        self.base_end = base_end;
        self.has_horizon = route.has_horizon;
        if matches!(self.phase, Phase::Waiting | Phase::Finished | Phase::Acting) {
            self.at_node(); // `current` is the first new segment, or there is none
        }
        Ok(())
    }

    pub fn pose(&self) -> Pose {
        self.pose
    }

    pub fn speed(&self) -> f64 {
        self.along.v
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    pub fn is_finished(&self) -> bool {
        self.phase == Phase::Finished
    }

    /// The robot stands at a node (start, arrival, or after an update): decide
    /// what it does next.
    /// - A `SOFT` or `HARD` action is unfinished: stand still and act.
    /// - Else, if there is a next segment: turn first if the heading is off, else drive.
    /// - Else the base is done: act while actions are still running, then wait
    ///   (horizon) or finish.
    fn at_node(&mut self) {
        self.along = Along { s: 0.0, v: 0.0 };
        self.phase = if !self.queue.driving_allowed() {
            Phase::Acting
        } else {
            match self.segments.get(self.current) {
                Some(seg) if angle_diff(self.pose.heading, seg.heading).abs() < 1e-9 => {
                    Phase::Driving
                }
                Some(_) => Phase::Rotating,
                None if !self.queue.is_idle() => Phase::Acting,
                None if self.has_horizon => Phase::Waiting,
                None => Phase::Finished,
            }
        };
    }

    /// Advance the simulation by `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        // actions run in parallel with everything the robot does
        self.queue.step(dt);
        match self.phase {
            Phase::Finished | Phase::Waiting => {}
            Phase::Acting => self.at_node(),
            Phase::Rotating => {
                let seg = self.segments[self.current];
                let (h, done) = rotate(
                    self.pose.heading,
                    seg.heading,
                    self.limits.max_angular_speed,
                    dt,
                );
                self.pose.heading = h;
                if done {
                    self.phase = Phase::Driving;
                }
            }
            Phase::Driving => {
                let seg = self.segments[self.current];
                let goal = EdgeGoal {
                    length: seg.length,
                    v_limit: self.limits.max_speed,
                    v_end: 0.0,
                };
                let (along, arrived) = advance(self.along, goal, &self.limits, dt);
                self.along = along;
                let f = along.s / seg.length;
                self.pose.x = seg.ax + (seg.bx - seg.ax) * f;
                self.pose.y = seg.ay + (seg.by - seg.ay) * f;
                if arrived {
                    let actions = std::mem::take(&mut self.arrive_actions[self.current]);
                    self.queue.push(actions.iter().map(|a| self.times.plan(a)));
                    self.current += 1;
                    self.at_node();
                }
            }
        }
    }
}

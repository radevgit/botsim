//! One robot driving a fixed route (a list of edges) on its own, without ECS.
//!
//! First version: differential kinematics only; the robot stops at every node,
//! turns on the spot to the heading the edge asks for (forwards, or backwards
//! if the edge has orientation π), then drives it. At the end of the released
//! base of an order with a horizon it waits; an order update extends the route.
//! Speed limits per edge and passing nodes without stopping come later.

use std::collections::HashSet;
use std::f64::consts::FRAC_PI_2;
use std::fmt;
use std::sync::Arc;

use crate::actions::{ActionQueue, ActionSpec, ActionTimes, TIME_EPSILON};
use crate::motion::{Along, EdgeGoal, advance, rotate, rotate_in};
use crate::order_route::{OrderRoute, PlanEdge, PlanNode};
use botsim_link::ids::OrderId;
use botsim_link::state::{ActionState, ActionStatus};
use botsim_model::angle::{angle_diff, normalize_angle};
use botsim_model::graph::{EdgeIdx, Graph, MapIdx, NodeIdx, Rotation, THREE_WHEEL_TOLERANCE};
use botsim_model::orientation::{OrientationError, OrientationType, edge_heading};
use botsim_model::path::Path;
use botsim_model::spec::{Kinematics, Limits, RobotSpec};

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
    /// Standing at a node because the next edge must not be started yet (it
    /// enters a RELEASE zone that has not been granted).
    WaitingForClearance,
    /// Standing at the start node of a lift ride, waiting until the world lets
    /// the robot into the cabin ([`Driver::board`]).
    WaitingForLift,
    /// In the cabin, until the world lets it out at the other floor
    /// ([`Driver::alight`]). The pose stays where the ride began.
    Riding,
    /// The whole order is done.
    Finished,
}

#[derive(Debug, Clone, PartialEq)]
pub enum DriveError {
    /// The edge does not start at the node where the previous one ended.
    NotConnected { edge: String },
    /// The edge has no usable length (start and end node coincide, or the length
    /// is not a finite number), so it has no direction.
    ZeroLengthEdge { edge: String },
    /// The edge leads to another map, and it is not a ride of a lift (a lift ride
    /// is accepted if the world has told the robot so, see [`Driver::from_order_route_with_rides`]).
    MapTransition { edge: String },
    /// The edge leads through a door that is narrower than the robot is wide.
    DoorTooNarrow { edge: String },
    /// The edge needs the robot to turn at its start node, and the layout allows
    /// no rotation there (S64).
    RotationNotAllowed { edge: String, node: String },
    /// A three-wheel (car-like) robot cannot turn on the spot, and the edge starts
    /// in another direction than the robot is heading (S70).
    CannotTurnOnTheSpot { edge: String, node: String },
    /// The curve of the edge is so tight that even at the lowest speed the robot
    /// can drive, it would have to turn faster than it can (S70).
    CurveTooTight { edge: String },
    /// The layout does not allow this robot's type on the node or edge (`what`
    /// names it, e.g. `edge 'A-B'`).
    NotAllowedForType { what: String },
    /// The robot cannot take the orientation the order asks for on this edge
    /// (VDA 5050 error `ORIENTATION_UNREACHABLE`, level WARNING).
    Orientation {
        edge: String,
        error: OrientationError,
    },
    /// An order update does not start at the end of the base driven so far.
    NotContinuation { expected: String, found: String },
    /// The first node of an order update has another `sequenceId` than the end
    /// of the base driven so far: the numbers of an order run on through its
    /// updates (VDA 5050 §6.1).
    SequenceNotContinued { expected: u32, found: u32 },
}

impl fmt::Display for DriveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
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
            DriveError::DoorTooNarrow { edge } => write!(
                f,
                "edge '{edge}' leads through a door that is narrower than the robot"
            ),
            DriveError::CannotTurnOnTheSpot { edge, node } => write!(
                f,
                "this robot cannot turn on the spot: edge '{edge}' starts at node '{node}' in another direction than the robot is heading"
            ),
            DriveError::CurveTooTight { edge } => write!(
                f,
                "the curve of edge '{edge}' is too tight for this robot: it would have to turn faster than it can at its lowest speed"
            ),
            DriveError::RotationNotAllowed { edge, node } => write!(
                f,
                "edge '{edge}' needs a turn at node '{node}' that the layout does not allow"
            ),
            DriveError::NotAllowedForType { what } => {
                write!(f, "the layout does not allow this robot type on {what}")
            }
            DriveError::Orientation { edge, error } => write!(f, "edge '{edge}': {error}"),
            DriveError::NotContinuation { expected, found } => write!(
                f,
                "order update starts at node '{found}', but the base ends at node '{expected}'"
            ),
            DriveError::SequenceNotContinued { expected, found } => write!(
                f,
                "order update starts at sequenceId {found}, but the base ends at sequenceId {expected}"
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

/// The whole order the robot is executing (base and horizon), for the state report.
/// Entry i of `nodes` is the node where segment i starts; `edges[i]` follows it.
#[derive(Debug, Clone)]
struct Plan {
    order_id: OrderId,
    order_update_id: u32,
    nodes: Vec<PlanNode>,
    edges: Vec<PlanEdge>,
}

/// Snapshot of what the state report carries, taken from the driver.
#[derive(Debug, Clone, PartialEq)]
pub struct Progress {
    /// `None` if the driver was built from a bare route (no order), or the order
    /// has an empty `orderId`.
    pub order_id: Option<OrderId>,
    pub order_update_id: u32,
    /// The last node reached (or the node the robot stands on); `None` without an order.
    pub last_node: Option<PlanNode>,
    /// Nodes still to be traversed, base and horizon (the last node is not in here).
    pub remaining_nodes: Vec<PlanNode>,
    /// Edges not yet left, base and horizon (including the one being driven).
    pub remaining_edges: Vec<PlanEdge>,
    pub driving: bool,
    /// Stopped (or braking to a stop) by a [`Driver::hold`]; reported as a
    /// protective field violation.
    pub held: bool,
    pub pose: Pose,
    /// Speed in the robot frame (negative when driving backwards), m/s.
    pub vx: f64,
    /// Sideways speed in the robot frame (positive to the left), m/s; 0 unless the
    /// robot is omnidirectional and travels across its heading.
    pub vy: f64,
    /// Rotation speed, rad/s (counter-clockwise positive).
    pub omega: f64,
    /// All actions: queued ones, then those still to be triggered.
    pub actions: Vec<ActionState>,
}

/// See [`Driver::glance`]; the fields mean what they mean in [`Progress`].
#[derive(Debug, Clone, Copy)]
pub(crate) struct Glance<'a> {
    pub last_node: Option<&'a PlanNode>,
    pub remaining_nodes: &'a [PlanNode],
    pub remaining_edges: &'a [PlanEdge],
    pub driving: bool,
    pub held: bool,
    /// An action is running (`actionStatus` RUNNING).
    pub acting: bool,
}

/// A straight edge copied out of the graph, so the driver needs no graph reference.
#[derive(Debug, Clone)]
struct Segment {
    /// The path the robot follows on the edge: a straight line, or the curve of
    /// the edge's trajectory for this robot's type (S69).
    path: Arc<Path>,
    /// The length of the path (m).
    length: f64,
    /// Heading the robot must have on this edge (edge direction, or + π when
    /// driving backwards; for an omnidirectional robot any orientation the order
    /// names). Travel is always along the edge. `None`: the robot keeps the
    /// heading it has (an omnidirectional robot, order without orientation, S63).
    heading: Option<f64>,
    /// The direction the path starts in and the direction it ends in (radians).
    /// Equal on a straight edge.
    start_dir: f64,
    end_dir: f64,
    /// On a curve the heading follows the direction of the path: the heading is
    /// the direction plus this (0 forwards, π backwards, any value for an
    /// omnidirectional robot with a tangential orientation). `None`: the heading
    /// is fixed (straight edges, a global orientation, or kept).
    follow: Option<f64>,
    /// The robot drives this edge backwards (its heading points against the travel direction).
    reversed: bool,
    /// A lift ride: no driving, the robot appears here when the ride is over.
    ride: Option<RideEnd>,
    /// The speed limit of the edge for this robot's type (LIF `maximumSpeed`).
    max_speed: Option<f64>,
    /// The speed limit of the curve: where the heading follows the path the robot
    /// turns at speed × curvature, which may not exceed its maximum angular
    /// speed. One limit for the whole segment, from its sharpest turn (S70).
    curve_speed: Option<f64>,
    /// How the robot turns at the start node to get its heading: `Some(true)`
    /// counter-clockwise, `Some(false)` clockwise — chosen when the layout allows
    /// only one direction and it is not the shorter one (S64); `None`: the
    /// shorter way (or no turn).
    turn_ccw: Option<bool>,
    /// The rotation the layout allows at the end node of this edge (`None` for a
    /// lift ride), to be combined with the next edge's start node rule.
    end_rotation: Option<Rotation>,
}

impl Driver {
    /// How fast the robot turns while it drives along the segment (rad/s,
    /// counter-clockwise positive): the speed times the curvature of the path where
    /// the heading follows the path, else 0.
    fn turning_rate(&self, s: &Segment) -> f64 {
        if s.follow.is_some() {
            self.along.v * s.path.curvature_at(self.along.s)
        } else {
            0.0
        }
    }
}

impl Segment {
    /// The heading the robot has when it has driven the segment: the heading it
    /// started with, or on a curve that follows the path the one at the end;
    /// `None` if the robot keeps whatever heading it has.
    fn end_heading(&self) -> Option<f64> {
        match self.follow {
            Some(offset) => Some(normalize_angle(self.end_dir + offset)),
            None => self.heading,
        }
    }
}

/// What a route continues from: the heading the robot will have at its start node
/// and the rotation the edge it comes in on allows there (`None` if unknown).
#[derive(Debug, Clone, Copy)]
struct Prev {
    heading: f64,
    end_rotation: Option<Rotation>,
}

/// Where a lift ride ends.
#[derive(Debug, Clone, Copy)]
struct RideEnd {
    edge: EdgeIdx,
    map: MapIdx,
    x: f64,
    y: f64,
}

/// Copy the edges of `route`, which must run on from `start`, out of the graph.
/// Returns the segments and the node where the last one ends (`start` if empty).
/// An edge to another map is accepted only if it is one of `rides`.
fn build_segments(
    graph: &Graph,
    spec: &RobotSpec,
    start: NodeIdx,
    route: &[RouteEdge],
    rides: &HashSet<EdgeIdx>,
    mut prev: Prev,
) -> Result<(Vec<Segment>, NodeIdx), DriveError> {
    let kinematics = spec.kinematics;
    let mut segments = Vec::with_capacity(route.len());
    let mut at = start;
    for re in route {
        let e = &graph.edges[re.edge];
        if e.start != at {
            return Err(DriveError::NotConnected { edge: e.id.clone() });
        }
        if e.map_transition {
            if !rides.contains(&re.edge) {
                return Err(DriveError::MapTransition { edge: e.id.clone() });
            }
            let (a, b) = (&graph.nodes[e.start], &graph.nodes[e.end]);
            segments.push(Segment {
                path: Arc::new(Path::straight((a.x, a.y), (a.x, a.y))),
                length: 0.0,
                heading: None, // the robot keeps its heading on a ride
                start_dir: 0.0,
                end_dir: 0.0,
                follow: None,
                reversed: false,
                max_speed: None,
                curve_speed: None,
                turn_ccw: None,
                end_rotation: None,
                ride: Some(RideEnd {
                    edge: re.edge,
                    map: b.map,
                    x: b.x,
                    y: b.y,
                }),
            });
            at = e.end;
            prev.end_rotation = None; // the rule of the edge before the ride does not carry over
            continue;
        }
        // The path: the trajectory of the edge for this robot's type, or the straight
        // line (S69). Also guards the division by the length when the pose is
        // computed in `step`.
        let path = Arc::new(graph.edge_path(re.edge, &spec.manufacturer, &spec.series_name));
        let length = path.length();
        if !(e.length.is_finite() && e.length > 0.0 && length.is_finite() && length > 0.0) {
            return Err(DriveError::ZeroLengthEdge { edge: e.id.clone() });
        }
        let (start_dir, end_dir) = path.end_directions();
        let heading = edge_heading(kinematics, start_dir, re.orientation, re.orientation_type)
            .map_err(|error| DriveError::Orientation {
                edge: e.id.clone(),
                error,
            })?;
        // An omnidirectional robot given no orientation needs none: the standard
        // lets it choose, and it keeps its heading (S63). Differential robots
        // always drive forwards then (S13).
        let keeps_heading = kinematics == Kinematics::Omnidirectional && re.orientation.is_none();

        // Turning at the start node (S64): the layout says which ways the robot may
        // turn there — what the edge it came in on allows at its end node and what
        // this edge allows at its start node, both (LIF §8.3.13.1). The shorter way
        // is taken if allowed, else the other one; if neither, the edge cannot be
        // driven after the edge before it, and the order is refused.
        let (start_rotation, end_rotation) =
            graph.edge_rotation(re.edge, &spec.manufacturer, &spec.series_name);
        let mut turn_ccw = None;
        let three_wheel = kinematics == Kinematics::ThreeWheel;
        if !keeps_heading {
            let diff = angle_diff(prev.heading, heading);
            if three_wheel {
                // a car-like robot cannot turn on the spot: it must already head the
                // way the edge starts (a little off is taken as aligned, S70)
                if diff.abs() > THREE_WHEEL_TOLERANCE {
                    return Err(DriveError::CannotTurnOnTheSpot {
                        edge: e.id.clone(),
                        node: graph.nodes[e.start].id.clone(),
                    });
                }
            } else if diff.abs() > 1e-9 {
                let allowed = prev
                    .end_rotation
                    .map_or(start_rotation, |r| r.and(start_rotation));
                let shortest_ccw = diff > 0.0;
                if allowed.allows(shortest_ccw) {
                    // the shorter way
                } else if allowed.allows(!shortest_ccw) {
                    turn_ccw = Some(!shortest_ccw);
                } else {
                    return Err(DriveError::RotationNotAllowed {
                        edge: e.id.clone(),
                        node: graph.nodes[e.start].id.clone(),
                    });
                }
            }
            prev.heading = heading;
        }
        // On a curve the heading follows the direction of the path (a differential
        // robot drives it forwards or backwards; an omnidirectional robot with a
        // tangential orientation keeps that angle to it); a global orientation or no
        // orientation (kept) leaves the heading fixed.
        let tangential = !matches!(re.orientation_type, Some(OrientationType::Global));
        let follow = (!path.is_straight() && !keeps_heading && tangential)
            .then(|| angle_diff(start_dir, heading));
        if let Some(offset) = follow {
            prev.heading = normalize_angle(end_dir + offset); // the heading it ends with
        }
        // Turning at speed × curvature may not be faster than the robot can turn
        // (S70): the speed on the curve is limited, and a curve so tight that the
        // limit is below the lowest speed the robot can drive at is refused for a
        // car-like robot (a robot that can turn on the spot just goes slower).
        let curve_speed = follow.and_then(|_| {
            let k = path.max_curvature();
            (k > 1e-9).then(|| spec.limits.max_angular_speed / k)
        });
        if three_wheel && curve_speed.is_some_and(|v| v < spec.min_speed) {
            return Err(DriveError::CurveTooTight { edge: e.id.clone() });
        }
        prev.end_rotation = Some(end_rotation);
        segments.push(Segment {
            path,
            length,
            heading: (!keeps_heading).then_some(heading),
            start_dir,
            end_dir,
            follow,
            reversed: !keeps_heading && angle_diff(start_dir, heading).abs() > FRAC_PI_2,
            ride: None,
            max_speed: graph.edge_max_speed(re.edge, &spec.manufacturer, &spec.series_name),
            curve_speed,
            turn_ccw,
            end_rotation: Some(end_rotation),
        });
        at = e.end;
    }
    Ok((segments, at))
}

#[derive(Debug, Clone)]
pub struct Driver {
    limits: Limits,
    segments: Vec<Segment>,
    /// Actions to queue when segment i ends, i.e. at its end node (parallel to `segments`).
    arrive_actions: Vec<Vec<ActionSpec>>,
    times: ActionTimes,
    queue: ActionQueue,
    /// The whole order, for the state report; `None` when built from a bare route.
    plan: Option<Plan>,
    /// Node where the released base ends (the decision point).
    base_end: NodeIdx,
    /// The order goes on beyond the base: wait at its end for an update.
    has_horizon: bool,
    /// The order was cancelled (`cancelOrder`); no updates are accepted.
    canceled: bool,
    /// Index of the segment being driven, or turned towards.
    current: usize,
    phase: Phase,
    pose: Pose,
    /// Distance along the current segment and speed.
    along: Along,
    /// Seconds a [`Driver::hold`] still lasts.
    hold: f64,
    /// `sequenceId`s of edges that must not be started yet.
    blocked: HashSet<u32>,
    /// Standing still after losing a zone release (see [`Driver::set_halted`]).
    halted: bool,
    /// The protective field is blocked (see [`Driver::set_obstructed`]).
    obstructed: bool,
    /// The edges that are lift rides (they change the map).
    rides: HashSet<EdgeIdx>,
    /// What the robot is like (its type picks the edge properties in the layout).
    spec: RobotSpec,
    /// Drive through nodes where possible (S66).
    pass_through: bool,
    /// `sequenceId`s of edges the robot must stop before (see
    /// [`Driver::set_stop_before_edges`]).
    stop_before: HashSet<u32>,
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
            HashSet::new(),
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
        Driver::from_order_route_with_rides(graph, spec, heading, route, times, HashSet::new())
    }

    /// Like [`Driver::from_order_route`]; the edges in `rides` are lift rides:
    /// the robot may have them in its route (any other edge to another map is
    /// refused with [`DriveError::MapTransition`]). See [`Driver::board`].
    pub fn from_order_route_with_rides(
        graph: &Graph,
        spec: &RobotSpec,
        heading: f64,
        route: &OrderRoute,
        times: ActionTimes,
        rides: HashSet<EdgeIdx>,
    ) -> Result<Driver, DriveError> {
        let mut d = Driver::build(
            graph,
            spec,
            route.start,
            heading,
            &route.edges,
            route.has_horizon,
            &route.node_actions[0],
            route.node_actions[1..].to_vec(),
            times,
            rides,
        )?;
        d.plan = Some(Plan {
            order_id: route.order_id.clone(),
            order_update_id: route.order_update_id,
            nodes: route.plan_nodes.clone(),
            edges: route.plan_edges.clone(),
        });
        Ok(d)
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
        mut times: ActionTimes,
        rides: HashSet<EdgeIdx>,
    ) -> Result<Driver, DriveError> {
        let before = Prev {
            heading: normalize_angle(heading),
            end_rotation: None,
        };
        let (segments, base_end) = build_segments(graph, spec, start, route, &rides, before)?;

        let s = &graph.nodes[start];
        let mut queue = ActionQueue::new();
        queue.push(start_actions.iter().map(|a| times.plan(a)));
        let mut d = Driver {
            limits: spec.limits,
            segments,
            arrive_actions,
            times,
            queue,
            plan: None,
            base_end,
            has_horizon,
            canceled: false,
            current: 0,
            phase: Phase::Finished,
            pose: Pose {
                map: s.map,
                x: s.x,
                y: s.y,
                heading: normalize_angle(heading),
            },
            along: Along { s: 0.0, v: 0.0 },
            hold: 0.0,
            blocked: HashSet::new(),
            halted: false,
            obstructed: false,
            rides,
            spec: spec.clone(),
            pass_through: false,
            stop_before: HashSet::new(),
        };
        d.at_node();
        Ok(d)
    }

    /// The lift ride the robot is waiting to start or is on: the edge of the
    /// ride. `None` in every other phase.
    pub fn ride_edge(&self) -> Option<EdgeIdx> {
        if !matches!(self.phase, Phase::WaitingForLift | Phase::Riding) {
            return None;
        }
        self.segments.get(self.current)?.ride.map(|r| r.edge)
    }

    /// Get into the cabin: only from [`Phase::WaitingForLift`], and not while
    /// the edge is blocked (a zone that is not granted) or the robot is halted.
    /// Returns whether the robot is in the cabin now.
    pub fn board(&mut self) -> bool {
        if self.phase != Phase::WaitingForLift || self.edge_blocked(self.current) || self.halted {
            return false;
        }
        self.phase = Phase::Riding;
        true
    }

    /// Get out of the cabin at the other floor: only from [`Phase::Riding`]. The
    /// pose moves to the end node of the ride (the heading is kept), the actions
    /// of that node start, and the robot goes on with its order.
    pub fn alight(&mut self) -> bool {
        if self.phase != Phase::Riding {
            return false;
        }
        let Some(end) = self.segments.get(self.current).and_then(|s| s.ride) else {
            return false; // (cannot happen: a riding robot is on a ride segment)
        };
        self.pose.map = end.map;
        self.pose.x = end.x;
        self.pose.y = end.y;
        let actions = std::mem::take(&mut self.arrive_actions[self.current]);
        self.queue.push(actions.iter().map(|a| self.times.plan(a)));
        self.current += 1;
        self.at_node();
        true
    }

    /// The edges (by `sequenceId`) the robot must not start driving yet, e.g.
    /// because they enter a RELEASE zone that is not granted. The robot may
    /// still turn towards such an edge, but waits at its start node. The set
    /// replaces the previous one; call it again when clearance changes.
    pub fn set_blocked_edges(&mut self, blocked: HashSet<u32>) {
        self.blocked = blocked;
    }

    /// Must the edge of segment `i` not be started?
    fn edge_blocked(&self, i: usize) -> bool {
        self.plan
            .as_ref()
            .and_then(|p| p.edges.get(i))
            .is_some_and(|e| self.blocked.contains(&e.sequence_id))
    }

    /// The layout edge the robot is on: it is driving and has left the start
    /// node. `None` while turning, standing or at a node. (A door does not close
    /// while a robot is on its edge.)
    pub fn driving_edge(&self) -> Option<EdgeIdx> {
        if self.phase != Phase::Driving || self.along.s <= 0.0 {
            return None;
        }
        self.plan.as_ref()?.edges.get(self.current).map(|e| e.edge)
    }

    /// Let the robot drive **through** nodes without stopping where it can (S66).
    /// Off by default: it stops at every node.
    pub fn set_pass_through(&mut self, on: bool) {
        self.pass_through = on;
    }

    /// The edges (by `sequenceId`) the robot must stop before even if they could
    /// be driven on to (a door's edges: the door must be seen open from a
    /// standstill). Set by the robot; replaces the previous set.
    pub fn set_stop_before_edges(&mut self, edges: HashSet<u32>) {
        self.stop_before = edges;
    }

    /// The speed the robot wants at the end of the edge it is driving: 0 to stop at
    /// the node, more to drive through it (S66). It drives through when
    /// - it is switched on, and the robot is not being stopped (hold, halt, safety
    ///   stop, or a cancelled order);
    /// - the next edge is part of the released route, is not a lift ride, and is
    ///   not one it must not be started or stop before (not granted, a door);
    /// - nothing is to be done at the node (no actions);
    /// - it goes straight on: the same direction, and the heading the next edge
    ///   needs is the heading the robot has (no turn, no wait for a turn);
    ///
    /// Then the speed is the lowest of the two edges' limits, and low enough to
    /// stop at the end of the next edge.
    fn speed_at_end(&self) -> f64 {
        if !self.pass_through || self.canceled || self.is_stopping() {
            return 0.0;
        }
        let (Some(cur), Some(next)) = (
            self.segments.get(self.current),
            self.segments.get(self.current + 1),
        ) else {
            return 0.0;
        };
        let next_seq = self
            .plan
            .as_ref()
            .and_then(|p| p.edges.get(self.current + 1))
            .map(|e| e.sequence_id);
        let barred =
            next_seq.is_some_and(|s| self.blocked.contains(&s) || self.stop_before.contains(&s));
        if cur.ride.is_some()
            || next.ride.is_some()
            || barred
            || !self.arrive_actions[self.current].is_empty()
            // the curves join smoothly: the path ends in the direction the next one starts in
            || angle_diff(cur.end_dir, next.start_dir).abs() > 1e-6
            // and the heading the robot ends this edge with is the one the next needs
            || next.heading.is_some_and(|h| {
                let ends_with = cur.end_heading().unwrap_or(self.pose.heading);
                angle_diff(ends_with, h).abs() > 1e-9
            })
        {
            return 0.0;
        }
        let limit = |s: &Segment| self.segment_speed_limit(s);
        let stoppable = (2.0 * self.limits.max_deceleration * next.length).sqrt();
        limit(cur).min(limit(next)).min(stoppable)
    }

    /// The fastest the robot may drive on a segment: the lowest of its own maximum,
    /// the edge's `maximumSpeed` for its type, and the speed the curve allows (S70).
    fn segment_speed_limit(&self, s: &Segment) -> f64 {
        let mut v = self.limits.max_speed;
        if let Some(m) = s.max_speed {
            v = v.min(m);
        }
        if let Some(c) = s.curve_speed {
            v = v.min(c);
        }
        v
    }

    /// Is the robot driving or turning (not standing at a node waiting or acting)?
    pub fn is_moving(&self) -> bool {
        matches!(self.phase, Phase::Driving | Phase::Rotating)
    }

    /// Stand still (brake to a stop, no turning) until `set_halted(false)`: the
    /// robot has lost the release of a zone it is in and the zone says `STOP`.
    /// Unlike a hold it is not a disturbance and not a protective field violation.
    pub fn set_halted(&mut self, halted: bool) {
        self.halted = halted;
    }

    /// The robot's protective field is not clear (another robot is in it): brake
    /// and stand still until the world says it is clear again (S54). Unlike a
    /// hold it comes from the situation, not from a disturbance, so it lasts as
    /// long as the field is blocked.
    pub fn set_obstructed(&mut self, obstructed: bool) {
        self.obstructed = obstructed;
    }

    /// Is the robot being stopped by its protective field right now? (Only while
    /// it is driving or turning.)
    pub fn is_obstructed(&self) -> bool {
        self.is_moving() && self.obstructed
    }

    /// The direction the robot travels in (unit vector in map coordinates), the
    /// direction of the path where it is on the edge it is driving; `None` unless it
    /// is driving a real edge. Backwards driving travels along the path too,
    /// against the heading.
    pub fn travel_direction(&self) -> Option<(f64, f64)> {
        if self.phase != Phase::Driving {
            return None;
        }
        let s = self.segments.get(self.current)?;
        if s.ride.is_some() {
            return None;
        }
        let t = s.path.tangent_at(self.along.s);
        Some((t.cos(), t.sin()))
    }

    /// Is the robot being stopped, by a hold, by `set_halted` or by its
    /// protective field? (It only counts while it is driving or turning.)
    fn is_stopping(&self) -> bool {
        self.is_moving()
            && (self.hold > TIME_EPSILON || self.halted || self.obstructed || self.must_not_go_on())
    }

    /// The robot is part-way along an edge that it must not be on (its zone grant
    /// was taken back before it entered the zone): it brakes and waits there.
    fn must_not_go_on(&self) -> bool {
        self.phase == Phase::Driving && self.along.s > 0.0 && self.edge_blocked(self.current)
    }

    /// Is a [`Driver::hold`] stopping the robot right now?
    pub fn is_held(&self) -> bool {
        self.is_moving() && self.hold > TIME_EPSILON
    }

    /// Stop the robot for `seconds` (an obstacle in its way, a disturbance):
    /// while driving it brakes with its maximum deceleration, while turning it
    /// stops at once; then it goes on with its order. A longer hold already
    /// running is kept. Actions run on during a hold.
    pub fn hold(&mut self, seconds: f64) {
        self.hold = self.hold.max(seconds);
    }

    /// Id, type and status of every action queued so far (for the state report).
    pub fn action_states(&self) -> impl Iterator<Item = (&str, &str, ActionStatus)> {
        self.queue.states()
    }

    /// `cancelOrder`: stop as soon as possible. While driving an edge the robot
    /// brakes to the node it is heading for (it cannot stop between nodes); at a
    /// node (turning, acting, waiting) it stops at once. Actions that are
    /// scheduled or running become `FAILED`; the rest of the order is dropped
    /// (`nodeStates` and `edgeStates` end at the node where the robot stops).
    /// `orderId` and `orderUpdateId` are kept. No update is accepted afterwards.
    pub fn cancel(&mut self) {
        self.canceled = true;
        self.has_horizon = false;
        self.queue.cancel_all();

        // actions of nodes not yet reached were never triggered: FAILED
        let pending: Vec<ActionSpec> = self
            .arrive_actions
            .iter_mut()
            .skip(self.current)
            .flat_map(std::mem::take)
            .collect();
        let mut times = self.times.clone();
        self.queue
            .push_failed(pending.iter().map(|a| times.plan(a)));

        // keep the segment being driven (to its end node), or none if standing at a
        // node; a robot in the cabin cannot stop there and finishes the ride
        let on_the_way = matches!(self.phase, Phase::Driving | Phase::Riding);
        let keep = if on_the_way {
            self.current + 1
        } else {
            self.current
        };
        self.segments.truncate(keep);
        self.arrive_actions.truncate(keep);
        if let Some(plan) = &mut self.plan {
            plan.nodes.truncate(keep + 1);
            plan.edges.truncate(keep);
        }
        if !on_the_way {
            self.at_node();
        }
    }

    pub fn is_canceled(&self) -> bool {
        self.canceled
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
        // the numbers go on from the end of the base
        if let Some(plan) = &self.plan
            && let (Some(end), Some(first)) = (
                plan.nodes.get(self.segments.len()),
                route.plan_nodes.first(),
            )
            && end.sequence_id != first.sequence_id
        {
            return Err(DriveError::SequenceNotContinued {
                expected: end.sequence_id,
                found: first.sequence_id,
            });
        }
        // the update goes on from the end of what is planned: the heading the robot
        // will have there, and what the last edge allows at that node
        let before = Prev {
            heading: self
                .segments
                .iter()
                .rev()
                .find_map(|s| s.end_heading())
                .unwrap_or(self.pose.heading),
            end_rotation: self.segments.last().and_then(|s| s.end_rotation),
        };
        let (segments, base_end) = build_segments(
            graph,
            &self.spec,
            route.start,
            &route.edges,
            &self.rides,
            before,
        )?;
        if let Some(plan) = &mut self.plan {
            // plan entry i is the node/edge at segment i: keep what came before the
            // decision point (index = segments so far), the update replaces the rest
            let at = self.segments.len();
            plan.order_update_id = route.order_update_id;
            plan.nodes.truncate(at);
            plan.nodes.extend(route.plan_nodes.iter().cloned());
            plan.edges.truncate(at);
            plan.edges.extend(route.plan_edges.iter().cloned());
        }
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

    /// The part of [`Driver::progress`] the simulator reads itself every step,
    /// borrowed instead of copied.
    pub(crate) fn glance(&self) -> Glance<'_> {
        let (last_node, remaining_nodes, remaining_edges) = match &self.plan {
            Some(p) => (
                p.nodes.get(self.current),
                p.nodes.get(self.current + 1..).unwrap_or(&[]),
                p.edges.get(self.current..).unwrap_or(&[]),
            ),
            None => (None, &[][..], &[][..]),
        };
        let stopped = self.is_stopping() && (self.phase == Phase::Rotating || self.along.v <= 1e-9);
        Glance {
            last_node,
            remaining_nodes,
            remaining_edges,
            // a held robot that has come to a halt is not driving. Standing at a
            // node (Acting, Waiting, Finished) is not driving; turning on the spot
            // is counted as driving (the drive wheels move)
            driving: self.is_moving() && !stopped,
            // held or obstructed, the robot reports a protective field violation
            held: self.is_held() || self.is_obstructed(),
            // (the actions still to be triggered are WAITING)
            acting: self
                .queue
                .states()
                .any(|(_, _, s)| s == ActionStatus::Running),
        }
    }

    /// What the state report says about the order and the robot's movement.
    pub fn progress(&self) -> Progress {
        let (order_id, order_update_id) = match &self.plan {
            Some(p) => (
                Some(p.order_id.clone()).filter(|id| !id.as_str().is_empty()),
                p.order_update_id,
            ),
            None => (None, 0),
        };
        let now = self.glance();

        // Actions: those queued so far, then those still to be triggered (WAITING).
        let mut actions: Vec<ActionState> = self.queue.reported().collect();
        for a in self.arrive_actions.iter().skip(self.current).flatten() {
            actions.push(ActionState {
                id: a.action_id.clone(),
                kind: Some(a.action_type.clone()),
                status: ActionStatus::Waiting,
            });
        }

        let seg = self.segments.get(self.current);
        let stopping = self.is_stopping();
        let (vx, vy, omega) = match (self.phase, seg) {
            (Phase::Rotating, _) if stopping => (0.0, 0.0, 0.0),
            // an omnidirectional robot moves sideways as well: the speed along the
            // edge, in the robot's frame (x forward, y left)
            (Phase::Driving, Some(s)) if self.spec.kinematics == Kinematics::Omnidirectional => {
                let rel = s.path.tangent_at(self.along.s) - self.pose.heading;
                let omega = self.turning_rate(s);
                (self.along.v * rel.cos(), self.along.v * rel.sin(), omega)
            }
            (Phase::Driving, Some(s)) => (
                if s.reversed {
                    -self.along.v
                } else {
                    self.along.v
                },
                0.0,
                self.turning_rate(s),
            ),
            (Phase::Rotating, Some(s)) => (
                0.0,
                0.0,
                match s.turn_ccw {
                    Some(ccw) => {
                        self.limits
                            .max_angular_speed
                            .copysign(if ccw { 1.0 } else { -1.0 })
                    }
                    None => self.limits.max_angular_speed.copysign(angle_diff(
                        self.pose.heading,
                        s.heading.unwrap_or(self.pose.heading),
                    )),
                },
            ),
            _ => (0.0, 0.0, 0.0),
        };

        Progress {
            order_id,
            order_update_id,
            last_node: now.last_node.cloned(),
            remaining_nodes: now.remaining_nodes.to_vec(),
            remaining_edges: now.remaining_edges.to_vec(),
            driving: now.driving,
            held: now.held,
            pose: self.pose,
            vx,
            vy,
            omega,
            actions,
        }
    }

    /// `orderId` and `orderUpdateId` of the order being executed or last finished;
    /// `None` if the driver was built without an order.
    pub fn order(&self) -> Option<(&str, u32)> {
        self.plan
            .as_ref()
            .map(|p| (p.order_id.as_str(), p.order_update_id))
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
        // a car-like robot never turns on the spot: a heading a little off (it was
        // accepted when the order was taken) is taken as the one wanted (S70)
        let tolerance = if self.spec.kinematics == Kinematics::ThreeWheel {
            THREE_WHEEL_TOLERANCE
        } else {
            1e-9
        };
        if tolerance > 1e-9
            && let Some(h) = self.segments.get(self.current).and_then(|s| s.heading)
            && angle_diff(self.pose.heading, h).abs() <= tolerance
        {
            self.pose.heading = h;
        }
        self.phase = if !self.queue.driving_allowed() {
            Phase::Acting
        } else {
            match self.segments.get(self.current) {
                Some(seg) if seg.ride.is_some() => Phase::WaitingForLift,
                // no heading wanted, or already the heading wanted: drive
                Some(seg)
                    if seg
                        .heading
                        .is_none_or(|h| angle_diff(self.pose.heading, h).abs() <= tolerance) =>
                {
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
        let held = self.is_stopping(); // a hold, or halted after losing a zone release
        if self.hold > 0.0 {
            self.hold = (self.hold - dt).max(0.0);
        }
        match self.phase {
            Phase::Finished | Phase::Waiting => {}
            // the world decides when the robot gets in and out of the cabin
            Phase::WaitingForLift | Phase::Riding => {}
            Phase::Acting => self.at_node(),
            Phase::WaitingForClearance => {
                if !self.edge_blocked(self.current) {
                    self.phase = Phase::Driving;
                }
            }
            // about to start an edge that must not be started: wait at its start node
            Phase::Driving if self.along.s == 0.0 && self.edge_blocked(self.current) => {
                self.phase = Phase::WaitingForClearance;
                self.along.v = 0.0; // (it was driving through the node: it stops)
            }
            Phase::Rotating if held => {} // stopped while turning
            Phase::Rotating => {
                let Some(seg) = self.segments.get(self.current).cloned() else {
                    self.phase = Phase::Driving; // (cannot happen)
                    return;
                };
                // (a robot turns only towards a heading; none: nothing to turn to)
                let Some(target) = seg.heading else {
                    self.phase = Phase::Driving;
                    return;
                };
                let rate = self.limits.max_angular_speed;
                let (h, done) = match seg.turn_ccw {
                    // the layout allows only one direction, and it is the long way
                    Some(ccw) => rotate_in(self.pose.heading, target, rate, dt, ccw),
                    None => rotate(self.pose.heading, target, rate, dt),
                };
                self.pose.heading = h;
                if done {
                    self.phase = Phase::Driving;
                }
            }
            Phase::Driving => {
                let seg = self.segments[self.current].clone();
                let goal = EdgeGoal {
                    length: seg.length,
                    // held: a speed limit of 0 makes it brake and stay stopped; the
                    // edge may allow less than the robot can do (S55)
                    v_limit: if held {
                        0.0
                    } else {
                        self.segment_speed_limit(&seg)
                    },
                    // 0: stop at the node; more: drive through it (S66)
                    v_end: self.speed_at_end(),
                };
                let (along, arrived) = advance(self.along, goal, &self.limits, dt);
                self.along = along;
                let (x, y) = seg.path.point_at(along.s);
                self.pose.x = x;
                self.pose.y = y;
                // on a curve the heading follows the direction of the path
                if let Some(offset) = seg.follow {
                    self.pose.heading = normalize_angle(seg.path.tangent_at(along.s) + offset);
                }
                if arrived {
                    let actions = std::mem::take(&mut self.arrive_actions[self.current]);
                    self.queue.push(actions.iter().map(|a| self.times.plan(a)));
                    self.current += 1;
                    if goal.v_end > 0.0 {
                        // through the node: the next edge starts at its start, at speed
                        self.along = Along {
                            s: 0.0,
                            v: goal.v_end,
                        };
                    } else {
                        self.at_node();
                    }
                }
            }
        }
    }
}

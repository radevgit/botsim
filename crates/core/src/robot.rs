//! A simulated robot as the controller sees it: it receives VDA 5050 orders,
//! accepts or rejects them, executes them, and reports its state.
//!
//! Order acceptance follows VDA 5050 §6.1.2 (Figure 8), in this order:
//! 1. the order must be valid (structure, release rules, ids in the layout);
//! 2. a **different `orderId`** is a new order:
//!    - the robot must be idle (order done, and no horizon to wait for), else
//!      `OTHER_ORDER_ACTIVE`;
//!    - `orderUpdateId` must be 0 (the standard names no error: `VALIDATION_FAILURE`);
//!    - the first node must be where the robot stands, else `START_NODE_OUT_OF_RANGE`;
//! 3. the **same `orderId`** is an update: a lower `orderUpdateId` →
//!    `OUTDATED_ORDER_UPDATE`, after a `cancelOrder` → `ORDER_UPDATE_FOLLOWING_CANCEL`,
//!    the same → `SAME_ORDER_UPDATE_ID`, higher → it must start at the decision
//!    point (the end of the base so far).
//!
//! A rejected message changes nothing except that its error is reported in the
//! state ("until new order is accepted"); an accepted order or update clears the
//! errors.
//!
//! Instant actions: only `cancelOrder` (see [`Robot::receive_instant_actions`]).
//!
//! Not yet: operating modes (`MOBILE_ROBOT_NOT_AVAILABLE`), other instant actions,
//! a deviation range around the start node (a fixed tolerance is used).

use std::collections::{HashMap, HashSet};

use vda5050::instant_actions::InstantActions;
use vda5050::order::Order;
use vda5050::state::{ActionStatus, State};

use crate::actions::ActionTimes;
use crate::drive::{DriveError, Driver, Phase, Pose};
use crate::errors::{ErrorType, RobotError};
use crate::graph::{EdgeIdx, Graph, NodeIdx};
use crate::order_route::route_from_order;
use crate::spec::RobotSpec;
use crate::state_report::{Header, InstantActionState, build_state};
use crate::zone_requests::{ZoneRequestTiming, ZoneRequester};
use crate::zones::Zones;

/// How close (metres) the robot must be to the first node of a new order.
pub const START_NODE_TOLERANCE: f64 = 0.1;

/// What happened to a received order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reception {
    Accepted,
    /// Rejected; the error is in the robot's error list.
    Rejected(ErrorType),
}

#[derive(Debug, Clone)]
pub struct Robot {
    spec: RobotSpec,
    times: ActionTimes,
    driver: Driver,
    errors: Vec<RobotError>,
    /// All instant actions received, with their status (kept, as the standard says).
    instant: Vec<InstantActionState>,
    /// Index in `instant` of a `cancelOrder` that is still stopping the robot.
    cancel_action: Option<usize>,
    /// RELEASE zone requests; `None` if the run has no zones.
    zone_requests: Option<ZoneRequester>,
    /// The time now, milliseconds since 1970 (set by the world).
    now_ms: i64,
    /// The layout edges that are lift rides (set by the world).
    rides: HashSet<EdgeIdx>,
    /// Ids of the edges of doors that are not fully open (set by the world).
    closed_door_edges: HashSet<String>,
    /// Passage width of the doors, by edge id (set by the world).
    door_widths: HashMap<String, f64>,
    /// The last update blocked an edge because of a closed door.
    door_blocked: bool,
}

impl Robot {
    /// An idle robot standing on `start` with `heading`.
    pub fn new(
        graph: &Graph,
        spec: RobotSpec,
        times: ActionTimes,
        start: NodeIdx,
        heading: f64,
    ) -> Result<Robot, DriveError> {
        let driver = Driver::new(graph, &spec, start, heading, &[])?;
        Ok(Robot {
            spec,
            times,
            driver,
            errors: Vec::new(),
            instant: Vec::new(),
            cancel_action: None,
            zone_requests: None,
            now_ms: 0,
            rides: HashSet::new(),
            closed_door_edges: HashSet::new(),
            door_widths: HashMap::new(),
            door_blocked: false,
        })
    }

    /// Tell the robot the passage width of the doors, by edge id: it refuses
    /// orders with an edge of a door that is narrower than the robot.
    pub fn set_door_widths(&mut self, widths: HashMap<String, f64>) {
        self.door_widths = widths;
    }

    /// Tell the robot which door edges it must not start now (the door is not
    /// fully open). Set by the world every step.
    pub fn set_closed_door_edges(&mut self, closed: HashSet<String>) {
        if closed != self.closed_door_edges {
            self.closed_door_edges = closed;
            self.update_zone_requests();
        }
    }

    /// The id of the edge the robot is driving on, if it has left the node.
    pub fn driving_edge_id(&self) -> Option<&str> {
        self.driver.driving_edge_id()
    }

    /// How wide the robot is (metres), across its direction of travel.
    pub fn width(&self) -> f64 {
        match self.spec.shape {
            crate::spec::Shape::Rect { half_width, .. } => 2.0 * half_width,
            crate::spec::Shape::Circle { radius, .. } => 2.0 * radius,
        }
    }

    /// The first edge of `edges` that leads through a door narrower than the robot.
    fn too_narrow(&self, edges: &[crate::order_route::PlanEdge]) -> Option<String> {
        edges
            .iter()
            .find(|e| {
                self.door_widths
                    .get(&e.edge_id)
                    .is_some_and(|&w| self.width() > w)
            })
            .map(|e| e.edge_id.clone())
    }

    /// Tell the robot which edges of the layout are lift rides. Orders with such
    /// an edge are accepted from now on; the world lets the robot in and out of
    /// the cabin (see [`crate::drive::Driver::board`]).
    pub fn set_rides(&mut self, rides: HashSet<EdgeIdx>) {
        self.rides = rides;
    }

    /// The lift ride the robot waits to start or is on (its layout edge).
    pub fn ride_edge(&self) -> Option<EdgeIdx> {
        self.driver.ride_edge()
    }

    /// Is the robot in a lift cabin?
    pub fn is_riding(&self) -> bool {
        self.driver.phase() == Phase::Riding
    }

    /// Get into the cabin; see [`crate::drive::Driver::board`].
    pub fn board(&mut self) -> bool {
        self.driver.board()
    }

    /// Get out of the cabin; see [`crate::drive::Driver::alight`].
    pub fn alight(&mut self) -> bool {
        let out = self.driver.alight();
        if out {
            self.update_zone_requests();
        }
        out
    }

    /// Give the robot the zones of the run: it will state RELEASE zone requests
    /// (see [`crate::zone_requests`]), by `timing`.
    pub fn set_zones(&mut self, zones: Zones, timing: ZoneRequestTiming) {
        self.zone_requests = Some(ZoneRequester::new(zones, timing));
    }

    /// Change when this robot requests zones. No effect without zones.
    pub fn set_zone_request_timing(&mut self, timing: ZoneRequestTiming) {
        if let Some(z) = self.zone_requests.as_mut() {
            z.set_timing(timing);
        }
    }

    /// The robot's `zoneRequests` as it states them (empty without zones).
    pub fn zone_requests(&self) -> Vec<vda5050::state::ZoneRequest> {
        self.zone_requests
            .as_ref()
            .map(ZoneRequester::to_state)
            .unwrap_or_default()
    }

    /// Bring the zone requests up to date with where the robot is and what is ahead.
    fn update_zone_requests(&mut self) {
        // nothing to keep up to date, nothing blocked before: skip the work
        if self.zone_requests.is_none() && self.closed_door_edges.is_empty() && !self.door_blocked {
            return;
        }
        let p = self.driver.progress();
        let mut blocked = HashSet::new();
        if let Some(zr) = self.zone_requests.as_mut() {
            zr.update(
                &p.remaining_edges,
                p.pose.map,
                p.pose.x,
                p.pose.y,
                self.now_ms,
            );
            // do not start an edge that enters a zone that is not granted
            blocked = zr.blocked_edges(&p.remaining_edges);
            // after losing the release of a zone it is in: STOP means stand still
            self.driver.set_halted(zr.halt());
        }
        // nor an edge through a door that is not fully open
        let doors: Vec<u32> = p
            .remaining_edges
            .iter()
            .filter(|e| self.closed_door_edges.contains(&e.edge_id))
            .map(|e| e.sequence_id)
            .collect();
        self.door_blocked = !doors.is_empty();
        blocked.extend(doors);
        self.driver.set_blocked_edges(blocked);
    }

    /// The robot's clock: milliseconds since 1970, set by the world before every
    /// step. Used for lease expiry of zone grants.
    pub fn set_time(&mut self, now_ms: i64) {
        self.now_ms = now_ms;
    }

    /// Receive a `responses` message: the controller's answers to zone requests.
    /// Answers to requests the robot does not have are ignored.
    pub fn receive_responses(&mut self, msg: &vda5050::responses::Responses) {
        if let Some(zr) = self.zone_requests.as_mut() {
            for r in &msg.responses {
                // a lease that cannot be read is treated as no lease
                let lease = r
                    .lease_expiry
                    .as_deref()
                    .and_then(crate::world::unix_ms_from_iso8601);
                zr.respond(&r.request_id, r.grant_type, lease);
            }
        }
        self.update_zone_requests();
    }

    pub fn pose(&self) -> Pose {
        self.driver.pose()
    }

    pub fn phase(&self) -> Phase {
        self.driver.phase()
    }

    /// Driving or turning (not standing at a node).
    pub fn is_moving(&self) -> bool {
        self.driver.is_moving()
    }

    /// Give every action a random extra time of up to `variation` (0.2 = 20 %),
    /// drawn from `rng` (S38). Applies to the orders received from now on.
    pub fn set_duration_noise(&mut self, rng: crate::jobs::Rng, variation: f64) {
        self.times.noise = Some(crate::actions::DurationNoise { rng, variation });
    }

    /// Is a hold stopping the robot right now?
    pub fn is_held(&self) -> bool {
        self.driver.is_held()
    }

    /// Stop for `seconds`, see [`Driver::hold`].
    pub fn hold(&mut self, seconds: f64) {
        self.driver.hold(seconds);
    }

    pub fn spec(&self) -> &RobotSpec {
        &self.spec
    }

    /// The body outline, relative to the kinematic center.
    pub fn shape(&self) -> &crate::spec::Shape {
        &self.spec.shape
    }

    pub fn errors(&self) -> &[RobotError] {
        &self.errors
    }

    /// Advance by `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        self.driver.step(dt);
        self.update_cancel();
        self.update_zone_requests();
    }

    /// The state message.
    pub fn state(&self, graph: &Graph, header: &Header) -> State {
        let mut state = build_state(
            graph,
            header,
            &self.driver.progress(),
            &self.errors,
            &self.instant,
        );
        // an empty array if there are zones but no request (as the standard says)
        if let Some(zr) = &self.zone_requests {
            state.zone_requests = Some(self.zone_requests());
            state
                .errors
                .extend(zr.errors().iter().map(RobotError::to_state_error));
        }
        state
    }

    fn reject(&mut self, error: RobotError) -> Reception {
        let t = error.error_type;
        self.errors.push(error);
        Reception::Rejected(t)
    }

    fn reject_order(&mut self, t: ErrorType, order: &Order, text: &str) -> Reception {
        let e = RobotError::for_order(t, &order.order_id, order.order_update_id, text.to_string());
        self.reject(e)
    }

    /// Receive an `order` message.
    pub fn receive_order(&mut self, graph: &Graph, order: &Order) -> Reception {
        // 1. valid?
        let route = match route_from_order(graph, order) {
            Ok(r) => r,
            Err(e) => {
                let err = RobotError::from_order_error(&e, &order.order_id, order.order_update_id);
                return self.reject(err);
            }
        };

        if let Some(edge) = self.too_narrow(&route.plan_edges) {
            let e = DriveError::DoorTooNarrow { edge };
            let err = RobotError::from_drive_error(&e, &order.order_id, order.order_update_id);
            return self.reject(err);
        }

        let current = self.driver.order().map(|(id, u)| (id.to_string(), u));
        let is_update = current
            .as_ref()
            .is_some_and(|(id, _)| *id == order.order_id);

        if !is_update {
            // 2. a new order
            if self.driver.phase() != Phase::Finished {
                return self.reject_order(
                    ErrorType::OtherOrderActive,
                    order,
                    "another order is still active",
                );
            }
            if order.order_update_id != 0 {
                return self.reject_order(
                    ErrorType::ValidationFailure,
                    order,
                    "a new order must have orderUpdateId 0",
                );
            }
            let p = self.driver.pose();
            let start = &graph.nodes[route.start];
            let near =
                p.map == start.map && (p.x - start.x).hypot(p.y - start.y) <= START_NODE_TOLERANCE;
            if !near {
                return self.reject_order(
                    ErrorType::StartNodeOutOfRange,
                    order,
                    "the first node of the order is not where the robot stands",
                );
            }
            return match Driver::from_order_route_with_rides(
                graph,
                &self.spec,
                p.heading,
                &route,
                self.times.for_next_order(),
                self.rides.clone(),
            ) {
                Ok(d) => {
                    self.driver = d;
                    self.errors.clear();
                    if let Some(zr) = self.zone_requests.as_mut() {
                        zr.forget_edges();
                        zr.learn_edges(graph, &route.plan_edges);
                    }
                    self.update_zone_requests();
                    Reception::Accepted
                }
                Err(e) => {
                    let err =
                        RobotError::from_drive_error(&e, &order.order_id, order.order_update_id);
                    self.reject(err)
                }
            };
        }

        // 3. an update of the current order
        let (_, current_update) = current.expect("is_update implies a current order");
        if order.order_update_id < current_update {
            return self.reject_order(
                ErrorType::OutdatedOrderUpdate,
                order,
                "orderUpdateId is lower than the current one",
            );
        }
        if self.driver.is_canceled() {
            return self.reject_order(
                ErrorType::OrderUpdateFollowingCancel,
                order,
                "the order was cancelled; no updates are accepted",
            );
        }
        if order.order_update_id == current_update {
            return self.reject_order(
                ErrorType::SameOrderUpdateId,
                order,
                "orderUpdateId is the same as the current one",
            );
        }
        match self.driver.extend(graph, &route) {
            Ok(()) => {
                self.errors.clear();
                if let Some(zr) = self.zone_requests.as_mut() {
                    zr.learn_edges(graph, &route.plan_edges);
                }
                self.update_zone_requests();
                Reception::Accepted
            }
            Err(e) => {
                let err = RobotError::from_drive_error(&e, &order.order_id, order.order_update_id);
                self.reject(err)
            }
        }
    }

    /// Receive an `instantActions` message. Every action is listed in
    /// `instantActionStates`. Supported: `cancelOrder` (optional parameter
    /// `orderId`). It fails (`FAILED` + `NO_ORDER_TO_CANCEL`, reference `actionId`)
    /// if the robot is idle or the `orderId` is not the active order. Any other
    /// type fails with `INVALID_INSTANT_ACTION`.
    pub fn receive_instant_actions(&mut self, msg: &InstantActions) {
        for a in &msg.actions {
            self.instant.push(InstantActionState {
                action_id: a.action_id.clone(),
                action_type: a.action_type.clone(),
                status: ActionStatus::Failed,
            });
            let idx = self.instant.len() - 1;

            if a.action_type != "cancelOrder" {
                self.errors.push(action_error(
                    ErrorType::InvalidInstantAction,
                    &a.action_id,
                    "unsupported instant action",
                ));
                continue;
            }

            let wanted = a
                .action_parameters
                .iter()
                .flatten()
                .find(|p| p.key == "orderId")
                .and_then(|p| p.value.as_str());
            let active = self.driver.phase() != Phase::Finished;
            let matches = match (wanted, self.driver.order()) {
                (Some(w), Some((id, _))) => w == id,
                (Some(_), None) => false,
                (None, _) => true,
            };
            if !active || !matches {
                self.errors.push(action_error(
                    ErrorType::NoOrderToCancel,
                    &a.action_id,
                    "no active order to cancel",
                ));
                continue;
            }

            self.driver.cancel();
            self.instant[idx].status = ActionStatus::Running;
            self.cancel_action = Some(idx);
            self.update_cancel();
        }
    }

    /// The `cancelOrder` action is FINISHED once all movement and actions have stopped.
    fn update_cancel(&mut self) {
        if let Some(idx) = self.cancel_action
            && self.driver.phase() == Phase::Finished
        {
            self.instant[idx].status = ActionStatus::Finished;
            self.cancel_action = None;
        }
    }
}

fn action_error(t: ErrorType, action_id: &str, text: &str) -> RobotError {
    RobotError {
        error_type: t,
        references: vec![("actionId".to_string(), action_id.to_string())],
        description: Some(text.to_string()),
    }
}

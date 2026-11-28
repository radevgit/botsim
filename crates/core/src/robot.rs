//! A simulated robot as the fleet server sees it: it receives orders, accepts or
//! rejects them, executes them, and reports its state, all in the compact form of
//! [`botsim_link`] (VDA 5050 meaning).
//!
//! Order acceptance follows VDA 5050 §6.1.2 (Figure 8), in this order:
//! 1. the order must be valid (see [`crate::order_route`]: numbers in the layout,
//!    edges that connect, a new order from `sequenceId` 0);
//! 2. a **different `orderId`** is a new order:
//!    - the robot must be idle (order done, and no horizon to wait for), else
//!      `OTHER_ORDER_ACTIVE`;
//!    - `orderUpdateId` must be 0 (the standard names no error: `VALIDATION_FAILURE`);
//!    - the first node must be where the robot stands, else `START_NODE_OUT_OF_RANGE`;
//! 3. the **same `orderId`** is an update: a lower `orderUpdateId` →
//!    `OUTDATED_ORDER_UPDATE`, after a `cancelOrder` → `ORDER_UPDATE_FOLLOWING_CANCEL`,
//!    the same → `SAME_ORDER_UPDATE_ID`, higher → it must start at the decision
//!    point (the end of the base so far), with the `sequenceId` of that node.
//!
//! A rejected message changes nothing except that its error is reported in the
//! state ("until new order is accepted"); an accepted order or update clears the
//! errors. A message that whoever reads the robot's messages could not read is
//! reported the same way ([`Robot::take_unread`]).
//!
//! Instant actions: see [`Robot::take_instant_actions`].
//!
//! Not yet: operating modes (`MOBILE_ROBOT_NOT_AVAILABLE`), other instant actions,
//! a deviation range around the start node (a fixed tolerance is used).

use std::collections::{HashMap, HashSet};

use botsim_link::ids::{ActionId, ActionType, ZoneSetId};
use botsim_link::instant::{InstantAction, InstantKind};
use botsim_link::order::Order;
use botsim_link::response::Response;
use botsim_link::state::{
    ActionState, ActionStatus, Error, ErrorKind, ErrorLevel, State, ZoneRequest, ZoneSetInfo,
};
use botsim_link::zone_set::ZoneSet;

use crate::actions::ActionTimes;
use crate::battery::Battery;
use crate::drive::{DriveError, Driver, Glance, Phase, Pose};
use crate::errors::RobotError;
use crate::order_route::route_from_order;
use crate::robot_zones::RobotZones;
use crate::state_report::build_state;
use crate::zone_requests::{ZoneRequestTiming, ZoneRequester};
use botsim_model::errors::ErrorType;
use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_model::spec::RobotSpec;
use botsim_model::zones::Zones;

/// How close (metres) the robot must be to the first node of a new order.
pub const START_NODE_TOLERANCE: f64 = 0.1;

/// What happened to a received order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reception {
    Accepted,
    /// Rejected; the error is in the robot's error list.
    Rejected(ErrorType),
    /// Not delivered yet (the world has a message delay, S58): the sender cannot
    /// know the outcome; it shows in the robot's state later, as in VDA 5050
    /// (the `orderId` of the state, or an error that names the order).
    InTransit,
}

/// How long (ms) the state reports a zone set that was not taken over: "a
/// reasonable amount of time for the fleet control to notice" (VDA 5050 §6.4.2);
/// our choice, 10 s.
pub const ZONE_SET_ERROR_MS: i64 = 10_000;

/// What was sent to a robot and could not be read, as whoever reads its messages
/// reports it (a translator): the error it would be refused with.
#[derive(Debug, Clone, PartialEq)]
pub enum Unread {
    /// An order or order update.
    Order(RobotError),
    /// A zone set.
    ZoneSet(RobotError),
    /// A message of instant actions of which not all could be read: all its
    /// entries, in their order.
    InstantActions(Vec<InstantEntry>),
}

/// An entry of a message of instant actions.
#[derive(Debug, Clone, PartialEq)]
pub enum InstantEntry {
    Read(InstantAction),
    /// One that could not be read: it is listed `FAILED`, with its error.
    Unread {
        id: ActionId,
        /// Its `actionType`, if that much is known.
        kind: Option<ActionType>,
        error: RobotError,
    },
}

/// A zone set on the robot, with its `zoneSetStatus`.
#[derive(Debug, Clone)]
struct StoredZoneSet {
    zones: RobotZones,
    enabled: bool,
}

#[derive(Debug, Clone)]
pub struct Robot {
    spec: RobotSpec,
    times: ActionTimes,
    driver: Driver,
    errors: Vec<RobotError>,
    /// All instant actions received, with their status (kept, as the standard says).
    instant: Vec<ActionState>,
    /// Index in `instant` of a `cancelOrder` that is still stopping the robot.
    cancel_action: Option<usize>,
    /// RELEASE zone requests; `None` while no zone set is enabled.
    zone_requests: Option<ZoneRequester>,
    /// The zone sets the controller has sent (S62), enabled or not. At most one is
    /// enabled (simplification: the standard allows one per map).
    zone_sets: Vec<StoredZoneSet>,
    /// When this robot requests RELEASE zones (S41); used when a zone set is enabled.
    zone_timing: ZoneRequestTiming,
    /// Errors about zone sets that were not taken over, with the time (ms since
    /// 1970) until which the state reports them.
    zone_set_errors: Vec<(RobotError, i64)>,
    /// The time now, milliseconds since 1970 (set by the world).
    now_ms: i64,
    /// The layout edges that are lift rides (set by the world).
    rides: HashSet<EdgeIdx>,
    /// The edges of doors that are not fully open (set by the world).
    closed_door_edges: HashSet<EdgeIdx>,
    /// Passage width of the doors, by edge (set by the world).
    door_widths: HashMap<EdgeIdx, f64>,
    /// The last update blocked an edge because of a closed door.
    door_blocked: bool,
    /// All edges that have a door (set by the world).
    door_edges: HashSet<EdgeIdx>,
    /// Drive through nodes where possible (S66).
    pass_through: bool,
    /// The battery; `None`: no battery model, the state reports 100 %.
    battery: Option<Battery>,
    /// The power (W) of the charger the robot stands on; set by the world.
    charger_power_w: Option<f64>,
    /// `startCharging` was received and not ended by `stopCharging`, moving or a
    /// full battery.
    charging_requested: bool,
    /// The robot gains charge right now.
    charging: bool,
    /// The battery has been empty at some time.
    ran_empty: bool,
    /// Simulated seconds spent charging.
    charging_seconds: f64,
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
            zone_sets: Vec::new(),
            zone_timing: ZoneRequestTiming::default(),
            zone_set_errors: Vec::new(),
            now_ms: 0,
            rides: HashSet::new(),
            closed_door_edges: HashSet::new(),
            door_widths: HashMap::new(),
            door_blocked: false,
            door_edges: HashSet::new(),
            pass_through: false,
            battery: None,
            charger_power_w: None,
            charging_requested: false,
            charging: false,
            ran_empty: false,
            charging_seconds: 0.0,
        })
    }

    /// The world tells the robot whether it stands on a charger and its power (W).
    pub fn set_charger_power(&mut self, power_w: Option<f64>) {
        self.charger_power_w = power_w;
    }

    /// Does the robot gain charge right now?
    pub fn is_charging(&self) -> bool {
        self.charging
    }

    /// Is the battery empty (and not charging)? The robot then stands still.
    pub fn is_out_of_charge(&self) -> bool {
        !self.charging && self.battery.as_ref().is_some_and(|b| b.is_empty())
    }

    /// Has the battery been empty at some time?
    pub fn ran_empty(&self) -> bool {
        self.ran_empty
    }

    /// Simulated seconds the robot has been charging.
    pub fn charging_seconds(&self) -> f64 {
        self.charging_seconds
    }

    /// Give the robot a battery (see [`crate::battery`]); its charge is reported in
    /// the state and falls as the robot drives, works and stands.
    pub fn set_battery(&mut self, battery: Battery) {
        self.battery = Some(battery);
    }

    /// The battery, if the robot has one.
    pub fn battery(&self) -> Option<&Battery> {
        self.battery.as_ref()
    }

    /// Drive through nodes without stopping where it can (S66, see
    /// [`Driver::set_pass_through`]); applies to this and the later orders.
    pub fn set_pass_through(&mut self, on: bool) {
        self.pass_through = on;
        self.driver.set_pass_through(on);
    }

    /// Tell the robot which edges have a door (all of them, open or not): it does
    /// not drive through the node in front of such an edge (S66).
    pub fn set_door_edges(&mut self, edges: HashSet<EdgeIdx>) {
        self.door_edges = edges;
    }

    /// Tell the robot the passage width of the doors, by edge: it refuses orders
    /// with an edge of a door that is narrower than the robot.
    pub fn set_door_widths(&mut self, widths: HashMap<EdgeIdx, f64>) {
        self.door_widths = widths;
    }

    /// Tell the robot which door edges it must not start now (the door is not
    /// fully open). Set by the world every step.
    pub fn set_closed_door_edges(&mut self, closed: HashSet<EdgeIdx>) {
        if closed != self.closed_door_edges {
            self.closed_door_edges = closed;
            self.update_zone_requests();
        }
    }

    /// The edge the robot is driving on, if it has left the node.
    pub fn driving_edge(&self) -> Option<EdgeIdx> {
        self.driver.driving_edge()
    }

    /// How wide the robot is (metres), across its direction of travel.
    pub fn width(&self) -> f64 {
        match self.spec.shape {
            botsim_model::spec::Shape::Rect { half_width, .. } => 2.0 * half_width,
            botsim_model::spec::Shape::Circle { radius, .. } => 2.0 * radius,
        }
    }

    /// The first node or edge of the order that the layout does not allow for
    /// this robot's type (LIF `mobileRobotTypeNodeProperties` and
    /// `mobileRobotTypeEdgeProperties`), as text for the error.
    fn not_allowed(&self, graph: &Graph, route: &crate::order_route::OrderRoute) -> Option<String> {
        let (m, s) = (&self.spec.manufacturer, &self.spec.series_name);
        // (the numbers of a route are in the layout)
        let node = route
            .plan_nodes
            .iter()
            .find(|n| !graph.node_allows(n.node, m, s));
        if let Some(n) = node {
            return Some(format!("node '{}'", graph.nodes[n.node].id));
        }
        route
            .plan_edges
            .iter()
            .find(|e| !graph.edge_allows(e.edge, m, s))
            .map(|e| format!("edge '{}'", graph.edges[e.edge].id))
    }

    /// The first edge of `edges` that leads through a door narrower than the robot.
    fn too_narrow(&self, edges: &[crate::order_route::PlanEdge]) -> Option<EdgeIdx> {
        edges
            .iter()
            .find(|e| {
                self.door_widths
                    .get(&e.edge)
                    .is_some_and(|&w| self.width() > w)
            })
            .map(|e| e.edge)
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

    /// Give the robot a zone set that is already **enabled**, as if the controller
    /// had sent it and enabled it (a shortcut for set-up and tests; the real way
    /// is [`Robot::take_zone_set`] and the `enableZoneSet` instant action). It
    /// will state RELEASE zone requests (see [`crate::zone_requests`]), by `timing`.
    pub fn set_zones(&mut self, zones: Zones, timing: ZoneRequestTiming) {
        let zones = RobotZones::from_setup(&zones);
        self.zone_timing = timing;
        self.zone_sets.clear();
        self.zone_sets.push(StoredZoneSet {
            zones: zones.clone(),
            enabled: true,
        });
        self.zone_requests = Some(ZoneRequester::new(zones, timing));
    }

    /// Change when this robot requests zones (also for a zone set enabled later).
    pub fn set_zone_request_timing(&mut self, timing: ZoneRequestTiming) {
        self.zone_timing = timing;
        if let Some(z) = self.zone_requests.as_mut() {
            z.set_timing(timing);
        }
    }

    /// Receive a zone set (S62). A zone set with an id the robot already has is
    /// **not** taken over: error `DUPLICATE_ZONE_SET` (WARNING) in the state for
    /// [`ZONE_SET_ERROR_MS`]. A zone set that does not fit the layout (see
    /// [`RobotZones::from_message`]) is refused with `UNKNOWN_MAP_ID` or
    /// `VALIDATION_FAILURE`. A new zone set is stored **disabled** and is used
    /// only after the `enableZoneSet` instant action.
    pub fn take_zone_set(&mut self, graph: &Graph, set: &ZoneSet) {
        let id = &set.id;
        let refuse = |t: ErrorType, text: String| RobotError {
            error_type: t,
            references: vec![("zoneSetId".to_string(), id.to_string())],
            description: Some(text),
        };
        if self.zone_sets.iter().any(|z| z.zones.id == *id) {
            let e = refuse(
                ErrorType::DuplicateZoneSet,
                "a zone set with this zoneSetId exists already".to_string(),
            );
            self.refuse_zone_set(e);
            return;
        }
        match RobotZones::from_message(graph, set) {
            Ok(zones) => self.zone_sets.push(StoredZoneSet {
                zones,
                enabled: false,
            }),
            Err(e) => {
                let t = match e {
                    botsim_model::zones::ZoneError::UnknownMap(_) => ErrorType::UnknownMapId,
                    _ => ErrorType::ValidationFailure,
                };
                self.refuse_zone_set(refuse(t, e.to_string()));
            }
        }
    }

    /// Report an error about a zone set that was not taken over, for a while.
    fn refuse_zone_set(&mut self, error: RobotError) {
        let until = self.now_ms.saturating_add(ZONE_SET_ERROR_MS);
        self.zone_set_errors.push((error, until));
    }

    /// Something sent to the robot could not be read: its error is reported as if
    /// the robot had refused it (an order's until an order is accepted, a zone
    /// set's for [`ZONE_SET_ERROR_MS`]), and instant actions that could not be
    /// read are listed `FAILED`. The instant actions that could be read are
    /// carried out, in their places.
    pub fn take_unread(&mut self, graph: &Graph, unread: &Unread) {
        match unread {
            Unread::Order(error) => self.errors.push(error.clone()),
            Unread::ZoneSet(error) => self.refuse_zone_set(error.clone()),
            Unread::InstantActions(entries) => {
                for entry in entries {
                    match entry {
                        InstantEntry::Read(action) => self.take_instant_action(graph, action),
                        InstantEntry::Unread { id, kind, error } => {
                            self.instant.push(ActionState {
                                id: id.clone(),
                                kind: kind.clone(),
                                status: ActionStatus::Failed,
                            });
                            self.errors.push(error.clone());
                        }
                    }
                }
            }
        }
    }

    /// Enable the stored zone set `id` (instant action `enableZoneSet`): the robot
    /// states RELEASE zone requests for its zones from now on, and every other zone
    /// set is disabled. `false` if the robot has no such zone set.
    fn enable_zone_set(&mut self, graph: &Graph, id: &ZoneSetId) -> bool {
        let Some(i) = self.zone_sets.iter().position(|z| z.zones.id == *id) else {
            return false;
        };
        if self.zone_sets[i].enabled {
            return true;
        }
        for (j, z) in self.zone_sets.iter_mut().enumerate() {
            z.enabled = j == i;
        }
        let mut zr = ZoneRequester::new(self.zone_sets[i].zones.clone(), self.zone_timing);
        // the edges of an order that is under way count for the new zones too
        zr.learn_edges(graph, &self.driver.progress().remaining_edges);
        self.zone_requests = Some(zr);
        self.update_zone_requests();
        true
    }

    /// Delete the stored zone set `id` (instant action `deleteZoneSet`). `false` if
    /// the robot has no such zone set, or it is enabled (in use).
    fn delete_zone_set(&mut self, id: &ZoneSetId) -> bool {
        match self.zone_sets.iter().position(|z| z.zones.id == *id) {
            Some(i) if !self.zone_sets[i].enabled => {
                self.zone_sets.remove(i);
                true
            }
            _ => false,
        }
    }

    /// The robot's zone sets as it states them, none if it has none.
    pub fn zone_set_infos(&self) -> Vec<ZoneSetInfo> {
        self.zone_sets
            .iter()
            .map(|z| ZoneSetInfo {
                id: z.zones.id.clone(),
                map: z.zones.map,
                enabled: z.enabled,
            })
            .collect()
    }

    /// The robot's zone requests as it states them (empty without zones).
    pub fn zone_requests(&self) -> Vec<ZoneRequest> {
        self.zone_requests
            .as_ref()
            .map(ZoneRequester::to_report)
            .unwrap_or_default()
    }

    /// Bring the zone requests up to date with where the robot is and what is ahead.
    fn update_zone_requests(&mut self) {
        // nothing to keep up to date, nothing blocked before: skip the work
        if self.zone_requests.is_none()
            && self.closed_door_edges.is_empty()
            && !self.door_blocked
            && self.door_edges.is_empty()
        {
            return;
        }
        let p = self.driver.progress();
        // the robot stops before the edges of a door, never drives through the
        // node in front of one (S66)
        self.driver.set_stop_before_edges(
            p.remaining_edges
                .iter()
                .filter(|e| self.door_edges.contains(&e.edge))
                .map(|e| e.sequence_id)
                .collect(),
        );
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
            .filter(|e| self.closed_door_edges.contains(&e.edge))
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

    /// Receive responses: the controller's answers to zone requests. Answers to
    /// requests the robot does not have are ignored.
    pub fn take_responses(&mut self, responses: &[Response]) {
        if let Some(zr) = self.zone_requests.as_mut() {
            for r in responses {
                let lease = r.lease_expiry.map(|time| time.0);
                zr.respond(r.request.as_str(), r.grant, lease);
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

    /// The protective field is blocked (S54): brake and stand still.
    pub fn set_obstructed(&mut self, obstructed: bool) {
        self.driver.set_obstructed(obstructed);
    }

    /// Is the protective field stopping the robot right now?
    pub fn is_obstructed(&self) -> bool {
        self.driver.is_obstructed()
    }

    /// The robot's protective field (see [`botsim_model::safety`]), if it is driving an
    /// edge; `dt` is the length of a time step.
    pub fn protective_field(&self, dt: f64) -> Option<togo::prelude::OrientedRect> {
        let dir = self.driver.travel_direction()?;
        let p = self.driver.pose();
        Some(botsim_model::safety::protective_field(
            &self.spec.shape,
            (p.x, p.y, p.heading),
            dir,
            self.driver.speed(),
            self.spec.limits.max_deceleration,
            dt,
        ))
    }

    pub fn spec(&self) -> &RobotSpec {
        &self.spec
    }

    /// The body outline, relative to the kinematic center.
    pub fn shape(&self) -> &botsim_model::spec::Shape {
        &self.spec.shape
    }

    pub fn errors(&self) -> &[RobotError] {
        &self.errors
    }

    /// Advance by `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        let before = self.driver.pose();
        // an empty battery stops the robot where it is (it brakes as for a hold)
        if self.is_out_of_charge() {
            self.ran_empty = true;
            self.driver.hold(2.0 * dt);
        }
        self.driver.step(dt);
        if let Some(b) = self.battery.as_mut() {
            let after = self.driver.pose();
            let moved = (after.x - before.x).hypot(after.y - before.y);
            // moving or a full battery ends the charging
            if moved > 0.0 || b.is_full() {
                self.charging_requested = false;
            }
            match self.charger_power_w {
                Some(p) if self.charging_requested => {
                    self.charging = true;
                    self.charging_seconds += dt;
                    b.charge(dt, p);
                }
                _ => {
                    self.charging = false;
                    b.drain(dt, moved, self.driver.phase() == Phase::Acting);
                }
            }
        }
        self.update_cancel();
        self.update_zone_requests();
    }

    /// What [`Robot::report`] says about the order, driving and actions, without
    /// building the report.
    pub(crate) fn glance(&self) -> Glance<'_> {
        self.driver.glance()
    }

    /// The state report.
    pub fn report(&self) -> State {
        let mut state = build_state(&self.driver.progress(), &self.errors, &self.instant);
        if let Some(b) = &self.battery {
            state.charge = b.percent();
            state.charging = self.charging;
            if self.is_out_of_charge() {
                // not one of the predefined types: the standard allows others
                state.errors.push(Error {
                    kind: ErrorKind::Other("BATTERY_EMPTY".into()),
                    level: ErrorLevel::Fatal,
                    references: Vec::new(),
                    description: Some("the battery is empty, the robot cannot move".into()),
                    hint: Some("charge the robot".into()),
                });
            }
        }
        // an empty list if there are zones but no request (as the standard says)
        if let Some(zr) = &self.zone_requests {
            state.zone_requests = Some(self.zone_requests());
            state
                .errors
                .extend(zr.errors().iter().map(RobotError::to_report));
        }
        // the zone sets stored on the robot, and errors about ones not taken over
        state.zone_sets = self.zone_set_infos();
        state.errors.extend(
            self.zone_set_errors
                .iter()
                .filter(|(_, until)| *until > self.now_ms)
                .map(|(e, _)| e.to_report()),
        );
        state
    }

    fn reject(&mut self, error: RobotError) -> Reception {
        let t = error.error_type;
        self.errors.push(error);
        Reception::Rejected(t)
    }

    fn reject_order(&mut self, t: ErrorType, order: &Order, text: &str) -> Reception {
        let id = order.order_id().as_str();
        let e = RobotError::for_order(t, id, order.update_id(), text.to_string());
        self.reject(e)
    }

    /// Receive an order or order update.
    pub fn take_order(&mut self, graph: &Graph, order: &Order) -> Reception {
        let (order_id, update_id) = (order.order_id().as_str(), order.update_id());
        // 1. valid?
        let route = match route_from_order(graph, order) {
            Ok(r) => r,
            Err(e) => {
                let err = RobotError::from_order_error(&e, order_id, update_id);
                return self.reject(err);
            }
        };

        if let Some(what) = self.not_allowed(graph, &route) {
            let e = DriveError::NotAllowedForType { what };
            let err = RobotError::from_drive_error(&e, order_id, update_id);
            return self.reject(err);
        }
        if let Some(edge) = self.too_narrow(&route.plan_edges) {
            let edge = graph.edges[edge].id.clone();
            let e = DriveError::DoorTooNarrow { edge };
            let err = RobotError::from_drive_error(&e, order_id, update_id);
            return self.reject(err);
        }

        let current = self.driver.order().map(|(id, u)| (id.to_string(), u));
        let is_update = current.as_ref().is_some_and(|(id, _)| id == order_id);

        if !is_update {
            // 2. a new order
            if self.driver.phase() != Phase::Finished {
                return self.reject_order(
                    ErrorType::OtherOrderActive,
                    order,
                    "another order is still active",
                );
            }
            if update_id != 0 {
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
                Ok(mut d) => {
                    d.set_pass_through(self.pass_through);
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
                    let err = RobotError::from_drive_error(&e, order_id, update_id);
                    self.reject(err)
                }
            };
        }

        // 3. an update of the current order
        let Some((_, current_update)) = current else {
            // (cannot happen: `is_update` means there is a current order)
            return self.reject_order(
                ErrorType::ValidationFailure,
                order,
                "there is no order to update",
            );
        };
        if update_id < current_update {
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
        if update_id == current_update {
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
                let err = RobotError::from_drive_error(&e, order_id, update_id);
                self.reject(err)
            }
        }
    }

    /// Receive instant actions, in their order. Every action is listed among the
    /// instant action states. Supported: `cancelOrder` (optional parameter
    /// `orderId`). It fails (`FAILED` + `NO_ORDER_TO_CANCEL`, reference `actionId`)
    /// if the robot is idle or the `orderId` is not the active order. Also
    /// (S62): `enableZoneSet` and `deleteZoneSet` (parameter `zoneSetId`); they
    /// finish at once, or fail with `INVALID_INSTANT_ACTION` if the robot has no
    /// such zone set (or, for `deleteZoneSet`, it is enabled). `startCharging`
    /// and `stopCharging`. Any other type fails with `INVALID_INSTANT_ACTION`.
    pub fn take_instant_actions(&mut self, graph: &Graph, actions: &[InstantAction]) {
        for action in actions {
            self.take_instant_action(graph, action);
        }
    }

    fn take_instant_action(&mut self, graph: &Graph, a: &InstantAction) {
        let id = a.id.as_str();
        self.instant.push(ActionState {
            id: a.id.clone(),
            kind: Some(a.kind.action_type()),
            status: ActionStatus::Failed,
        });
        let idx = self.instant.len() - 1;

        match &a.kind {
            InstantKind::EnableZoneSet { zone_set } | InstantKind::DeleteZoneSet { zone_set } => {
                let done = if matches!(a.kind, InstantKind::EnableZoneSet { .. }) {
                    self.enable_zone_set(graph, zone_set)
                } else {
                    self.delete_zone_set(zone_set)
                };
                if done {
                    self.instant[idx].status = ActionStatus::Finished;
                } else {
                    self.errors.push(action_error(
                        ErrorType::InvalidInstantAction,
                        id,
                        "the robot has no such zone set (or it is in use)",
                    ));
                }
            }
            InstantKind::StopCharging => {
                self.charging_requested = false;
                self.instant[idx].status = ActionStatus::Finished;
            }
            InstantKind::StartCharging => {
                if self.battery.is_some() && self.charger_power_w.is_some() {
                    // charging runs while the robot stands on the charger and the
                    // battery is not full
                    self.charging_requested = true;
                    self.instant[idx].status = ActionStatus::Finished;
                } else {
                    self.errors.push(action_error(
                        ErrorType::InvalidInstantAction,
                        id,
                        "startCharging: the robot has no battery or does not stand on a charger",
                    ));
                }
            }
            InstantKind::Other(_) => {
                self.errors.push(action_error(
                    ErrorType::InvalidInstantAction,
                    id,
                    "unsupported instant action",
                ));
            }
            InstantKind::CancelOrder { order_id } => {
                let active = self.driver.phase() != Phase::Finished;
                let matches = match (order_id, self.driver.order()) {
                    (Some(w), Some((id, _))) => w.as_str() == id,
                    (Some(_), None) => false,
                    (None, _) => true,
                };
                if !active || !matches {
                    self.errors.push(action_error(
                        ErrorType::NoOrderToCancel,
                        id,
                        "no active order to cancel",
                    ));
                    return;
                }

                self.driver.cancel();
                self.instant[idx].status = ActionStatus::Running;
                self.cancel_action = Some(idx);
                self.update_cancel();
            }
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

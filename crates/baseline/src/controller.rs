//! The baseline controller: jobs go first come first served to the nearest
//! free robot; the route is released node by node with a window of
//! [`RELEASE_WINDOW`] nodes.
//!
//! It sees the robots only through their VDA 5050 `state` messages and talks to
//! them only with `order` messages, like any real controller. Call
//! [`Controller::step`] once per simulation step, after `World::step`.
//!
//! - **Free robot**: no job assigned. Its place is the layout node nearest to
//!   its reported position (within [`NODE_TOLERANCE`]); a robot that is not on a
//!   node is not used. A robot without a job stays where it is (S31).
//! - **Nearest**: the shortest route length from the robot's node to the
//!   pickup node; ties go to the robot added first.
//! - **FCFS**: the oldest job is assigned first. A job no free robot can reach
//!   stays in the queue and does not block the jobs behind it.
//! - **Release**: when the robot reaches plan node `i`, the base is extended up
//!   to node `i + 2` (an order update from the end of the base), as far as the
//!   way is free.
//! - **Reservations** (S34): a node, and the edge leading to it, are reserved
//!   for a robot when they are released to it, and given back when the robot
//!   has driven on to the next node. A robot is released only onto nodes and
//!   edges nobody else holds; otherwise it waits at the end of its base and the
//!   controller tries again every cycle. A robot without a job holds the node
//!   it stands on.
//! - **Lifts** (S47): a route with a lift ride needs the lift; one robot uses a
//!   lift at a time, the controller calls, releases and sends the cabin (see
//!   [`crate::lifts`]).
//! - **Doors** (S51): a route with an edge through a door needs the door open;
//!   one robot uses a door at a time, the controller opens and releases it (see
//!   [`crate::doors`]).
//! - **Done**: the robot stands on the last node, nothing is driving, and all
//!   actions are finished.
//!
//! - **Re-routing** (S53): a robot that has stood at the end of its released
//!   route for [`REROUTE_AFTER_SECONDS`] because another robot holds the way gets
//!   a new route around the nodes others hold (an order update that changes the
//!   unreleased part).
//!
//! Not yet: deadlock resolution beyond that (robots can still wait for each other
//! for ever when there is no way round), look-ahead beyond the window, robot types
//! and edge rules, robots that fail.

use std::collections::VecDeque;

use botsim_core::errors::ErrorType;
use botsim_core::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_core::jobs::Job;
use botsim_core::metrics::{Metrics, is_waiting, summarize};
use botsim_core::robot::Reception;
use botsim_core::spec::Kinematics;
use botsim_core::world::{RobotId, World};
use vda5050::order::Order;
use vda5050::state::{ActionStatus, State};

use crate::doors::DoorDesk;
use crate::lifts::{LiftDesk, LiftRobot};
use crate::node_grid::NodeGrid;
use crate::order_builder::{OrderHeader, Plan, RELEASE_WINDOW, build_order};
use crate::reservations::Reservations;
use crate::route::{Arrival, Mover, Turns, distances_to, in_edges, plan_path, travel_lengths};
use crate::zone_grants::{RobotView, ZoneGrants};
use vda5050::responses::Responses;

/// A robot that has been held at the end of its released route this long by
/// other robots gets a new route around them (S53, my choice: well before the
/// 30 s a deadlock needs to be reported).
pub const REROUTE_AFTER_SECONDS: f64 = 10.0;

/// How close (metres) a robot must be to a node to count as standing on it.
pub const NODE_TOLERANCE: f64 = 0.1;

/// A job the controller finished.
#[derive(Debug, Clone, PartialEq)]
pub struct Completed {
    pub job: Job,
    pub robot: RobotId,
    pub assigned_tick: u64,
    pub finished_tick: u64,
    /// Time the robot stood still with this job, not doing an action.
    pub waiting_seconds: f64,
}

/// A job a robot refused (it stays out of the queue).
#[derive(Debug, Clone, PartialEq)]
pub struct Rejected {
    pub job: Job,
    pub robot: RobotId,
    pub reason: ErrorType,
}

#[derive(Debug)]
struct Work {
    job: Job,
    plan: Plan,
    order_id: String,
    update_id: u32,
    /// Plan index of the last released node.
    released_through: usize,
    assigned_tick: u64,
    waiting_seconds: f64,
    /// How long the robot has stood at the end of its released route because
    /// the next node or edge is held by another robot (S53).
    blocked_seconds: f64,
    /// The order was sent with a message delay and the robot's state has not
    /// shown it yet (S58): the controller does not know whether it was accepted.
    awaiting_ack: bool,
    /// The robot refused an update of the order (S70, e.g. a car-like robot that
    /// cannot take a turn): the order was cancelled, and the job is given up as soon
    /// as the robot has stopped. Until then the robot keeps what it holds.
    cancelling: Option<ErrorType>,
}

/// What a robot type may drive (S55): the layout lists the types that may use
/// each node and edge.
#[derive(Debug)]
struct TypeRouting {
    /// `ok[e]`: the type may drive edge `e` (the edge and both its nodes allow it).
    ok: Vec<bool>,
    /// The turns the type cannot make (S73); `None` if it can make them all.
    turns: Option<Turns>,
    /// The distance the type drives on each edge (S74): a curve is longer than the
    /// straight line between its nodes.
    cost: Vec<f64>,
    /// The allowed edges into each node.
    into: Vec<Vec<EdgeIdx>>,
}

impl TypeRouting {
    /// What the type can drive, for the route search.
    fn mover(&self) -> Mover<'_> {
        Mover {
            ok: &self.ok,
            cost: &self.cost,
            turns: self.turns.as_ref(),
        }
    }
}

#[derive(Debug)]
struct Slot {
    id: RobotId,
    /// Index into `Controller::types`.
    type_idx: usize,
    work: Option<Work>,
    header_id: u32,
    /// `headerId` of the next `responses` message to this robot.
    response_header_id: u32,
    /// The `zoneSet` message was sent to this robot (S62).
    zone_set_sent: bool,
    /// The `enableZoneSet` instant action was sent to this robot (S62).
    enable_sent: bool,
    /// `headerId` of the next `zoneSet` message and of the next `instantActions`
    /// message (each topic counts for itself).
    zone_set_header_id: u32,
    instant_header_id: u32,
}

#[derive(Debug)]
pub struct Controller {
    graph: Graph,
    pending: VecDeque<Job>,
    robots: Vec<Slot>,
    completed: Vec<Completed>,
    rejected: Vec<Rejected>,
    /// The tick of the previous control cycle.
    last_tick: u64,
    reservations: Reservations,
    /// Answers to RELEASE zone requests; `None` if the run has no zones.
    zone_grants: Option<ZoneGrants>,
    /// The zone set of the site, which the controller sends to every robot and
    /// enables (S62); `None` if the run has no zones.
    zone_set: Option<vda5050::zone_set::ZoneSet>,
    /// Rides in the routes: who owns which lift (see [`crate::lifts`]).
    lifts: LiftDesk,
    /// Guarded edges in the routes: who owns which door (see [`crate::doors`]).
    doors: DoorDesk,
    /// Which node a position is on (S52).
    node_grid: NodeGrid,
    /// What each robot type may drive and the edges into each node for it, for
    /// the route lengths to a pickup (S52, S55). Indexed by `Slot::type_idx`.
    types: Vec<TypeRouting>,
    /// (job, robot type) pairs whose drop cannot be reached from their pickup
    /// by that type (the job waits for ever for such a robot).
    unserviceable: std::collections::HashSet<(u64, usize)>,
    /// How many times a route was changed because the way was held (S53).
    reroutes: usize,
}

impl Controller {
    /// A controller for all robots now in `world`.
    pub fn new(world: &World) -> Controller {
        // the robot types (manufacturer, series) in the order they first appear
        let graph = world.graph();
        let mut keys: Vec<(String, String)> = Vec::new();
        let mut kinds: Vec<Kinematics> = Vec::new();
        let robots: Vec<Slot> = world
            .robot_ids()
            .filter_map(|id| {
                // the robot's type comes from its factsheet message (S61); a robot
                // that has none cannot be read and gets no jobs
                let f = world.factsheet(id).ok()?;
                let kinematics = world.spec(id).ok()?.kinematics;
                let key = (f.manufacturer, f.type_specification.series_name);
                let type_idx = keys.iter().position(|k| *k == key).unwrap_or_else(|| {
                    keys.push(key);
                    kinds.push(kinematics);
                    keys.len() - 1
                });
                Some(Slot {
                    id: id.clone(),
                    type_idx,
                    work: None,
                    header_id: 0,
                    response_header_id: 0,
                    zone_set_sent: false,
                    enable_sent: false,
                    zone_set_header_id: 0,
                    instant_header_id: 0,
                })
            })
            .collect();
        let types: Vec<TypeRouting> = keys
            .iter()
            .zip(&kinds)
            .map(|((m, s), &kinematics)| {
                let ok: Vec<bool> = (0..graph.edges.len())
                    .map(|e| graph.edge_allows(e, m, s))
                    .collect();
                TypeRouting {
                    into: in_edges(graph, &ok),
                    turns: Turns::for_type(graph, kinematics, m, s),
                    cost: travel_lengths(graph, m, s),
                    ok,
                }
            })
            .collect();
        Controller {
            graph: world.graph().clone(),
            pending: VecDeque::new(),
            robots,
            completed: Vec::new(),
            rejected: Vec::new(),
            last_tick: world.tick(),
            reservations: Reservations::new(world.graph()),
            zone_grants: world.zones().cloned().map(ZoneGrants::new),
            zone_set: world.zones().map(|z| vda5050::zone_set::ZoneSet {
                map_id: world.graph().maps[z.map].clone(),
                zone_set_id: z.zone_set_id.clone(),
                zone_set_descriptor: None,
                zones: z.zones.clone(),
            }),
            lifts: LiftDesk::new(world),
            doors: DoorDesk::new(world),
            node_grid: NodeGrid::new(world.graph()),
            types,
            unserviceable: std::collections::HashSet::new(),
            reroutes: 0,
        }
    }

    /// How many times a robot's route was changed because another robot held the
    /// way for longer than [`REROUTE_AFTER_SECONDS`].
    pub fn reroutes(&self) -> usize {
        self.reroutes
    }

    /// Who holds which node and edge.
    pub fn reservations(&self) -> &Reservations {
        &self.reservations
    }

    /// Throughput and waiting of the jobs done so far (see
    /// [`botsim_core::metrics`]); `elapsed_seconds` is the simulated time.
    pub fn metrics(&self, elapsed_seconds: f64) -> Metrics {
        let waits: Vec<f64> = self.completed.iter().map(|c| c.waiting_seconds).collect();
        summarize(&waits, elapsed_seconds)
    }

    /// Jobs not yet assigned, oldest first.
    pub fn pending(&self) -> impl Iterator<Item = &Job> {
        self.pending.iter()
    }

    pub fn completed(&self) -> &[Completed] {
        &self.completed
    }

    pub fn rejected(&self) -> &[Rejected] {
        &self.rejected
    }

    /// Number of robots working on a job.
    pub fn busy(&self) -> usize {
        self.robots.iter().filter(|s| s.work.is_some()).count()
    }

    /// One control cycle: take the new jobs, read every robot's state, move
    /// the release windows on, finish jobs, and assign waiting jobs.
    pub fn step(&mut self, world: &mut World, new_jobs: impl IntoIterator<Item = Job>) {
        self.pending.extend(new_jobs);
        let tick = world.tick();
        // time since the last cycle: a robot seen waiting now is counted as
        // waiting for all of it
        let elapsed = (tick - self.last_tick) as f64 * world.dt();
        self.last_tick = tick;

        // (the robots are the ones the world had; if one cannot be read, this cycle
        // does nothing)
        let Ok(states) = self
            .robots
            .iter()
            .map(|s| world.state(&s.id))
            .collect::<Result<Vec<State>, _>>()
        else {
            return;
        };

        for (i, state) in states.iter().enumerate() {
            self.update_work(world, i, state, tick, elapsed);
        }
        self.update_lifts(world, &states);
        // a robot without a job holds the node it stands on (S34)
        for (i, state) in states.iter().enumerate() {
            if self.robots[i].work.is_none()
                && let Some(n) = self.node_of(state)
            {
                self.reservations.reserve_node(n, i);
            }
        }
        self.update_zone_sets(world, &states);
        self.answer_zone_requests(world, &states);
        self.dispatch(world, &states, tick);

        // tell the observer what is reserved (only if someone watches)
        if world.observing() {
            world.publish_reservations(self.reservations.to_observer());
        }
    }

    /// Release more of the route, or finish the job, for robot `i`.
    fn update_work(&mut self, world: &mut World, i: usize, state: &State, tick: u64, elapsed: f64) {
        let slot = &mut self.robots[i];
        let Some(work) = slot.work.as_mut() else {
            return;
        };
        if let Some(reason) = work.cancelling {
            // wait until the robot has stopped (it brakes to the next node); until
            // then the way it is on stays reserved
            if !state.driving {
                let Some(work) = slot.work.take() else {
                    return;
                };
                give_back(
                    &mut self.reservations,
                    &mut self.lifts,
                    &mut self.doors,
                    i,
                    &work.plan,
                    work.released_through,
                );
                self.rejected.push(Rejected {
                    job: work.job,
                    robot: slot.id.clone(),
                    reason,
                });
            }
            return;
        }
        if work.awaiting_ack {
            if state.order_id == work.order_id {
                work.awaiting_ack = false; // the robot has taken the order
            } else if let Some(reason) = order_error(state, &work.order_id) {
                // The robot refused the order (the state says so): give the job up.
                let Some(work) = slot.work.take() else {
                    return;
                };
                give_back(
                    &mut self.reservations,
                    &mut self.lifts,
                    &mut self.doors,
                    i,
                    &work.plan,
                    work.released_through,
                );
                self.rejected.push(Rejected {
                    job: work.job,
                    robot: slot.id.clone(),
                    reason,
                });
                return;
            } else {
                return; // not answered yet
            }
        }
        if state.order_id != work.order_id {
            return; // the robot has not taken the order yet
        }
        let at = (state.last_node_sequence_id / 2) as usize;
        let last = work.plan.last();

        if at == last && is_idle(state) {
            let Some(work) = slot.work.take() else {
                return;
            };
            self.completed.push(Completed {
                job: work.job,
                robot: slot.id.clone(),
                assigned_tick: work.assigned_tick,
                finished_tick: tick,
                waiting_seconds: work.waiting_seconds,
            });
            return;
        }

        // S32: waiting = standing still with a job, and not busy with an action
        if is_waiting(state) {
            work.waiting_seconds += elapsed;
        }

        // the robot has driven on: give back what it has left behind (S34)
        // (a route can pass the same node or edge again, e.g. a lap of a ring:
        // what is still ahead stays reserved)
        let ahead_nodes = &work.plan.nodes[at..=work.released_through];
        let ahead_edges = &work.plan.edges[at..work.released_through];
        for j in 0..at {
            if !ahead_nodes.contains(&work.plan.nodes[j]) {
                self.reservations.release_node(work.plan.nodes[j], i);
            }
            if !ahead_edges.contains(&work.plan.edges[j]) {
                self.reservations.release_edge(work.plan.edges[j], i);
            }
        }

        // release as far as the window allows and nobody else holds the way
        let from = work.released_through;
        let wanted = work.plan.released_through(at, RELEASE_WINDOW);
        // a ride in the window needs its lift (S47)
        let wanted = self.lifts.window(i, &work.plan, at, from, wanted);
        // so does an edge through a door (S51)
        let wanted = self.doors.window(i, &work.plan, at, from, wanted);
        let rt = reserve_up_to(&mut self.reservations, &work.plan, i, from, wanted);
        self.doors.settle(i, rt);

        // Timeout re-routing (S53): the robot stands at the end of its released
        // route and wants to go on, but another robot holds the way (not a lift
        // or a door, which have their own rules).
        let stuck = at == from && wanted > from && rt == from && is_waiting(state);
        work.blocked_seconds = if stuck {
            work.blocked_seconds + elapsed
        } else {
            0.0
        };
        if stuck && work.blocked_seconds >= REROUTE_AFTER_SECONDS {
            work.blocked_seconds = 0.0; // try again after another timeout
            if let Some(plan) = reroute(
                &self.graph,
                &self.reservations,
                &self.lifts,
                &self.doors,
                &self.types[slot.type_idx].mover(),
                i,
                &work.plan,
                at,
            ) {
                work.plan = plan;
                work.update_id += 1;
                let header = OrderHeader {
                    robot: &slot.id,
                    header_id: slot.header_id,
                    timestamp: &world.timestamp(),
                    order_id: &work.order_id,
                    order_update_id: work.update_id,
                };
                // nothing new is released; the unreleased part of the route changes
                let msg = build_order(&self.graph, &header, &work.plan, from, from);
                slot.header_id += 1;
                match deliver(world, msg) {
                    Ok(_) => self.reroutes += 1,
                    Err(reason) => {
                        // the robot cannot drive the changed route: give the job up
                        cancel_order(world, slot, reason);
                        return;
                    }
                }
            }
        }

        if rt > from {
            work.update_id += 1;
            let header = OrderHeader {
                robot: &slot.id,
                header_id: slot.header_id,
                timestamp: &world.timestamp(),
                order_id: &work.order_id,
                order_update_id: work.update_id,
            };
            let msg = build_order(&self.graph, &header, &work.plan, from, rt);
            slot.header_id += 1;
            match deliver(world, msg) {
                Ok(_) => work.released_through = rt,
                Err(reason) => {
                    // the robot cannot drive the next part (e.g. a car-like robot
                    // that cannot take the turn): give the job up (S70)
                    cancel_order(world, slot, reason);
                }
            }
        }
    }

    /// Call, release and send the lifts that robots use (see [`crate::lifts`]).
    fn update_lifts(&mut self, world: &mut World, states: &[State]) {
        let nodes: Vec<Option<NodeIdx>> = states.iter().map(|s| self.node_of(s)).collect();
        let ats = self.plan_positions(states);
        let robots: Vec<LiftRobot> = self
            .robots
            .iter()
            .enumerate()
            .map(|(i, slot)| LiftRobot {
                id: &slot.id,
                at: ats[i],
                node: nodes[i],
            })
            .collect();
        self.lifts.update(world, &robots);
        self.doors.update(world, &ats);
    }

    /// The plan index of each robot's last node; `None` for a robot without a job.
    fn plan_positions(&self, states: &[State]) -> Vec<Option<usize>> {
        self.robots
            .iter()
            .enumerate()
            .map(|(i, slot)| {
                slot.work
                    .as_ref()
                    .filter(|w| states[i].order_id == w.order_id)
                    .map(|_| (states[i].last_node_sequence_id / 2) as usize)
            })
            .collect()
    }

    /// The state of the robot's copy of the site's zone set, `None` if it has none
    /// (S62).
    fn zone_set_status(&self, state: &State) -> Option<vda5050::state::EnabledStatus> {
        let id = &self.zone_set.as_ref()?.zone_set_id;
        state
            .zone_sets
            .iter()
            .flatten()
            .find(|z| z.zone_set_id == *id)
            .map(|z| z.zone_set_status)
    }

    /// Is the robot ready for jobs as far as zones go: the run has no zones, or
    /// the robot reports the site's zone set as enabled? A robot without its
    /// zones would drive into RELEASE zones without asking.
    fn zones_ready(&self, state: &State) -> bool {
        self.zone_set.is_none()
            || self.zone_set_status(state) == Some(vda5050::state::EnabledStatus::Enabled)
    }

    /// Give every robot the site's zone set (S62): send the `zoneSet` message; when
    /// the robot's state shows it (disabled), send the `enableZoneSet` instant
    /// action. Each is sent once; the controller reads the answer from the state
    /// (which may be old, with a message delay).
    fn update_zone_sets(&mut self, world: &mut World, states: &[State]) {
        let Some(zs) = self.zone_set.clone() else {
            return;
        };
        for i in 0..self.robots.len() {
            let status = self.zone_set_status(&states[i]);
            let slot = &mut self.robots[i];
            if status.is_none() && !slot.zone_set_sent {
                let msg = serde_json::from_value(serde_json::json!({
                    "headerId": slot.zone_set_header_id,
                    "timestamp": world.timestamp(),
                    "version": botsim_core::world::PROTOCOL_VERSION,
                    "manufacturer": slot.id.manufacturer,
                    "serialNumber": slot.id.serial_number,
                    "zoneSet": zs,
                }));
                slot.zone_set_header_id += 1;
                slot.zone_set_sent = true;
                // (the message has all its fields and the robot is in the world; if
                // not, the robot gets no zone set)
                if let Ok(msg) = msg {
                    let _ = world.send_zone_set(&msg);
                }
            } else if status == Some(vda5050::state::EnabledStatus::Disabled) && !slot.enable_sent {
                let msg = serde_json::from_value(serde_json::json!({
                    "headerId": slot.instant_header_id,
                    "timestamp": world.timestamp(),
                    "version": botsim_core::world::PROTOCOL_VERSION,
                    "manufacturer": slot.id.manufacturer,
                    "serialNumber": slot.id.serial_number,
                    "actions": [{
                        "actionType": "enableZoneSet",
                        "actionId": format!("enable-{}", zs.zone_set_id),
                        "blockingType": "NONE",
                        "actionParameters": [{"key": "zoneSetId", "value": zs.zone_set_id}],
                    }],
                }));
                slot.instant_header_id += 1;
                slot.enable_sent = true;
                if let Ok(msg) = msg {
                    let _ = world.send_instant_actions(&msg);
                }
            }
        }
    }

    /// Answer the RELEASE zone requests the robots state (see [`crate::zone_grants`]).
    fn answer_zone_requests(&mut self, world: &mut World, states: &[State]) {
        let Some(grants) = self.zone_grants.as_mut() else {
            return;
        };
        let views: Vec<RobotView> = states
            .iter()
            .map(|s| RobotView {
                requests: s.zone_requests.clone().unwrap_or_default(),
                position: s
                    .mobile_robot_position
                    .as_ref()
                    .and_then(|p| self.graph.map_idx(&p.map_id).map(|m| (m, p.x, p.y))),
            })
            .collect();
        let answers = grants.decide(&views);
        // one `responses` message per robot
        for (i, slot) in self.robots.iter_mut().enumerate() {
            let mine: Vec<_> = answers
                .iter()
                .filter(|a| a.robot == i)
                .map(|a| serde_json::json!({"requestId": a.request_id, "grantType": a.grant}))
                .collect();
            if mine.is_empty() {
                continue;
            }
            let msg: Result<Responses, _> = serde_json::from_value(serde_json::json!({
                "headerId": slot.response_header_id,
                "timestamp": world.timestamp(),
                "version": botsim_core::world::PROTOCOL_VERSION,
                "manufacturer": slot.id.manufacturer,
                "serialNumber": slot.id.serial_number,
                "responses": mine,
            }));
            slot.response_header_id += 1;
            if let Ok(msg) = msg {
                let _ = world.send_responses(&msg);
            }
        }
    }

    /// Give waiting jobs to free robots.
    fn dispatch(&mut self, world: &mut World, states: &[State], tick: u64) {
        // free robots with the node they stand on
        let mut free: Vec<(usize, NodeIdx)> = (0..self.robots.len())
            .filter(|&i| self.robots[i].work.is_none())
            .filter(|&i| self.zones_ready(&states[i]))
            .filter_map(|i| Some((i, self.node_of(&states[i])?)))
            .collect();

        let mut waiting = VecDeque::new();
        while let Some(job) = self.pending.pop_front() {
            if free.is_empty() {
                waiting.push_back(job); // nobody to give it to: no search needed
                continue;
            }
            // The nearest free robot that can do the job. One search backwards
            // from the pickup gives the route length from every node (S52), so
            // a job costs one search per robot type, not one per free robot. A
            // type may drive only some of the edges (S55), so its lengths differ.
            let mut dist: Vec<Option<Vec<f64>>> = vec![None; self.types.len()];
            let mut order: Vec<(i64, usize, usize)> = Vec::new();
            for (k, &(i, node)) in free.iter().enumerate() {
                let t = self.robots[i].type_idx;
                // A job whose drop cannot be reached from its pickup cannot be done
                // by any robot of the type; the layout does not change, so ask only
                // once (S52: it was retried against every free robot every cycle).
                if self.unserviceable.contains(&(job.id, t)) {
                    continue;
                }
                if dist[t].is_none() {
                    // (however the robot arrives at the pickup: the most it can do)
                    if self
                        .lifts
                        .route_from(&self.types[t].mover(), Arrival::Free, job.pickup, job.drop)
                        .is_none()
                    {
                        self.unserviceable.insert((job.id, t));
                        continue;
                    }
                    dist[t] = Some(distances_to(
                        &self.graph,
                        &self.types[t].into,
                        &self.types[t].cost,
                        job.pickup,
                    ));
                }
                let Some(dist_t) = dist[t].as_ref() else {
                    continue; // (computed just above)
                };
                let d = dist_t[node];
                if d.is_finite() {
                    // smallest distance (to a nanometre); equal → the robot added first
                    order.push(((d * 1e9).round() as i64, i, k));
                }
            }
            order.sort_unstable();
            // the first robot in that order that has a plan for the job
            let chosen = order.iter().find_map(|&(_, i, k)| {
                let (_, node) = free[k];
                let mover = self.types[self.robots[i].type_idx].mover();
                // the robot's heading matters if it cannot turn on the spot (S73)
                let arrival = states[i]
                    .mobile_robot_position
                    .as_ref()
                    .map_or(Arrival::Free, |p| Arrival::Heading(p.theta));
                let plan = Plan::for_job_with(&self.graph, node, arrival, &job, |a, arr, b| {
                    self.lifts.route_from(&mover, arr, a, b)
                })?;
                Some((i, k, plan))
            });
            let Some((i, k, plan)) = chosen else {
                waiting.push_back(job);
                continue;
            };
            free.remove(k);
            self.assign(world, i, job, plan, tick);
        }
        self.pending = waiting;
    }

    fn assign(&mut self, world: &mut World, i: usize, job: Job, plan: Plan, tick: u64) {
        let slot = &mut self.robots[i];
        let order_id = format!("job-{}", job.id);
        // node 0 is where the robot stands (it holds it already)
        self.reservations.reserve_node(plan.nodes[0], i);
        let wanted = self
            .lifts
            .window(i, &plan, 0, 0, plan.released_through(0, RELEASE_WINDOW));
        let wanted = self.doors.window(i, &plan, 0, 0, wanted);
        let released_through = reserve_up_to(&mut self.reservations, &plan, i, 0, wanted);
        self.doors.settle(i, released_through);
        let header = OrderHeader {
            robot: &slot.id,
            header_id: slot.header_id,
            timestamp: &world.timestamp(),
            order_id: &order_id,
            order_update_id: 0,
        };
        let msg = build_order(&self.graph, &header, &plan, 0, released_through);
        slot.header_id += 1;
        match deliver(world, msg) {
            Ok(reception) => {
                slot.work = Some(Work {
                    job,
                    plan,
                    order_id,
                    update_id: 0,
                    released_through,
                    assigned_tick: tick,
                    waiting_seconds: 0.0,
                    blocked_seconds: 0.0,
                    awaiting_ack: reception == Reception::InTransit,
                    cancelling: None,
                });
            }
            Err(reason) => {
                give_back(
                    &mut self.reservations,
                    &mut self.lifts,
                    &mut self.doors,
                    i,
                    &plan,
                    released_through,
                );
                self.rejected.push(Rejected {
                    job,
                    robot: slot.id.clone(),
                    reason,
                });
            }
        }
    }

    /// The layout node a robot stands on, from its reported position.
    fn node_of(&self, state: &State) -> Option<NodeIdx> {
        let p = state.mobile_robot_position.as_ref()?;
        let map = self.graph.map_idx(&p.map_id)?;
        self.node_grid.node_near(map, p.x, p.y, NODE_TOLERANCE)
    }
}

/// The order of a job was refused: give back what was reserved for it (the robot
/// still holds the node it stands on) and any lift or door it was given.
fn give_back(
    reservations: &mut Reservations,
    lifts: &mut LiftDesk,
    doors: &mut DoorDesk,
    robot: usize,
    plan: &Plan,
    released_through: usize,
) {
    lifts.forget(robot);
    doors.forget(robot);
    for j in 1..=released_through {
        reservations.release_node(plan.nodes[j], robot);
        reservations.release_edge(plan.edges[j - 1], robot);
    }
}

/// Send an order message to its robot: `Ok` with `Accepted` or `InTransit`, or the
/// error type if the robot refused it, if there is no message (it could not be
/// built), or if the robot is unknown to the world.
fn deliver(world: &mut World, msg: Option<Order>) -> Result<Reception, ErrorType> {
    let msg = msg.ok_or(ErrorType::ValidationFailure)?;
    match world.send_order(&msg) {
        Ok(Reception::Rejected(reason)) => Err(reason),
        Ok(reception) => Ok(reception),
        Err(_) => Err(ErrorType::ValidationFailure),
    }
}

/// The robot refused an update of its order: tell it to cancel the order (it brakes
/// to the next node and stops), and note that the job is to be given up once it has
/// stopped (see `Work::cancelling`). If the message cannot be built or sent, the
/// robot waits at the end of what it was given and the job is still given up.
fn cancel_order(world: &mut World, slot: &mut Slot, reason: ErrorType) {
    let Some(work) = slot.work.as_mut() else {
        return;
    };
    work.cancelling = Some(reason);
    let message = serde_json::from_value(serde_json::json!({
        "headerId": slot.instant_header_id,
        "timestamp": world.timestamp(),
        "version": botsim_core::world::PROTOCOL_VERSION,
        "manufacturer": slot.id.manufacturer,
        "serialNumber": slot.id.serial_number,
        "actions": [{
            "actionType": "cancelOrder",
            "actionId": format!("cancel-{}", work.order_id),
            "blockingType": "NONE",
        }],
    }));
    slot.instant_header_id += 1;
    if let Ok(msg) = message {
        let _ = world.send_instant_actions(&msg);
    }
}

/// The error type of an error in `state` that names the order `order_id`: the
/// robot has refused that order (VDA 5050 puts the `orderId` in the references).
fn order_error(state: &State, order_id: &str) -> Option<ErrorType> {
    let e = state.errors.iter().find(|e| {
        e.error_references
            .iter()
            .flatten()
            .any(|r| r.reference_key == "orderId" && r.reference_value == order_id)
    })?;
    Some(
        ErrorType::ALL
            .iter()
            .copied()
            .find(|t| t.as_str() == e.error_type)
            .unwrap_or(ErrorType::ValidationFailure),
    )
}

/// A new route for the rest of the job of `robot`, which stands at plan node `at`
/// at the end of its released route: it goes to the same places (the nodes of the
/// rest of the plan that have actions, and the last node) and does the same
/// actions, but **does not enter a node another robot holds** (it holds or has
/// had released). `None` if there is no such route, if the route would change
/// nothing, or if the job uses a lift or a door (those have their own rules and
/// claims that depend on the route).
#[allow(clippy::too_many_arguments)]
fn reroute(
    graph: &Graph,
    reservations: &Reservations,
    lifts: &LiftDesk,
    doors: &DoorDesk,
    mover: &Mover,
    robot: usize,
    plan: &Plan,
    at: usize,
) -> Option<Plan> {
    let special = |e| lifts.is_ride(e) || doors.is_guarded(e);
    if lifts.owns(robot) || doors.owns(robot) || plan.edges[at..].iter().any(|&e| special(e)) {
        return None;
    }
    // no lift ride and no door; the edges the robot's type may drive (S55) and the
    // turns it can make (S73) are in `mover`
    let blocked: std::collections::HashSet<NodeIdx> = (0..graph.nodes.len())
        .filter(|&n| reservations.node_holder(n).is_some_and(|h| h != robot))
        .collect();

    let mut nodes = plan.nodes[..=at].to_vec();
    let mut edges = plan.edges[..at].to_vec();
    let mut actions = plan.actions[..=at].to_vec();
    let mut here = plan.nodes[at];
    // how the robot arrives at `here`: by the edge it has driven (not known at the
    // very start)
    let mut arrival = if at > 0 {
        Arrival::Edge(plan.edges[at - 1])
    } else {
        Arrival::Free
    };
    for j in at + 1..=plan.last() {
        if j != plan.last() && plan.actions[j].is_empty() {
            continue; // not a place the job needs to stop at
        }
        let target = plan.nodes[j];
        for e in plan_path(graph, mover, arrival, here, target, &blocked, special)? {
            edges.push(e);
            nodes.push(graph.edges[e].end);
            actions.push(Vec::new());
            arrival = Arrival::Edge(e);
        }
        // (there is at least the node the robot stands on)
        actions.last_mut()?.extend(plan.actions[j].iter().cloned());
        here = target;
    }
    (edges != plan.edges).then_some(Plan {
        nodes,
        edges,
        actions,
    })
}

/// Reserve plan nodes `from + 1 ..= wanted` for `robot`, in order, each together
/// with the edge leading to it; stop at the first one somebody else holds.
/// Returns the index of the last node now released (`from` if none).
fn reserve_up_to(
    reservations: &mut Reservations,
    plan: &Plan,
    robot: usize,
    from: usize,
    wanted: usize,
) -> usize {
    let mut rt = from;
    while rt < wanted && reservations.reserve_step(plan.edges[rt], plan.nodes[rt + 1], robot) {
        rt += 1;
    }
    rt
}

/// Not driving, and every action is done.
fn is_idle(state: &State) -> bool {
    !state.driving
        && state.node_states.is_empty()
        && state.action_states.iter().all(|a| {
            matches!(
                a.action_status,
                ActionStatus::Finished | ActionStatus::Failed
            )
        })
}

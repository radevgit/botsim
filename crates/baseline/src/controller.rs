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
//! Not yet: deadlock resolution (two robots can wait for each other for ever),
//! look-ahead beyond the window, timeouts and re-routing, robot types and edge
//! rules, robots that fail.

use std::collections::VecDeque;

use botsim_core::errors::ErrorType;
use botsim_core::graph::{Graph, NodeIdx};
use botsim_core::jobs::Job;
use botsim_core::metrics::{Metrics, is_waiting, summarize};
use botsim_core::robot::Reception;
use botsim_core::world::{RobotId, World};
use vda5050::state::{ActionStatus, State};

use crate::doors::DoorDesk;
use crate::lifts::{LiftDesk, LiftRobot};
use crate::order_builder::{OrderHeader, Plan, RELEASE_WINDOW, build_order};
use crate::reservations::Reservations;
use crate::zone_grants::{RobotView, ZoneGrants};
use vda5050::responses::Responses;

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
}

#[derive(Debug)]
struct Slot {
    id: RobotId,
    work: Option<Work>,
    header_id: u32,
    /// `headerId` of the next `responses` message to this robot.
    response_header_id: u32,
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
    /// Rides in the routes: who owns which lift (see [`crate::lifts`]).
    lifts: LiftDesk,
    /// Guarded edges in the routes: who owns which door (see [`crate::doors`]).
    doors: DoorDesk,
}

impl Controller {
    /// A controller for all robots now in `world`.
    pub fn new(world: &World) -> Controller {
        Controller {
            graph: world.graph().clone(),
            pending: VecDeque::new(),
            robots: world
                .robot_ids()
                .map(|id| Slot {
                    id: id.clone(),
                    work: None,
                    header_id: 0,
                    response_header_id: 0,
                })
                .collect(),
            completed: Vec::new(),
            rejected: Vec::new(),
            last_tick: world.tick(),
            reservations: Reservations::new(world.graph()),
            zone_grants: world.zones().cloned().map(ZoneGrants::new),
            lifts: LiftDesk::new(world),
            doors: DoorDesk::new(world),
        }
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

        let states: Vec<State> = self
            .robots
            .iter()
            .map(|s| world.state(&s.id).expect("the robot exists"))
            .collect();

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
        if state.order_id != work.order_id {
            return; // the robot has not taken the order (cannot happen: orders are sent synchronously)
        }
        let at = (state.last_node_sequence_id / 2) as usize;
        let last = work.plan.last();

        if at == last && is_idle(state) {
            let work = slot.work.take().expect("checked above");
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
            match world.send_order(&msg) {
                Ok(Reception::Accepted) => work.released_through = rt,
                other => panic!("the robot refused an order update: {other:?}"),
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
            let msg: Responses = serde_json::from_value(serde_json::json!({
                "headerId": slot.response_header_id,
                "timestamp": world.timestamp(),
                "version": botsim_core::world::PROTOCOL_VERSION,
                "manufacturer": slot.id.manufacturer,
                "serialNumber": slot.id.serial_number,
                "responses": mine,
            }))
            .expect("the message has all required fields");
            slot.response_header_id += 1;
            world.send_responses(&msg).expect("the robot exists");
        }
    }

    /// Give waiting jobs to free robots.
    fn dispatch(&mut self, world: &mut World, states: &[State], tick: u64) {
        // free robots with the node they stand on
        let mut free: Vec<(usize, NodeIdx)> = (0..self.robots.len())
            .filter(|&i| self.robots[i].work.is_none())
            .filter_map(|i| Some((i, self.node_of(&states[i])?)))
            .collect();

        let mut waiting = VecDeque::new();
        while let Some(job) = self.pending.pop_front() {
            // the nearest free robot that can do the job
            let best = free
                .iter()
                .enumerate()
                .filter_map(|(k, &(i, node))| {
                    let plan =
                        Plan::for_job_with(&self.graph, node, &job, |a, b| self.lifts.route(a, b))?;
                    let to_pickup = self.lifts.route(node, job.pickup)?;
                    let d: f64 = to_pickup.iter().map(|&e| self.graph.edges[e].length).sum();
                    Some((d, i, k, plan))
                })
                // smallest distance; equal distance → the robot added first
                .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            let Some((_, i, k, plan)) = best else {
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
        match world.send_order(&msg).expect("the robot exists") {
            Reception::Accepted => {
                slot.work = Some(Work {
                    job,
                    plan,
                    order_id,
                    update_id: 0,
                    released_through,
                    assigned_tick: tick,
                    waiting_seconds: 0.0,
                });
            }
            Reception::Rejected(reason) => {
                self.lifts.forget(i);
                self.doors.forget(i);
                // give back what was reserved for this order (the robot still holds its node)
                for j in 1..=released_through {
                    self.reservations.release_node(plan.nodes[j], i);
                    self.reservations.release_edge(plan.edges[j - 1], i);
                }
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
        self.graph
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| n.map == map)
            .map(|(k, n)| (k, (n.x - p.x).hypot(n.y - p.y)))
            .filter(|&(_, d)| d <= NODE_TOLERANCE)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(k, _)| k)
    }
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

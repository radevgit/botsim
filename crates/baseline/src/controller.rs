//! The baseline controller: jobs go first come first served to the nearest
//! free robot; the route is released node by node with a window of
//! [`RELEASE_WINDOW`](crate::order_builder::RELEASE_WINDOW) nodes.
//!
//! It sees the robots only through their state reports and talks to them, the
//! doors and the lifts only with messages, like any real controller. The messages
//! are those of [`botsim_link`], which for robots have the meaning of VDA 5050. It
//! is given the site once ([`Config`]) and runs in cycles: each call of
//! [`Controller::step`] takes what has arrived and gives back the messages to send
//! (see [`crate::cycle`]). In a simulation it runs once per step, after the robots
//! have moved.
//!
//! **Refusals** show only in the state reports. The controller takes each order
//! and update as accepted when it sends it, and notes what to take back. A state
//! that does not show it yet and has an error naming the `orderId`, or a new
//! `ORIENTATION_UNREACHABLE` error for an edge sent (that error names no order),
//! says it was refused: a new order is given up at once; for an update the order
//! is cancelled, and the job is given up as soon as the robot has the cancel and
//! has stopped.
//!
//! - **Free robot**: no job assigned. Its place is the layout node nearest to
//!   its reported position (within [`NODE_TOLERANCE`]); a robot that is not on a
//!   node is not used.
//! - **Tasks**: which robot gets which job, and which one drives to a charger or
//!   a parking place, is decided by a [`Dispatcher`] (the baseline's: the oldest
//!   job first, to the nearest robot; see
//!   [`botsim_dispatcher::baseline`]).
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
//!
//! **Parts:** this module carries out the tasks (messages, releases, re-planning,
//! lifts, doors, what becomes of a robot's work). Who gets which task is a
//! [`Dispatcher`]. Who may go now (reservations, conflicts, go-first, the wait
//! timeout, zone answers) is a [`TrafficPolicy`] (brief `traffic_policy_slot.md`);
//! the rules above that say so are the baseline's, [`crate::baseline_policy`].

use std::collections::{HashMap, VecDeque};
use std::sync::Arc;

use botsim_dispatcher::baseline::BaselineDispatcher;
use botsim_dispatcher::{Answer, ChargePolicy, Control, Cycle, Dispatcher, Purpose, Site, Task};
use botsim_link::device::FromLift;
use botsim_link::ids::{ActionId, OrderId};
use botsim_link::instant::{InstantAction, InstantKind};
use botsim_link::message::{FromRobot, Header, Millis, ToRobot, ToRobotBody};
use botsim_link::response::Response;
use botsim_link::state::{ActionStatus, Error, ErrorKind, RequestStatus, State};
use botsim_link::zone_set::ZoneSet;
use botsim_model::errors::ErrorType;
use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_model::job::Job;
use botsim_model::robot::{RobotId, RobotNo};
use botsim_model::spec::Shape;
use botsim_model::waiting::{HeldBack, WaitReason};
use botsim_router::load::{self, Load};
use botsim_router::route::{Arrival, Mover, plan_path};
use botsim_router::types::TypeRouting;

use crate::baseline_policy::BaselinePolicy;
pub use crate::baseline_policy::{
    GIVE_WAY_AFTER_SECONDS, GIVE_WAY_HOLD_SECONDS, REROUTE_AFTER_SECONDS,
};
use crate::conflicts::{ConflictInput, NodeConflictInput, NodeNodeConflictInput};
use crate::cycle::{Config, Input, Message, Output, robot_no};
use crate::doors::DoorDesk;
use crate::lifts::{LiftDesk, LiftRobot};
use crate::metrics::{Metrics, summarize};
use crate::node_grid::NodeGrid;
use crate::order_builder::{OrderHeader, Plan, build_order_with};
use crate::policy::{Ask, Fleet, PolicyRobot, RouteView, Setup, TrafficPolicy};
use crate::reservations::Reservations;

/// The time constant of the waiting average per node that the route cost uses
/// (route with load).
pub const WAIT_AVERAGE_SECONDS: f64 = 60.0;

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

/// The start of the order id of a task.
fn order_prefix(purpose: Purpose) -> &'static str {
    match purpose {
        Purpose::Job => "job",
        Purpose::Charge => "charge",
        Purpose::Park => "park",
    }
}

#[derive(Debug)]
struct Work {
    job: Job,
    plan: Plan,
    order_id: OrderId,
    update_id: u32,
    /// Plan index of the last released node.
    released_through: usize,
    assigned_tick: u64,
    waiting_seconds: f64,
    /// A transport job, or a trip the dispatcher sends the robot on. The `job` of
    /// a trip is a stand-in (the robot's node and the goal), the route has no
    /// actions, and a trip is not counted as a completed or rejected job.
    purpose: Purpose,
    /// The order and the updates the robot's states have not shown yet, oldest
    /// first: any of them may still be refused.
    unconfirmed: VecDeque<Sent>,
    /// The robot refused an update of the order (S70, e.g. a car-like robot that
    /// cannot take a turn): the order was cancelled, and the job is given up as soon
    /// as the robot has stopped. Until then the robot keeps what it holds.
    cancelling: Option<Cancelling>,
}

impl Work {
    /// The robot's states have not shown the order yet (S58): the controller does
    /// not know whether it was accepted.
    fn awaiting_ack(&self) -> bool {
        self.unconfirmed.front().is_some_and(|s| s.update_id == 0)
    }

    /// Take off the order and the updates that `state` shows as accepted.
    fn confirm(&mut self, state: &State) {
        if state.order_id.as_ref() != Some(&self.order_id) {
            return;
        }
        let before = self.unconfirmed.len();
        while self
            .unconfirmed
            .front()
            .is_some_and(|s| s.update_id <= state.update_id)
        {
            self.unconfirmed.pop_front();
        }
        // a robot clears its errors when it accepts an order, so those it reports
        // now are all new for the messages that come after
        if self.unconfirmed.len() < before {
            for sent in &mut self.unconfirmed {
                sent.orientation_errors = 0;
            }
        }
    }
}

/// An order or update on its way: what to take back if the robot refuses it.
#[derive(Debug)]
struct Sent {
    update_id: u32,
    /// The released node index before this message.
    released_before: usize,
    /// The route before this message, if the message added a job to a trip.
    plan_before: Option<Plan>,
    /// The message carried another route (S53).
    rerouted: bool,
    /// The edges the message names.
    edges: Vec<EdgeIdx>,
    /// The `ORIENTATION_UNREACHABLE` errors naming one of these edges in the
    /// robot's state when the message was sent.
    orientation_errors: usize,
}

/// A refused update: the order is being cancelled.
#[derive(Debug)]
struct Cancelling {
    reason: ErrorType,
    /// The id of the `cancelOrder` action.
    action: ActionId,
}

#[derive(Debug)]
struct Slot {
    id: RobotId,
    /// The robot's number in the messages: its place in the fleet.
    no: RobotNo,
    /// Index into `Controller::types`.
    type_idx: usize,
    work: Option<Work>,
    /// The series name of the robot's type (with the manufacturer in `id`).
    series: String,
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

impl Slot {
    /// A message to this robot, as the `seq`-th of its kind.
    fn message(&self, seq: u32, time: Millis, body: ToRobotBody) -> Message {
        let header = Header {
            robot: self.no,
            seq,
            time,
        };
        Message::Robot(ToRobot { header, body })
    }
}

/// The controller, with the traffic rules behind `P` (by default the baseline's,
/// [`BaselinePolicy`]) and a [`Dispatcher`] (by default the baseline's,
/// [`BaselineDispatcher`]).
#[derive(Debug)]
pub struct Controller<P: TrafficPolicy = BaselinePolicy> {
    graph: Graph,
    /// Which robot gets which task; it holds the jobs not yet given out.
    dispatcher: Box<dyn Dispatcher>,
    robots: Vec<Slot>,
    completed: Vec<Completed>,
    rejected: Vec<Rejected>,
    /// The tick of the previous control cycle.
    last_tick: u64,
    /// The seconds between two ticks.
    step_seconds: f64,
    /// Who may go now.
    policy: P,
    /// The site has zones: a robot is ready for tasks only with its zone set
    /// enabled.
    zoned: bool,
    /// The zone set of the site, which the controller sends to every robot and
    /// enables (S62); `None` if the run has no zones, or if it has them but the
    /// controller was given no message for them ([`Config::zone_set`]).
    zone_set: Option<Arc<ZoneSet>>,
    /// Rides in the routes: who owns which lift (see [`crate::lifts`]).
    lifts: LiftDesk,
    /// Guarded edges in the routes: who owns which door (see [`crate::doors`]).
    doors: DoorDesk,
    /// Which node a position is on (S52).
    node_grid: NodeGrid,
    /// What each robot type may drive and the edges into each node for it (S52,
    /// S55). Indexed by `Slot::type_idx`.
    types: Vec<TypeRouting>,
    /// Every lift ride with the waiting node of its entrance (a copy of the lift
    /// desk's, which does not change), for the dispatcher's routes.
    rides: HashMap<EdgeIdx, Option<NodeIdx>>,
    /// How many times a route was changed because the way was held (S53).
    reroutes: usize,
    /// Metres of route cost added for every robot that holds an edge, or the node
    /// it ends at, when a route is planned; 0 (default) is the plain shortest route.
    load_weight: f64,
    /// Metres of route cost added per robot that has recently waited on the node an
    /// edge ends at (see `node_wait`); 0 (default) is off.
    wait_weight: f64,
    /// For each node, how many robots stood waiting on it, as a moving average over
    /// [`WAIT_AVERAGE_SECONDS`] (1.0 = one robot waiting all the time).
    node_wait: Vec<f64>,
}

impl Controller {
    /// A controller with the baseline's traffic rules for the fleet of `config`.
    pub fn new(config: &Config) -> Controller {
        Controller::with_policy(config, BaselinePolicy::new(&config.graph))
    }

    /// Swept-area reservation of the baseline rules, see
    /// [`BaselinePolicy::set_conflicts`].
    pub fn set_conflicts(&mut self, input: &[ConflictInput]) -> usize {
        self.policy.set_conflicts(input)
    }

    /// Node-edge conflicts of the baseline rules, see
    /// [`BaselinePolicy::set_node_conflicts`].
    pub fn set_node_conflicts(&mut self, input: &[NodeConflictInput]) -> usize {
        self.policy.set_node_conflicts(input)
    }

    /// Node-node conflicts of the baseline rules, see
    /// [`BaselinePolicy::set_node_node_conflicts`].
    pub fn set_node_node_conflicts(&mut self, input: &[NodeNodeConflictInput]) -> usize {
        self.policy.set_node_node_conflicts(input)
    }

    /// Who holds which node and edge.
    pub fn reservations(&self) -> &Reservations {
        self.policy.table()
    }

    /// How many times a robot went first, ahead of the robot stuck behind it
    /// (S77, [`GIVE_WAY_AFTER_SECONDS`]).
    pub fn give_ways(&self) -> usize {
        self.policy.give_ways()
    }
}

impl<P: TrafficPolicy> Controller<P> {
    /// A controller for the fleet of `config` with the traffic rules of `policy`
    /// and the baseline's dispatcher.
    pub fn with_policy(config: &Config, policy: P) -> Controller<P> {
        Controller::with_dispatcher(config, policy, Box::new(BaselineDispatcher::new()))
    }

    /// A controller for the fleet of `config` with the traffic rules of `policy`
    /// and the tasks given out by `dispatcher` (which it starts).
    pub fn with_dispatcher(
        config: &Config,
        mut policy: P,
        mut dispatcher: Box<dyn Dispatcher>,
    ) -> Controller<P> {
        // the robot types (manufacturer, series) in the order they first appear
        let graph = &config.graph;
        let mut keys: Vec<(String, String)> = Vec::new();
        let mut kinds = Vec::new();
        // the body outline of each robot, for the policy
        let mut shapes: Vec<Shape> = Vec::new();
        let robots: Vec<Slot> = config
            .robots
            .iter()
            .enumerate()
            .map(|(i, robot)| {
                let key = (robot.id.manufacturer.clone(), robot.series.clone());
                let type_idx = keys.iter().position(|k| *k == key).unwrap_or_else(|| {
                    keys.push(key);
                    kinds.push(robot.kinematics);
                    keys.len() - 1
                });
                shapes.push(robot.shape);
                Slot {
                    id: robot.id.clone(),
                    no: robot_no(i),
                    type_idx,
                    work: None,
                    series: robot.series.clone(),
                    header_id: 0,
                    response_header_id: 0,
                    zone_set_sent: false,
                    enable_sent: false,
                    zone_set_header_id: 0,
                    instant_header_id: 0,
                }
            })
            .collect();
        let types: Vec<TypeRouting> = keys
            .iter()
            .zip(&kinds)
            .map(|((m, s), &kinematics)| TypeRouting::new(graph, kinematics, m, s))
            .collect();
        let type_names: Vec<String> = keys.iter().map(|(m, s)| format!("{m}/{s}")).collect();
        let policy_robots: Vec<PolicyRobot> = robots
            .iter()
            .zip(shapes)
            .map(|(slot, shape)| PolicyRobot {
                id: slot.id.clone(),
                series: slot.series.clone(),
                type_idx: slot.type_idx,
                shape,
                omnidirectional: types[slot.type_idx].omnidirectional,
            })
            .collect();
        let zones = config.zones.as_ref().filter(|z| z.map < graph.maps.len());
        policy.start(&Setup {
            graph,
            types: &type_names,
            robots: &policy_robots,
            zones,
        });
        dispatcher.start(graph, robots.len());
        let lifts = LiftDesk::new(graph, &config.lifts);
        let rides = lifts.waiting_nodes().clone();
        Controller {
            graph: graph.clone(),
            dispatcher,
            robots,
            completed: Vec::new(),
            rejected: Vec::new(),
            last_tick: config.start_tick,
            step_seconds: config.step_seconds,
            policy,
            zoned: zones.is_some(),
            zone_set: zones.and(config.zone_set.clone()),
            lifts,
            doors: DoorDesk::new(&config.doors),
            node_grid: NodeGrid::new(graph),
            types,
            rides,
            reroutes: 0,
            load_weight: 0.0,
            wait_weight: 0.0,
            node_wait: vec![0.0; graph.nodes.len()],
        }
    }

    /// The robots, in the order of the states [`Controller::step`] takes.
    pub fn robots(&self) -> impl Iterator<Item = &RobotId> {
        self.robots.iter().map(|s| &s.id)
    }

    /// The parking places of the layout (U13): the interaction nodes of the stations
    /// of type `parking`. With none, idle robots stay where they are.
    pub fn parking_places(&self) -> usize {
        self.dispatcher.counts().parking_places
    }

    /// Trips to a parking place started so far.
    pub fn park_trips(&self) -> usize {
        self.dispatcher.counts().park_trips
    }

    /// How often a robot without a job found no free parking place (or none it can
    /// reach) and stayed where it was; counted once until it gets a route again.
    pub fn no_free_parking(&self) -> usize {
        self.dispatcher.counts().no_free_parking
    }

    /// Send robots to chargers (charging brief, step 2): a free robot whose reported
    /// charge is below `policy.low` drives to the nearest charger that is free (no
    /// robot on it, none on its way there), and the controller sends `startCharging`
    /// when it has arrived. Chargers are the stations of type `charger` (any case) of
    /// the layout. Returns the number of charger nodes (0: nothing will happen).
    pub fn set_charging(&mut self, policy: ChargePolicy) -> usize {
        self.dispatcher.set_charging(policy)
    }

    /// Trips to a charger started so far.
    pub fn charge_trips(&self) -> usize {
        self.dispatcher.counts().charge_trips
    }

    /// Simulated seconds free robots below the low limit have waited for a charger
    /// (all robots together; robots that are busy with a job are not counted).
    pub fn charge_wait_seconds(&self) -> f64 {
        self.dispatcher.counts().charge_wait_seconds
    }

    /// Route with load, second part: every edge also costs `weight` metres for each
    /// robot that has recently waited on the node it ends at (a moving average over
    /// [`WAIT_AVERAGE_SECONDS`]), so that routes avoid places where robots keep
    /// waiting, also far ahead of a robot. 0 turns it off; a negative or non-finite
    /// weight is taken as 0.
    pub fn set_wait_weight(&mut self, weight: f64) {
        self.wait_weight = if weight.is_finite() && weight > 0.0 {
            weight
        } else {
            0.0
        };
    }

    /// Route with load: when a route is planned (at the start of a job and after a
    /// wait timeout), every edge costs its length plus `weight` metres for each
    /// other robot that holds the edge and for each that holds the node it ends at,
    /// so that an equally long empty lane is preferred to a busy one. 0 turns it
    /// off. A negative or non-finite weight is taken as 0.
    pub fn set_load_weight(&mut self, weight: f64) {
        self.load_weight = if weight.is_finite() && weight > 0.0 {
            weight
        } else {
            0.0
        };
    }

    /// How many times a robot's route was changed because the policy asked for it
    /// (the baseline: another robot held the way for longer than
    /// [`REROUTE_AFTER_SECONDS`]).
    pub fn reroutes(&self) -> usize {
        self.reroutes
    }

    /// The traffic policy.
    pub fn policy(&self) -> &P {
        &self.policy
    }

    /// The traffic policy, e.g. to give it the conflict lists
    /// ([`TrafficPolicy::conflicts`]).
    pub fn policy_mut(&mut self) -> &mut P {
        &mut self.policy
    }

    /// Throughput and waiting of the jobs done so far (see
    /// [`crate::metrics`]); `elapsed_seconds` is the simulated time.
    pub fn metrics(&self, elapsed_seconds: f64) -> Metrics {
        let waits: Vec<f64> = self.completed.iter().map(|c| c.waiting_seconds).collect();
        summarize(&waits, elapsed_seconds)
    }

    /// Jobs not yet assigned, oldest first.
    pub fn pending(&self) -> impl Iterator<Item = &Job> {
        self.dispatcher.pending()
    }

    pub fn completed(&self) -> &[Completed] {
        &self.completed
    }

    pub fn rejected(&self) -> &[Rejected] {
        &self.rejected
    }

    /// Number of robots working on a job (a trip to a charger counts, a trip to a
    /// parking place does not).
    pub fn busy(&self) -> usize {
        self.robots
            .iter()
            .filter(|s| s.work.as_ref().is_some_and(|w| w.purpose != Purpose::Park))
            .count()
    }

    /// One control cycle: take the new jobs, read every robot's state, move
    /// the release windows on, finish jobs, and assign waiting jobs.
    pub fn step(&mut self, input: &Input, new_jobs: impl IntoIterator<Item = Job>) -> Output {
        for job in new_jobs {
            self.dispatcher.add_job(job);
        }
        let tick = input.time.tick;
        // time since the last cycle: a robot seen waiting now is counted as
        // waiting for all of it
        let elapsed = tick.saturating_sub(self.last_tick) as f64 * self.step_seconds;
        self.last_tick = tick;

        // (without a report of every robot, in the fleet's order, this cycle does
        // nothing)
        let states = input.states;
        let in_place = || {
            let reports = states.iter().zip(&self.robots);
            reports
                .into_iter()
                .all(|(r, slot)| r.header.robot == slot.no)
        };
        if states.len() != self.robots.len() || !in_place() {
            return Output::default();
        }
        let time = input.time.millis();
        let mut out = Vec::new();

        self.learn_refusals(states, time, &mut out);

        // the waiting average of the nodes (only needed for the route cost)
        if self.wait_weight > 0.0 {
            let keep = (-elapsed / WAIT_AVERAGE_SECONDS).exp();
            for w in &mut self.node_wait {
                *w *= keep;
            }
            for (i, report) in states.iter().enumerate() {
                if self.robots[i].work.is_some()
                    && report.state.is_waiting()
                    && let Some(n) = self.node_of(&report.state)
                {
                    self.node_wait[n] += 1.0 - keep;
                }
            }
        }

        // the policy sees everyone's progress before the releases
        self.policy.cycle_start(&Fleet {
            tick,
            elapsed,
            states,
            routes: &route_views(&self.robots),
        });
        for (i, report) in states.iter().enumerate() {
            self.update_work(i, &report.state, (tick, elapsed), time, &mut out);
        }
        // a charging robot that has reached the upper limit stops charging and is
        // free for jobs again
        for (i, report) in states.iter().enumerate() {
            if self.dispatcher.stop_charging(i, report.state.charge) {
                let slot = &mut self.robots[i];
                let id = ActionId::new(format!("stop-charge-{}", slot.instant_header_id));
                send_instant(&mut out, time, slot, InstantKind::StopCharging, id);
            }
        }
        self.policy.after_releases(&Fleet {
            tick,
            elapsed,
            states,
            routes: &route_views(&self.robots),
        });
        self.update_lifts(input.lifts, states, time, &mut out);
        // the policy hears of every robot without a job and where it stands
        for (i, report) in states.iter().enumerate() {
            if self.robots[i].work.is_none()
                && let Some(n) = self.node_of(&report.state)
            {
                self.policy.idle(i, n);
            }
        }
        self.update_zone_sets(states, time, &mut out);
        self.answer_zone_requests(states, (tick, elapsed), time, &mut out);
        self.dispatch(states, (tick, elapsed), time, &mut out);
        Output {
            messages: out,
            // why each robot is held back, and by whom (who waits for whom:
            // deadlocks)
            held_back: Some(self.held_back(states)),
            // what is reserved (only if someone watches)
            reservations: input.watched.then(|| self.policy.reservations()),
        }
    }

    /// Take off what the states show as accepted, and act on what they show as
    /// refused: a new order is given up at once; for an update, the release and
    /// the route it brought are taken back and the order is cancelled (S70).
    fn learn_refusals(&mut self, states: &[FromRobot], time: Millis, out: &mut Vec<Message>) {
        for (i, (slot, report)) in self.robots.iter_mut().zip(states).enumerate() {
            let state = &report.state;
            let Some(work) = slot.work.as_mut() else {
                continue;
            };
            if work.cancelling.is_some() {
                continue;
            }
            work.confirm(state);
            let Some(reason) = refusal(&self.graph, state, work) else {
                continue;
            };
            if work.awaiting_ack() {
                // The robot refused the order: give the job up.
                let Some(work) = slot.work.take() else {
                    continue;
                };
                give_back(
                    &mut self.policy,
                    &mut self.lifts,
                    &mut self.doors,
                    i,
                    &work.plan,
                    work.released_through,
                );
                if work.purpose == Purpose::Job {
                    self.rejected.push(Rejected {
                        job: work.job,
                        robot: slot.id.clone(),
                        reason,
                    });
                }
                self.dispatcher.given_up(i, work.purpose);
                continue;
            }
            // the oldest unconfirmed update was not accepted, so neither was what
            // came with it or after it
            let mut sent = std::mem::take(&mut work.unconfirmed);
            if let Some(first) = sent.front() {
                work.released_through = first.released_before;
            }
            self.reroutes = self
                .reroutes
                .saturating_sub(sent.iter().filter(|s| s.rerouted).count());
            if let Some(plan) = sent.iter_mut().find_map(|s| s.plan_before.take()) {
                work.plan = plan;
                self.policy.route_changed(i, &work.plan);
            }
            cancel_order(out, time, slot, reason);
        }
    }

    /// For each robot that stands still on its order: why it is held back, and by
    /// whom (see [`botsim_model::waiting`]).
    /// - Road released ahead of it and a zone request not granted: the zone, with
    ///   the robots the policy names.
    /// - At the end of its released route, with more to go: the lift or door another
    ///   robot owns, else the release, with the policy's holder.
    fn held_back(&self, states: &[FromRobot]) -> Vec<Option<HeldBack>> {
        self.robots
            .iter()
            .zip(states)
            .enumerate()
            .map(|(i, (slot, report))| {
                let state = &report.state;
                let work = slot.work.as_ref()?;
                if work.cancelling.is_some()
                    || state.order_id.as_ref() != Some(&work.order_id)
                    || !state.is_waiting()
                {
                    return None;
                }
                let at = state.last_step() as usize;
                let rt = work.released_through;
                let (reason, blockers) = if at < rt {
                    let asks = state.zone_requests.iter().flatten().any(|q| {
                        matches!(q.status, RequestStatus::Requested | RequestStatus::Queued)
                    });
                    if !asks {
                        return None;
                    }
                    (WaitReason::Zone, self.policy.zone_blockers(i))
                } else if at == rt && rt < work.plan.last() {
                    if let Some(owner) = self.lifts.held_by(i, &work.plan, rt) {
                        (WaitReason::Lift, vec![owner])
                    } else if let Some(owner) = self.doors.held_by(i, &work.plan, rt) {
                        (WaitReason::Door, vec![owner])
                    } else {
                        let holder = self.policy.blocker(i, &work.plan, rt);
                        (WaitReason::Release, holder.into_iter().collect())
                    }
                } else {
                    return None;
                };
                Some(HeldBack { reason, blockers })
            })
            .collect()
    }

    /// Release more of the route, or finish the job, for robot `i`.
    fn update_work(
        &mut self,
        i: usize,
        state: &State,
        (tick, elapsed): (u64, f64),
        time: Millis,
        out: &mut Vec<Message>,
    ) {
        let slot = &mut self.robots[i];
        let Some(work) = slot.work.as_mut() else {
            return;
        };
        if let Some(cancel) = &work.cancelling {
            // wait until the robot has the cancel (its state lists the action) and
            // has stopped (it brakes to the next node); until then the way it is on
            // stays reserved
            let reason = cancel.reason;
            let has_it = state.instant_actions.iter().any(|a| a.id == cancel.action);
            if has_it && !state.driving {
                let Some(work) = slot.work.take() else {
                    return;
                };
                give_back(
                    &mut self.policy,
                    &mut self.lifts,
                    &mut self.doors,
                    i,
                    &work.plan,
                    work.released_through,
                );
                if work.purpose == Purpose::Job {
                    self.rejected.push(Rejected {
                        job: work.job,
                        robot: slot.id.clone(),
                        reason,
                    });
                }
                self.dispatcher.given_up(i, work.purpose);
            }
            return;
        }
        if work.awaiting_ack() {
            return; // not answered yet
        }
        if state.order_id.as_ref() != Some(&work.order_id) {
            return; // the robot has not taken the order yet
        }
        let at = state.last_step() as usize;
        let last = work.plan.last();

        if at == last && is_idle(state) {
            let Some(work) = slot.work.take() else {
                return;
            };
            self.policy.route_finished(i, &work.plan);
            self.dispatcher.arrived(i, work.purpose);
            match work.purpose {
                Purpose::Job => {}
                Purpose::Charge => {
                    // arrived at the charger: start charging; no job was done
                    let id = ActionId::new(format!("charge-{}", work.order_id));
                    send_instant(out, time, slot, InstantKind::StartCharging, id);
                    return;
                }
                // the robot stands on its place now
                Purpose::Park => return,
            }
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
        if state.is_waiting() {
            work.waiting_seconds += elapsed;
        }

        // ask the policy how far the route may be released now
        let from = work.released_through;
        let wanted = work.plan.released_through(at, self.policy.window());
        // a ride in the window needs its lift (S47)
        let wanted = self.lifts.window(i, &work.plan, at, from, wanted);
        // so does an edge through a door (S51)
        let wanted = self.doors.window(i, &work.plan, at, from, wanted);
        let answer = self.policy.release(&Ask {
            robot: i,
            state,
            route: &work.plan,
            at,
            released_through: from,
            limit: wanted,
            elapsed,
        });
        // (what is released cannot be taken back, and the limit holds)
        let rt = answer.through.min(wanted).max(from);
        self.doors.settle(i, rt);

        // the policy may ask for another route beyond what is released (S53)
        let mut changed = false;
        if let Some(wish) = answer.reroute {
            let loaded = loaded_cost(
                &self.types[slot.type_idx].cost,
                &self.graph,
                &self.policy,
                (self.load_weight, self.wait_weight, &self.node_wait),
                i,
            );
            let mover = match &loaded {
                Some(c) => self.types[slot.type_idx].mover_with(c),
                None => self.types[slot.type_idx].mover(),
            };
            if let Some(plan) = reroute(
                &self.graph,
                &wish.avoid,
                &self.lifts,
                &self.doors,
                &mover,
                i,
                &work.plan,
                rt,
            ) {
                work.plan = plan;
                changed = true;
                self.policy.route_changed(i, &work.plan);
            }
        }

        // one order update for a changed route and for newly released nodes
        if changed || rt > from {
            work.update_id += 1;
            let header = OrderHeader {
                robot: slot.no,
                seq: slot.header_id,
                time,
                order_id: &work.order_id,
                order_update_id: work.update_id,
            };
            let orient = self.types[slot.type_idx]
                .omnidirectional
                .then_some((slot.id.manufacturer.as_str(), slot.series.as_str()));
            let msg = build_order_with(&self.graph, &header, &work.plan, from, rt, orient);
            slot.header_id += 1;
            match msg {
                Some(msg) => {
                    out.push(Message::Robot(msg));
                    work.unconfirmed.push_back(sent(
                        &self.graph,
                        state,
                        work.update_id,
                        from,
                        changed,
                        work.plan.edges[from..].to_vec(),
                    ));
                    work.released_through = rt;
                    if changed {
                        self.reroutes += 1;
                    }
                }
                // no message can be made for the next part or the changed route:
                // give the job up, as for a refused update (S70)
                None => cancel_order(out, time, slot, ErrorType::ValidationFailure),
            }
        }
    }

    /// Call, release and send the lifts that robots use (see [`crate::lifts`]).
    fn update_lifts(
        &mut self,
        lifts: &[FromLift],
        states: &[FromRobot],
        time: Millis,
        out: &mut Vec<Message>,
    ) {
        let nodes: Vec<Option<NodeIdx>> = states.iter().map(|s| self.node_of(&s.state)).collect();
        let ats = self.plan_positions(states);
        let robots: Vec<LiftRobot> = ats
            .iter()
            .zip(nodes)
            .map(|(&at, node)| LiftRobot { at, node })
            .collect();
        self.lifts.update(lifts, &robots, time, out);
        self.doors.update(&ats, time, out);
    }

    /// The plan index of each robot's last node; `None` for a robot without a job.
    fn plan_positions(&self, states: &[FromRobot]) -> Vec<Option<usize>> {
        self.robots
            .iter()
            .zip(states)
            .map(|(slot, report)| {
                let state = &report.state;
                slot.work
                    .as_ref()
                    .filter(|w| state.order_id.as_ref() == Some(&w.order_id))
                    .map(|_| state.last_step() as usize)
            })
            .collect()
    }

    /// Whether the robot's copy of the site's zone set is enabled, `None` if it has
    /// none (S62).
    fn zone_set_enabled(&self, state: &State) -> Option<bool> {
        let id = &self.zone_set.as_ref()?.id;
        state
            .zone_sets
            .iter()
            .find(|z| z.id == *id)
            .map(|z| z.enabled)
    }

    /// Is the robot ready for jobs as far as zones go: the run has no zones, or
    /// the robot reports the site's zone set as enabled? A robot without its
    /// zones would drive into RELEASE zones without asking.
    fn zones_ready(&self, state: &State) -> bool {
        !self.zoned || self.zone_set_enabled(state) == Some(true)
    }

    /// Give every robot the site's zone set (S62): send the `zoneSet` message; when
    /// the robot's state shows it (disabled), send the `enableZoneSet` instant
    /// action. Each is sent once; the controller reads the answer from the state
    /// (which may be old, with a message delay).
    fn update_zone_sets(&mut self, states: &[FromRobot], time: Millis, out: &mut Vec<Message>) {
        let Some(zs) = self.zone_set.clone() else {
            return;
        };
        for (i, report) in states.iter().enumerate() {
            let enabled = self.zone_set_enabled(&report.state);
            let slot = &mut self.robots[i];
            if enabled.is_none() && !slot.zone_set_sent {
                let body = ToRobotBody::ZoneSet(zs.clone());
                out.push(slot.message(slot.zone_set_header_id, time, body));
                slot.zone_set_header_id += 1;
                slot.zone_set_sent = true;
            } else if enabled == Some(false) && !slot.enable_sent {
                let id = ActionId::new(format!("enable-{}", zs.id));
                let kind = InstantKind::EnableZoneSet {
                    zone_set: zs.id.clone(),
                };
                send_instant(out, time, slot, kind, id);
                slot.enable_sent = true;
            }
        }
    }

    /// Send the policy's answers to the RELEASE zone requests the robots state.
    fn answer_zone_requests(
        &mut self,
        states: &[FromRobot],
        (tick, elapsed): (u64, f64),
        time: Millis,
        out: &mut Vec<Message>,
    ) {
        let answers = self.policy.zone_answers(&Fleet {
            tick,
            elapsed,
            states,
            routes: &route_views(&self.robots),
        });
        // one `responses` message per robot
        for (i, slot) in self.robots.iter_mut().enumerate() {
            let mine: Vec<Response> = answers
                .iter()
                .filter(|a| a.robot == i)
                .map(|a| Response {
                    request: a.request_id.clone(),
                    grant: a.grant,
                    lease_expiry: None,
                })
                .collect();
            if mine.is_empty() {
                continue;
            }
            let body = ToRobotBody::Responses(mine);
            out.push(slot.message(slot.response_header_id, time, body));
            slot.response_header_id += 1;
        }
    }

    /// Let the dispatcher give out tasks: waiting jobs, and trips to chargers and
    /// parking places. It acts through [`Orders`].
    fn dispatch(
        &mut self,
        states: &[FromRobot],
        (tick, elapsed): (u64, f64),
        time: Millis,
        out: &mut Vec<Message>,
    ) {
        let robots: Vec<botsim_dispatcher::Robot> = states
            .iter()
            .enumerate()
            .map(|(i, report)| self.dispatch_view(i, &report.state))
            .collect();
        let site = Site {
            graph: &self.graph,
            types: &self.types,
            rides: &self.rides,
        };
        let mut orders = Orders {
            graph: &self.graph,
            types: &self.types,
            node_grid: &self.node_grid,
            weights: (self.load_weight, self.wait_weight, &self.node_wait),
            robots: &mut self.robots,
            policy: &mut self.policy,
            lifts: &mut self.lifts,
            doors: &mut self.doors,
            rejected: &mut self.rejected,
            tick,
            time,
            out,
            states,
        };
        self.dispatcher
            .dispatch(&site, &robots, Cycle { tick, elapsed }, &mut orders);
    }

    /// Robot `i` as the dispatcher sees it.
    fn dispatch_view(&self, i: usize, state: &State) -> botsim_dispatcher::Robot {
        let slot = &self.robots[i];
        let work = match &slot.work {
            None => botsim_dispatcher::Work::None,
            Some(w) if takes_extension(w, i, state, &self.lifts, &self.doors) => {
                let at = state.last_step() as usize;
                let rt = w.released_through;
                botsim_dispatcher::Work::Parking {
                    from: w.plan.nodes[rt],
                    ahead: w.plan.edges.get(at..rt).unwrap_or_default().to_vec(),
                    arrival: (rt > 0).then(|| w.plan.edges[rt - 1]),
                }
            }
            Some(_) => botsim_dispatcher::Work::Busy,
        };
        botsim_dispatcher::Robot {
            type_idx: slot.type_idx,
            node: self.node_of(state),
            heading: state.position.as_ref().map(|p| p.theta),
            charge: state.charge,
            ready: self.zones_ready(state),
            work,
        }
    }

    /// The layout node a robot stands on, from its reported position.
    fn node_of(&self, state: &State) -> Option<NodeIdx> {
        node_of(&self.node_grid, state)
    }
}

/// What the controller does for the dispatcher during one dispatch: it answers its
/// questions and carries out its tasks (see [`Control`]).
struct Orders<'a, P: TrafficPolicy> {
    graph: &'a Graph,
    types: &'a [TypeRouting],
    node_grid: &'a NodeGrid,
    /// (`load_weight`, `wait_weight`, `node_wait`), see [`loaded_cost`].
    weights: (f64, f64, &'a [f64]),
    robots: &'a mut [Slot],
    policy: &'a mut P,
    lifts: &'a mut LiftDesk,
    doors: &'a mut DoorDesk,
    rejected: &'a mut Vec<Rejected>,
    tick: u64,
    time: Millis,
    /// The messages of the cycle.
    out: &'a mut Vec<Message>,
    states: &'a [FromRobot],
}

impl<P: TrafficPolicy> Orders<'_, P> {
    /// Does the route of `task` start at `from` and connect edge by edge, and, for
    /// a job, lead to its pickup node after `pickup_at` edges and end at its drop
    /// node?
    fn connects(&self, from: NodeIdx, task: &Task) -> bool {
        if task.from != from {
            return false;
        }
        let mut here = from;
        let mut pickup = (task.pickup_at == 0).then_some(from);
        for (k, &e) in task.edges.iter().enumerate() {
            match self.graph.edges.get(e) {
                Some(edge) if edge.start == here => here = edge.end,
                _ => return false,
            }
            if k + 1 == task.pickup_at {
                pickup = Some(here);
            }
        }
        task.purpose != Purpose::Job || (pickup == Some(task.job.pickup) && here == task.job.drop)
    }

    /// The route of `task` with its actions.
    fn plan(&self, task: Task) -> Option<Plan> {
        match task.purpose {
            Purpose::Job => Plan::for_edges(self.graph, task.from, task.edges, task.pickup_at),
            Purpose::Charge | Purpose::Park => Some(Plan::new(self.graph, task.from, task.edges)),
        }
    }

    /// Place the route of a free robot with the traffic rules and send the order.
    fn assign(&mut self, i: usize, job: Job, mut plan: Plan, purpose: Purpose) -> Answer {
        let order_id = OrderId::new(format!("{}-{}", order_prefix(purpose), job.id));
        let wanted = plan.released_through(0, self.policy.window());
        let wanted = self.lifts.window(i, &plan, 0, 0, wanted);
        let wanted = self.doors.window(i, &plan, 0, 0, wanted);
        let answer = self.policy.place(&Ask {
            robot: i,
            state: &self.states[i].state,
            route: &plan,
            at: 0,
            released_through: 0,
            limit: wanted,
            elapsed: 0.0,
        });
        let released_through = answer.through.min(wanted);
        self.doors.settle(i, released_through);
        if let Some(wish) = answer.reroute {
            let kind = &self.types[self.robots[i].type_idx];
            let loaded = self.route_cost(i);
            let mover = match &loaded {
                Some(c) => kind.mover_with(c),
                None => kind.mover(),
            };
            if let Some(other) = reroute(
                self.graph,
                &wish.avoid,
                self.lifts,
                self.doors,
                &mover,
                i,
                &plan,
                released_through,
            ) {
                plan = other;
                self.policy.route_changed(i, &plan);
            }
        }
        let slot = &mut self.robots[i];
        let header = OrderHeader {
            robot: slot.no,
            seq: slot.header_id,
            time: self.time,
            order_id: &order_id,
            order_update_id: 0,
        };
        let orient = self.types[slot.type_idx]
            .omnidirectional
            .then_some((slot.id.manufacturer.as_str(), slot.series.as_str()));
        let msg = build_order_with(self.graph, &header, &plan, 0, released_through, orient);
        slot.header_id += 1;
        match msg {
            Some(msg) => {
                self.out.push(Message::Robot(msg));
                let state = &self.states[i].state;
                let order = sent(self.graph, state, 0, 0, false, plan.edges.clone());
                slot.work = Some(Work {
                    job,
                    plan,
                    order_id,
                    update_id: 0,
                    released_through,
                    assigned_tick: self.tick,
                    waiting_seconds: 0.0,
                    purpose,
                    unconfirmed: VecDeque::from([order]),
                    cancelling: None,
                });
                Answer::Taken
            }
            // no message can be made of this route
            None => {
                give_back(
                    self.policy,
                    self.lifts,
                    self.doors,
                    i,
                    &plan,
                    released_through,
                );
                if purpose == Purpose::Job {
                    self.rejected.push(Rejected {
                        job,
                        robot: slot.id.clone(),
                        reason: ErrorType::ValidationFailure,
                    });
                }
                Answer::Refused
            }
        }
    }
}

impl<P: TrafficPolicy> Control for Orders<'_, P> {
    fn node_holder(&self, node: NodeIdx) -> Option<usize> {
        if node >= self.graph.nodes.len() {
            return None;
        }
        self.policy.node_holder(node)
    }

    fn route_cost(&self, robot: usize) -> Option<Vec<f64>> {
        let kind = &self.types[self.robots.get(robot)?.type_idx];
        loaded_cost(&kind.cost, self.graph, &*self.policy, self.weights, robot)
    }

    fn give(&mut self, i: usize, task: Task) -> Answer {
        let (Some(slot), Some(state)) = (self.robots.get(i), self.states.get(i)) else {
            return Answer::Invalid;
        };
        let here = node_of(self.node_grid, &state.state);
        if slot.work.is_some() || !here.is_some_and(|n| self.connects(n, &task)) {
            return Answer::Invalid;
        }
        let (job, purpose) = (task.job, task.purpose);
        match self.plan(task) {
            Some(plan) => self.assign(i, job, plan, purpose),
            None => Answer::Invalid,
        }
    }

    /// An update of the trip's order from the last released node: the released part
    /// as before, then the job. Nothing more is released here; that happens in the
    /// next cycle.
    fn extend(&mut self, i: usize, task: Task) -> Answer {
        let (Some(slot), Some(state)) = (self.robots.get(i), self.states.get(i)) else {
            return Answer::Invalid;
        };
        let from = slot
            .work
            .as_ref()
            .filter(|w| takes_extension(w, i, &state.state, self.lifts, self.doors))
            .and_then(|w| w.plan.nodes.get(w.released_through));
        // (a robot does not run the actions of the node an update starts at, so the
        // pickup must lie beyond it)
        if task.purpose != Purpose::Job
            || task.pickup_at == 0
            || !from.is_some_and(|&n| self.connects(n, &task))
        {
            return Answer::Invalid;
        }
        let job = task.job;
        let Some(then) = self.plan(task) else {
            return Answer::Invalid;
        };
        let slot = &mut self.robots[i];
        let Some(work) = slot.work.as_mut() else {
            return Answer::Invalid;
        };
        let plan = work.plan.continued(work.released_through, then);
        work.job = job;
        work.purpose = Purpose::Job;
        work.assigned_tick = self.tick;
        work.waiting_seconds = 0.0;
        work.update_id += 1;
        let header = OrderHeader {
            robot: slot.no,
            seq: slot.header_id,
            time: self.time,
            order_id: &work.order_id,
            order_update_id: work.update_id,
        };
        let orient = self.types[slot.type_idx]
            .omnidirectional
            .then_some((slot.id.manufacturer.as_str(), slot.series.as_str()));
        let from = work.released_through;
        let msg = build_order_with(self.graph, &header, &plan, from, from, orient);
        slot.header_id += 1;
        match msg {
            Some(msg) => {
                self.out.push(Message::Robot(msg));
                let mut update = sent(
                    self.graph,
                    &self.states[i].state,
                    work.update_id,
                    from,
                    false,
                    plan.edges[from..].to_vec(),
                );
                update.plan_before = Some(std::mem::replace(&mut work.plan, plan));
                work.unconfirmed.push_back(update);
                self.policy.route_changed(i, &work.plan);
                Answer::Taken
            }
            // no message can be made of this route: cancel the order and give the
            // job up, as for any refused update (S70)
            None => {
                cancel_order(self.out, self.time, slot, ErrorType::ValidationFailure);
                Answer::Refused
            }
        }
    }

    fn charge_here(&mut self, i: usize, trip: usize) {
        if let Some(slot) = self.robots.get_mut(i).filter(|s| s.work.is_none()) {
            let id = ActionId::new(format!("charge-here-{trip}"));
            send_instant(self.out, self.time, slot, InstantKind::StartCharging, id);
        }
    }
}

/// The layout node a robot stands on, from its reported position.
fn node_of(grid: &NodeGrid, state: &State) -> Option<NodeIdx> {
    let p = state.position.as_ref()?;
    grid.node_near(p.map, p.x, p.y, NODE_TOLERANCE)
}

/// Can the trip to a parking place that robot `i` is on take a job now (S144)? Its
/// order is answered and shown by the robot, it is not being cancelled, and the
/// robot owns no lift or door (those claims depend on the route).
fn takes_extension(
    work: &Work,
    i: usize,
    state: &State,
    lifts: &LiftDesk,
    doors: &DoorDesk,
) -> bool {
    work.purpose == Purpose::Park
        && work.cancelling.is_none()
        && !work.awaiting_ack()
        && state.order_id.as_ref() == Some(&work.order_id)
        && !lifts.owns(i)
        && !doors.owns(i)
}

/// The order of a job was refused: give back any lift or door it was given, and tell
/// the policy that the route is gone.
fn give_back<P: TrafficPolicy>(
    policy: &mut P,
    lifts: &mut LiftDesk,
    doors: &mut DoorDesk,
    robot: usize,
    plan: &Plan,
    released_through: usize,
) {
    lifts.forget(robot);
    doors.forget(robot);
    policy.route_dropped(robot, plan, released_through);
}

/// The routes as the policy sees them, one entry per robot.
fn route_views(robots: &[Slot]) -> Vec<Option<RouteView<'_>>> {
    robots
        .iter()
        .map(|s| {
            s.work.as_ref().map(|w| RouteView {
                route: &w.plan,
                released_through: w.released_through,
                cancelling: w.cancelling.is_some(),
            })
        })
        .collect()
}

/// The robot refused an update of its order: tell it to cancel the order (it brakes
/// to the next node and stops), and note that the job is to be given up once it has
/// stopped (see `Work::cancelling`).
fn cancel_order(out: &mut Vec<Message>, time: Millis, slot: &mut Slot, reason: ErrorType) {
    let Some(order_id) = slot.work.as_ref().map(|w| w.order_id.clone()) else {
        return;
    };
    let action = ActionId::new(format!("cancel-{order_id}"));
    let kind = InstantKind::CancelOrder { order_id: None };
    send_instant(out, time, slot, kind, action.clone());
    if let Some(work) = slot.work.as_mut() {
        work.cancelling = Some(Cancelling { reason, action });
    }
}

/// Send one instant action to the robot of `slot`.
fn send_instant(
    out: &mut Vec<Message>,
    time: Millis,
    slot: &mut Slot,
    kind: InstantKind,
    id: ActionId,
) {
    let body = ToRobotBody::InstantActions(vec![InstantAction { id, kind }]);
    out.push(slot.message(slot.instant_header_id, time, body));
    slot.instant_header_id += 1;
}

/// The note on an order or update that names `edges`, made when the robot reports
/// `state`.
fn sent(
    graph: &Graph,
    state: &State,
    update_id: u32,
    released_before: usize,
    rerouted: bool,
    edges: Vec<EdgeIdx>,
) -> Sent {
    let orientation_errors = orientation_errors(graph, state, &edges);
    Sent {
        update_id,
        released_before,
        plan_before: None,
        rerouted,
        edges,
        orientation_errors,
    }
}

/// Is this an `ORIENTATION_UNREACHABLE` error that names one of `edges`?
fn names_orientation(graph: &Graph, error: &Error, edges: &[EdgeIdx]) -> bool {
    error.kind == ErrorKind::Predefined(ErrorType::OrientationUnreachable)
        && error.references.iter().any(|(key, value)| {
            &**key == "edgeId"
                && edges
                    .iter()
                    .any(|&e| graph.edges.get(e).is_some_and(|e| *e.id == **value))
        })
}

/// How many errors of `state` say that an orientation on one of `edges` cannot be
/// taken.
fn orientation_errors(graph: &Graph, state: &State, edges: &[EdgeIdx]) -> usize {
    state
        .errors
        .iter()
        .filter(|e| names_orientation(graph, e, edges))
        .count()
}

/// The error type of an error in `state` that says the robot has refused an order
/// or update of `work` not yet confirmed: an error that names the order (VDA 5050
/// puts the `orderId` in the references), or a new `ORIENTATION_UNREACHABLE` error
/// for an edge sent (it names only the edge).
fn refusal(graph: &Graph, state: &State, work: &Work) -> Option<ErrorType> {
    if work.unconfirmed.is_empty() {
        return None;
    }
    let error = state.errors.iter().find(|e| {
        e.references
            .iter()
            .any(|(key, value)| &**key == "orderId" && &**value == work.order_id.as_str())
            || work.unconfirmed.iter().any(|s| {
                names_orientation(graph, e, &s.edges)
                    && orientation_errors(graph, state, &s.edges) > s.orientation_errors
            })
    })?;
    Some(match error.kind {
        ErrorKind::Predefined(t) => t,
        ErrorKind::Other(_) => ErrorType::ValidationFailure,
    })
}

/// The route cost of every edge for `robot`, with the penalties of
/// [`load::loaded_cost`]: this says to the router which edges and nodes the policy
/// gives to other robots. `weights` is (`load_weight`, `wait_weight`,
/// `node_wait`). `None` if both weights are 0.
fn loaded_cost<P: TrafficPolicy>(
    cost: &[f64],
    graph: &Graph,
    policy: &P,
    weights: (f64, f64, &[f64]),
    robot: usize,
) -> Option<Vec<f64>> {
    let (load_weight, wait_weight, node_wait) = weights;
    let other = |h: Option<usize>| h.is_some_and(|h| h != robot);
    // (who holds what matters only with a load weight)
    let (edge_held, node_held): (Vec<bool>, Vec<bool>) = if load_weight > 0.0 {
        (
            (0..graph.edges.len())
                .map(|e| other(policy.edge_holder(e)))
                .collect(),
            (0..graph.nodes.len())
                .map(|n| other(policy.node_holder(n)))
                .collect(),
        )
    } else {
        Default::default()
    };
    load::loaded_cost(
        graph,
        cost,
        &Load {
            load_weight,
            wait_weight,
            edge_held: &edge_held,
            node_held: &node_held,
            node_wait,
        },
    )
}

/// A new route for the rest of the job of `robot`, which stands at plan node `at`
/// at the end of its released route: it goes to the same places (the nodes of the
/// rest of the plan that have actions, and the last node) and does the same
/// actions, but **does not enter a node in `blocked`** (the policy's wish: the
/// baseline names the nodes other robots hold). `None` if there is no such route, if
/// the route would change nothing, or if the job uses a lift or a door (those have
/// their own rules and claims that depend on the route).
#[allow(clippy::too_many_arguments)]
fn reroute(
    graph: &Graph,
    blocked: &std::collections::HashSet<NodeIdx>,
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
        for e in plan_path(graph, mover, arrival, here, target, blocked, special)? {
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

/// Not driving, and every action is done.
fn is_idle(state: &State) -> bool {
    !state.driving
        && state.route.ahead().is_empty()
        && state
            .actions
            .iter()
            .all(|a| matches!(a.status, ActionStatus::Finished | ActionStatus::Failed))
}

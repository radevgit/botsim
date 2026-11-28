//! The baseline controller: jobs go first come first served to the nearest
//! free robot; the route is released node by node with a window of
//! [`RELEASE_WINDOW`](crate::order_builder::RELEASE_WINDOW) nodes.
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
//!
//! **Two parts** (brief `traffic_policy_slot.md`): this module is the dispatcher
//! (jobs, routes, re-planning, charging, lifts, doors, messages). Who may go now
//! (reservations, conflicts, go-first, the wait timeout, zone answers) is a
//! [`TrafficPolicy`]; the rules above that say so are the baseline's,
//! [`crate::baseline_policy`].

use std::collections::VecDeque;

use botsim_core::jobs::Job;
use botsim_core::metrics::{Metrics, is_waiting, summarize};
use botsim_core::robot::Reception;
use botsim_core::waiting::{HeldBack, WaitReason};
use botsim_core::world::World;
use botsim_model::errors::ErrorType;
use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_model::robot::RobotId;
use botsim_model::spec::{Kinematics, Shape};
use vda5050::order::Order;
use vda5050::state::{ActionStatus, RequestStatus, State};

use crate::baseline_policy::BaselinePolicy;
pub use crate::baseline_policy::{
    GIVE_WAY_AFTER_SECONDS, GIVE_WAY_HOLD_SECONDS, REROUTE_AFTER_SECONDS,
};
use crate::conflicts::{ConflictInput, NodeConflictInput, NodeNodeConflictInput};
use crate::doors::DoorDesk;
use crate::lifts::{LiftDesk, LiftRobot};
use crate::node_grid::NodeGrid;
use crate::order_builder::{OrderHeader, Plan, build_order_with};
use crate::policy::{Ask, Fleet, PolicyRobot, RouteView, Setup, TrafficPolicy};
use crate::reservations::Reservations;
use crate::route::{Arrival, Mover, Turns, distances_to, in_edges, plan_path, travel_lengths};
use vda5050::responses::Responses;

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

/// What a route is for. The `job` of a trip is a stand-in (the robot's node and the
/// goal), the route has no actions, and a trip is not counted as a completed or
/// rejected job.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Purpose {
    Job,
    /// To a charger; on arrival the controller starts the charging.
    Charge,
    /// To a free parking place (U13).
    Park,
}

impl Purpose {
    /// The start of the order id.
    fn order_prefix(self) -> &'static str {
        match self {
            Purpose::Job => "job",
            Purpose::Charge => "charge",
            Purpose::Park => "park",
        }
    }
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
    /// A transport job, or a trip the dispatcher sends the robot on.
    purpose: Purpose,
    /// The order was sent with a message delay and the robot's state has not
    /// shown it yet (S58): the controller does not know whether it was accepted.
    awaiting_ack: bool,
    /// The robot refused an update of the order (S70, e.g. a car-like robot that
    /// cannot take a turn): the order was cancelled, and the job is given up as soon
    /// as the robot has stopped. Until then the robot keeps what it holds.
    cancelling: Option<ErrorType>,
}

/// A robot that a job can be given to.
#[derive(Debug, Clone, Copy)]
struct Candidate {
    robot: usize,
    /// Where its route to the pickup starts: the node it stands on, or the last
    /// released node of its trip to a parking place.
    from: NodeIdx,
    /// Metres it still drives to get there.
    ahead: f64,
    /// It is on its way to a parking place: a job extends that order (S144).
    parking: bool,
}

/// What a robot type may drive (S55): the layout lists the types that may use
/// each node and edge.
#[derive(Debug)]
struct TypeRouting {
    /// An omnidirectional type drives off in any direction without turning.
    omnidirectional: bool,
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

    /// Like [`mover`](Self::mover), with another cost per edge (the length plus the
    /// load penalty).
    fn mover_with<'a>(&'a self, cost: &'a [f64]) -> Mover<'a> {
        Mover {
            ok: &self.ok,
            cost,
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
    /// The robot stands on a charger and `startCharging` was sent: it gets no jobs.
    charging: bool,
    /// It was counted as finding no free parking place; reset when it gets a route.
    unparked: bool,
    /// It refused an order to a parking place: not sent again until it has had a job.
    park_refused: bool,
}

/// When the baseline sends robots to a charger (percent of the charge the robot
/// reports in its state).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChargePolicy {
    /// A free robot below this goes to a charger.
    pub low: f64,
    /// A charging robot gets jobs again from this on.
    pub high: f64,
    /// Below this a robot takes no more jobs and waits for a charger.
    pub critical: f64,
}

/// The controller: the dispatcher, with the traffic rules behind `P` (by default the
/// baseline's, [`BaselinePolicy`]).
#[derive(Debug)]
pub struct Controller<P: TrafficPolicy = BaselinePolicy> {
    graph: Graph,
    pending: VecDeque<Job>,
    robots: Vec<Slot>,
    completed: Vec<Completed>,
    rejected: Vec<Rejected>,
    /// The tick of the previous control cycle.
    last_tick: u64,
    /// Who may go now.
    policy: P,
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
    /// Metres of route cost added for every robot that holds an edge, or the node
    /// it ends at, when a route is planned; 0 (default) is the plain shortest route.
    load_weight: f64,
    /// Metres of route cost added per robot that has recently waited on the node an
    /// edge ends at (see `node_wait`); 0 (default) is off.
    wait_weight: f64,
    /// For each node, how many robots stood waiting on it, as a moving average over
    /// [`WAIT_AVERAGE_SECONDS`] (1.0 = one robot waiting all the time).
    node_wait: Vec<f64>,
    /// Sending robots to chargers; `None` (default) is off.
    charge_policy: Option<ChargePolicy>,
    /// The nodes of the stations of type `charger`.
    charger_nodes: Vec<NodeIdx>,
    /// Charger node → the robot that is on its way there.
    charger_targets: std::collections::HashMap<NodeIdx, usize>,
    /// Trips to a charger started so far (also the id of the stand-in jobs).
    charge_trips: usize,
    /// Simulated seconds that free robots below the low limit have waited for a
    /// charger (all robots together).
    charge_wait_seconds: f64,
    /// The parking places: the nodes of the stations of type `parking` (U13).
    parking_nodes: Vec<NodeIdx>,
    /// Parking place → the robot that is on its way there.
    parking_targets: std::collections::HashMap<NodeIdx, usize>,
    /// Trips to a parking place started so far (also the id of the stand-in jobs).
    park_trips: usize,
    /// How often a robot without a job got no parking place.
    no_free_parking: usize,
}

impl Controller {
    /// A controller with the baseline's traffic rules for all robots now in `world`.
    pub fn new(world: &World) -> Controller {
        Controller::with_policy(world, BaselinePolicy::new(world.graph()))
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
    /// A controller for all robots now in `world` with the traffic rules of `policy`.
    pub fn with_policy(world: &World, mut policy: P) -> Controller<P> {
        // the robot types (manufacturer, series) in the order they first appear
        let graph = world.graph();
        let mut keys: Vec<(String, String)> = Vec::new();
        let mut kinds: Vec<Kinematics> = Vec::new();
        // the body outline of each robot, for the policy
        let mut shapes: Vec<Shape> = Vec::new();
        let robots: Vec<Slot> = world
            .robot_ids()
            .filter_map(|id| {
                // the robot's type comes from its factsheet message (S61); a robot
                // that has none cannot be read and gets no jobs
                let f = world.factsheet(id).ok()?;
                let spec = world.spec(id).ok()?;
                let (kinematics, shape) = (spec.kinematics, spec.shape);
                let series = f.type_specification.series_name.clone();
                let key = (f.manufacturer, f.type_specification.series_name);
                let type_idx = keys.iter().position(|k| *k == key).unwrap_or_else(|| {
                    keys.push(key);
                    kinds.push(kinematics);
                    keys.len() - 1
                });
                shapes.push(shape);
                Some(Slot {
                    id: id.clone(),
                    type_idx,
                    work: None,
                    series,
                    header_id: 0,
                    response_header_id: 0,
                    zone_set_sent: false,
                    enable_sent: false,
                    zone_set_header_id: 0,
                    instant_header_id: 0,
                    charging: false,
                    unparked: false,
                    park_refused: false,
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
                    omnidirectional: kinematics == Kinematics::Omnidirectional,
                    into: in_edges(graph, &ok),
                    turns: Turns::for_type(graph, kinematics, m, s),
                    cost: travel_lengths(graph, m, s),
                    ok,
                }
            })
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
        policy.start(&Setup {
            graph: world.graph(),
            types: &type_names,
            robots: &policy_robots,
            zones: world.zones(),
        });
        Controller {
            graph: world.graph().clone(),
            pending: VecDeque::new(),
            robots,
            completed: Vec::new(),
            rejected: Vec::new(),
            last_tick: world.tick(),
            policy,
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
            load_weight: 0.0,
            wait_weight: 0.0,
            node_wait: vec![0.0; world.graph().nodes.len()],
            charge_policy: None,
            charger_nodes: Vec::new(),
            charger_targets: std::collections::HashMap::new(),
            charge_trips: 0,
            charge_wait_seconds: 0.0,
            parking_nodes: station_nodes(world.graph(), "parking"),
            parking_targets: std::collections::HashMap::new(),
            park_trips: 0,
            no_free_parking: 0,
        }
    }

    /// The parking places of the layout (U13): the interaction nodes of the stations
    /// of type `parking`. With none, idle robots stay where they are.
    pub fn parking_places(&self) -> usize {
        self.parking_nodes.len()
    }

    /// Trips to a parking place started so far.
    pub fn park_trips(&self) -> usize {
        self.park_trips
    }

    /// How often a robot without a job found no free parking place (or none it can
    /// reach) and stayed where it was; counted once until it gets a route again.
    pub fn no_free_parking(&self) -> usize {
        self.no_free_parking
    }

    /// Send robots to chargers (charging brief, step 2): a free robot whose reported
    /// charge is below `policy.low` drives to the nearest charger that is free (no
    /// robot on it, none on its way there), and the controller sends `startCharging`
    /// when it has arrived. Chargers are the stations of type `charger` (any case) of
    /// the layout. Returns the number of charger nodes (0: nothing will happen).
    pub fn set_charging(&mut self, policy: ChargePolicy) -> usize {
        self.charger_nodes = station_nodes(&self.graph, "charger");
        self.charge_policy = Some(policy);
        self.charger_nodes.len()
    }

    /// Trips to a charger started so far.
    pub fn charge_trips(&self) -> usize {
        self.charge_trips
    }

    /// Simulated seconds free robots below the low limit have waited for a charger
    /// (all robots together; robots that are busy with a job are not counted).
    pub fn charge_wait_seconds(&self) -> f64 {
        self.charge_wait_seconds
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

    /// The route cost of every edge for a robot of type `t` that is not `robot`:
    /// the length plus the load penalty. `None` if the load weight is 0 (use the
    /// plain lengths).
    fn loaded_cost(&self, t: usize, robot: usize) -> Option<Vec<f64>> {
        loaded_cost(
            &self.types[t].cost,
            &self.graph,
            &self.policy,
            (self.load_weight, self.wait_weight, &self.node_wait),
            robot,
        )
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

        // the waiting average of the nodes (only needed for the route cost)
        if self.wait_weight > 0.0 {
            let keep = (-elapsed / WAIT_AVERAGE_SECONDS).exp();
            for w in &mut self.node_wait {
                *w *= keep;
            }
            for (i, state) in states.iter().enumerate() {
                if self.robots[i].work.is_some()
                    && is_waiting(state)
                    && let Some(n) = self.node_of(state)
                {
                    self.node_wait[n] += 1.0 - keep;
                }
            }
        }

        // the policy sees everyone's progress before the releases
        self.policy.cycle_start(&Fleet {
            tick,
            elapsed,
            states: &states,
            routes: &route_views(&self.robots),
        });
        for (i, state) in states.iter().enumerate() {
            self.update_work(world, i, state, tick, elapsed);
        }
        // a charging robot that has reached the upper limit stops charging and is
        // free for jobs again
        if let Some(policy) = self.charge_policy {
            for (i, state) in states.iter().enumerate() {
                let slot = &mut self.robots[i];
                if slot.charging && state.power_supply.state_of_charge >= policy.high {
                    slot.charging = false;
                    let id = format!("stop-charge-{}", slot.instant_header_id);
                    send_instant(world, slot, "stopCharging", &id);
                }
            }
        }
        self.policy.after_releases(&Fleet {
            tick,
            elapsed,
            states: &states,
            routes: &route_views(&self.robots),
        });
        self.update_lifts(world, &states);
        // the policy hears of every robot without a job and where it stands
        for (i, state) in states.iter().enumerate() {
            if self.robots[i].work.is_none()
                && let Some(n) = self.node_of(state)
            {
                self.policy.idle(i, n);
            }
        }
        self.update_zone_sets(world, &states);
        self.answer_zone_requests(world, &states, tick, elapsed);
        self.dispatch(world, &states, tick, elapsed);
        // why each robot is held back, and by whom (who waits for whom: deadlocks)
        world.publish_held_back(self.held_back(&states));

        // tell the observer what is reserved (only if someone watches)
        if world.observing() {
            world.publish_reservations(self.policy.reservations());
        }
    }

    /// For each robot that stands still on its order: why it is held back, and by
    /// whom (see [`botsim_core::waiting`]).
    /// - Road released ahead of it and a zone request not granted: the zone, with
    ///   the robots the policy names.
    /// - At the end of its released route, with more to go: the lift or door another
    ///   robot owns, else the release, with the policy's holder.
    fn held_back(&self, states: &[State]) -> Vec<Option<HeldBack>> {
        self.robots
            .iter()
            .zip(states)
            .enumerate()
            .map(|(i, (slot, state))| {
                let work = slot.work.as_ref()?;
                if work.cancelling.is_some()
                    || state.order_id != work.order_id
                    || !is_waiting(state)
                {
                    return None;
                }
                let at = (state.last_node_sequence_id / 2) as usize;
                let rt = work.released_through;
                let (reason, blockers) = if at < rt {
                    let asks = state.zone_requests.iter().flatten().any(|q| {
                        matches!(
                            q.request_status,
                            RequestStatus::Requested | RequestStatus::Queued
                        )
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
                    &mut self.policy,
                    &mut self.lifts,
                    &mut self.doors,
                    i,
                    &work.plan,
                    work.released_through,
                );
                match work.purpose {
                    Purpose::Job => self.rejected.push(Rejected {
                        job: work.job,
                        robot: slot.id.clone(),
                        reason,
                    }),
                    Purpose::Charge => self.charger_targets.retain(|_, r| *r != i),
                    Purpose::Park => {
                        self.parking_targets.retain(|_, r| *r != i);
                        slot.park_refused = true;
                    }
                }
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
                    &mut self.policy,
                    &mut self.lifts,
                    &mut self.doors,
                    i,
                    &work.plan,
                    work.released_through,
                );
                match work.purpose {
                    Purpose::Job => self.rejected.push(Rejected {
                        job: work.job,
                        robot: slot.id.clone(),
                        reason,
                    }),
                    Purpose::Charge => self.charger_targets.retain(|_, r| *r != i),
                    Purpose::Park => {
                        self.parking_targets.retain(|_, r| *r != i);
                        slot.park_refused = true;
                    }
                }
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
            self.policy.route_finished(i, &work.plan);
            match work.purpose {
                Purpose::Job => {}
                Purpose::Charge => {
                    // arrived at the charger: start charging; no job was done
                    self.charger_targets.retain(|_, r| *r != i);
                    slot.charging = true;
                    send_instant(
                        world,
                        slot,
                        "startCharging",
                        &format!("charge-{}", work.order_id),
                    );
                    return;
                }
                Purpose::Park => {
                    // the robot stands on its place now
                    self.parking_targets.retain(|_, r| *r != i);
                    return;
                }
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
        if is_waiting(state) {
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
                robot: &slot.id,
                header_id: slot.header_id,
                timestamp: &world.timestamp(),
                order_id: &work.order_id,
                order_update_id: work.update_id,
            };
            let orient = self.types[slot.type_idx]
                .omnidirectional
                .then_some((slot.id.manufacturer.as_str(), slot.series.as_str()));
            let msg = build_order_with(&self.graph, &header, &work.plan, from, rt, orient);
            slot.header_id += 1;
            match deliver(world, msg) {
                Ok(_) => {
                    work.released_through = rt;
                    if changed {
                        self.reroutes += 1;
                    }
                }
                Err(reason) => {
                    // the robot cannot drive the next part (e.g. a car-like robot
                    // that cannot take the turn) or the changed route: give the job
                    // up (S70)
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

    /// Send the policy's answers to the RELEASE zone requests the robots state.
    fn answer_zone_requests(
        &mut self,
        world: &mut World,
        states: &[State],
        tick: u64,
        elapsed: f64,
    ) {
        let answers = self.policy.zone_answers(&Fleet {
            tick,
            elapsed,
            states,
            routes: &route_views(&self.robots),
        });
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
    fn dispatch(&mut self, world: &mut World, states: &[State], tick: u64, elapsed: f64) {
        // free robots with the node they stand on
        let mut free: Vec<(usize, NodeIdx)> = (0..self.robots.len())
            .filter(|&i| self.robots[i].work.is_none())
            .filter(|&i| !self.robots[i].charging)
            .filter(|&i| self.zones_ready(&states[i]))
            .filter_map(|i| Some((i, self.node_of(&states[i])?)))
            .collect();

        // a free robot with a low battery goes to a charger instead of getting a job
        // (the pair of a low robot and a free charger with the shortest route first, so
        // that the nearest robot gets a charger, not the first in the list)
        if let Some(policy) = self.charge_policy {
            let mut skipped: Vec<usize> = Vec::new();
            let mut dist: std::collections::HashMap<(usize, NodeIdx), Vec<f64>> =
                std::collections::HashMap::new();
            loop {
                let mut best: Option<(f64, usize, NodeIdx)> = None; // (length, index in free, charger)
                for (k, &(i, node)) in free.iter().enumerate() {
                    if skipped.contains(&i) || states[i].power_supply.state_of_charge >= policy.low
                    {
                        continue;
                    }
                    if self.charger_nodes.contains(&node) {
                        best = Some((-1.0, k, node)); // already on a charger
                        break;
                    }
                    let t = self.robots[i].type_idx;
                    for &c in &self.charger_nodes {
                        // a charger is free if no robot holds it and none is on its way
                        if self.policy.node_holder(c).is_some()
                            || self.charger_targets.contains_key(&c)
                        {
                            continue;
                        }
                        let d = dist.entry((t, c)).or_insert_with(|| {
                            distances_to(&self.graph, &self.types[t].into, &self.types[t].cost, c)
                        })[node];
                        if d.is_finite() && best.is_none_or(|(b, _, _)| d < b) {
                            best = Some((d, k, c));
                        }
                    }
                }
                let Some((_, k, charger)) = best else {
                    break;
                };
                let (i, node) = free[k];
                if self.start_charge(world, states, i, node, charger, tick) {
                    free.remove(k);
                } else {
                    skipped.push(i);
                }
            }
            // robots still free and below the low limit wait for a charger; below the
            // critical limit they take no more jobs
            for &(i, _) in &free {
                if states[i].power_supply.state_of_charge < policy.low {
                    self.charge_wait_seconds += elapsed;
                }
            }
            free.retain(|&(i, _)| states[i].power_supply.state_of_charge >= policy.critical);
        }

        // jobs go to the free robots and to those on their way to a parking place
        let mut candidates: Vec<Candidate> = free
            .iter()
            .map(|&(robot, from)| Candidate {
                robot,
                from,
                ahead: 0.0,
                parking: false,
            })
            .collect();
        candidates
            .extend((0..self.robots.len()).filter_map(|i| self.parking_candidate(i, &states[i])));

        let mut waiting = VecDeque::new();
        while let Some(job) = self.pending.pop_front() {
            if candidates.is_empty() {
                waiting.push_back(job); // nobody to give it to: no search needed
                continue;
            }
            // The nearest candidate that can do the job. One search backwards
            // from the pickup gives the route length from every node (S52), so
            // a job costs one search per robot type, not one per free robot. A
            // type may drive only some of the edges (S55), so its lengths differ.
            let mut dist: Vec<Option<Vec<f64>>> = vec![None; self.types.len()];
            let mut order: Vec<(i64, usize, usize)> = Vec::new();
            for (k, c) in candidates.iter().enumerate() {
                let i = c.robot;
                let t = self.robots[i].type_idx;
                // an order update does not run the actions of its first node
                if c.parking && c.from == job.pickup {
                    continue;
                }
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
                let d = dist_t[c.from] + c.ahead;
                if d.is_finite() {
                    // smallest distance (to a nanometre); equal → the robot added first
                    order.push(((d * 1e9).round() as i64, i, k));
                }
            }
            order.sort_unstable();
            // the first robot in that order that has a plan for the job
            let chosen = order.iter().find_map(|&(_, i, k)| {
                let c = candidates[k];
                let t = self.robots[i].type_idx;
                let loaded = self.loaded_cost(t, i);
                let mover = match &loaded {
                    Some(c) => self.types[t].mover_with(c),
                    None => self.types[t].mover(),
                };
                // the trip to a parking place that the job continues, if any
                let trip = self.robots[i].work.as_ref().filter(|_| c.parking);
                // the robot's heading matters if it cannot turn on the spot (S73):
                // on a trip, the way it will arrive at the last released node
                let arrival = match trip {
                    Some(w) if w.released_through > 0 => {
                        Arrival::Edge(w.plan.edges[w.released_through - 1])
                    }
                    _ => states[i]
                        .mobile_robot_position
                        .as_ref()
                        .map_or(Arrival::Free, |p| Arrival::Heading(p.theta)),
                };
                let plan = Plan::for_job_with(&self.graph, c.from, arrival, &job, |a, arr, b| {
                    self.lifts.route_from(&mover, arr, a, b)
                })?;
                let plan = match trip {
                    Some(w) => w.plan.continued(w.released_through, plan),
                    None => plan,
                };
                Some((k, plan))
            });
            let Some((k, plan)) = chosen else {
                waiting.push_back(job);
                continue;
            };
            let c = candidates.remove(k);
            if c.parking {
                self.extend_trip(world, c.robot, job, plan);
            } else {
                self.assign(world, &states[c.robot], c.robot, job, plan, Purpose::Job);
            }
        }
        self.pending = waiting;
        // the free robots that got nothing go to a parking place (U13)
        let idle: Vec<(usize, NodeIdx)> = candidates
            .iter()
            .filter(|c| !c.parking)
            .map(|c| (c.robot, c.from))
            .collect();
        self.park(world, states, &idle, tick);
    }

    /// Robot `i` as a candidate for a job while it is on its way to a parking place
    /// (S144); `None` if it is not, or cannot take one now: its order is not yet
    /// answered or is being cancelled, it owns a lift or a door (those claims depend
    /// on the route), or its battery is below the low limit (it goes to a charger
    /// once it is free).
    fn parking_candidate(&self, i: usize, state: &State) -> Option<Candidate> {
        let slot = &self.robots[i];
        let work = slot.work.as_ref()?;
        let low = self.charge_policy.map_or(0.0, |p| p.low);
        if work.purpose != Purpose::Park
            || work.cancelling.is_some()
            || work.awaiting_ack
            || state.order_id != work.order_id
            || self.lifts.owns(i)
            || self.doors.owns(i)
            || !self.zones_ready(state)
            || state.power_supply.state_of_charge < low
        {
            return None;
        }
        let at = (state.last_node_sequence_id / 2) as usize;
        let released = work.plan.edges.get(at..work.released_through);
        let cost = &self.types[slot.type_idx].cost;
        Some(Candidate {
            robot: i,
            from: work.plan.nodes[work.released_through],
            ahead: released.unwrap_or_default().iter().map(|&e| cost[e]).sum(),
            parking: true,
        })
    }

    /// Give `job` to robot `i` on its way to a parking place: an update of its order
    /// from the last released node, with `plan` (the released part as before, then
    /// the job). Nothing more is released here; that happens in the next cycle.
    fn extend_trip(&mut self, world: &mut World, i: usize, job: Job, plan: Plan) {
        let slot = &mut self.robots[i];
        let Some(work) = slot.work.as_mut() else {
            return;
        };
        self.parking_targets.retain(|_, r| *r != i);
        work.job = job;
        work.purpose = Purpose::Job;
        work.assigned_tick = world.tick();
        work.waiting_seconds = 0.0;
        work.update_id += 1;
        slot.park_refused = false;
        let header = OrderHeader {
            robot: &slot.id,
            header_id: slot.header_id,
            timestamp: &world.timestamp(),
            order_id: &work.order_id,
            order_update_id: work.update_id,
        };
        let orient = self.types[slot.type_idx]
            .omnidirectional
            .then_some((slot.id.manufacturer.as_str(), slot.series.as_str()));
        let from = work.released_through;
        let msg = build_order_with(&self.graph, &header, &plan, from, from, orient);
        slot.header_id += 1;
        match deliver(world, msg) {
            Ok(_) => {
                work.plan = plan;
                self.policy.route_changed(i, &work.plan);
            }
            // the robot refuses the route: cancel the order and give the job up, as
            // for any refused update (S70)
            Err(reason) => cancel_order(world, slot, reason),
        }
    }

    /// Send the robots of `idle` (without a job, with the node they stand on) that
    /// are not on a parking place to free ones, the pair with the shortest route
    /// first. A place is free if no robot stands on it or holds it and none is on
    /// its way there. A robot that gets none stays where it is and is counted.
    fn park(&mut self, world: &mut World, states: &[State], idle: &[(usize, NodeIdx)], tick: u64) {
        if self.parking_nodes.is_empty() {
            return;
        }
        let mut waiting: Vec<(usize, NodeIdx)> = idle
            .iter()
            .filter(|&&(i, node)| {
                !self.parking_nodes.contains(&node) && !self.robots[i].park_refused
            })
            .copied()
            .collect();
        if waiting.is_empty() {
            return;
        }
        let occupied: std::collections::HashSet<NodeIdx> =
            states.iter().filter_map(|s| self.node_of(s)).collect();
        let mut dist: std::collections::HashMap<(usize, NodeIdx), Vec<f64>> =
            std::collections::HashMap::new();
        // (robot, place) pairs for which no route was found
        let mut no_route: Vec<(usize, NodeIdx)> = Vec::new();
        loop {
            let mut best: Option<(f64, usize, NodeIdx)> = None; // (length, index, place)
            for (k, &(i, node)) in waiting.iter().enumerate() {
                let t = self.robots[i].type_idx;
                for &p in &self.parking_nodes {
                    if occupied.contains(&p)
                        || self.policy.node_holder(p).is_some()
                        || self.parking_targets.contains_key(&p)
                        || no_route.contains(&(i, p))
                    {
                        continue;
                    }
                    let d = dist.entry((t, p)).or_insert_with(|| {
                        distances_to(&self.graph, &self.types[t].into, &self.types[t].cost, p)
                    })[node];
                    if d.is_finite() && best.is_none_or(|(b, _, _)| d < b) {
                        best = Some((d, k, p));
                    }
                }
            }
            let Some((_, k, place)) = best else {
                break;
            };
            let (i, node) = waiting[k];
            let t = self.robots[i].type_idx;
            let loaded = self.loaded_cost(t, i);
            let mover = match &loaded {
                Some(c) => self.types[t].mover_with(c),
                None => self.types[t].mover(),
            };
            let arrival = states[i]
                .mobile_robot_position
                .as_ref()
                .map_or(Arrival::Free, |p| Arrival::Heading(p.theta));
            let Some(edges) = self.lifts.route_from(&mover, arrival, node, place) else {
                no_route.push((i, place));
                continue;
            };
            let plan = Plan::new(&self.graph, node, edges);
            let job = Job {
                id: self.park_trips as u64,
                pickup: node,
                drop: place,
                created_tick: tick,
            };
            self.park_trips += 1;
            self.parking_targets.insert(place, i);
            waiting.remove(k);
            self.assign(world, &states[i], i, job, plan, Purpose::Park);
        }
        for (i, _) in waiting {
            if !self.robots[i].unparked {
                self.robots[i].unparked = true;
                self.no_free_parking += 1;
            }
        }
    }

    /// Send the free robot `i`, standing on `node`, to `charger` and start charging
    /// there; `false` if it cannot be reached.
    fn start_charge(
        &mut self,
        world: &mut World,
        states: &[State],
        i: usize,
        node: NodeIdx,
        charger: NodeIdx,
        tick: u64,
    ) -> bool {
        // already on the charger: just start
        if node == charger {
            let slot = &mut self.robots[i];
            slot.charging = true;
            let id = format!("charge-here-{}", self.charge_trips);
            self.charge_trips += 1;
            send_instant(world, slot, "startCharging", &id);
            return true;
        }
        let t = self.robots[i].type_idx;
        let loaded = self.loaded_cost(t, i);
        let mover = match &loaded {
            Some(c) => self.types[t].mover_with(c),
            None => self.types[t].mover(),
        };
        let arrival = states[i]
            .mobile_robot_position
            .as_ref()
            .map_or(Arrival::Free, |p| Arrival::Heading(p.theta));
        let Some(edges) = self.lifts.route_from(&mover, arrival, node, charger) else {
            return false;
        };
        let plan = Plan::new(&self.graph, node, edges);
        let job = Job {
            id: self.charge_trips as u64,
            pickup: node,
            drop: charger,
            created_tick: tick,
        };
        self.charge_trips += 1;
        self.charger_targets.insert(charger, i);
        self.assign(world, &states[i], i, job, plan, Purpose::Charge);
        true
    }

    fn assign(
        &mut self,
        world: &mut World,
        state: &State,
        i: usize,
        job: Job,
        mut plan: Plan,
        purpose: Purpose,
    ) {
        let tick = world.tick();
        let order_id = format!("{}-{}", purpose.order_prefix(), job.id);
        let wanted = plan.released_through(0, self.policy.window());
        let wanted = self.lifts.window(i, &plan, 0, 0, wanted);
        let wanted = self.doors.window(i, &plan, 0, 0, wanted);
        let answer = self.policy.place(&Ask {
            robot: i,
            state,
            route: &plan,
            at: 0,
            released_through: 0,
            limit: wanted,
            elapsed: 0.0,
        });
        let released_through = answer.through.min(wanted);
        self.doors.settle(i, released_through);
        if let Some(wish) = answer.reroute {
            let t = self.robots[i].type_idx;
            let loaded = self.loaded_cost(t, i);
            let mover = match &loaded {
                Some(c) => self.types[t].mover_with(c),
                None => self.types[t].mover(),
            };
            if let Some(other) = reroute(
                &self.graph,
                &wish.avoid,
                &self.lifts,
                &self.doors,
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
            robot: &slot.id,
            header_id: slot.header_id,
            timestamp: &world.timestamp(),
            order_id: &order_id,
            order_update_id: 0,
        };
        let orient = self.types[slot.type_idx]
            .omnidirectional
            .then_some((slot.id.manufacturer.as_str(), slot.series.as_str()));
        let msg = build_order_with(&self.graph, &header, &plan, 0, released_through, orient);
        slot.header_id += 1;
        match deliver(world, msg) {
            Ok(reception) => {
                slot.unparked = false;
                if purpose == Purpose::Job {
                    slot.park_refused = false;
                }
                slot.work = Some(Work {
                    job,
                    plan,
                    order_id,
                    update_id: 0,
                    released_through,
                    assigned_tick: tick,
                    waiting_seconds: 0.0,
                    purpose,
                    awaiting_ack: reception == Reception::InTransit,
                    cancelling: None,
                });
            }
            Err(reason) => {
                give_back(
                    &mut self.policy,
                    &mut self.lifts,
                    &mut self.doors,
                    i,
                    &plan,
                    released_through,
                );
                match purpose {
                    Purpose::Job => self.rejected.push(Rejected {
                        job,
                        robot: slot.id.clone(),
                        reason,
                    }),
                    Purpose::Charge => self.charger_targets.retain(|_, r| *r != i),
                    Purpose::Park => {
                        self.parking_targets.retain(|_, r| *r != i);
                        slot.park_refused = true;
                    }
                }
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

/// The interaction nodes of the stations of the layout whose type is `kind` (any
/// case), sorted, each once.
fn station_nodes(graph: &Graph, kind: &str) -> Vec<NodeIdx> {
    let mut nodes: Vec<NodeIdx> = graph
        .stations
        .iter()
        .filter(|s| {
            s.station_type
                .as_deref()
                .is_some_and(|t| t.eq_ignore_ascii_case(kind))
        })
        .flat_map(|s| s.interaction_nodes.iter().copied())
        .collect();
    nodes.sort_unstable();
    nodes.dedup();
    nodes
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
    let action_id = format!("cancel-{}", work.order_id);
    send_instant(world, slot, "cancelOrder", &action_id);
}

/// Send one instant action (no parameters) to the robot of `slot`.
fn send_instant(world: &mut World, slot: &mut Slot, action_type: &str, action_id: &str) {
    let message = serde_json::from_value(serde_json::json!({
        "headerId": slot.instant_header_id,
        "timestamp": world.timestamp(),
        "version": botsim_core::world::PROTOCOL_VERSION,
        "manufacturer": slot.id.manufacturer,
        "serialNumber": slot.id.serial_number,
        "actions": [{
            "actionType": action_type,
            "actionId": action_id,
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

/// The route cost of every edge for `robot`: `cost` plus `reserved` for each other
/// robot that holds the edge and each that holds the node it ends at, plus `waited`
/// times the waiting average of that node. `weights` is (`reserved`, `waited`,
/// `node_wait`). `None` if both weights are 0.
fn loaded_cost<P: TrafficPolicy>(
    cost: &[f64],
    graph: &Graph,
    policy: &P,
    weights: (f64, f64, &[f64]),
    robot: usize,
) -> Option<Vec<f64>> {
    let (reserved, waited, node_wait) = weights;
    if reserved <= 0.0 && waited <= 0.0 {
        return None;
    }
    let other = |h: Option<usize>| h.is_some_and(|h| h != robot);
    Some(
        cost.iter()
            .enumerate()
            .map(|(e, &c)| {
                let end = graph.edges[e].end;
                let n = usize::from(other(policy.edge_holder(e)))
                    + usize::from(other(policy.node_holder(end)));
                c + reserved * n as f64 + waited * node_wait[end]
            })
            .collect(),
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
        && state.node_states.is_empty()
        && state.action_states.iter().all(|a| {
            matches!(
                a.action_status,
                ActionStatus::Finished | ActionStatus::Failed
            )
        })
}

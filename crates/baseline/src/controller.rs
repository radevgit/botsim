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

use botsim_core::collision::{overlap, place};
use botsim_core::errors::ErrorType;
use botsim_core::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_core::jobs::Job;
use botsim_core::metrics::{Metrics, is_waiting, summarize};
use botsim_core::robot::Reception;
use botsim_core::spec::{Kinematics, Shape};
use botsim_core::world::{RobotId, World};
use vda5050::order::Order;
use vda5050::state::{ActionStatus, State};

use crate::conflicts::{ConflictInput, ConflictTable, Held, NodeConflictInput};
use crate::doors::DoorDesk;
use crate::lifts::{LiftDesk, LiftRobot};
use crate::node_grid::NodeGrid;
use crate::order_builder::{OrderHeader, Plan, RELEASE_WINDOW, build_order_with};
use crate::reservations::Reservations;
use crate::route::{Arrival, Mover, Turns, distances_to, in_edges, plan_path, travel_lengths};
use crate::zone_grants::{RobotView, ZoneGrants};
use vda5050::responses::Responses;

/// A robot that has been held at the end of its released route this long by
/// other robots gets a new route around them (S53, my choice: well before the
/// 30 s a deadlock needs to be reported).
pub const REROUTE_AFTER_SECONDS: f64 = 10.0;

/// The time constant of the waiting average per node that the route cost uses
/// (route with load).
pub const WAIT_AVERAGE_SECONDS: f64 = 60.0;

/// A robot that has been held this long at the end of its released route by a
/// robot that is stuck behind it goes first (S77, my choice: before the re-route).
/// Extra distance (m) kept between the body swept by a turn on the spot and a stopped robot.
const TURN_CLEARANCE: f64 = 0.05;

pub const GIVE_WAY_AFTER_SECONDS: f64 = 5.0;

/// Go-first takes the node or edge of a robot H only if H has had its route released
/// and has not moved for this long (it is held by its protective field, not just
/// released: warehouse 4, 1102 s, two robots were released onto the same node in the
/// same second and drove into each other).
pub const GIVE_WAY_HOLD_SECONDS: f64 = 2.0;

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
    /// How long the robot has had a route released ahead of it and has not moved
    /// (stopped by its protective field, or just released). Go-first takes a node
    /// or edge from a robot only after [`GIVE_WAY_HOLD_SECONDS`] of it.
    still_seconds: f64,
    /// This is a trip to a charger, not a transport job: its `job` is a stand-in (the
    /// robot's node and the charger node), it has no actions, it is not counted as a
    /// completed or rejected job; at the end the controller starts the charging.
    charge: bool,
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
    /// The robot's body outline (to check that a turn on the spot is clear).
    shape: Shape,
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
    /// How many times a robot was let go first, ahead of the robot stuck behind it
    /// (S77).
    give_ways: usize,
    /// Metres of route cost added for every robot that holds an edge, or the node
    /// it ends at, when a route is planned; 0 (default) is the plain shortest route.
    load_weight: f64,
    /// Metres of route cost added per robot that has recently waited on the node an
    /// edge ends at (see `node_wait`); 0 (default) is off.
    wait_weight: f64,
    /// For each node, how many robots stood waiting on it, as a moving average over
    /// [`WAIT_AVERAGE_SECONDS`] (1.0 = one robot waiting all the time).
    node_wait: Vec<f64>,
    /// The robot types as `manufacturer/seriesName`, by `Slot::type_idx`.
    type_names: Vec<String>,
    /// The type of each robot (`Slot::type_idx`), by robot index.
    robot_types: Vec<usize>,
    /// Edges whose swept outlines overlap; `None` reserves nodes and edges only.
    conflicts: Option<ConflictTable>,
    /// Robot → the edge it may be released onto although it conflicts with an edge
    /// another robot holds (go-first, S77); cleared when it has been released.
    waived: std::collections::HashMap<usize, EdgeIdx>,
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
                let spec = world.spec(id).ok()?;
                let (kinematics, shape) = (spec.kinematics, spec.shape);
                let series = f.type_specification.series_name.clone();
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
                    shape,
                    series,
                    header_id: 0,
                    response_header_id: 0,
                    zone_set_sent: false,
                    enable_sent: false,
                    zone_set_header_id: 0,
                    instant_header_id: 0,
                    charging: false,
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
        let robot_types: Vec<usize> = robots.iter().map(|s| s.type_idx).collect();
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
            give_ways: 0,
            load_weight: 0.0,
            wait_weight: 0.0,
            node_wait: vec![0.0; world.graph().nodes.len()],
            type_names: keys.iter().map(|(m, s)| format!("{m}/{s}")).collect(),
            robot_types,
            conflicts: None,
            waived: std::collections::HashMap::new(),
            charge_policy: None,
            charger_nodes: Vec::new(),
            charger_targets: std::collections::HashMap::new(),
            charge_trips: 0,
            charge_wait_seconds: 0.0,
        }
    }

    /// Send robots to chargers (charging brief, step 2): a free robot whose reported
    /// charge is below `policy.low` drives to the nearest charger that is free (no
    /// robot on it, none on its way there), and the controller sends `startCharging`
    /// when it has arrived. Chargers are the stations of type `charger` (any case) of
    /// the layout. Returns the number of charger nodes (0: nothing will happen).
    pub fn set_charging(&mut self, policy: ChargePolicy) -> usize {
        let mut nodes: Vec<NodeIdx> = self
            .graph
            .stations
            .iter()
            .filter(|s| {
                s.station_type
                    .as_deref()
                    .is_some_and(|t| t.eq_ignore_ascii_case("charger"))
            })
            .flat_map(|s| s.interaction_nodes.iter().copied())
            .collect();
        nodes.sort_unstable();
        nodes.dedup();
        self.charger_nodes = nodes;
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

    /// Swept-area reservation (see [`crate::conflicts`]): from now on a robot is
    /// released onto an edge only if no other robot holds an edge that conflicts with
    /// it. Returns the number of conflicting edge pairs read.
    pub fn set_conflicts(&mut self, input: &[ConflictInput]) -> usize {
        let table = ConflictTable::new(&self.graph, &self.type_names, input);
        let n = table.pairs();
        self.conflicts = Some(table);
        n
    }

    /// Node-edge conflicts for the swept-area reservation: a robot is not released
    /// onto an edge that passes too close to a node another robot holds, nor to a node
    /// where it would stand next to an edge another robot holds. Call after
    /// [`Controller::set_conflicts`]; returns the number read (0 if that was not
    /// called).
    pub fn set_node_conflicts(&mut self, input: &[NodeConflictInput]) -> usize {
        let Some(table) = self.conflicts.as_mut() else {
            return 0;
        };
        table.add_node_conflicts(&self.graph, &self.type_names, input)
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
            &self.reservations,
            (self.load_weight, self.wait_weight, &self.node_wait),
            robot,
        )
    }

    /// How many times a robot's route was changed because another robot held the
    /// way for longer than [`REROUTE_AFTER_SECONDS`].
    pub fn reroutes(&self) -> usize {
        self.reroutes
    }

    /// How many times a robot went first, ahead of the robot stuck behind it
    /// (S77, [`GIVE_WAY_AFTER_SECONDS`]).
    pub fn give_ways(&self) -> usize {
        self.give_ways
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
        self.go_first(&states);
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
        self.dispatch(world, &states, tick, elapsed);

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
                if work.charge {
                    self.charger_targets.retain(|_, r| *r != i);
                } else {
                    self.rejected.push(Rejected {
                        job: work.job,
                        robot: slot.id.clone(),
                        reason,
                    });
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
                    &mut self.reservations,
                    &mut self.lifts,
                    &mut self.doors,
                    i,
                    &work.plan,
                    work.released_through,
                );
                if work.charge {
                    self.charger_targets.retain(|_, r| *r != i);
                } else {
                    self.rejected.push(Rejected {
                        job: work.job,
                        robot: slot.id.clone(),
                        reason,
                    });
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
            if work.charge {
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
        // A node taken over from a robot in the way (S77) is held before it is
        // released: it is the next node after the released ones. It is not given
        // back while the route has it there, even if the route passed it earlier;
        // but it is once the robot has driven on, also if the route has it again
        // much later (a lap of a ring: it then held the node for the whole lap,
        // warehouse 4, Y-007 held two nodes 158 m away).
        let last = work.plan.last();
        let next_nodes = &work.plan.nodes[at..=(work.released_through + 1).min(last)];
        let next_edges = &work.plan.edges[at..(work.released_through + 1).min(last)];
        for j in 0..at {
            let (n, e) = (work.plan.nodes[j], work.plan.edges[j]);
            if !ahead_nodes.contains(&n)
                && !(self.reservations.is_taken_over_node(n, i) && next_nodes.contains(&n))
            {
                self.reservations.release_node(n, i);
            }
            if !ahead_edges.contains(&e)
                && !(self.reservations.is_taken_over_edge(e, i) && next_edges.contains(&e))
            {
                self.reservations.release_edge(e, i);
            }
        }

        // release as far as the window allows and nobody else holds the way
        let from = work.released_through;
        let wanted = work.plan.released_through(at, RELEASE_WINDOW);
        // a ride in the window needs its lift (S47)
        let wanted = self.lifts.window(i, &work.plan, at, from, wanted);
        // so does an edge through a door (S51)
        let wanted = self.doors.window(i, &work.plan, at, from, wanted);
        let swept = self
            .conflicts
            .as_ref()
            .map(|c| (c, self.robot_types.as_slice()));
        let waived = self.waived.get(&i).copied();
        let rt = reserve_up_to(
            &mut self.reservations,
            &work.plan,
            i,
            from,
            wanted,
            swept,
            waived,
        );
        if rt > from {
            self.waived.remove(&i);
        }
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
        let not_moving = state
            .velocity
            .as_ref()
            .is_none_or(|v| v.vx.unwrap_or(0.0).abs() < 0.05);
        work.still_seconds = if rt > at && not_moving {
            work.still_seconds + elapsed
        } else {
            0.0
        };
        if stuck && work.blocked_seconds >= REROUTE_AFTER_SECONDS {
            work.blocked_seconds = 0.0; // try again after another timeout
            let loaded = loaded_cost(
                &self.types[slot.type_idx].cost,
                &self.graph,
                &self.reservations,
                (self.load_weight, self.wait_weight, &self.node_wait),
                i,
            );
            let mover = match &loaded {
                Some(c) => self.types[slot.type_idx].mover_with(c),
                None => self.types[slot.type_idx].mover(),
            };
            if let Some(plan) = reroute(
                &self.graph,
                &self.reservations,
                &self.lifts,
                &self.doors,
                &mover,
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
                let orient = self.types[slot.type_idx]
                    .omnidirectional
                    .then_some((slot.id.manufacturer.as_str(), slot.series.as_str()));
                let msg = build_order_with(&self.graph, &header, &work.plan, from, from, orient);
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
            let orient = self.types[slot.type_idx]
                .omnidirectional
                .then_some((slot.id.manufacturer.as_str(), slot.series.as_str()));
            let msg = build_order_with(&self.graph, &header, &work.plan, from, rt, orient);
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
                        if self.reservations.node_holder(c).is_some()
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
                let t = self.robots[i].type_idx;
                let loaded = self.loaded_cost(t, i);
                let mover = match &loaded {
                    Some(c) => self.types[t].mover_with(c),
                    None => self.types[t].mover(),
                };
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
            self.assign(world, i, job, plan, tick, false);
        }
        self.pending = waiting;
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
        self.assign(world, i, job, plan, tick, true);
        true
    }

    fn assign(
        &mut self,
        world: &mut World,
        i: usize,
        job: Job,
        plan: Plan,
        tick: u64,
        charge: bool,
    ) {
        let slot = &mut self.robots[i];
        let order_id = if charge {
            format!("charge-{}", job.id)
        } else {
            format!("job-{}", job.id)
        };
        // node 0 is where the robot stands (it holds it already)
        self.reservations.reserve_node(plan.nodes[0], i);
        let wanted = self
            .lifts
            .window(i, &plan, 0, 0, plan.released_through(0, RELEASE_WINDOW));
        let wanted = self.doors.window(i, &plan, 0, 0, wanted);
        let swept = self
            .conflicts
            .as_ref()
            .map(|c| (c, self.robot_types.as_slice()));
        let released_through =
            reserve_up_to(&mut self.reservations, &plan, i, 0, wanted, swept, None);
        self.doors.settle(i, released_through);
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
                slot.work = Some(Work {
                    job,
                    plan,
                    order_id,
                    update_id: 0,
                    released_through,
                    assigned_tick: tick,
                    waiting_seconds: 0.0,
                    blocked_seconds: 0.0,
                    still_seconds: 0.0,
                    charge,
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
                if charge {
                    self.charger_targets.retain(|_, r| *r != i);
                } else {
                    self.rejected.push(Rejected {
                        job,
                        robot: slot.id.clone(),
                        reason,
                    });
                }
            }
        }
    }

    /// Deadlocks with a safety link (S77): robot X stands at the end of its released
    /// route and waits for the next node, which robot H has released; H is stopped
    /// (its protective field sees X) and X is right in front of it, heading the same
    /// way. H cannot move until X does, so X goes first: it takes the node over (H
    /// gets it back when X lets go of it) and its release goes on as usual. Not
    /// done head-on (X heading against H): there both would run into each other.
    fn go_first(&mut self, states: &[State]) {
        for i in 0..self.robots.len() {
            let Some(work) = self.robots[i].work.as_ref() else {
                continue;
            };
            if work.blocked_seconds < GIVE_WAY_AFTER_SECONDS || work.cancelling.is_some() {
                continue;
            }
            let from = work.released_through;
            let (Some(&node), Some(&edge)) =
                (work.plan.nodes.get(from + 1), work.plan.edges.get(from))
            else {
                continue;
            };
            // X waits for the node, or for an edge whose swept area overlaps the one
            // of its next edge (swept-area reservation): the holder is H
            let all_node_holder = self.reservations.node_holder(node).filter(|&h| h != i);
            let all_swept = self
                .conflicts
                .as_ref()
                .and_then(|c| c.blocker(&self.reservations, &self.robot_types, i, edge));
            // or for a node (or an edge) that the next edge passes too close to
            // (node-edge conflict)
            let all_nblock = self
                .conflicts
                .as_ref()
                .and_then(|c| c.node_blocker(&self.reservations, &self.robot_types, i, edge));
            // X may be kept by several robots (the node holder and the holder of a
            // conflicting edge are often different ones: a robot passing through the
            // node, and a stopped one on a curve beside it). Each is tried in turn as
            // H; the first one that lets X go first is taken. A holder that is not
            // taken is simply waited for.
            for pass in 0..3 {
                let (node_holder, swept, nblock) = match pass {
                    // the node holder; if it also holds a conflicting edge, X is
                    // released onto its edge although it does
                    0 => (
                        all_node_holder,
                        all_swept.filter(|&(sh, _)| Some(sh) == all_node_holder),
                        None,
                    ),
                    1 => (None, all_swept, None),
                    _ => (None, None, all_nblock),
                };
                let Some(h) = node_holder
                    .or(swept.map(|(h, _)| h))
                    .or(nblock.map(|(h, _)| h))
                else {
                    continue;
                };
                // H must be on its way through the node (it has released it as part of
                // its route and does not stand on it); a robot that is parked there
                // holds it because it stands on it, and cannot be gone around. For a
                // conflicting edge: H has released it as part of its route.
                let on_its_way = match (node_holder, swept) {
                    (None, None) => nblock.is_some_and(|(_, held)| {
                        self.robots[h].work.as_ref().is_some_and(|w| match held {
                            Held::Node(n) => w
                                .plan
                                .nodes
                                .iter()
                                .take(w.released_through + 1)
                                .any(|&m| m == n),
                            Held::Edge(f) => w
                                .plan
                                .edges
                                .iter()
                                .take(w.released_through)
                                .any(|&e| e == f),
                        })
                    }),
                    (Some(_), _) => {
                        self.robots[h].work.as_ref().is_some_and(|w| {
                            w.plan
                                .nodes
                                .iter()
                                .take(w.released_through + 1)
                                .any(|&n| n == node)
                        }) && self.node_of(&states[h]) != Some(node)
                    }
                    (None, Some((_, f))) => self.robots[h].work.as_ref().is_some_and(|w| {
                        w.plan
                            .edges
                            .iter()
                            .take(w.released_through)
                            .any(|&e| e == f)
                    }),
                };
                if !on_its_way {
                    continue;
                }
                // H has not moved for a while with its route released
                if !self.robots[h]
                    .work
                    .as_ref()
                    .is_some_and(|w| w.still_seconds >= GIVE_WAY_HOLD_SECONDS)
                {
                    continue;
                }
                // the edge must be free, or held by the same robot
                if !self
                    .reservations
                    .edge_holder(edge)
                    .is_none_or(|e| e == i || e == h)
                {
                    continue;
                }
                let (Some(px), Some(ph)) = (
                    states[i].mobile_robot_position.as_ref(),
                    states[h].mobile_robot_position.as_ref(),
                ) else {
                    continue;
                };
                let stopped = states[h]
                    .velocity
                    .as_ref()
                    .is_none_or(|v| v.vx.unwrap_or(0.0).abs() < 0.05);
                if !stopped || px.map_id != ph.map_id {
                    continue;
                }
                // X is ahead of H, within 4 m and 60° of H's heading, and heads the same
                // way (within 45°)
                let (dx, dy) = (px.x - ph.x, px.y - ph.y);
                let off_heading = angle_between(dy.atan2(dx), ph.theta);
                // or its route after the node is the one H takes after it (a lane change
                // or merge: X stands on H's curve and both go on to the same node); then
                // X may also be beside H's curve (up to 100° off its heading, 5 m away).
                // The same holds if X stands on a node that H has released (H's way
                // goes through X).
                // (for a conflicting edge: or both routes meet at the end of H's edge and
                // go on to the same node, N908/T-009, warehouse 4)
                let same_route = self.same_route_after(i, h, node)
                    || swept.is_some_and(|(_, f)| self.meet_and_go_on(i, h, f));
                // (not by who holds the node: X may have taken it over earlier)
                let on_h_way = self.node_of(&states[i]).is_some_and(|n| {
                    self.robots[h].work.as_ref().is_some_and(|w| {
                        w.plan
                            .nodes
                            .iter()
                            .take(w.released_through + 1)
                            .any(|&m| m == n)
                    })
                });
                let wide = same_route || on_h_way;
                let near = dx.hypot(dy) <= if wide { 5.0 } else { 4.0 };
                // the way X will head once it is released: along its next edge (a robot
                // standing on a node has not turned to the edge yet; warehouse 4, O-008
                // stood facing west, 117° from T-008, while its edge ran 45°)
                let start = self
                    .graph
                    .edge_directions(
                        edge,
                        &self.robots[i].id.manufacturer,
                        &self.robots[i].series,
                    )
                    .0;
                // X standing on H's way may head any way as long as it does not face H
                let faces_away = angle_between(start, (-dy).atan2(-dx)) >= 90f64.to_radians();
                let same_way = angle_between(start, ph.theta) <= 45f64.to_radians()
                || same_route
                || (on_h_way && faces_away)
                // a wait for a conflicting edge or a close node (the map): X's edge
                // leads away from H, which is stopped, so X may drive off first
                || ((swept.is_some() || nblock.is_some()) && faces_away);
                let limit = if wide { 100.0 } else { 60.0 };
                let ahead = off_heading <= f64::to_radians(limit);
                // X may have to turn on the spot to drive off; its body then sweeps
                // through the angles to the start of the edge and must not touch H (a
                // forklift turning 1.7 m from a stopped robot hit it, warehouse 4, 309 s)
                // (an omnidirectional robot does not turn: it drives off the way it is
                // facing)
                let turns = !self.types[self.robots[i].type_idx].omnidirectional
                    && angle_between(px.theta, start) > 15f64.to_radians();
                let clear = !turns
                    || turn_is_clear(
                        &self.robots[i].shape,
                        (px.x, px.y, px.theta),
                        start,
                        &self.robots[h].shape,
                        (ph.x, ph.y, ph.theta),
                    );
                if !(near && ahead && same_way && clear) {
                    continue;
                }
                if node_holder.is_some() {
                    if self.reservations.take_over_node(node, i).is_none() {
                        continue;
                    }
                    self.reservations.take_over_edge(edge, i);
                }
                // the edge conflicts with one H holds: X may be released onto its edge
                // although H still holds the other (until X has been released)
                if swept.is_some_and(|(sh, _)| sh == h) || nblock.is_some_and(|(nh, _)| nh == h) {
                    self.waived.insert(i, edge);
                }
                self.give_ways += 1;
                break;
            }
        }
    }

    /// Do robots `a` and `b` both go on from `node` to the same next node? `a`
    /// has `node` next in its plan; `b` has it somewhere ahead in its own.
    fn same_route_after(&self, a: usize, b: usize, node: NodeIdx) -> bool {
        let (Some(wa), Some(wb)) = (self.robots[a].work.as_ref(), self.robots[b].work.as_ref())
        else {
            return false;
        };
        let after_a = wa.plan.nodes.get(wa.released_through + 2);
        let after_b = wb
            .plan
            .nodes
            .iter()
            .skip(wb.released_through.saturating_sub(1))
            .position(|&n| n == node)
            .and_then(|k| {
                wb.plan
                    .nodes
                    .get(wb.released_through.saturating_sub(1) + k + 1)
            });
        after_a.is_some() && after_a == after_b
    }

    /// Does the route of `a`, within its next three nodes, reach the end node of edge
    /// `f` of `b`, and do both go on from there to the same node?
    fn meet_and_go_on(&self, a: usize, b: usize, f: EdgeIdx) -> bool {
        let (Some(wa), Some(wb)) = (self.robots[a].work.as_ref(), self.robots[b].work.as_ref())
        else {
            return false;
        };
        let m = self.graph.edges[f].end;
        let first = wa.released_through + 1;
        let Some(p) = wa
            .plan
            .nodes
            .iter()
            .enumerate()
            .skip(first)
            .take(3)
            .find_map(|(k, &n)| (n == m).then_some(k))
        else {
            return false;
        };
        let after_a = wa.plan.nodes.get(p + 1);
        let after_b = wb
            .plan
            .nodes
            .iter()
            .skip(wb.released_through.saturating_sub(1))
            .position(|&n| n == m)
            .and_then(|k| {
                wb.plan
                    .nodes
                    .get(wb.released_through.saturating_sub(1) + k + 1)
            });
        after_a.is_some() && after_a == after_b
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

/// Can a robot at `x` (x, y, heading) turn on the spot to the heading `to` without
/// touching the robot `h` (standing still)? The bodies are placed like the world
/// does; `h` is grown by `TURN_CLEARANCE`, and the turn is checked every 5°.
fn turn_is_clear(
    shape: &Shape,
    x: (f64, f64, f64),
    to: f64,
    h_shape: &Shape,
    h: (f64, f64, f64),
) -> bool {
    let grown = match *h_shape {
        Shape::Rect {
            cx,
            cy,
            half_length,
            half_width,
        } => Shape::Rect {
            cx,
            cy,
            half_length: half_length + TURN_CLEARANCE,
            half_width: half_width + TURN_CLEARANCE,
        },
        Shape::Circle { cx, cy, radius } => Shape::Circle {
            cx,
            cy,
            radius: radius + TURN_CLEARANCE,
        },
    };
    let body_h = place(&grown, h.0, h.1, h.2);
    // the shortest way round, from -π to π
    let turn =
        (to - x.2 + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
    let steps = ((turn.abs() / 5f64.to_radians()).ceil() as usize).max(1);
    (0..=steps).all(|k| {
        let heading = x.2 + turn * k as f64 / steps as f64;
        !overlap(&place(shape, x.0, x.1, heading), &body_h)
    })
}

/// The angle between two directions, 0..π.
fn angle_between(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(std::f64::consts::TAU);
    if d > std::f64::consts::PI {
        std::f64::consts::TAU - d
    } else {
        d
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
fn loaded_cost(
    cost: &[f64],
    graph: &Graph,
    reservations: &Reservations,
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
                let n = usize::from(other(reservations.edge_holder(e)))
                    + usize::from(other(reservations.node_holder(end)));
                c + reserved * n as f64 + waited * node_wait[end]
            })
            .collect(),
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
/// with the edge leading to it; stop at the first one somebody else holds, or whose
/// edge conflicts with an edge another robot holds (`swept`: the conflict table and
/// the type of each robot). Returns the index of the last node now released (`from`
/// if none).
fn reserve_up_to(
    reservations: &mut Reservations,
    plan: &Plan,
    robot: usize,
    from: usize,
    wanted: usize,
    swept: Option<(&ConflictTable, &[usize])>,
    waived: Option<EdgeIdx>,
) -> usize {
    let mut rt = from;
    while rt < wanted {
        let e = plan.edges[rt];
        if waived != Some(e)
            && swept.is_some_and(|(table, types)| table.blocks(reservations, types, robot, e))
        {
            break;
        }
        if !reservations.reserve_step(e, plan.nodes[rt + 1], robot) {
            break;
        }
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

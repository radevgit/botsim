//! Run a scenario to its end and describe the result as JSON.
//!
//! The summary has no wall-clock times or file paths, so the same scenario
//! gives byte-identical output on every run and machine.
//!
//! With a `[jobs]` section the scenario creates jobs and the bundled baseline
//! controller runs (see [`botsim_baseline::controller`]); without it the robots
//! stand still. Metrics: throughput, waiting (avg, max), collisions.
//!
//! Not yet: deadlocks, controller time per step.

use botsim_baseline::conflicts::{ConflictInput, NodeConflictInput, NodeNodeConflictInput};
use botsim_baseline::controller::Controller;
use botsim_baseline::policy::TrafficPolicy;
use botsim_core::lift::DoorState;
use botsim_core::world::World;
use botsim_dispatcher::ChargePolicy;
use botsim_model::job::Job;
use botsim_space::{MapOptions, TypeRef};
use serde::Serialize;

use crate::scenario::{Scenario, ScenarioError};
use crate::waiting_summary::{WaitingSummary, waiting_summary};

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Summary {
    pub scenario: String,
    pub seed: u64,
    pub duration: f64,
    pub time_step: f64,
    /// The number of steps run: the scenario's duration, or fewer if the run was
    /// stopped at the first deadlock.
    pub ticks: u64,
    /// The run ended early because the scenario said `stop_at_first_deadlock` and
    /// a deadlock was reported (S59).
    pub stopped_at_first_deadlock: bool,
    /// The collision metric; it must be 0 for a good controller.
    pub collision_count: usize,
    pub collisions: Vec<CollisionSummary>,
    pub deadlock_count: usize,
    pub deadlocks: Vec<DeadlockSummary>,
    /// Random stops that started (0 without disturbances).
    pub random_stops: usize,
    /// Times a robot was stopped by its protective field (stops of one robot less
    /// than 1 s apart count once; see `World::safety_stops`).
    pub safety_stops: usize,
    /// The loaded zone set; left out without `zone_set` in the scenario.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub zone_set: Option<ZoneSetSummary>,
    /// The lifts as they stand at the end of the run; left out without lifts.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub lifts: Vec<LiftSummary>,
    /// The doors as they stand at the end of the run; left out without doors.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub doors: Vec<DoorSummary>,
    /// The batteries; left out when no robot has a battery model.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub battery: Option<BatterySummary>,
    /// Parking of idle robots; left out if the layout has no parking places (or no
    /// controller runs).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parking: Option<ParkingSummary>,
    pub jobs: JobsSummary,
    pub metrics: MetricsSummary,
    /// Waiting caused by other robots (U12); left out if no controller runs.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub waiting_for_others: Option<WaitingSummary>,
    pub robots: Vec<RobotSummary>,
    /// Wall-clock time of the controller; only with `--timing` (S39), because it
    /// differs from run to run and so breaks identical output.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timing: Option<TimingSummary>,
}

/// The batteries of the run (charging brief).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BatterySummary {
    /// Robots with a battery model.
    pub robots: usize,
    /// Robots whose battery was empty at some time (they stopped where they were).
    pub ran_empty: usize,
    /// Robots standing empty at the end.
    pub empty_at_end: usize,
    /// Simulated seconds all robots together spent charging.
    pub charging_seconds: f64,
    /// Trips the controller sent robots on to a charger (0 without chargers).
    pub charge_trips: usize,
    /// Simulated seconds free robots below the low limit waited for a charger (all
    /// robots together).
    pub waiting_for_charger_seconds: f64,
}

/// Idle robots and parking places (U13).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ParkingSummary {
    /// Parking places of the layout (interaction nodes of `parking` stations).
    pub places: usize,
    /// Trips to a parking place the controller started.
    pub trips: usize,
    /// Times a robot without a job got no place and stayed where it was (counted
    /// once until it gets a route again).
    pub no_free_place: usize,
}

/// How long the controller took per step (real time, not simulated time).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct TimingSummary {
    /// How many times the controller ran (0 without a `[jobs]` section).
    pub controller_steps: u64,
    pub controller_total_ms: f64,
    /// `null` if the controller did not run.
    pub controller_mean_us: Option<f64>,
    pub controller_max_us: Option<f64>,
}

/// A door at the end of the run.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DoorSummary {
    pub id: String,
    /// `closed`, `opening`, `open` or `closing`.
    pub state: String,
}

/// The name of a door state, for the summary.
fn door_name(s: DoorState) -> &'static str {
    match s {
        DoorState::Closed => "closed",
        DoorState::Opening => "opening",
        DoorState::Open => "open",
        DoorState::Closing => "closing",
    }
}

/// A lift at the end of the run.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct LiftSummary {
    pub id: String,
    /// The map the cabin is at (or left, if it is moving).
    pub floor_map_id: String,
    /// `closed`, `opening`, `open` or `closing`.
    pub door: String,
    /// The door that is open or was used last (`front` for a lift with one door).
    pub door_id: String,
    /// Calls still waiting.
    pub queued_calls: usize,
}

/// The zones that were loaded (they are not used by any behaviour yet).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ZoneSetSummary {
    pub zone_set_id: String,
    pub map_id: String,
    pub zones: Vec<ZoneEntry>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ZoneEntry {
    pub zone_id: String,
    /// `RELEASE`, `BLOCKED`, `SPEED_LIMIT`, …
    pub zone_type: String,
}

/// A cycle of robots waiting for each other for longer than the timeout.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DeadlockSummary {
    /// `manufacturer/serialNumber` of the robots in the cycle.
    pub robots: Vec<String>,
    /// Simulated seconds since the start when the cycle formed.
    pub first_seen_time: f64,
    /// When it had lasted for the timeout and was reported.
    pub detected_time: f64,
    /// `null` if it still lasted at the end of the run.
    pub ended_time: Option<f64>,
    /// From forming to its end, or to the end of the run.
    pub duration_seconds: f64,
}

/// What happened to the jobs (all 0 without a `[jobs]` section).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct JobsSummary {
    pub created: usize,
    pub done: usize,
    /// Created but no robot has taken them.
    pub waiting_for_robot: usize,
    /// A robot is working on them at the end of the run.
    pub in_progress: usize,
    /// Refused by a robot.
    pub rejected: usize,
    /// How many times the baseline controller gave a robot a new route because
    /// another robot had held the way for too long (S53).
    pub rerouted: usize,
}

/// The metrics of the run. Waiting is the time robots stood still with a job
/// and not doing an action; `null` if no job was finished.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MetricsSummary {
    pub throughput_per_hour: f64,
    /// Mean time of a finished job, from the moment a robot got it to its end
    /// (the time it waited for a robot is not in it); `null` if none was finished.
    pub job_time_avg_seconds: Option<f64>,
    pub waiting_avg_seconds: Option<f64>,
    pub waiting_max_seconds: Option<f64>,
}

/// Two robots started to overlap.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CollisionSummary {
    pub tick: u64,
    /// Simulated seconds since the start.
    pub time: f64,
    /// `manufacturer/serialNumber` of the two robots.
    pub robots: [String; 2],
}
/// Where a robot ended.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RobotSummary {
    pub manufacturer: String,
    /// The robot's type (factsheet `seriesName`); with `manufacturer` it is the
    /// type the layout lists on the nodes and edges it may use (S57).
    pub series_name: String,
    pub serial_number: String,
    /// Jobs this robot finished (0 without a `[jobs]` section).
    pub jobs_done: usize,
    pub map_id: String,
    pub x: f64,
    pub y: f64,
    pub heading: f64,
    /// The robot's top speed in this run (m/s), after any disturbance.
    pub max_speed: f64,
    /// Empty if the robot never had an order (VDA 5050 `lastNodeId` rule).
    pub last_node_id: String,
}

/// Adds up the real time of the controller steps.
#[derive(Default)]
struct StepClock {
    steps: u64,
    total: std::time::Duration,
    max: std::time::Duration,
}

impl StepClock {
    /// Run `f`; if `measure`, count its real time.
    fn time(&mut self, measure: bool, f: impl FnOnce()) {
        if !measure {
            return f();
        }
        let start = std::time::Instant::now();
        f();
        let took = start.elapsed();
        self.steps += 1;
        self.total += took;
        self.max = self.max.max(took);
    }

    fn report(&self) -> TimingSummary {
        let us = |d: std::time::Duration| d.as_secs_f64() * 1e6;
        TimingSummary {
            controller_steps: self.steps,
            controller_total_ms: round_micro(self.total.as_secs_f64() * 1e3),
            controller_mean_us: (self.steps > 0)
                .then(|| round_micro(us(self.total) / self.steps as f64)),
            controller_max_us: (self.steps > 0).then(|| round_micro(us(self.max))),
        }
    }
}

/// Round to a microsecond, so that sums like 0.05 × 1531 print as 76.55 and not
/// 76.55000000000001.
fn round_micro(x: f64) -> f64 {
    (x * 1e6).round() / 1e6
}

/// `ticks` steps of `dt` seconds, rounded for printing.
fn seconds(ticks: u64, dt: f64) -> f64 {
    round_micro(ticks as f64 * dt)
}

/// Number of steps that cover `duration` seconds (rounded to the nearest step).
pub fn ticks_for(duration: f64, time_step: f64) -> u64 {
    (duration / time_step).round() as u64
}

/// Step `world` for the scenario's duration and describe the robots.
pub fn run(scenario: &Scenario, world: World) -> Result<Summary, ScenarioError> {
    run_with(scenario, world, false)
}

/// Like [`run`]; with `measure_time` the real time of every controller step is
/// measured and reported in `timing` (S39).
///
/// Unless the scenario says `conflict_reservation = false`, a scenario read from a
/// file also gets swept-area reservation: the conflict map of `botsim-space` (made
/// from the layout and the factsheets) goes to the controller.
pub fn run_with(
    scenario: &Scenario,
    world: World,
    measure_time: bool,
) -> Result<Summary, ScenarioError> {
    run_controller(scenario, world, measure_time, botsim_bridge::controller)
}

/// Like [`run`], with the traffic rules of `policy` instead of the baseline's: the
/// controller dispatches as usual and asks `policy` who may go now (see
/// [`botsim_baseline::policy`]). The policy gets the same conflict lists the baseline
/// gets, computed by the runner (unless the scenario says
/// `conflict_reservation = false`). Without a `[jobs]` section no controller runs and
/// the policy is not used. To record the run, attach the observer to `world` first,
/// as for [`run`].
pub fn run_with_policy(
    scenario: &Scenario,
    world: World,
    policy: Box<dyn TrafficPolicy>,
) -> Result<Summary, ScenarioError> {
    run_controller(scenario, world, false, |w| {
        botsim_bridge::with_policy(w, policy)
    })
}

/// The run itself; `make` builds the controller for the world.
fn run_controller<P: TrafficPolicy>(
    scenario: &Scenario,
    mut world: World,
    measure_time: bool,
    make: impl FnOnce(&World) -> Controller<P>,
) -> Result<Summary, ScenarioError> {
    let ticks = ticks_for(scenario.duration, scenario.time_step);
    let mut generator = scenario.job_generator(world.graph())?;
    // (a controller only with jobs)
    let mut controller = generator.is_some().then(|| {
        let mut c = make(&world);
        c.set_load_weight(scenario.route_load_weight);
        c.set_wait_weight(scenario.route_wait_weight);
        // robots with a low battery go to a charger (only with chargers in the world)
        if scenario.charger_power_w.is_some() {
            c.set_charging(ChargePolicy {
                low: scenario.charge_low,
                high: 80.0_f64.max(scenario.charge_low),
                critical: 10.0,
            });
        }
        if scenario.conflict_reservation
            && let Some(path) = &scenario.source
        {
            let (edges, nodes, node_nodes) = conflict_input(scenario, path);
            c.policy_mut().conflicts(&edges);
            c.policy_mut().node_conflicts(&nodes);
            c.policy_mut().node_node_conflicts(&node_nodes);
        }
        c
    });
    let mut created = 0;
    let mut clock = StepClock::default();
    let mut stopped_at_first_deadlock = false;

    while world.tick() < ticks {
        // one cycle: new jobs → controller → the world moves
        if let (Some(g), Some(c)) = (generator.as_mut(), controller.as_mut()) {
            let new: Vec<Job> = g.step(world.tick()).into_iter().collect();
            created += new.len();
            clock.time(measure_time, || botsim_bridge::step(c, &mut world, new));
        }
        world.step();
        if scenario.stop_at_first_deadlock && !world.deadlocks().is_empty() {
            stopped_at_first_deadlock = true;
            break;
        }
    }
    if let Some(c) = controller.as_mut() {
        // see jobs that finished in the last step
        clock.time(measure_time, || botsim_bridge::step(c, &mut world, []));
    }
    let (jobs, metrics) = match &controller {
        Some(c) => {
            let m = c.metrics(world.time());
            let waiting_for_robot = c.pending().count();
            (
                JobsSummary {
                    created,
                    done: c.completed().len(),
                    waiting_for_robot,
                    in_progress: c.busy(),
                    rejected: c.rejected().len(),
                    rerouted: c.reroutes(),
                },
                MetricsSummary {
                    throughput_per_hour: m.throughput_per_hour,
                    job_time_avg_seconds: (!c.completed().is_empty()).then(|| {
                        let ticks: u64 = c
                            .completed()
                            .iter()
                            .map(|d| d.finished_tick.saturating_sub(d.assigned_tick))
                            .sum();
                        round_micro(ticks as f64 * world.dt() / c.completed().len() as f64)
                    }),
                    waiting_avg_seconds: m.waiting_avg_seconds.map(round_micro),
                    waiting_max_seconds: m.waiting_max_seconds.map(round_micro),
                },
            )
        }
        None => (
            JobsSummary {
                created: 0,
                done: 0,
                waiting_for_robot: 0,
                in_progress: 0,
                rejected: 0,
                rerouted: 0,
            },
            MetricsSummary {
                throughput_per_hour: 0.0,
                job_time_avg_seconds: None,
                waiting_avg_seconds: None,
                waiting_max_seconds: None,
            },
        ),
    };
    let ids: Vec<_> = world.robot_ids().cloned().collect();
    let robots = ids
        .iter()
        // (the ids come from the world, so every lookup succeeds; a robot that
        // somehow cannot be read is left out of the list)
        .filter_map(|id| {
            let p = world.pose(id).ok()?;
            let state = world.state(id).ok()?;
            let spec = world.spec(id).ok()?;
            Some(RobotSummary {
                manufacturer: id.manufacturer.clone(),
                series_name: spec.series_name.clone(),
                jobs_done: controller.as_ref().map_or(0, |c| {
                    c.completed().iter().filter(|d| d.robot == *id).count()
                }),
                serial_number: id.serial_number.clone(),
                map_id: world.graph().maps.get(p.map).cloned().unwrap_or_default(),
                x: p.x,
                y: p.y,
                heading: p.heading,
                max_speed: round_micro(spec.limits.max_speed),
                last_node_id: state.last_node_id,
            })
        })
        .collect();
    let name =
        |id: &botsim_model::robot::RobotId| format!("{}/{}", id.manufacturer, id.serial_number);
    let collisions: Vec<_> = world
        .collisions()
        .iter()
        .map(|c| CollisionSummary {
            tick: c.tick,
            time: seconds(c.tick, world.dt()),
            robots: [name(&c.a), name(&c.b)],
        })
        .collect();
    let dt = world.dt();
    let deadlocks: Vec<_> = world
        .deadlocks()
        .iter()
        .map(|d| DeadlockSummary {
            robots: d.robots.iter().map(name).collect(),
            first_seen_time: seconds(d.first_seen_tick, dt),
            detected_time: seconds(d.detected_tick, dt),
            ended_time: d.ended_tick.map(|t| seconds(t, dt)),
            duration_seconds: seconds(d.ended_tick.unwrap_or(world.tick()) - d.first_seen_tick, dt),
        })
        .collect();
    Ok(Summary {
        timing: measure_time.then(|| clock.report()),
        scenario: scenario.name.clone(),
        deadlock_count: deadlocks.len(),
        deadlocks,
        parking: controller
            .as_ref()
            .filter(|c| c.parking_places() > 0)
            .map(|c| ParkingSummary {
                places: c.parking_places(),
                trips: c.park_trips(),
                no_free_place: c.no_free_parking(),
            }),
        random_stops: world.random_stops(),
        safety_stops: world.safety_stops(),
        battery: world.battery_report().map(|b| BatterySummary {
            robots: b.robots,
            ran_empty: b.ran_empty,
            empty_at_end: b.empty_now,
            charging_seconds: round_micro(b.charging_seconds),
            charge_trips: controller.as_ref().map_or(0, |c| c.charge_trips()),
            waiting_for_charger_seconds: round_micro(
                controller.as_ref().map_or(0.0, |c| c.charge_wait_seconds()),
            ),
        }),
        lifts: world
            .lifts()
            .iter()
            .map(|l| LiftSummary {
                id: l.id.clone(),
                floor_map_id: world
                    .lift_floor_map(&l.id, l.floor)
                    .unwrap_or_default()
                    .to_string(),
                door: door_name(l.door).to_string(),
                door_id: l.door_id.clone(),
                queued_calls: l.queue.len(),
            })
            .collect(),
        doors: world
            .doors()
            .iter()
            .map(|d| DoorSummary {
                id: d.id.clone(),
                state: door_name(d.state).to_string(),
            })
            .collect(),
        zone_set: world.zones().map(|z| ZoneSetSummary {
            zone_set_id: z.zone_set_id.clone(),
            map_id: world.graph().maps[z.map].clone(),
            zones: z
                .zones
                .iter()
                .map(|zone| ZoneEntry {
                    zone_id: zone.zone_id.clone(),
                    zone_type: serde_json::to_value(&zone.kind)
                        .ok()
                        .and_then(|v| v["zoneType"].as_str().map(str::to_string))
                        .unwrap_or_default(),
                })
                .collect(),
        }),
        collision_count: collisions.len(),
        collisions,
        jobs,
        metrics,
        waiting_for_others: controller.as_ref().map(|_| waiting_summary(&world)),
        seed: scenario.seed,
        duration: scenario.duration,
        time_step: scenario.time_step,
        ticks: world.tick(),
        stopped_at_first_deadlock,
        robots,
    })
}

/// What the baseline reads of the conflict map: edge pairs, node-edge pairs and
/// node-node pairs.
type ConflictLists = (
    Vec<ConflictInput>,
    Vec<NodeConflictInput>,
    Vec<NodeNodeConflictInput>,
);

/// The conflict map of the layout for the fleet (`botsim-space`): edge pairs whose
/// swept outlines overlap or come nearer than the scenario's `conflict_clearance`
/// and that have no node in common (the node reservation covers the others), nodes
/// against edges, and all pairs of nodes too close for a robot on each. Empty if the
/// scenario's files cannot be read again or the layout is not a graph.
fn conflict_input(scenario: &Scenario, scenario_file: &std::path::Path) -> ConflictLists {
    let none = (Vec::new(), Vec::new(), Vec::new());
    let base = scenario_file.parent().unwrap_or(std::path::Path::new("."));
    let read = |path: &std::path::Path| std::fs::read_to_string(path).ok();
    let Some(lif) = read(&base.join(&scenario.lif))
        .and_then(|t| serde_json::from_str::<vdma_lif::Lif>(&t).ok())
    else {
        return none;
    };
    let mut types: Vec<TypeRef> = Vec::new();
    for g in &scenario.robots {
        let Some(factsheet) = read(&base.join(&g.factsheet))
            .and_then(|t| serde_json::from_str::<vda5050::factsheet::Factsheet>(&t).ok())
        else {
            return none;
        };
        let Ok(t) = TypeRef::from_factsheet(&factsheet) else {
            return none;
        };
        if !types.iter().any(|x| x.name == t.name) {
            types.push(t);
        }
    }
    let options = MapOptions {
        clearance: scenario.conflict_clearance,
        ..MapOptions::default()
    };
    let Some(map) = botsim_space::compute(&lif, &types, &options) else {
        return none;
    };
    let edges = map
        .pairs
        .into_iter()
        .map(|p| ConflictInput {
            type_a: p.type_a,
            type_b: p.type_b,
            edges: p
                .conflicts
                .into_iter()
                .filter(|c| !c.shares_node && c.junction_nodes.is_some())
                .map(|c| (c.edge_a, c.edge_b))
                .collect(),
        })
        .collect();
    // a robot standing on a node against an edge that does not touch it
    let nodes = map
        .node_pairs
        .into_iter()
        .map(|p| NodeConflictInput {
            type_node: p.type_node,
            type_edge: p.type_edge,
            pairs: p
                .conflicts
                .into_iter()
                .filter(|c| c.junction_nodes.is_some())
                .map(|c| (c.node, c.edge))
                .collect(),
        })
        .collect();
    // two robots on two nodes
    let node_nodes = map
        .node_node_pairs
        .into_iter()
        .map(|p| NodeNodeConflictInput {
            type_a: p.type_a,
            type_b: p.type_b,
            pairs: p
                .conflicts
                .into_iter()
                .map(|c| (c.node_a, c.node_b))
                .collect(),
        })
        .collect();
    (edges, nodes, node_nodes)
}

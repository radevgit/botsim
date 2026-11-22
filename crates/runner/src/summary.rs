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

use botsim_baseline::controller::Controller;
use botsim_core::jobs::Job;
use botsim_core::lift::DoorState;
use botsim_core::world::World;
use serde::Serialize;

use crate::scenario::{Scenario, ScenarioError};

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
    /// The loaded zone set; left out without `zone_set` in the scenario.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub zone_set: Option<ZoneSetSummary>,
    /// The lifts as they stand at the end of the run; left out without lifts.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub lifts: Vec<LiftSummary>,
    /// The doors as they stand at the end of the run; left out without doors.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub doors: Vec<DoorSummary>,
    pub jobs: JobsSummary,
    pub metrics: MetricsSummary,
    pub robots: Vec<RobotSummary>,
    /// Wall-clock time of the controller; only with `--timing` (S39), because it
    /// differs from run to run and so breaks identical output.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timing: Option<TimingSummary>,
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
pub fn run_with(
    scenario: &Scenario,
    mut world: World,
    measure_time: bool,
) -> Result<Summary, ScenarioError> {
    let ticks = ticks_for(scenario.duration, scenario.time_step);
    let mut generator = scenario.job_generator(world.graph())?;
    let mut controller = generator.as_ref().map(|_| Controller::new(&world));
    let mut created = 0;
    let mut clock = StepClock::default();
    let mut stopped_at_first_deadlock = false;

    while world.tick() < ticks {
        // one cycle: new jobs → controller → the world moves
        if let (Some(g), Some(c)) = (generator.as_mut(), controller.as_mut()) {
            let new: Vec<Job> = g.step(world.tick()).into_iter().collect();
            created += new.len();
            clock.time(measure_time, || c.step(&mut world, new));
        }
        world.step();
        if scenario.stop_at_first_deadlock && !world.deadlocks().is_empty() {
            stopped_at_first_deadlock = true;
            break;
        }
    }
    if let Some(c) = controller.as_mut() {
        // see jobs that finished in the last step
        clock.time(measure_time, || c.step(&mut world, []));
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
        |id: &botsim_core::world::RobotId| format!("{}/{}", id.manufacturer, id.serial_number);
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
        random_stops: world.random_stops(),
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
        seed: scenario.seed,
        duration: scenario.duration,
        time_step: scenario.time_step,
        ticks: world.tick(),
        stopped_at_first_deadlock,
        robots,
    })
}

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
use botsim_core::world::World;
use serde::Serialize;

use crate::scenario::{Scenario, ScenarioError};

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Summary {
    pub scenario: String,
    pub seed: u64,
    pub duration: f64,
    pub time_step: f64,
    pub ticks: u64,
    /// The collision metric; it must be 0 for a good controller.
    pub collision_count: usize,
    pub collisions: Vec<CollisionSummary>,
    pub deadlock_count: usize,
    pub deadlocks: Vec<DeadlockSummary>,
    pub jobs: JobsSummary,
    pub metrics: MetricsSummary,
    pub robots: Vec<RobotSummary>,
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
    pub serial_number: String,
    pub map_id: String,
    pub x: f64,
    pub y: f64,
    pub heading: f64,
    /// Empty if the robot never had an order (VDA 5050 `lastNodeId` rule).
    pub last_node_id: String,
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
pub fn run(scenario: &Scenario, mut world: World) -> Result<Summary, ScenarioError> {
    let ticks = ticks_for(scenario.duration, scenario.time_step);
    let mut generator = scenario.job_generator(world.graph())?;
    let mut controller = generator.as_ref().map(|_| Controller::new(&world));
    let mut created = 0;

    while world.tick() < ticks {
        // one cycle: new jobs → controller → the world moves
        if let (Some(g), Some(c)) = (generator.as_mut(), controller.as_mut()) {
            let new: Vec<Job> = g.step(world.tick()).into_iter().collect();
            created += new.len();
            c.step(&mut world, new);
        }
        world.step();
    }
    if let Some(c) = controller.as_mut() {
        c.step(&mut world, []); // see jobs that finished in the last step
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
        .map(|id| {
            let p = world.pose(id).expect("the id comes from the world");
            let state = world.state(id).expect("the id comes from the world");
            RobotSummary {
                manufacturer: id.manufacturer.clone(),
                serial_number: id.serial_number.clone(),
                map_id: world.graph().maps[p.map].clone(),
                x: p.x,
                y: p.y,
                heading: p.heading,
                last_node_id: state.last_node_id,
            }
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
        scenario: scenario.name.clone(),
        deadlock_count: deadlocks.len(),
        deadlocks,
        collision_count: collisions.len(),
        collisions,
        jobs,
        metrics,
        seed: scenario.seed,
        duration: scenario.duration,
        time_step: scenario.time_step,
        ticks,
        robots,
    })
}

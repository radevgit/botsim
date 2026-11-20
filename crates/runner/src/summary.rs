//! Run a scenario to its end and describe the result as JSON.
//!
//! The summary has no wall-clock times or file paths, so the same scenario
//! gives byte-identical output on every run and machine.
//!
//! Not yet: metrics (throughput, wait times, …) and deadlocks. There is
//! no controller yet either, so robots only move if something sends orders.

use botsim_core::world::World;
use serde::Serialize;

use crate::scenario::Scenario;

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
    pub robots: Vec<RobotSummary>,
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

/// Number of steps that cover `duration` seconds (rounded to the nearest step).
pub fn ticks_for(duration: f64, time_step: f64) -> u64 {
    (duration / time_step).round() as u64
}

/// Step `world` for the scenario's duration and describe the robots.
pub fn run(scenario: &Scenario, mut world: World) -> Summary {
    let ticks = ticks_for(scenario.duration, scenario.time_step);
    while world.tick() < ticks {
        world.step();
    }
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
            time: c.tick as f64 * world.dt(),
            robots: [name(&c.a), name(&c.b)],
        })
        .collect();
    Summary {
        scenario: scenario.name.clone(),
        collision_count: collisions.len(),
        collisions,
        seed: scenario.seed,
        duration: scenario.duration,
        time_step: scenario.time_step,
        ticks,
        robots,
    }
}

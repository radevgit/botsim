//! Run metrics that do not depend on which controller is used.
//!
//! - **Throughput**: finished transport jobs per simulated hour.
//! - **Waiting** (user decision S32): the time a robot **stands still while it
//!   has a job** and is **not busy with an action** (`pick`, `drop`, …). That is
//!   the time traffic control costs. It is counted per job; the metric is the
//!   average and the maximum over the finished jobs. (A job still running at the
//!   end of the run is not included.)
//!
//! Collisions are counted by the world ([`crate::world::World::collisions`]).
//! Not yet: deadlocks, controller time per step.

use vda5050::state::{ActionStatus, State};

/// Is this robot waiting, judged from its state message? Standing still means
/// not driving and not turning on the spot (`driving` covers both); any running
/// action makes it busy. Whether the robot has a job is for the caller to know.
pub fn is_waiting(state: &State) -> bool {
    // a robot stopped by an obstacle or a disturbance (protective field
    // violation) is not waiting for the controller
    !state.driving
        && !state.safety_state.field_violation
        && !state
            .action_states
            .iter()
            .any(|a| a.action_status == ActionStatus::Running)
}

#[derive(Debug, Clone, PartialEq)]
pub struct Metrics {
    pub jobs_done: usize,
    /// Jobs per simulated hour.
    pub throughput_per_hour: f64,
    /// `None` if no job is finished.
    pub waiting_avg_seconds: Option<f64>,
    pub waiting_max_seconds: Option<f64>,
}

/// Metrics from the waiting time (seconds) of each finished job and the
/// simulated time of the run (`elapsed_seconds` > 0, else the throughput is 0).
pub fn summarize(waiting_seconds: &[f64], elapsed_seconds: f64) -> Metrics {
    let n = waiting_seconds.len();
    let throughput_per_hour = if elapsed_seconds > 0.0 {
        n as f64 / (elapsed_seconds / 3600.0)
    } else {
        0.0
    };
    Metrics {
        jobs_done: n,
        throughput_per_hour,
        waiting_avg_seconds: (n > 0).then(|| waiting_seconds.iter().sum::<f64>() / n as f64),
        waiting_max_seconds: waiting_seconds.iter().copied().reduce(f64::max),
    }
}

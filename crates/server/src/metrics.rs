//! The figures of the jobs the controller has done.
//!
//! - **Throughput**: finished transport jobs per simulated hour.
//! - **Waiting** (see [`botsim_model::waiting`]): counted per job; the metric is
//!   the average and the maximum over the finished jobs. (A job still running at
//!   the end of the run is not included.)
//!
//! Collisions are counted by the simulator. Not yet: deadlocks, controller time
//! per step.

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

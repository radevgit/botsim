//! Seeded transport jobs: from a pickup node to a drop node, created at a set
//! rate between a list of job nodes.
//!
//! Random numbers come from our own small generator (SplitMix64), not a crate,
//! so a seed gives the same jobs on every platform and in every future version.
//! Each purpose gets its own stream ([`Rng::stream`]); adding a new random
//! feature later never changes the jobs of an existing seed.
//!
//! Jobs are created like events of a Poisson process, approximated per step:
//! in each tick a job appears with probability `rate × dt`, at most one per tick.

use std::fmt;

use crate::graph::NodeIdx;

/// SplitMix64: tiny, fast, good enough for scenario randomness.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed)
    }

    /// An independent stream for one purpose (e.g. "jobs", "disturbances"):
    /// the same seed and name always give the same stream.
    pub fn stream(seed: u64, purpose: &str) -> Rng {
        // FNV-1a hash of the name, mixed into the seed
        let mut h: u64 = 0xcbf29ce484222325;
        for b in purpose.bytes() {
            h = (h ^ u64::from(b)).wrapping_mul(0x100000001b3);
        }
        let mut r = Rng(seed ^ h);
        r.next_u64(); // spread the mix
        r
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e3779b97f4a7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Uniform in 0..n (`n` > 0).
    pub fn below(&mut self, n: usize) -> usize {
        ((u128::from(self.next_u64()) * n as u128) >> 64) as usize
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Job {
    pub id: u64,
    pub pickup: NodeIdx,
    pub drop: NodeIdx,
    /// The tick the job appeared.
    pub created_tick: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum JobError {
    /// Pickup and drop must differ, so at least two different nodes are needed.
    TooFewNodes,
    /// The rate must be a finite number >= 0.
    BadRate(f64),
    /// `rate × dt` is above 1: at most one job per step can be created.
    RateTooHighForStep,
}

impl fmt::Display for JobError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            JobError::TooFewNodes => write!(f, "jobs need at least two different nodes"),
            JobError::BadRate(r) => write!(f, "job rate must be a finite number >= 0, got {r}"),
            JobError::RateTooHighForStep => {
                write!(
                    f,
                    "job rate is too high for the time step (more than 1 job per step)"
                )
            }
        }
    }
}

impl std::error::Error for JobError {}

#[derive(Debug, Clone)]
pub struct JobGenerator {
    rng: Rng,
    nodes: Vec<NodeIdx>,
    /// Probability of a new job in one step.
    p_step: f64,
    next_id: u64,
}

impl JobGenerator {
    /// `rate_per_hour` jobs per simulated hour on average, between `nodes`
    /// (duplicates are ignored), with time step `dt` seconds.
    pub fn new(
        seed: u64,
        nodes: &[NodeIdx],
        rate_per_hour: f64,
        dt: f64,
    ) -> Result<JobGenerator, JobError> {
        let mut unique: Vec<NodeIdx> = Vec::new();
        for &n in nodes {
            if !unique.contains(&n) {
                unique.push(n);
            }
        }
        if unique.len() < 2 {
            return Err(JobError::TooFewNodes);
        }
        if !(rate_per_hour.is_finite() && rate_per_hour >= 0.0) {
            return Err(JobError::BadRate(rate_per_hour));
        }
        let p_step = rate_per_hour / 3600.0 * dt;
        if p_step > 1.0 {
            return Err(JobError::RateTooHighForStep);
        }
        Ok(JobGenerator {
            rng: Rng::stream(seed, "jobs"),
            nodes: unique,
            p_step,
            next_id: 1,
        })
    }

    /// Call once per step: a new job, or none.
    pub fn step(&mut self, tick: u64) -> Option<Job> {
        if self.rng.next_f64() >= self.p_step {
            return None;
        }
        let n = self.nodes.len();
        let pickup = self.rng.below(n);
        let mut drop = self.rng.below(n - 1);
        if drop >= pickup {
            drop += 1; // skip the pickup: drop is uniform over the others
        }
        let job = Job {
            id: self.next_id,
            pickup: self.nodes[pickup],
            drop: self.nodes[drop],
            created_tick: tick,
        };
        self.next_id += 1;
        Some(job)
    }
}

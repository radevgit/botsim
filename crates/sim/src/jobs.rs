//! Seeded transport jobs: from a pickup node to a drop node, created at a set
//! rate between a list of job nodes, or from a list of pickup places to a list of
//! drop places ([`JobGenerator::between_groups`]).
//!
//! Random numbers come from our own small generator (SplitMix64), not a crate,
//! so a seed gives the same jobs on every platform and in every future version.
//! Each purpose gets its own stream ([`Rng::stream`]); adding a new random
//! feature later never changes the jobs of an existing seed.
//!
//! Jobs are created like events of a Poisson process, approximated per step:
//! in each tick a job appears with probability `rate × dt`, at most one per tick.

use std::fmt;

use botsim_model::graph::NodeIdx;
use botsim_model::job::Job;
use botsim_model::time::Millis;

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

#[derive(Debug, Clone, PartialEq)]
pub enum JobError {
    /// Pickup and drop must differ, so at least two different nodes are needed.
    TooFewNodes,
    /// The rate must be a finite number >= 0.
    BadRate(f64),
    /// `rate × dt` is above 1: at most one job per step can be created.
    RateTooHighForStep,
    /// A list of pickup or drop places is empty.
    EmptyGroup,
    /// A node is both a pickup and a drop place.
    SharedNode,
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
            JobError::EmptyGroup => {
                write!(f, "jobs need at least one pickup place and one drop place")
            }
            JobError::SharedNode => {
                write!(f, "a node cannot be both a pickup and a drop place")
            }
        }
    }
}

impl std::error::Error for JobError {}

#[derive(Debug, Clone)]
pub struct JobGenerator {
    rng: Rng,
    /// The places jobs start and end at; each is a list of nodes where it can be
    /// reached (one node for a plain node, the interaction nodes of a station).
    /// With `drops` set these are the pickup places only.
    places: Vec<Vec<NodeIdx>>,
    /// Jobs go from `places` to these, and with `true` also the other way round;
    /// `None`: between any two of `places`.
    drops: Option<(Vec<Vec<NodeIdx>>, bool)>,
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
        let places: Vec<Vec<NodeIdx>> = unique.into_iter().map(|n| vec![n]).collect();
        JobGenerator::between_places(seed, &places, rate_per_hour, dt)
    }

    /// Like [`JobGenerator::new`], but between **places**, each a list of nodes
    /// where it can be reached (S60: a station with its interaction nodes). A job
    /// goes from one place to another; the node of a place with several is drawn
    /// at random (seeded) for each job. A place with one node draws nothing, so
    /// a run between plain nodes is the same as with [`JobGenerator::new`]. At
    /// least two different places are needed (identical lists count once).
    pub fn between_places(
        seed: u64,
        places: &[Vec<NodeIdx>],
        rate_per_hour: f64,
        dt: f64,
    ) -> Result<JobGenerator, JobError> {
        let unique = unique_places(places);
        if unique.len() < 2 {
            return Err(JobError::TooFewNodes);
        }
        Ok(JobGenerator {
            rng: Rng::stream(seed, "jobs"),
            places: unique,
            drops: None,
            p_step: per_step(rate_per_hour, dt)?,
            next_id: 1,
        })
    }

    /// Jobs from one group of places to another: picked up at one of `pickups`,
    /// dropped at one of `drops`, each drawn evenly. With `both_directions` each job
    /// is, with equal chance, that way round or the reverse (picked up at one of
    /// `drops`, dropped at one of `pickups`). The two groups must not share a node.
    /// Per job the random numbers are drawn in this order: the direction (only with
    /// `both_directions`), the place of `pickups`, the place of `drops`, then the
    /// node of the pickup place and of the drop place if they have several.
    pub fn between_groups(
        seed: u64,
        pickups: &[Vec<NodeIdx>],
        drops: &[Vec<NodeIdx>],
        both_directions: bool,
        rate_per_hour: f64,
        dt: f64,
    ) -> Result<JobGenerator, JobError> {
        let (pickups, drops) = (unique_places(pickups), unique_places(drops));
        if pickups.is_empty() || drops.is_empty() {
            return Err(JobError::EmptyGroup);
        }
        if pickups
            .iter()
            .flatten()
            .any(|n| drops.iter().flatten().any(|m| m == n))
        {
            return Err(JobError::SharedNode);
        }
        Ok(JobGenerator {
            rng: Rng::stream(seed, "jobs"),
            places: pickups,
            drops: Some((drops, both_directions)),
            p_step: per_step(rate_per_hour, dt)?,
            next_id: 1,
        })
    }

    /// Call once per step, with the time of the step: a new job (which appeared
    /// at `now`), or none.
    pub fn step(&mut self, now: Millis) -> Option<Job> {
        if self.rng.next_f64() >= self.p_step {
            return None;
        }
        let (from, to) = match &self.drops {
            // between any two places
            None => {
                let n = self.places.len();
                let from = self.rng.below(n);
                let mut to = self.rng.below(n - 1);
                if to >= from {
                    to += 1; // skip the pickup: drop is uniform over the others
                }
                (&self.places[from], &self.places[to])
            }
            // from a pickup place to a drop place, or back
            Some((drops, both)) => {
                let reverse = *both && self.rng.below(2) == 1;
                let a = &self.places[self.rng.below(self.places.len())];
                let b = &drops[self.rng.below(drops.len())];
                if reverse { (b, a) } else { (a, b) }
            }
        };
        let node_of = |rng: &mut Rng, place: &[NodeIdx]| match place {
            [only] => *only,
            _ => place[rng.below(place.len())],
        };
        let pickup = node_of(&mut self.rng, from);
        let drop = node_of(&mut self.rng, to);
        let job = Job {
            id: self.next_id,
            pickup,
            drop,
            created: now,
        };
        self.next_id += 1;
        Some(job)
    }
}

/// The places without empty ones and without identical lists, in their order.
fn unique_places(places: &[Vec<NodeIdx>]) -> Vec<Vec<NodeIdx>> {
    let mut unique: Vec<Vec<NodeIdx>> = Vec::new();
    for p in places {
        if !p.is_empty() && !unique.contains(p) {
            unique.push(p.clone());
        }
    }
    unique
}

/// The probability of a new job in one step of `dt` seconds.
fn per_step(rate_per_hour: f64, dt: f64) -> Result<f64, JobError> {
    if !(rate_per_hour.is_finite() && rate_per_hour >= 0.0) {
        return Err(JobError::BadRate(rate_per_hour));
    }
    let p_step = rate_per_hour / 3600.0 * dt;
    if p_step > 1.0 {
        return Err(JobError::RateTooHighForStep);
    }
    Ok(p_step)
}

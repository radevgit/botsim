//! Seeded random disturbances (user decision S36: start with speed variation).
//!
//! **Speed variation**: each robot's top speed is lowered by a random share
//! between 0 and `speed_variation` (e.g. 0.05 = up to 5 %), fixed for the whole
//! run. Acceleration and turning speed are not changed. The shares come from
//! their own random stream ("disturbances"), so switching disturbances on or off
//! never changes the jobs of a seed. One number is drawn per robot, in the
//! order the robots are added, so a robot's share does not depend on how many
//! robots follow it.
//!
//! **Random short stops** (S37) and **varying action durations** (S38) are
//! described at [`StopSpec`] and [`crate::actions::DurationNoise`].
//!
//! Not yet: breakdowns.

use std::fmt;

use crate::jobs::Rng;
use crate::spec::RobotSpec;

#[derive(Debug, Clone, PartialEq)]
pub enum DisturbanceError {
    /// The speed variation must be a finite number from 0 up to (not including) 1.
    BadSpeedVariation(f64),
    /// `stops_per_hour` must be a finite number >= 0, and small enough for one
    /// stop per step at most.
    BadStopRate(f64),
    /// The stop duration range must be finite with 0 <= min <= max.
    BadStopDuration { min: f64, max: f64 },
    /// The action duration variation must be a finite number >= 0.
    BadDurationVariation(f64),
}

impl fmt::Display for DisturbanceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DisturbanceError::BadSpeedVariation(v) => write!(
                f,
                "speed_variation must be a number from 0 up to (not including) 1, got {v}"
            ),
            DisturbanceError::BadStopRate(r) => write!(
                f,
                "stops_per_hour must be a number >= 0 and at most one stop per time step, got {r}"
            ),
            DisturbanceError::BadDurationVariation(v) => write!(
                f,
                "action_variation must be a finite number >= 0 (0.2 = up to 20 % longer), got {v}"
            ),
            DisturbanceError::BadStopDuration { min, max } => write!(
                f,
                "stop_seconds_min and stop_seconds_max must satisfy 0 <= min <= max, got {min} and {max}"
            ),
        }
    }
}

/// Check the action duration variation (S38): finite and not negative.
pub fn check_duration_variation(variation: f64) -> Result<(), DisturbanceError> {
    if variation.is_finite() && variation >= 0.0 {
        Ok(())
    } else {
        Err(DisturbanceError::BadDurationVariation(variation))
    }
}

/// Random short stops (S37): a robot that is driving or turning is stopped for a
/// random time, as if something stood in its way.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StopSpec {
    /// Average number of stops per hour **of driving or turning** (a robot that
    /// stands still cannot be stopped).
    pub per_hour: f64,
    pub min_seconds: f64,
    pub max_seconds: f64,
}

impl StopSpec {
    /// Check the numbers for time step `dt`.
    pub fn check(&self, dt: f64) -> Result<(), DisturbanceError> {
        if !(self.per_hour.is_finite()
            && self.per_hour >= 0.0
            && self.per_hour / 3600.0 * dt <= 1.0)
        {
            return Err(DisturbanceError::BadStopRate(self.per_hour));
        }
        let ok = self.min_seconds.is_finite()
            && self.max_seconds.is_finite()
            && 0.0 <= self.min_seconds
            && self.min_seconds <= self.max_seconds;
        if !ok {
            return Err(DisturbanceError::BadStopDuration {
                min: self.min_seconds,
                max: self.max_seconds,
            });
        }
        Ok(())
    }
}

/// The stops of one robot. Every robot has its own random stream, so a robot's
/// stops do not depend on how many other robots there are.
#[derive(Debug, Clone)]
pub struct StopGenerator {
    rng: Rng,
    spec: StopSpec,
    p_step: f64,
}

impl StopGenerator {
    /// For robot number `robot` (its index in the world), time step `dt`.
    pub fn new(
        seed: u64,
        robot: usize,
        spec: StopSpec,
        dt: f64,
    ) -> Result<StopGenerator, DisturbanceError> {
        spec.check(dt)?;
        Ok(StopGenerator {
            rng: Rng::stream(seed, &format!("stops-{robot}")),
            spec,
            p_step: spec.per_hour / 3600.0 * dt,
        })
    }

    /// Call once per step while the robot is moving: the length of a new stop in
    /// seconds, or `None`.
    pub fn step(&mut self) -> Option<f64> {
        if self.rng.next_f64() >= self.p_step {
            return None;
        }
        let s = self.spec;
        Some(s.min_seconds + self.rng.next_f64() * (s.max_seconds - s.min_seconds))
    }
}

impl std::error::Error for DisturbanceError {}

#[derive(Debug, Clone)]
pub struct Disturbances {
    rng: Rng,
    speed_variation: f64,
}

impl Disturbances {
    pub fn new(seed: u64, speed_variation: f64) -> Result<Disturbances, DisturbanceError> {
        if !(speed_variation.is_finite() && (0.0..1.0).contains(&speed_variation)) {
            return Err(DisturbanceError::BadSpeedVariation(speed_variation));
        }
        Ok(Disturbances {
            rng: Rng::stream(seed, "disturbances"),
            speed_variation,
        })
    }

    /// The speed factor of the next robot: 1.0 minus a random share of at most
    /// `speed_variation`, so in `(1 - speed_variation, 1]`.
    pub fn next_speed_factor(&mut self) -> f64 {
        1.0 - self.rng.next_f64() * self.speed_variation
    }
}

impl RobotSpec {
    /// The same robot with its top speed multiplied by `factor` (0 < `factor` <= 1).
    pub fn with_speed_factor(&self, factor: f64) -> RobotSpec {
        assert!(factor > 0.0 && factor <= 1.0, "speed factor {factor}");
        let mut s = self.clone();
        s.limits.max_speed *= factor;
        s
    }
}

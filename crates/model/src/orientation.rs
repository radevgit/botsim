//! Which heading the robot must have while driving an edge (VDA 5050 edge
//! `orientation` and `orientationType`).
//!
//! Rules (VDA 5050 §6, edge attributes):
//! - TANGENTIAL (the default): relative to the direction of the edge; 0.0 = driving
//!   forwards, π = driving backwards. Differential and three-wheel robots can only do
//!   these two; omnidirectional robots can take any orientation.
//! - GLOBAL: relative to the map; only valid for omnidirectional robots.
//! - No orientation given: the standard lets the robot choose. Decision S13:
//!   always forwards (tangential, 0.0). Backwards must be asked for with π.
//!
//! The result is a target heading. Travel direction is always along the edge,
//! so a heading of "edge direction + π" means driving backwards.

use std::fmt;

use vda5050::order::OrientationType;

use crate::angle::{angle_diff, normalize_angle};
use crate::spec::Kinematics;

/// An orientation of exactly 0 or π is expected, but real layouts write
/// e.g. 3.14 for π; values within this many radians count as 0 or π.
pub const ORIENTATION_TOLERANCE: f64 = 0.01;

/// The robot cannot take the requested orientation. Maps to the VDA 5050 error
/// type `ORIENTATION_UNREACHABLE` (level WARNING); the order is rejected.
#[derive(Debug, Clone, PartialEq)]
pub enum OrientationError {
    /// This kinematics can only drive an edge forwards (0.0) or backwards (π).
    OnlyForwardsOrBackwards { orientation: f64 },
    /// GLOBAL orientation is only valid for omnidirectional robots.
    GlobalNeedsOmnidirectional,
    /// The orientation is not a finite number.
    NotFinite,
}

impl fmt::Display for OrientationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            OrientationError::OnlyForwardsOrBackwards { orientation } => write!(
                f,
                "orientation {orientation} is unreachable: this robot can only drive forwards (0) or backwards (pi)"
            ),
            OrientationError::GlobalNeedsOmnidirectional => {
                write!(
                    f,
                    "GLOBAL orientation is only valid for omnidirectional robots"
                )
            }
            OrientationError::NotFinite => write!(f, "orientation is not a finite number"),
        }
    }
}

impl std::error::Error for OrientationError {}

/// Heading the robot must have on an edge whose direction (start to end) is
/// `edge_direction`, in (-π, π].
pub fn edge_heading(
    kinematics: Kinematics,
    edge_direction: f64,
    orientation: Option<f64>,
    orientation_type: Option<OrientationType>,
) -> Result<f64, OrientationError> {
    let Some(o) = orientation else {
        return Ok(normalize_angle(edge_direction)); // S13: always forwards
    };
    if !o.is_finite() {
        return Err(OrientationError::NotFinite);
    }

    let omni = kinematics == Kinematics::Omnidirectional;
    match orientation_type.unwrap_or(OrientationType::Tangential) {
        OrientationType::Global => {
            if omni {
                Ok(normalize_angle(o))
            } else {
                Err(OrientationError::GlobalNeedsOmnidirectional)
            }
        }
        OrientationType::Tangential => {
            if omni {
                Ok(normalize_angle(edge_direction + o))
            } else if angle_diff(o, 0.0).abs() <= ORIENTATION_TOLERANCE {
                Ok(normalize_angle(edge_direction))
            } else if angle_diff(o, std::f64::consts::PI).abs() <= ORIENTATION_TOLERANCE {
                Ok(normalize_angle(edge_direction + std::f64::consts::PI))
            } else {
                Err(OrientationError::OnlyForwardsOrBackwards { orientation: o })
            }
        }
    }
}

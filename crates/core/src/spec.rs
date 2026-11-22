//! Robot type specification taken from a VDA 5050 factsheet: kinematics and
//! motion limits and body outline.

use std::fmt;

use vda5050::factsheet::Factsheet;

/// Kinematics of the robot (factsheet `mobileRobotKinematics`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kinematics {
    /// Forwards/backwards along the path, turns on the spot.
    Differential,
    /// Any travel direction, heading independent of travel direction.
    Omnidirectional,
    /// Car-like; usually cannot turn on the spot.
    ThreeWheel,
}

/// Motion limits. Units: m/s, m/s², rad/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    pub max_speed: f64,
    pub max_acceleration: f64,
    pub max_deceleration: f64,
    /// From the factsheet, or [`DEFAULT_ANGULAR_SPEED`] if the factsheet has none
    /// (see [`RobotSpec::angular_speed_is_default`]).
    pub max_angular_speed: f64,
}

/// Angular speed (rad/s) used when the factsheet has no `maximumAngularSpeed`.
pub const DEFAULT_ANGULAR_SPEED: f64 = 1.0;

/// Body outline, given in the robot frame (x forward, y left) as an offset
/// from the kinematic center. Units: metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Shape {
    /// Rectangle with its centre at (`cx`, `cy`) from the kinematic center,
    /// axes along the robot frame.
    Rect {
        cx: f64,
        cy: f64,
        half_length: f64,
        half_width: f64,
    },
    Circle {
        cx: f64,
        cy: f64,
        radius: f64,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct RobotSpec {
    pub manufacturer: String,
    pub series_name: String,
    pub kinematics: Kinematics,
    pub limits: Limits,
    /// True if `limits.max_angular_speed` is the simulator default, not a factsheet value.
    /// Reported in the run summary.
    pub angular_speed_is_default: bool,
    /// The lowest speed the robot can drive at continuously (factsheet
    /// `minimumSpeed`, m/s); it can stop, but not creep slower. 0 if the factsheet
    /// gives none or an unusable value.
    pub min_speed: f64,
    pub shape: Shape,
    /// The factsheet this spec was made from; the robot sends it as its
    /// `factsheet` message ([`crate::world::World::factsheet`], S61).
    pub factsheet: Factsheet,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SpecError {
    /// The envelope vertices give no usable rectangle (empty, non-finite or zero size).
    BadShape,
    /// The simulator cannot model this (manufacturer-specific) kinematics.
    UnsupportedKinematics(String),
    /// A limit that must be a finite number > 0 is not.
    BadLimit { name: &'static str, value: f64 },
}

impl fmt::Display for SpecError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SpecError::BadShape => write!(f, "body outline is empty, non-finite or has zero size"),
            SpecError::UnsupportedKinematics(k) => {
                write!(f, "unsupported mobileRobotKinematics '{k}'")
            }
            SpecError::BadLimit { name, value } => {
                write!(f, "{name} must be finite and > 0, got {value}")
            }
        }
    }
}

impl std::error::Error for SpecError {}

fn positive(name: &'static str, value: f64) -> Result<f64, SpecError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(SpecError::BadLimit { name, value })
    }
}

/// An envelope is a safety field (not the body) if its id says so.
fn is_safety_field(id: &str) -> bool {
    let id = id.to_ascii_lowercase();
    id.contains("safety") || id.contains("field")
}

/// Body outline: bounding rectangle (robot frame axes) of all `envelopes2d`
/// vertices whose id does not mark a safety field; `length` × `width` centered
/// on the kinematic center if there are none.
fn body_shape(f: &Factsheet) -> Result<Shape, SpecError> {
    let p = &f.physical_parameters;
    let points: Vec<_> = f
        .mobile_robot_geometry
        .envelopes2d
        .iter()
        .flatten()
        .filter(|e| !is_safety_field(&e.envelope_2d_id))
        .flat_map(|e| e.vertices.iter())
        .collect();

    if points.is_empty() {
        let (half_length, half_width) = (
            positive("length", p.length)? / 2.0,
            positive("width", p.width)? / 2.0,
        );
        return Ok(Shape::Rect {
            cx: 0.0,
            cy: 0.0,
            half_length,
            half_width,
        });
    }

    if points.iter().any(|v| !v.x.is_finite() || !v.y.is_finite()) {
        return Err(SpecError::BadShape);
    }
    let min_x = points.iter().map(|v| v.x).fold(f64::INFINITY, f64::min);
    let max_x = points.iter().map(|v| v.x).fold(f64::NEG_INFINITY, f64::max);
    let min_y = points.iter().map(|v| v.y).fold(f64::INFINITY, f64::min);
    let max_y = points.iter().map(|v| v.y).fold(f64::NEG_INFINITY, f64::max);
    if max_x <= min_x || max_y <= min_y {
        return Err(SpecError::BadShape);
    }
    Ok(Shape::Rect {
        cx: (min_x + max_x) / 2.0,
        cy: (min_y + max_y) / 2.0,
        half_length: (max_x - min_x) / 2.0,
        half_width: (max_y - min_y) / 2.0,
    })
}

impl RobotSpec {
    pub fn from_factsheet(f: &Factsheet) -> Result<RobotSpec, SpecError> {
        let kinematics = match f.type_specification.mobile_robot_kinematics.as_str() {
            "DIFFERENTIAL" => Kinematics::Differential,
            "OMNIDIRECTIONAL" => Kinematics::Omnidirectional,
            "THREE_WHEEL" => Kinematics::ThreeWheel,
            other => return Err(SpecError::UnsupportedKinematics(other.to_string())),
        };

        let p = &f.physical_parameters;
        let (max_angular_speed, angular_speed_is_default) = match p.maximum_angular_speed {
            Some(v) => (positive("maximumAngularSpeed", v)?, false),
            None => (DEFAULT_ANGULAR_SPEED, true),
        };
        let limits = Limits {
            max_speed: positive("maximumSpeed", p.maximum_speed)?,
            max_acceleration: positive("maximumAcceleration", p.maximum_acceleration)?,
            max_deceleration: positive("maximumDeceleration", p.maximum_deceleration)?,
            max_angular_speed,
        };

        Ok(RobotSpec {
            manufacturer: f.manufacturer.clone(),
            series_name: f.type_specification.series_name.clone(),
            kinematics,
            limits,
            angular_speed_is_default,
            min_speed: if p.minimum_speed.is_finite() && p.minimum_speed > 0.0 {
                p.minimum_speed
            } else {
                0.0
            },
            shape: body_shape(f)?,
            factsheet: f.clone(),
        })
    }
}

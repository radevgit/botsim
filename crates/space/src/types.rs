//! The robot type as the sweeps need it.

use botsim_model::spec::{Kinematics, RobotSpec, Shape, SpecError};
use vda5050::factsheet::Factsheet;

/// How far the protective field reaches in front of the body (m): from the
/// factsheet's `envelopes2d` whose id says "safety" or "field" (the rule
/// `botsim-model` uses to tell them from the body); if the factsheet has none,
/// the fixed margin of the simulator's own protective field.
/// The second value says whether the number is from the factsheet.
fn field_margin(f: &Factsheet, body: &Shape) -> (f64, bool) {
    let body_front = match body {
        Shape::Rect {
            cx, half_length, ..
        } => cx + half_length,
        Shape::Circle { cx, radius, .. } => cx + radius,
    };
    let field_front = f
        .mobile_robot_geometry
        .envelopes2d
        .iter()
        .flatten()
        .filter(|e| {
            let id = e.envelope_2d_id.to_ascii_lowercase();
            id.contains("safety") || id.contains("field")
        })
        .flat_map(|e| e.vertices.iter())
        .map(|v| v.x)
        .filter(|x| x.is_finite())
        .fold(f64::NEG_INFINITY, f64::max);
    if field_front.is_finite() {
        ((field_front - body_front).max(0.0), true)
    } else {
        (botsim_model::safety::FIELD_MARGIN, false)
    }
}

/// A robot type of the fleet: what the sweeps and the checks need of its factsheet.
#[derive(Debug, Clone)]
pub struct TypeRef {
    /// `manufacturer/seriesName`.
    pub name: String,
    pub manufacturer: String,
    pub series: String,
    pub kinematics: Kinematics,
    /// Length of the body outline (m), along the driving direction.
    pub length: f64,
    /// Width of the body outline (m), across the driving direction.
    pub width: f64,
    /// How far the protective field reaches in front of the body (m).
    pub field_margin: f64,
    /// False if the factsheet has no protective field and `field_margin` is the
    /// simulator's default.
    pub field_from_factsheet: bool,
    /// Body outline in the robot frame (x forward, y left), from the kinematic center.
    pub shape: Shape,
}

impl TypeRef {
    /// The type of a VDA 5050 factsheet: kinematics and body outline as the simulator
    /// reads them (`RobotSpec::from_factsheet`), the protective field from the
    /// factsheet. The error says why the factsheet cannot be used.
    pub fn from_factsheet(f: &Factsheet) -> Result<TypeRef, SpecError> {
        let s = RobotSpec::from_factsheet(f)?;
        let (field_margin, field_from_factsheet) = field_margin(f, &s.shape);
        let (length, width) = match s.shape {
            Shape::Rect {
                half_length,
                half_width,
                ..
            } => (2.0 * half_length, 2.0 * half_width),
            Shape::Circle { radius, .. } => (2.0 * radius, 2.0 * radius),
        };
        Ok(TypeRef {
            name: format!("{}/{}", f.manufacturer, f.type_specification.series_name),
            manufacturer: f.manufacturer.clone(),
            series: f.type_specification.series_name.clone(),
            kinematics: s.kinematics,
            length,
            width,
            field_margin,
            field_from_factsheet,
            shape: s.shape,
        })
    }

    /// A type with a rectangle body centred on the kinematic center and the default
    /// protective field of 0.3 m (for tests and examples; real types come from the
    /// factsheet).
    pub fn test_rect(name: &str, kinematics: Kinematics, length: f64, width: f64) -> TypeRef {
        let (manufacturer, series) = name.split_once('/').unwrap_or(("ACME", name));
        TypeRef {
            name: name.into(),
            manufacturer: manufacturer.into(),
            series: series.into(),
            kinematics,
            length,
            width,
            field_margin: 0.3,
            field_from_factsheet: false,
            shape: Shape::Rect {
                cx: 0.0,
                cy: 0.0,
                half_length: length / 2.0,
                half_width: width / 2.0,
            },
        }
    }
}

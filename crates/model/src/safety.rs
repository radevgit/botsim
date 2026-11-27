//! The robot's onboard safety stop (S54): a **protective field** ahead of the
//! robot, in the direction of travel (not the heading: a robot driving
//! backwards looks backwards). When another robot's body enters the field the
//! robot brakes and stands still until the field is clear again. It reports a
//! protective field violation in its state while it does.
//!
//! The field is a rectangle that starts at the front of the robot's body and
//! reaches `v² / (2 d) + v · dt + FIELD_MARGIN` ahead (braking distance at the
//! speed `v` and deceleration `d`, the distance covered in one step, and a
//! fixed margin); it is as wide as the body plus [`LATERAL_MARGIN`] on each side.
//! The numbers of the margins are our choice, not from a standard.
//!
//! Only robots are obstacles (there are no walls or people yet). A robot that
//! is turning or standing at a node has no field.

use togo::prelude::{OrientedRect, point};

use crate::spec::Shape;

/// Extra length of the field beyond the braking distance (m).
pub const FIELD_MARGIN: f64 = 0.3;
/// Extra width of the field on each side of the body (m).
pub const LATERAL_MARGIN: f64 = 0.1;

/// The protective field of a robot with `shape` at (`x`, `y`) with `heading`,
/// travelling along the unit vector `dir` at `speed` (m/s) with braking
/// deceleration `decel` (m/s²), in time steps of `dt` seconds.
pub fn protective_field(
    shape: &Shape,
    (x, y, heading): (f64, f64, f64),
    dir: (f64, f64),
    speed: f64,
    decel: f64,
    dt: f64,
) -> OrientedRect {
    let (s, c) = heading.sin_cos();
    let (fwd, left) = ((c, s), (-s, c));
    let across = (-dir.1, dir.0);
    let dot = |a: (f64, f64), b: (f64, f64)| a.0 * b.0 + a.1 * b.1;

    // centre of the body, and its half extent along and across the travel direction
    let (bx, by, ahead, side) = match *shape {
        Shape::Rect {
            cx,
            cy,
            half_length,
            half_width,
        } => (
            x + c * cx - s * cy,
            y + s * cx + c * cy,
            dot(dir, fwd).abs() * half_length + dot(dir, left).abs() * half_width,
            dot(across, fwd).abs() * half_length + dot(across, left).abs() * half_width,
        ),
        Shape::Circle { cx, cy, radius } => {
            (x + c * cx - s * cy, y + s * cx + c * cy, radius, radius)
        }
    };

    let length = speed * speed / (2.0 * decel) + speed * dt + FIELD_MARGIN;
    let reach = ahead + length / 2.0;
    OrientedRect::new(
        point(bx + dir.0 * reach, by + dir.1 * reach),
        point(dir.0, dir.1),
        length / 2.0,
        side + LATERAL_MARGIN,
    )
}

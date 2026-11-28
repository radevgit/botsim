//! Narrow-phase overlap of robot bodies (shapes placed in the world).
//!
//! A body is a rotated rectangle or a circle (from [`Shape`], an offset from
//! the kinematic center, turning with the robot's heading). Shapes are filled
//! and **touching counts as overlap** (togo's rule; circle–circle follows it).
//! Different maps never overlap; the caller checks that.

use togo::prelude::{
    Circle, OrientedRect, dist_circle_orect, dist_orect_orect, overlap_circle_orect,
    overlap_orect_orect, point,
};

use crate::spec::Shape;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Body {
    Rect(OrientedRect),
    Circle(Circle),
}

/// The shape of a robot standing at (`x`, `y`) with `heading`.
pub fn place(shape: &Shape, x: f64, y: f64, heading: f64) -> Body {
    let (s, c) = heading.sin_cos();
    // offset in the robot frame (x forward, y left) → world
    let at = |cx: f64, cy: f64| point(x + c * cx - s * cy, y + s * cx + c * cy);
    match *shape {
        Shape::Rect {
            cx,
            cy,
            half_length,
            half_width,
        } => Body::Rect(OrientedRect::new(
            at(cx, cy),
            point(c, s),
            half_length,
            half_width,
        )),
        Shape::Circle { cx, cy, radius } => Body::Circle(Circle::new(at(cx, cy), radius)),
    }
}

/// The shortest distance between the two bodies; 0 if they overlap or touch.
pub fn gap(a: &Body, b: &Body) -> f64 {
    match (a, b) {
        (Body::Rect(a), Body::Rect(b)) => dist_orect_orect(a, b),
        (Body::Circle(c), Body::Rect(r)) | (Body::Rect(r), Body::Circle(c)) => {
            dist_circle_orect(c, r)
        }
        (Body::Circle(a), Body::Circle(b)) => {
            ((a.c.x - b.c.x).hypot(a.c.y - b.c.y) - a.r - b.r).max(0.0)
        }
    }
}

/// Do the two bodies overlap (or touch)?
pub fn overlap(a: &Body, b: &Body) -> bool {
    match (a, b) {
        (Body::Rect(a), Body::Rect(b)) => overlap_orect_orect(a, b),
        (Body::Circle(c), Body::Rect(r)) | (Body::Rect(r), Body::Circle(c)) => {
            overlap_circle_orect(c, r)
        }
        (Body::Circle(a), Body::Circle(b)) => (a.c.x - b.c.x).hypot(a.c.y - b.c.y) <= a.r + b.r,
    }
}

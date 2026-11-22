//! Motion along one edge: a pure function, one fixed time step at a time.
//!
//! The robot is a point moving along a path of known length. Each step it
//! accelerates towards the speed limit, but never faster than it can still
//! brake to the wanted speed at the end of the path. Rotation at nodes is a
//! separate step.

use crate::spec::Limits;

/// Position along the edge and speed. Units: m, m/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Along {
    pub s: f64,
    pub v: f64,
}

/// What the robot is asked to do on this edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeGoal {
    /// Edge length in metres.
    pub length: f64,
    /// Speed limit on this edge (already the minimum of edge and robot limit), m/s.
    pub v_limit: f64,
    /// Speed wanted at the end of the edge: 0 to stop, > 0 to drive on into the next edge.
    pub v_end: f64,
}

/// Advance one step of `dt` seconds. Returns the new state and whether the
/// end of the edge was reached (then `s == length` and `v == v_end`).
///
/// Acceleration and deceleration limits are respected, except in the very last
/// step, where the speed is set to `v_end` on arrival.
pub fn advance(st: Along, goal: EdgeGoal, limits: &Limits, dt: f64) -> (Along, bool) {
    let remaining = goal.length - st.s;
    if remaining <= 0.0 {
        return (
            Along {
                s: goal.length,
                v: goal.v_end,
            },
            true,
        );
    }

    let v_limit = goal.v_limit.min(limits.max_speed);
    // Speed after accelerating (or slowing down towards a lowered limit).
    let v_acc = (st.v + limits.max_acceleration * dt).min(v_limit);
    let s_acc = 0.5 * (st.v + v_acc) * dt;
    if s_acc >= remaining {
        // The end is within reach this step. Braking has already made the speed
        // small here, so setting it to `v_end` is a small jump.
        return (
            Along {
                s: goal.length,
                v: goal.v_end,
            },
            true,
        );
    }
    // Fastest speed from which the end can still be reached at `v_end`.
    let v_brake = (goal.v_end * goal.v_end
        + 2.0 * limits.max_deceleration * (remaining - s_acc).max(0.0))
    .sqrt();
    let v_new = v_acc
        .min(v_brake)
        .max(st.v - limits.max_deceleration * dt)
        .max(0.0);

    let ds = 0.5 * (st.v + v_new) * dt;
    if ds >= remaining {
        return (
            Along {
                s: goal.length,
                v: goal.v_end,
            },
            true,
        );
    }
    (
        Along {
            s: st.s + ds,
            v: v_new,
        },
        false,
    )
}

/// Normalise an angle to (-π, π].
pub fn normalize_angle(a: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    let mut r = a % two_pi; // (-2π, 2π)
    if r > std::f64::consts::PI {
        r -= two_pi;
    } else if r <= -std::f64::consts::PI {
        r += two_pi;
    }
    r
}

/// Signed shortest rotation from `from` to `to`, in (-π, π]. Positive = counter-clockwise.
pub fn angle_diff(from: f64, to: f64) -> f64 {
    normalize_angle(to - from)
}

/// Like [`rotate`], but in the given direction (`ccw` or clockwise), even if that
/// is the long way round (S64: the layout may allow only one direction).
pub fn rotate_in(heading: f64, target: f64, rate: f64, dt: f64, ccw: bool) -> (f64, bool) {
    // how far there is to go in that direction, in (0, 2π]
    let remaining = if ccw {
        (target - heading).rem_euclid(std::f64::consts::TAU)
    } else {
        (heading - target).rem_euclid(std::f64::consts::TAU)
    };
    let step = rate * dt;
    if remaining <= step {
        (normalize_angle(target), true)
    } else {
        let turn = if ccw { step } else { -step };
        (normalize_angle(heading + turn), false)
    }
}

/// Rotate on the spot by one step of `dt` seconds towards `target`, the
/// shortest way, at constant angular speed `rate` (rad/s, > 0). Angles in
/// radians, counter-clockwise positive. Returns the new heading in (-π, π] and
/// whether the target was reached (then the heading equals `target`
/// normalised).
pub fn rotate(heading: f64, target: f64, rate: f64, dt: f64) -> (f64, bool) {
    let diff = angle_diff(heading, target);
    let max_turn = rate * dt;
    if diff.abs() <= max_turn {
        (normalize_angle(target), true)
    } else {
        (normalize_angle(heading + max_turn.copysign(diff)), false)
    }
}

//! Angles in radians, counter-clockwise positive.

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

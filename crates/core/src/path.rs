//! Paths a robot follows along an edge (S69): a straight line, or the curve of a LIF
//! `trajectory` (a NURBS), sampled into a polyline with a known length.
//!
//! A [`Path`] answers three questions for a distance `s` along it: where the
//! robot is, which way the path points there (the **tangent**), and how sharply it
//! turns. The tangent at the two ends is exact (it comes from the control
//! points); in between it is the direction of the sampled line piece, which is
//! within about 0.01 rad of the true one for curves of a metre radius or more.

use std::fmt;

/// Distance between two samples of a curve (m).
const SAMPLE_STEP: f64 = 0.02;

/// A NURBS curve in the plane (LIF 2.0 `trajectory`): degree, knots, control
/// points with weights.
#[derive(Debug, Clone, PartialEq)]
pub struct Nurbs {
    degree: usize,
    knots: Vec<f64>,
    /// (x, y, weight) of the control points.
    points: Vec<(f64, f64, f64)>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum NurbsError {
    /// The degree must be at least 1.
    BadDegree,
    /// At least `degree + 1` control points are needed, all finite.
    TooFewPoints,
    /// A weight must be a finite number above 0.
    BadWeight,
    /// The knot vector must have `points + degree + 1` finite values, not
    /// decreasing.
    BadKnots,
}

impl fmt::Display for NurbsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NurbsError::BadDegree => write!(f, "the degree must be at least 1"),
            NurbsError::TooFewPoints => {
                write!(
                    f,
                    "a curve of degree d needs at least d + 1 finite control points"
                )
            }
            NurbsError::BadWeight => write!(f, "a weight must be a finite number above 0"),
            NurbsError::BadKnots => write!(
                f,
                "the knot vector must have (points + degree + 1) values that do not decrease"
            ),
        }
    }
}

impl std::error::Error for NurbsError {}

impl Nurbs {
    /// `degree` (default in LIF: 1), `knots` (default: uniform and clamped, the
    /// curve starts at the first and ends at the last control point) and
    /// `points` as (x, y, weight; default 1).
    pub fn new(
        degree: usize,
        knots: Option<Vec<f64>>,
        points: &[(f64, f64, Option<f64>)],
    ) -> Result<Nurbs, NurbsError> {
        if degree < 1 {
            return Err(NurbsError::BadDegree);
        }
        if points.len() < degree + 1 || points.iter().any(|p| !(p.0.is_finite() && p.1.is_finite()))
        {
            return Err(NurbsError::TooFewPoints);
        }
        let points: Vec<(f64, f64, f64)> = points
            .iter()
            .map(|&(x, y, w)| (x, y, w.unwrap_or(1.0)))
            .collect();
        if points.iter().any(|p| !(p.2.is_finite() && p.2 > 0.0)) {
            return Err(NurbsError::BadWeight);
        }
        let n = points.len();
        let knots = match knots {
            Some(k) => {
                if k.len() != n + degree + 1
                    || k.iter().any(|v| !v.is_finite())
                    || k.windows(2).any(|w| w[0] > w[1])
                    || k[degree] >= k[n]
                {
                    return Err(NurbsError::BadKnots);
                }
                k
            }
            None => {
                // clamped uniform: degree + 1 zeros, the interior knots, degree + 1 ones
                let interior = n - degree - 1;
                let mut k = vec![0.0; degree + 1];
                k.extend((1..=interior).map(|i| i as f64 / (interior + 1) as f64));
                k.extend(vec![1.0; degree + 1]);
                k
            }
        };
        Ok(Nurbs {
            degree,
            knots,
            points,
        })
    }

    /// The parameter range of the curve.
    fn domain(&self) -> (f64, f64) {
        (self.knots[self.degree], self.knots[self.points.len()])
    }

    /// The point of the curve at parameter `u` (de Boor's algorithm on the weighted
    /// control points).
    pub fn point(&self, u: f64) -> (f64, f64) {
        let (lo, hi) = self.domain();
        let u = u.clamp(lo, hi);
        let n = self.points.len();
        let p = self.degree;
        // the knot span k with knots[k] <= u < knots[k + 1] (the last span at the end)
        let mut k = p;
        while k < n - 1 && u >= self.knots[k + 1] {
            k += 1;
        }
        let mut d: Vec<(f64, f64, f64)> = (0..=p)
            .map(|j| {
                let (x, y, w) = self.points[j + k - p];
                (x * w, y * w, w)
            })
            .collect();
        for r in 1..=p {
            for j in (r..=p).rev() {
                let i = j + k - p;
                let span = self.knots[i + p + 1 - r] - self.knots[i];
                let a = if span == 0.0 {
                    0.0
                } else {
                    (u - self.knots[i]) / span
                };
                d[j] = (
                    (1.0 - a) * d[j - 1].0 + a * d[j].0,
                    (1.0 - a) * d[j - 1].1 + a * d[j].1,
                    (1.0 - a) * d[j - 1].2 + a * d[j].2,
                );
            }
        }
        (d[p].0 / d[p].2, d[p].1 / d[p].2)
    }

    /// The first and the last point of the curve.
    pub fn ends(&self) -> ((f64, f64), (f64, f64)) {
        let (lo, hi) = self.domain();
        (self.point(lo), self.point(hi))
    }

    /// The length of the control polygon (an upper bound of the curve's length).
    fn polygon_length(&self) -> f64 {
        self.points
            .windows(2)
            .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
            .sum()
    }

    /// The direction of the curve at its two ends (radians): towards the next
    /// distinct control point, which is exact for a curve that starts and ends at
    /// its first and last control point.
    pub fn end_tangents(&self) -> (f64, f64) {
        let dir = |a: (f64, f64, f64), b: (f64, f64, f64)| (b.1 - a.1).atan2(b.0 - a.0);
        let p = &self.points;
        let first = p
            .iter()
            .skip(1)
            .find(|q| (q.0 - p[0].0).hypot(q.1 - p[0].1) > 1e-12)
            .map_or(0.0, |&q| dir(p[0], q));
        let last = p[..p.len() - 1]
            .iter()
            .rev()
            .find(|q| (q.0 - p[p.len() - 1].0).hypot(q.1 - p[p.len() - 1].1) > 1e-12)
            .map_or(0.0, |&q| dir(q, p[p.len() - 1]));
        (first, last)
    }

    /// The curve as a path: sampled every few centimetres.
    pub fn to_path(&self) -> Path {
        let (lo, hi) = self.domain();
        let steps = ((self.polygon_length() / SAMPLE_STEP).ceil() as usize).clamp(8, 20_000);
        let pts: Vec<(f64, f64)> = (0..=steps)
            .map(|i| self.point(lo + (hi - lo) * i as f64 / steps as f64))
            .collect();
        let (first, last) = self.end_tangents();
        Path::from_points(pts, Some((first, last)))
    }
}

/// A path with a length: points joined by straight pieces.
#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    pts: Vec<(f64, f64)>,
    /// Distance from the start to each point.
    cum: Vec<f64>,
    /// Exact direction at the start and at the end (radians).
    ends: (f64, f64),
}

impl Path {
    /// The straight path from `a` to `b`.
    pub fn straight(a: (f64, f64), b: (f64, f64)) -> Path {
        let dir = (b.1 - a.1).atan2(b.0 - a.0);
        Path::from_points(vec![a, b], Some((dir, dir)))
    }

    fn from_points(pts: Vec<(f64, f64)>, ends: Option<(f64, f64)>) -> Path {
        let mut cum = vec![0.0];
        let mut total = 0.0;
        for w in pts.windows(2) {
            total += (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1);
            cum.push(total);
        }
        let ends = ends.unwrap_or_else(|| {
            let d = |a: (f64, f64), b: (f64, f64)| (b.1 - a.1).atan2(b.0 - a.0);
            match pts.as_slice() {
                [first, second, .., before_last, last] => {
                    (d(*first, *second), d(*before_last, *last))
                }
                [a, b] => (d(*a, *b), d(*a, *b)),
                _ => (0.0, 0.0),
            }
        });
        Path { pts, cum, ends }
    }

    /// The length of the path (m).
    pub fn length(&self) -> f64 {
        self.cum.last().copied().unwrap_or(0.0)
    }

    /// Is it a straight line (two points)?
    pub fn is_straight(&self) -> bool {
        self.pts.len() == 2
    }

    /// The direction the path starts in, and the direction it ends in (radians).
    pub fn end_directions(&self) -> (f64, f64) {
        self.ends
    }

    /// Index of the piece that contains distance `s` (clamped to the path).
    fn piece(&self, s: f64) -> usize {
        let s = s.clamp(0.0, self.length());
        match self.cum.binary_search_by(|c| c.total_cmp(&s)) {
            Ok(i) => i.min(self.pts.len() - 2),
            Err(i) => (i - 1).min(self.pts.len() - 2),
        }
    }

    /// The point at distance `s` from the start (clamped to the path).
    pub fn point_at(&self, s: f64) -> (f64, f64) {
        let s = s.clamp(0.0, self.length());
        let i = self.piece(s);
        let len = self.cum[i + 1] - self.cum[i];
        let f = if len > 0.0 {
            (s - self.cum[i]) / len
        } else {
            0.0
        };
        let (a, b) = (self.pts[i], self.pts[i + 1]);
        (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f)
    }

    /// The direction of the path at distance `s` (radians): exact at the two ends,
    /// the direction of the sampled piece in between.
    pub fn tangent_at(&self, s: f64) -> f64 {
        if s <= 0.0 {
            return self.ends.0;
        }
        if s >= self.length() {
            return self.ends.1;
        }
        let i = self.piece(s);
        let (a, b) = (self.pts[i], self.pts[i + 1]);
        (b.1 - a.1).atan2(b.0 - a.0)
    }

    /// The sharpest turn on the path: the largest |curvature| (rad/m), found every
    /// 10 cm. 0 for a straight path.
    pub fn max_curvature(&self) -> f64 {
        if self.is_straight() {
            return 0.0;
        }
        let steps = (self.length() / 0.1).ceil().max(1.0) as usize;
        (0..=steps)
            .map(|i| {
                self.curvature_at(self.length() * i as f64 / steps as f64)
                    .abs()
            })
            .fold(0.0, f64::max)
    }

    /// How fast the direction changes with the distance at `s` (rad/m, positive =
    /// turning left), looked at over 0.2 m.
    pub fn curvature_at(&self, s: f64) -> f64 {
        if self.is_straight() {
            return 0.0;
        }
        let half = 0.1;
        let (a, b) = ((s - half).max(0.0), (s + half).min(self.length()));
        if b - a < 1e-9 {
            return 0.0;
        }
        let d = (self.tangent_at(b) - self.tangent_at(a)).rem_euclid(std::f64::consts::TAU);
        let d = if d > std::f64::consts::PI {
            d - std::f64::consts::TAU
        } else {
            d
        };
        d / (b - a)
    }
}

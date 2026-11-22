//! NURBS curves and the paths made of them (S69).

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI};

use botsim_core::path::{Nurbs, NurbsError, Path};

/// A quarter of a circle of radius `r` around the origin, from (r, 0) to (0, r): the
/// exact rational quadratic NURBS (weights 1, √2/2, 1).
fn quarter_circle(r: f64) -> Nurbs {
    Nurbs::new(
        2,
        None,
        &[
            (r, 0.0, Some(1.0)),
            (r, r, Some(FRAC_1_SQRT_2)),
            (0.0, r, Some(1.0)),
        ],
    )
    .unwrap()
}

#[test]
fn a_degree_one_curve_through_two_points_is_a_straight_line() {
    let c = Nurbs::new(1, None, &[(0.0, 0.0, None), (6.0, 8.0, None)]).unwrap();
    assert_eq!(c.ends(), ((0.0, 0.0), (6.0, 8.0)));
    let p = c.to_path();
    assert!((p.length() - 10.0).abs() < 1e-9);
    let (x, y) = p.point_at(5.0);
    assert!((x - 3.0).abs() < 1e-9 && (y - 4.0).abs() < 1e-9);
    let dir = 8.0_f64.atan2(6.0);
    assert!((p.tangent_at(3.0) - dir).abs() < 1e-12);
}

#[test]
fn a_rational_quadratic_is_an_exact_circle() {
    let r = 3.0;
    let c = quarter_circle(r);
    let ((x0, y0), (x1, y1)) = c.ends();
    assert!((x0 - r).abs() < 1e-12 && y0.abs() < 1e-12);
    assert!(x1.abs() < 1e-12 && (y1 - r).abs() < 1e-12);
    for i in 0..=20 {
        let (x, y) = c.point(i as f64 / 20.0);
        assert!(
            (x.hypot(y) - r).abs() < 1e-9,
            "u = {}: radius {}",
            i,
            x.hypot(y)
        );
    }
    // the sampled length is the arc length, π r / 2
    let p = c.to_path();
    assert!((p.length() - PI * r / 2.0).abs() < 1e-3, "{}", p.length());
}

#[test]
fn the_path_has_the_direction_of_the_curve_everywhere() {
    let r = 2.0;
    let p = quarter_circle(r).to_path();
    // exact at the ends: up at the start, left at the end
    assert!((p.tangent_at(0.0) - FRAC_PI_2).abs() < 1e-12);
    assert!((p.tangent_at(p.length()) - PI).abs() < 1e-12);
    // in between: the tangent of a circle is perpendicular to the radius
    for i in 1..20 {
        let s = p.length() * i as f64 / 20.0;
        let (x, y) = p.point_at(s);
        let expected = y.atan2(x) + FRAC_PI_2;
        let diff = (p.tangent_at(s) - expected + PI).rem_euclid(2.0 * PI) - PI;
        assert!(diff.abs() < 0.02, "s = {s}: off by {diff}");
        // and it turns left with 1/r
        assert!(
            (p.curvature_at(s) - 1.0 / r).abs() < 0.05,
            "{}",
            p.curvature_at(s)
        );
    }
}

#[test]
fn points_at_a_distance_are_that_far_along_and_clamped() {
    let p = quarter_circle(5.0).to_path();
    let at = |s| p.point_at(s);
    assert_eq!(at(-1.0), at(0.0));
    assert_eq!(at(p.length() + 1.0), at(p.length()));
    // the chord of a short piece is as long as the distance walked
    let (a, b) = (at(2.0), at(2.5));
    assert!(((b.0 - a.0).hypot(b.1 - a.1) - 0.5).abs() < 1e-3);
}

#[test]
fn a_straight_path_is_straight_and_has_no_curvature() {
    let p = Path::straight((1.0, 1.0), (1.0, 6.0));
    assert!(p.is_straight());
    assert_eq!(p.length(), 5.0);
    assert_eq!(p.end_directions(), (FRAC_PI_2, FRAC_PI_2));
    assert_eq!(p.curvature_at(2.0), 0.0);
    assert!(!quarter_circle(1.0).to_path().is_straight());
}

#[test]
fn a_cubic_with_the_default_knots_runs_between_its_end_points() {
    // an S-bend from (0,0) to (10,4)
    let c = Nurbs::new(
        3,
        None,
        &[
            (0.0, 0.0, None),
            (4.0, 0.0, None),
            (6.0, 4.0, None),
            (10.0, 4.0, None),
        ],
    )
    .unwrap();
    assert_eq!(c.ends(), ((0.0, 0.0), (10.0, 4.0)));
    let p = c.to_path();
    // it starts and ends going east, and is longer than the chord
    assert!((p.end_directions().0).abs() < 1e-12 && (p.end_directions().1).abs() < 1e-12);
    assert!(p.length() > 10.77 && p.length() < 14.0, "{}", p.length());
    // the middle turns the other way (left, then right)
    assert!(p.curvature_at(p.length() * 0.25) > 0.0);
    assert!(p.curvature_at(p.length() * 0.75) < 0.0);
}

#[test]
fn a_given_knot_vector_is_used() {
    // the same quadratic, with the knots written out
    let c = Nurbs::new(
        2,
        Some(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]),
        &[
            (3.0, 0.0, Some(1.0)),
            (3.0, 3.0, Some(FRAC_1_SQRT_2)),
            (0.0, 3.0, Some(1.0)),
        ],
    )
    .unwrap();
    assert_eq!(c, quarter_circle(3.0));
}

#[test]
fn wrong_curves_are_refused() {
    let two = [(0.0, 0.0, None), (1.0, 0.0, None)];
    assert_eq!(Nurbs::new(0, None, &two), Err(NurbsError::BadDegree));
    assert_eq!(Nurbs::new(2, None, &two), Err(NurbsError::TooFewPoints));
    assert_eq!(
        Nurbs::new(1, None, &[(0.0, f64::NAN, None), (1.0, 0.0, None)]),
        Err(NurbsError::TooFewPoints)
    );
    assert_eq!(
        Nurbs::new(1, None, &[(0.0, 0.0, Some(0.0)), (1.0, 0.0, None)]),
        Err(NurbsError::BadWeight)
    );
    // wrong number of knots, decreasing knots
    assert_eq!(
        Nurbs::new(1, Some(vec![0.0, 1.0]), &two),
        Err(NurbsError::BadKnots)
    );
    assert_eq!(
        Nurbs::new(1, Some(vec![0.0, 1.0, 0.5, 1.0]), &two),
        Err(NurbsError::BadKnots)
    );
}

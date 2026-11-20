// The tests use 3.14 on purpose: real layouts write it for π.
#![allow(clippy::approx_constant)]

use std::f64::consts::{FRAC_PI_2, PI};

use botsim_core::orientation::{OrientationError, edge_heading};
use botsim_core::spec::Kinematics::{Differential, Omnidirectional, ThreeWheel};
use vda5050::order::OrientationType::{Global, Tangential};

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-12
}

#[test]
fn no_orientation_means_forwards() {
    // edge pointing north (π/2)
    for k in [Differential, Omnidirectional, ThreeWheel] {
        assert!(close(
            edge_heading(k, FRAC_PI_2, None, None).unwrap(),
            FRAC_PI_2
        ));
    }
    // an orientation type without an orientation changes nothing
    assert!(close(
        edge_heading(Differential, 0.5, None, Some(Global)).unwrap(),
        0.5
    ));
}

#[test]
fn tangential_zero_is_forwards_and_pi_is_backwards() {
    let h = edge_heading(Differential, FRAC_PI_2, Some(0.0), Some(Tangential)).unwrap();
    assert!(close(h, FRAC_PI_2));
    // type omitted = TANGENTIAL
    let h = edge_heading(Differential, FRAC_PI_2, Some(PI), None).unwrap();
    assert!(close(h, -FRAC_PI_2)); // facing south while the edge goes north
    // same for three-wheel
    let h = edge_heading(ThreeWheel, 0.0, Some(PI), None).unwrap();
    assert!(close(h, PI));
}

#[test]
fn layouts_that_write_3_14_for_pi_are_accepted() {
    let h = edge_heading(Differential, 0.0, Some(3.14), None).unwrap();
    assert!(close(h, PI));
    // but 3.0 is not π
    assert_eq!(
        edge_heading(Differential, 0.0, Some(3.0), None),
        Err(OrientationError::OnlyForwardsOrBackwards { orientation: 3.0 })
    );
}

#[test]
fn differential_cannot_take_other_orientations_or_global() {
    assert_eq!(
        edge_heading(Differential, 0.0, Some(FRAC_PI_2), None),
        Err(OrientationError::OnlyForwardsOrBackwards {
            orientation: FRAC_PI_2
        })
    );
    assert_eq!(
        edge_heading(Differential, 0.0, Some(0.0), Some(Global)),
        Err(OrientationError::GlobalNeedsOmnidirectional)
    );
    assert_eq!(
        edge_heading(Differential, 0.0, Some(f64::NAN), None),
        Err(OrientationError::NotFinite)
    );
}

#[test]
fn omnidirectional_takes_any_orientation() {
    // tangential: relative to the edge direction (edge north, orientation +90° → west)
    let h = edge_heading(
        Omnidirectional,
        FRAC_PI_2,
        Some(FRAC_PI_2),
        Some(Tangential),
    )
    .unwrap();
    assert!(close(h, PI));
    // global: absolute, the edge direction does not matter
    let h = edge_heading(Omnidirectional, FRAC_PI_2, Some(-FRAC_PI_2), Some(Global)).unwrap();
    assert!(close(h, -FRAC_PI_2));
    // result is normalised
    let h = edge_heading(Omnidirectional, 3.0, Some(1.0), None).unwrap();
    assert!(h > -PI && h <= PI && close(h, 4.0 - 2.0 * PI));
}

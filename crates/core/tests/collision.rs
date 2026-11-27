use std::f64::consts::FRAC_PI_2;

use botsim_core::collision::{overlap, place};
use botsim_model::spec::Shape;

const RECT: Shape = Shape::Rect {
    cx: 0.0,
    cy: 0.0,
    half_length: 0.6,
    half_width: 0.4,
};

const CIRCLE: Shape = Shape::Circle {
    cx: 0.0,
    cy: 0.0,
    radius: 0.5,
};

// a pose is (x, y, heading) for each of the two shapes; kept flat so the tests read as tables
#[allow(clippy::too_many_arguments)]
fn hit(a: &Shape, ax: f64, ay: f64, ah: f64, b: &Shape, bx: f64, by: f64, bh: f64) -> bool {
    let (pa, pb) = (place(a, ax, ay, ah), place(b, bx, by, bh));
    let r = overlap(&pa, &pb);
    assert_eq!(r, overlap(&pb, &pa), "overlap must be symmetric");
    r
}

#[test]
fn rectangles_overlap_touch_and_separate() {
    // side by side along x: touch at distance 1.2 (counts), gap at 1.21
    assert!(hit(&RECT, 0.0, 0.0, 0.0, &RECT, 1.0, 0.0, 0.0));
    assert!(hit(&RECT, 0.0, 0.0, 0.0, &RECT, 1.2, 0.0, 0.0), "touching");
    assert!(!hit(&RECT, 0.0, 0.0, 0.0, &RECT, 1.21, 0.0, 0.0));
    // one turned by 90°: now the long axes cross, reach 0.6 + 0.4 = 1.0
    assert!(hit(&RECT, 0.0, 0.0, 0.0, &RECT, 0.0, 0.99, FRAC_PI_2));
    assert!(!hit(&RECT, 0.0, 0.0, 0.0, &RECT, 0.0, 1.01, FRAC_PI_2));
    // 45° turn: the turned box reaches (0.6 + 0.4) / √2 = 0.707 in x, so the
    // two touch at a centre distance of 0.6 + 0.707 = 1.307
    let q = std::f64::consts::FRAC_PI_4;
    assert!(hit(&RECT, 0.0, 0.0, 0.0, &RECT, 1.30, 0.0, q));
    assert!(!hit(&RECT, 0.0, 0.0, 0.0, &RECT, 1.32, 0.0, q));
}

#[test]
fn circles_overlap_touch_and_separate() {
    assert!(hit(&CIRCLE, 0.0, 0.0, 0.0, &CIRCLE, 0.9, 0.0, 0.0));
    assert!(
        hit(&CIRCLE, 0.0, 0.0, 0.0, &CIRCLE, 1.0, 0.0, 0.0),
        "touching"
    );
    assert!(!hit(&CIRCLE, 0.0, 0.0, 0.0, &CIRCLE, 1.01, 0.0, 0.0));
    // a diagonal distance of 1.0 is touching, not more
    assert!(!hit(&CIRCLE, 0.0, 0.0, 0.0, &CIRCLE, 0.8, 0.8, 0.0));
}

#[test]
fn circle_against_rectangle() {
    // rectangle reaches x = 0.6; circle radius 0.5 centred at x = 1.0 → reaches 0.5
    assert!(hit(&RECT, 0.0, 0.0, 0.0, &CIRCLE, 1.0, 0.0, 0.0));
    assert!(!hit(&RECT, 0.0, 0.0, 0.0, &CIRCLE, 1.2, 0.0, 0.0));
    // the rectangle turned by 90° only reaches 0.4 in x
    assert!(!hit(&RECT, 0.0, 0.0, FRAC_PI_2, &CIRCLE, 1.0, 0.0, 0.0));
}

#[test]
fn the_shape_offset_turns_with_the_heading() {
    // body centre 1 m ahead of the kinematic center
    let ahead = Shape::Circle {
        cx: 1.0,
        cy: 0.0,
        radius: 0.1,
    };
    let probe = Shape::Circle {
        cx: 0.0,
        cy: 0.0,
        radius: 0.1,
    };
    // facing east the body is at (1, 0); facing north at (0, 1)
    assert!(hit(&ahead, 0.0, 0.0, 0.0, &probe, 1.0, 0.0, 0.0));
    assert!(!hit(&ahead, 0.0, 0.0, 0.0, &probe, 0.0, 1.0, 0.0));
    assert!(hit(&ahead, 0.0, 0.0, FRAC_PI_2, &probe, 0.0, 1.0, 0.0));
    // sideways offset (y is left): facing east it is at (0, 1)
    let left = Shape::Circle {
        cx: 0.0,
        cy: 1.0,
        radius: 0.1,
    };
    assert!(hit(&left, 0.0, 0.0, 0.0, &probe, 0.0, 1.0, 0.0));
    assert!(hit(&left, 0.0, 0.0, FRAC_PI_2, &probe, -1.0, 0.0, 0.0));
}

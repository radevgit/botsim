//! The protective field (S54). The safety stop itself is tested in `botsim-sim`.

use botsim_model::safety::{FIELD_MARGIN, LATERAL_MARGIN, protective_field};
use botsim_model::spec::{RobotSpec, Shape};
use vda5050::factsheet::Factsheet;

const DT: f64 = 0.05;

fn spec() -> RobotSpec {
    // 1.2 m long, 0.8 m wide, up to 1.5 m/s, braking 0.8 m/s²
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

#[test]
fn the_field_starts_at_the_front_of_the_body_and_reaches_the_braking_distance() {
    let s = spec();
    let Shape::Rect {
        cx,
        half_length,
        half_width,
        ..
    } = s.shape
    else {
        panic!("a rectangle");
    };
    let d = s.limits.max_deceleration;

    // standing, facing east, travelling east: only the margin ahead
    let f = protective_field(&s.shape, (0.0, 0.0, 0.0), (1.0, 0.0), 0.0, d, DT);
    let front = cx + half_length;
    assert!((f.c.x - (front + FIELD_MARGIN / 2.0)).abs() < 1e-9, "{f}");
    assert!((f.half_len - FIELD_MARGIN / 2.0).abs() < 1e-9);
    assert!((f.half_wid - (half_width + LATERAL_MARGIN)).abs() < 1e-9);

    // at 1.5 m/s: the braking distance, one step, and the margin
    let v = 1.5;
    let f = protective_field(&s.shape, (0.0, 0.0, 0.0), (1.0, 0.0), v, d, DT);
    let length = v * v / (2.0 * d) + v * DT + FIELD_MARGIN;
    assert!((f.half_len * 2.0 - length).abs() < 1e-9);
    assert!((f.c.x - (front + length / 2.0)).abs() < 1e-9);

    // travelling west with the heading east (backwards): the field is behind
    let f = protective_field(&s.shape, (0.0, 0.0, 0.0), (-1.0, 0.0), v, d, DT);
    // (it starts at the back of the body, which is at x = cx - half_length)
    assert!(
        (f.c.x - (cx - half_length - length / 2.0)).abs() < 1e-9,
        "behind the robot: {f}"
    );
    assert_eq!((f.dir.x, f.dir.y), (-1.0, 0.0));

    // heading north, travelling north: the field is north of the body
    let f = protective_field(
        &s.shape,
        (3.0, 4.0, std::f64::consts::FRAC_PI_2),
        (0.0, 1.0),
        0.0,
        d,
        DT,
    );
    assert!(
        (f.c.x - 3.0).abs() < 1e-9 && f.c.y > 4.0 + half_length,
        "{f}"
    );
    assert_eq!((f.dir.x, f.dir.y), (0.0, 1.0));
}

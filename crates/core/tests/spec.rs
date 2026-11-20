use botsim_core::spec::{DEFAULT_ANGULAR_SPEED, Kinematics, RobotSpec, Shape, SpecError};
use vda5050::factsheet::{Envelope2d, Factsheet, Point2d};

const DIFF_ROBOT: &str = include_str!("data/diff_robot.factsheet.json");

fn factsheet() -> Factsheet {
    serde_json::from_str(DIFF_ROBOT).unwrap()
}

#[test]
fn spec_from_factsheet() {
    let s = RobotSpec::from_factsheet(&factsheet()).unwrap();
    assert_eq!(s.manufacturer, "ACME");
    assert_eq!(s.series_name, "X1");
    assert_eq!(s.kinematics, Kinematics::Differential);
    assert_eq!(s.limits.max_speed, 1.5);
    assert_eq!(s.limits.max_acceleration, 0.5);
    assert_eq!(s.limits.max_deceleration, 0.8);
    assert_eq!(s.limits.max_angular_speed, 1.0);
    assert!(!s.angular_speed_is_default);
}

#[test]
fn missing_angular_speed_gets_the_default_and_is_flagged() {
    let mut f = factsheet();
    f.physical_parameters.maximum_angular_speed = None;
    let s = RobotSpec::from_factsheet(&f).unwrap();
    assert_eq!(s.limits.max_angular_speed, DEFAULT_ANGULAR_SPEED);
    assert_eq!(DEFAULT_ANGULAR_SPEED, 1.0);
    assert!(s.angular_speed_is_default);

    // a stated but invalid value is still an error, not replaced by the default
    f.physical_parameters.maximum_angular_speed = Some(-1.0);
    assert!(matches!(
        RobotSpec::from_factsheet(&f),
        Err(SpecError::BadLimit {
            name: "maximumAngularSpeed",
            ..
        })
    ));
}

#[test]
fn other_kinematics_names() {
    let mut f = factsheet();
    f.type_specification.mobile_robot_kinematics = "OMNIDIRECTIONAL".into();
    assert_eq!(
        RobotSpec::from_factsheet(&f).unwrap().kinematics,
        Kinematics::Omnidirectional
    );
    f.type_specification.mobile_robot_kinematics = "THREE_WHEEL".into();
    assert_eq!(
        RobotSpec::from_factsheet(&f).unwrap().kinematics,
        Kinematics::ThreeWheel
    );
    f.type_specification.mobile_robot_kinematics = "MY_OWN".into();
    assert_eq!(
        RobotSpec::from_factsheet(&f).unwrap_err(),
        SpecError::UnsupportedKinematics("MY_OWN".into())
    );
}

#[test]
fn rejects_bad_limits() {
    let mut f = factsheet();
    f.physical_parameters.maximum_deceleration = 0.0;
    assert_eq!(
        RobotSpec::from_factsheet(&f).unwrap_err(),
        SpecError::BadLimit {
            name: "maximumDeceleration",
            value: 0.0
        }
    );
    let mut f = factsheet();
    f.physical_parameters.maximum_speed = f64::NAN;
    assert!(matches!(
        RobotSpec::from_factsheet(&f),
        Err(SpecError::BadLimit {
            name: "maximumSpeed",
            ..
        })
    ));
}

fn envelope(id: &str, pts: &[(f64, f64)]) -> Envelope2d {
    Envelope2d {
        envelope_2d_id: id.into(),
        vertices: pts.iter().map(|&(x, y)| Point2d { x, y }).collect(),
        description: None,
    }
}

#[test]
fn shape_falls_back_to_length_and_width() {
    let s = RobotSpec::from_factsheet(&factsheet()).unwrap();
    assert_eq!(
        s.shape,
        Shape::Rect {
            cx: 0.0,
            cy: 0.0,
            half_length: 0.6,
            half_width: 0.4
        }
    );
}

#[test]
fn shape_is_bounding_rect_of_body_envelopes_without_safety_fields() {
    let mut f = factsheet();
    f.mobile_robot_geometry.envelopes2d = Some(vec![
        // unloaded body: 1.0 long, 0.6 wide, centre 0.1 ahead of the kinematic center
        envelope(
            "unloaded",
            &[(-0.4, -0.3), (0.6, -0.3), (0.6, 0.3), (-0.4, 0.3)],
        ),
        // loaded: sticks out 0.2 more at the front
        envelope(
            "loaded",
            &[(-0.4, -0.3), (0.8, -0.3), (0.8, 0.3), (-0.4, 0.3)],
        ),
        // safety field must be ignored
        envelope(
            "Safety_field_fast",
            &[(-3.0, -2.0), (3.0, -2.0), (3.0, 2.0), (-3.0, 2.0)],
        ),
    ]);
    let s = RobotSpec::from_factsheet(&f).unwrap();
    let Shape::Rect {
        cx,
        cy,
        half_length,
        half_width,
    } = s.shape
    else {
        panic!("expected a rectangle");
    };
    assert!((cx - 0.2).abs() < 1e-12);
    assert!(cy.abs() < 1e-12);
    assert!((half_length - 0.6).abs() < 1e-12);
    assert!((half_width - 0.3).abs() < 1e-12);
}

#[test]
fn only_safety_fields_means_fallback_and_degenerate_envelope_is_rejected() {
    let mut f = factsheet();
    f.mobile_robot_geometry.envelopes2d =
        Some(vec![envelope("safetyField", &[(-3.0, -2.0), (3.0, 2.0)])]);
    assert!(matches!(
        RobotSpec::from_factsheet(&f).unwrap().shape,
        Shape::Rect { half_length, .. } if half_length == 0.6
    ));

    f.mobile_robot_geometry.envelopes2d = Some(vec![envelope("body", &[(0.0, 0.0), (1.0, 0.0)])]);
    assert_eq!(
        RobotSpec::from_factsheet(&f).unwrap_err(),
        SpecError::BadShape
    );
}

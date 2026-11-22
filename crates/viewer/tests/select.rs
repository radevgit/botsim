use botsim_export::View;
use botsim_record::{RobotRecord, RouteRecord, ShapeRecord, StatusRecord};
use botsim_viewer::{PICK_TOLERANCE_PX, pick_robot};

/// 50 pixels per metre, the world point (0, 0) in the middle of a 1000 × 600 image.
fn view() -> View {
    View {
        width: 1000,
        height: 600,
        scale: 50.0,
        centre: (0.0, 0.0),
    }
}

fn robot(x: f64, y: f64, heading: f64) -> RobotRecord {
    RobotRecord {
        id: "ACME/R".into(),
        map: 0,
        x,
        y,
        heading,
        shape: ShapeRecord::Rect([0.0, 0.0, 0.6, 0.4]),
        status: StatusRecord::Idle,
        speed: 0.0,
        order: String::new(),
        last_node: None,
        released: RouteRecord::default(),
        planned: RouteRecord::default(),
    }
}

fn click(view: &View, x: f64, y: f64) -> (f32, f32) {
    view.to_px(x, y)
}

#[test]
fn a_click_on_the_body_picks_the_robot_and_a_click_far_away_nothing() {
    let (v, robots) = (view(), vec![robot(5.0, 2.0, 0.0)]);
    assert_eq!(pick_robot(&robots, 0, &v, click(&v, 5.0, 2.0)), Some(0));
    assert_eq!(pick_robot(&robots, 0, &v, click(&v, 5.4, 2.3)), Some(0));
    assert_eq!(pick_robot(&robots, 0, &v, click(&v, 8.0, 2.0)), None);
    assert_eq!(pick_robot(&robots, 0, &v, click(&v, 5.0, -3.0)), None);
    assert_eq!(pick_robot(&[], 0, &v, click(&v, 5.0, 2.0)), None);
}

#[test]
fn a_click_just_beside_a_small_robot_still_hits_it() {
    let (v, robots) = (view(), vec![robot(0.0, 0.0, 0.0)]);
    // the body reaches 0.72 m (half the diagonal); the tolerance adds 10 px = 0.2 m
    let reach = 0.6f64.hypot(0.4) + PICK_TOLERANCE_PX / v.scale;
    assert_eq!(
        pick_robot(&robots, 0, &v, click(&v, reach - 0.02, 0.0)),
        Some(0)
    );
    assert_eq!(
        pick_robot(&robots, 0, &v, click(&v, reach + 0.05, 0.0)),
        None
    );
}

#[test]
fn the_nearest_robot_wins_where_two_are_close() {
    let v = view();
    let robots = vec![robot(0.0, 0.0, 0.0), robot(1.0, 0.0, 0.0)];
    assert_eq!(pick_robot(&robots, 0, &v, click(&v, 0.3, 0.0)), Some(0));
    assert_eq!(pick_robot(&robots, 0, &v, click(&v, 0.7, 0.0)), Some(1));
}

#[test]
fn the_body_turns_with_the_heading_and_a_circle_works_too() {
    let v = view();
    // pointing up: the 1.2 m length is along y; a point 0.5 m up is on the body
    // (the pick circle reaches 0.72 m + tolerance either way, so test the shift of
    // the middle with an offset shape instead)
    let mut offset = robot(0.0, 0.0, std::f64::consts::FRAC_PI_2);
    offset.shape = ShapeRecord::Rect([1.0, 0.0, 0.2, 0.2]); // 1 m ahead of the centre
    let robots = vec![offset];
    // heading up (+y): the body is at (0, 1)
    assert_eq!(pick_robot(&robots, 0, &v, click(&v, 0.0, 1.0)), Some(0));
    assert_eq!(pick_robot(&robots, 0, &v, click(&v, 1.0, 0.0)), None);

    let mut circle = robot(3.0, 3.0, 0.0);
    circle.shape = ShapeRecord::Circle([0.0, 0.0, 0.5]);
    let robots = vec![circle];
    assert_eq!(pick_robot(&robots, 0, &v, click(&v, 3.3, 3.3)), Some(0));
    assert_eq!(pick_robot(&robots, 0, &v, click(&v, 4.0, 4.0)), None);
}

#[test]
fn robots_that_are_not_drawn_are_not_picked() {
    let v = view();
    let mut other_floor = robot(0.0, 0.0, 0.0);
    other_floor.map = 1;
    let mut in_lift = robot(0.0, 0.0, 0.0);
    in_lift.status = StatusRecord::RidingLift;
    let robots = vec![other_floor, in_lift, robot(0.0, 0.0, 0.0)];
    // only the third is on floor 0 and in the open
    assert_eq!(pick_robot(&robots, 0, &v, click(&v, 0.0, 0.0)), Some(2));
    assert_eq!(pick_robot(&robots, 1, &v, click(&v, 0.0, 0.0)), Some(0));
}

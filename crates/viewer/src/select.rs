//! Choosing a robot by a click in the picture.

use botsim_export::View;
use botsim_record::{RobotRecord, ShapeRecord, StatusRecord};

/// A click this close (pixels) to a robot's body still picks it, so that small
/// robots in a big picture can be hit.
pub const PICK_TOLERANCE_PX: f64 = 10.0;

/// Is the robot drawn on this floor? (A robot in a lift cabin is on no floor.)
pub fn on_floor(robot: &RobotRecord, map: usize) -> bool {
    robot.map == map && robot.status != StatusRecord::RidingLift
}

/// The middle of the body and the radius of the smallest circle around it.
fn body_circle(robot: &RobotRecord) -> ((f64, f64), f64) {
    let (cx, cy, radius) = match robot.shape {
        ShapeRecord::Rect([cx, cy, half_length, half_width]) => {
            (cx, cy, half_length.hypot(half_width))
        }
        ShapeRecord::Circle([cx, cy, radius]) => (cx, cy, radius),
    };
    let (sin, cos) = robot.heading.sin_cos();
    (
        (robot.x + cx * cos - cy * sin, robot.y + cx * sin + cy * cos),
        radius,
    )
}

/// The robot (its position in `robots`) that a click at image pixel `(px, py)`
/// hits: the one whose body middle is nearest, among those the click is on or
/// within [`PICK_TOLERANCE_PX`] of. Robots that are not drawn on this floor are
/// not picked. `None` if the click is on nothing.
pub fn pick_robot(
    robots: &[RobotRecord],
    map: usize,
    view: &View,
    (px, py): (f32, f32),
) -> Option<usize> {
    let (wx, wy) = view.to_world(px, py);
    let tolerance = PICK_TOLERANCE_PX / view.scale;
    robots
        .iter()
        .enumerate()
        .filter(|(_, r)| on_floor(r, map))
        .filter_map(|(i, r)| {
            let ((x, y), radius) = body_circle(r);
            let distance = (wx - x).hypot(wy - y);
            (distance <= radius + tolerance).then_some((i, distance))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i)
}

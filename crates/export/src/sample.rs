//! The world at any moment, from the snapshots of a recording: robot poses are
//! interpolated between the two snapshots around the moment, so a video at 30
//! frames per second is smooth although the recording has fewer snapshots.

use std::f64::consts::{PI, TAU};

use botsim_record::SnapshotLine;

/// The world at simulated time `t` (seconds).
///
/// Between two snapshots, the x and y of each robot move linearly and the
/// heading turns the short way; everything else (status, routes, blocking, lifts,
/// ...) is that of the earlier snapshot. A robot is interpolated only if both
/// snapshots list the same robots in the same order and the robot is on the same
/// map in both; otherwise it stays as it was in the earlier snapshot, because it
/// did not move continuously. Before the first and after the last snapshot, that
/// snapshot is returned. `snapshots` must be ordered by time (a recording is).
/// `None` if there are no snapshots. The result's `time` is `t`, clamped to the
/// recording.
pub fn sample(snapshots: &[SnapshotLine], t: f64) -> Option<SnapshotLine> {
    let first = snapshots.first()?;
    let last = snapshots.last()?;
    if t <= first.time {
        return Some(first.clone());
    }
    if t >= last.time {
        return Some(last.clone());
    }
    // the first snapshot after t; the one before it is at or before t
    let next = snapshots.partition_point(|s| s.time <= t);
    let (a, b) = (&snapshots[next - 1], &snapshots[next]);
    let span = b.time - a.time;
    let u = if span > 0.0 { (t - a.time) / span } else { 0.0 };

    let mut out = a.clone();
    out.time = t;
    let same_robots = a.robots.len() == b.robots.len()
        && a.robots.iter().zip(&b.robots).all(|(p, q)| p.id == q.id);
    if same_robots {
        for (robot, target) in out.robots.iter_mut().zip(&b.robots) {
            if robot.map == target.map {
                robot.x += (target.x - robot.x) * u;
                robot.y += (target.y - robot.y) * u;
                robot.heading = turn(robot.heading, target.heading, u);
            }
        }
    }
    Some(out)
}

/// From heading `from` towards `to` (radians), the short way, `u` of the way.
fn turn(from: f64, to: f64, u: f64) -> f64 {
    let mut delta = (to - from) % TAU;
    if delta > PI {
        delta -= TAU;
    } else if delta < -PI {
        delta += TAU;
    }
    from + delta * u
}

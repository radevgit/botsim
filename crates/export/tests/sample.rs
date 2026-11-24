use std::f64::consts::PI;

use botsim_export::sample;
use botsim_record::{RobotRecord, RouteRecord, ShapeRecord, SnapshotLine, StatusRecord};

fn robot(id: &str, map: usize, x: f64, y: f64, heading: f64, status: StatusRecord) -> RobotRecord {
    RobotRecord {
        id: id.into(),
        map,
        x,
        y,
        heading,
        shape: ShapeRecord::Circle([0.0, 0.0, 0.4]),
        status,
        speed: 0.0,
        order: String::new(),
        last_node: None,
        released: RouteRecord::default(),
        planned: RouteRecord::default(),
        charge: None,
        charging: false,
    }
}

fn snap(time: f64, robots: Vec<RobotRecord>) -> SnapshotLine {
    SnapshotLine {
        tick: (time * 20.0).round() as u64,
        time,
        robots,
        reservations: vec![],
        blocking: vec![[0, 1]],
        overlapping: vec![],
        deadlocks: vec![],
        lifts: vec![],
        doors: vec![],
    }
}

fn two() -> Vec<SnapshotLine> {
    vec![
        snap(
            1.0,
            vec![robot("a", 0, 0.0, 0.0, 0.0, StatusRecord::Driving)],
        ),
        snap(
            2.0,
            vec![robot("a", 0, 10.0, -4.0, 1.0, StatusRecord::Idle)],
        ),
    ]
}

#[test]
fn no_snapshots_no_sample() {
    assert_eq!(sample(&[], 1.0), None);
}

#[test]
fn a_robot_moves_linearly_between_two_snapshots() {
    let s = sample(&two(), 1.25).unwrap();
    let r = &s.robots[0];
    assert_eq!((r.x, r.y, r.heading), (2.5, -1.0, 0.25));
    assert_eq!(s.time, 1.25);
    // everything but the pose is the earlier snapshot's
    assert_eq!(r.status, StatusRecord::Driving);
    assert_eq!(s.blocking, vec![[0, 1]]);
}

#[test]
fn on_a_snapshot_and_outside_the_recording_it_is_that_snapshot() {
    let snaps = two();
    assert_eq!(sample(&snaps, 1.0).unwrap(), snaps[0]);
    assert_eq!(sample(&snaps, 2.0).unwrap(), snaps[1]);
    assert_eq!(sample(&snaps, 0.0).unwrap(), snaps[0]);
    assert_eq!(sample(&snaps, 99.0).unwrap(), snaps[1]);
}

#[test]
fn the_right_pair_is_found_among_many_snapshots() {
    let snaps: Vec<SnapshotLine> = (1..=100)
        .map(|i| {
            let x = i as f64;
            snap(
                i as f64 * 0.05,
                vec![robot("a", 0, x, 0.0, 0.0, StatusRecord::Driving)],
            )
        })
        .collect();
    // between the snapshots 40 (t = 2.0, x = 40) and 41 (t = 2.05, x = 41)
    let s = sample(&snaps, 2.025).unwrap();
    assert!((s.robots[0].x - 40.5).abs() < 1e-9);
}

#[test]
fn the_heading_turns_the_short_way() {
    // from 3.0 over the half turn to -3.0: 0.2832 rad, not 6 rad the other way
    let snaps = vec![
        snap(
            0.0,
            vec![robot("a", 0, 0.0, 0.0, 3.0, StatusRecord::Driving)],
        ),
        snap(
            1.0,
            vec![robot("a", 0, 0.0, 0.0, -3.0, StatusRecord::Driving)],
        ),
    ];
    let mid = sample(&snaps, 0.5).unwrap().robots[0].heading;
    assert!((mid - (3.0 + (2.0 * PI - 6.0) / 2.0)).abs() < 1e-9, "{mid}");
    // and the other direction
    let back = vec![
        snap(
            0.0,
            vec![robot("a", 0, 0.0, 0.0, -3.0, StatusRecord::Driving)],
        ),
        snap(
            1.0,
            vec![robot("a", 0, 0.0, 0.0, 3.0, StatusRecord::Driving)],
        ),
    ];
    let mid = sample(&back, 0.5).unwrap().robots[0].heading;
    assert!(
        (mid - (-3.0 - (2.0 * PI - 6.0) / 2.0)).abs() < 1e-9,
        "{mid}"
    );
}

#[test]
fn a_robot_that_changes_map_does_not_slide() {
    // in a lift: it appears on the other floor; it stays where it was until then
    let snaps = vec![
        snap(
            0.0,
            vec![robot("a", 0, 1.0, 1.0, 0.0, StatusRecord::Driving)],
        ),
        snap(
            1.0,
            vec![robot("a", 1, 50.0, 50.0, 0.0, StatusRecord::Driving)],
        ),
    ];
    let r = &sample(&snaps, 0.5).unwrap().robots[0];
    assert_eq!((r.map, r.x, r.y), (0, 1.0, 1.0));
}

#[test]
fn a_changed_robot_list_is_not_interpolated() {
    let snaps = vec![
        snap(0.0, vec![robot("a", 0, 0.0, 0.0, 0.0, StatusRecord::Idle)]),
        snap(
            1.0,
            vec![
                robot("a", 0, 8.0, 0.0, 0.0, StatusRecord::Idle),
                robot("b", 0, 0.0, 0.0, 0.0, StatusRecord::Idle),
            ],
        ),
    ];
    let s = sample(&snaps, 0.5).unwrap();
    assert_eq!(s.robots.len(), 1);
    assert_eq!(s.robots[0].x, 0.0);
}

#[test]
fn a_standing_robot_stays_put_and_the_result_is_repeatable() {
    let snaps = vec![
        snap(0.0, vec![robot("a", 0, 3.0, 4.0, 0.5, StatusRecord::Idle)]),
        snap(1.0, vec![robot("a", 0, 3.0, 4.0, 0.5, StatusRecord::Idle)]),
    ];
    let s = sample(&snaps, 0.3).unwrap();
    assert_eq!(
        (s.robots[0].x, s.robots[0].y, s.robots[0].heading),
        (3.0, 4.0, 0.5)
    );
    assert_eq!(sample(&snaps, 0.3).unwrap(), s);
}

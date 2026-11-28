mod common;

use botsim_model::graph::Graph;
use botsim_model::waiting::WaitReason;
use botsim_record::{ShapeRecord, SnapshotLine, StatusRecord};
use botsim_sim::observer::{Observer, Snapshot};
use botsim_sim::waiting::Wait;
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Debug)]
struct Collect(Rc<RefCell<Vec<Snapshot>>>);

impl Observer for Collect {
    fn on_layout(&mut self, _: &Graph) {}
    fn on_step(&mut self, s: &Snapshot) {
        self.0.borrow_mut().push(s.clone());
    }
}

/// Corridor A - B - C, one robot R-1 on A driving to B (then C is planned).
fn run() -> Vec<Snapshot> {
    let mut w = common::corridor_world();
    let log = Rc::new(RefCell::new(Vec::new()));
    w.set_observer(Box::new(Collect(log.clone())));
    for _ in 0..400 {
        w.step();
    }
    log.borrow().clone()
}

#[test]
fn a_snapshot_line_copies_the_snapshot() {
    let snaps = run();
    let first = SnapshotLine::from(&snaps[0]);
    assert_eq!((first.tick, first.time), (snaps[0].tick, snaps[0].time));
    assert_eq!(first.robots.len(), 1);
    let r = &first.robots[0];
    assert_eq!(r.id, "ACME/R-1");
    assert_eq!(r.order, "o1");
    assert!(matches!(r.shape, ShapeRecord::Rect(_)));
    assert!(!r.released.nodes.is_empty() && !r.planned.nodes.is_empty());
    assert!(first.reservations.is_empty() && first.blocking.is_empty());
    assert!(first.lifts.is_empty() && first.doors.is_empty());

    // the status changes over the run
    let statuses: Vec<StatusRecord> = snaps
        .iter()
        .map(|s| SnapshotLine::from(s).robots[0].status)
        .collect();
    assert!(statuses.contains(&StatusRecord::Driving));
    assert!(statuses.contains(&StatusRecord::WaitingForRelease));
}

#[test]
fn every_snapshot_survives_a_json_round_trip() {
    for s in run() {
        let line = SnapshotLine::from(&s);
        let text = serde_json::to_string(&line).unwrap();
        assert!(!text.contains('\n'), "one line");
        assert!(text.starts_with(r#"{"type":"snapshot","tick":"#), "{text}");
        let back: SnapshotLine = serde_json::from_str(&text).unwrap();
        assert_eq!(back, line);
    }
}

#[test]
fn a_wait_is_copied_with_its_blocker_and_reason() {
    let mut s = run().remove(0);
    s.robots[0].waiting = Some(Wait {
        reason: WaitReason::SafetyStop,
        blocker: Some(0),
    });
    let text = serde_json::to_string(&SnapshotLine::from(&s)).unwrap();
    assert!(
        text.contains(r#""waiting":{"waiting_for":0,"reason":"SafetyStop"}"#),
        "{text}"
    );
    // an unknown blocker is `null`
    s.robots[0].waiting = Some(Wait {
        reason: WaitReason::Release,
        blocker: None,
    });
    let line = SnapshotLine::from(&s);
    let text = serde_json::to_string(&line).unwrap();
    assert!(
        text.contains(r#""waiting":{"waiting_for":null,"reason":"Release"}"#),
        "{text}"
    );
    assert_eq!(serde_json::from_str::<SnapshotLine>(&text).unwrap(), line);
}

#[test]
fn a_robot_that_does_not_wait_has_no_field_and_old_lines_still_read() {
    let mut s = run().remove(0);
    s.robots[0].waiting = None;
    // a line without the field is also what a recording from before it looks like
    let line = SnapshotLine::from(&s);
    let text = serde_json::to_string(&line).unwrap();
    assert!(!text.contains("waiting"), "{text}");
    let back: SnapshotLine = serde_json::from_str(&text).unwrap();
    assert_eq!(back.robots[0].waiting, None);
}

#[test]
fn the_json_has_the_agreed_field_names() {
    let line = SnapshotLine::from(&run()[0]);
    let v: serde_json::Value = serde_json::to_value(&line).unwrap();
    for key in [
        "tick",
        "time",
        "robots",
        "reservations",
        "blocking",
        "overlapping",
        "deadlocks",
        "lifts",
        "doors",
    ] {
        assert!(v.get(key).is_some(), "missing {key}");
    }
    let r = &v["robots"][0];
    for key in [
        "id",
        "map",
        "x",
        "y",
        "heading",
        "shape",
        "status",
        "speed",
        "order",
        "last_node",
        "released",
        "planned",
    ] {
        assert!(r.get(key).is_some(), "missing robot.{key}");
    }
    assert!(r["shape"].get("rect").is_some());
}

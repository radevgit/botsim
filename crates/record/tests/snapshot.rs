use std::cell::RefCell;
use std::rc::Rc;

use botsim_core::actions::ActionTimes;
use botsim_core::graph::Graph;
use botsim_core::observer::{Observer, Snapshot};
use botsim_core::robot::Reception::Accepted;
use botsim_core::spec::RobotSpec;
use botsim_core::world::{RobotId, World};
use botsim_record::{ShapeRecord, SnapshotLine, StatusRecord};
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

const DT: f64 = 0.05;
const DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../scenarios/");

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
    let lif: Lif =
        serde_json::from_str(&std::fs::read_to_string(format!("{DIR}corridor.lif.json")).unwrap())
            .unwrap();
    let graph = Graph::from_lif(&lif).unwrap();
    let a = graph.node_idx("A").unwrap();
    let f: Factsheet = serde_json::from_str(
        &std::fs::read_to_string(format!("{DIR}diff_robot.factsheet.json")).unwrap(),
    )
    .unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let mut w = World::new(graph, DT, 0).unwrap();
    w.add_robot(
        RobotId::new("ACME", "R-1"),
        spec,
        ActionTimes::new(2.0),
        a,
        0.0,
    )
    .unwrap();
    let log = Rc::new(RefCell::new(Vec::new()));
    w.set_observer(Box::new(Collect(log.clone())));
    let order: Order = serde_json::from_str(
        r#"{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"A","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"B","sequenceId":2,"released":true,"actions":[]},
                 {"nodeId":"C","sequenceId":4,"released":false,"actions":[]}],
        "edges":[{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]},
                 {"edgeId":"B-C","sequenceId":3,"released":false,"actions":[]}]}"#,
    )
    .unwrap();
    assert_eq!(w.send_order(&order), Ok(Accepted));
    for _ in 0..400 {
        w.step();
    }
    let snaps = log.borrow().clone();
    snaps
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

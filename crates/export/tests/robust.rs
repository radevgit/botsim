//! A recording that does not fit together (a bad node number, a bad robot number)
//! must not make the exporter panic: what cannot be drawn is left out.

use botsim_export::Renderer;
use botsim_record::{EdgeRecord, LayoutLine, NodeRecord, SnapshotLine};

fn layout() -> LayoutLine {
    let node = |id: &str, x: f64| NodeRecord {
        id: id.into(),
        map: 0,
        x,
        y: 0.0,
    };
    let edge = |id: &str, start, end| EdgeRecord {
        id: id.into(),
        start,
        end,
        length: 10.0,
        map_transition: false,
        paths: vec![],
    };
    LayoutLine {
        version: 1,
        maps: vec!["M".into()],
        nodes: vec![node("A", 0.0), node("B", 10.0)],
        // the second edge names a node that does not exist, the third too
        edges: vec![edge("A-B", 0, 1), edge("A-X", 0, 99), edge("Y-B", 77, 1)],
        geometry: vec![],
        robots: vec![],
    }
}

#[test]
fn an_edge_with_a_bad_node_number_is_left_out() {
    let layout = layout();
    let r = Renderer::new(&layout, 0, 640, 360).unwrap();
    let frame = r.empty_frame();
    assert_eq!((frame.width(), frame.height()), (640, 360));
    // the good edge is there: the middle of the picture is on it
    let c = frame.pixel(320, 180).unwrap();
    assert_ne!([c.red(), c.green(), c.blue()], [255, 255, 255]);
}

#[test]
fn bad_robot_numbers_in_a_snapshot_are_left_out() {
    let layout = layout();
    let r = Renderer::new(&layout, 0, 640, 360).unwrap();
    let snapshot = SnapshotLine {
        tick: 1,
        time: 0.05,
        robots: vec![],
        reservations: vec![],
        blocking: vec![[0, 1], [5, 9]],
        overlapping: vec![[3, 4]],
        deadlocks: vec![vec![0, 8]],
        lifts: vec![],
        doors: vec![],
    };
    let frame = r.frame(&snapshot);
    assert_eq!((frame.width(), frame.height()), (640, 360));
    // also with a robot selected that does not exist
    let _ = r.frame_selected(&snapshot, Some("nobody"));
}

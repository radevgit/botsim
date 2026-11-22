//! Curved edges: all curves of an edge dimmed, the selected robot's own strong.

use botsim_export::palette::{self, Rgb};
use botsim_export::{Renderer, View};
use botsim_record::{EdgePath, EdgeRecord, LayoutLine, LayoutRobot, NodeRecord, SnapshotLine};
use tiny_skia::Pixmap;

const W: u32 = 1920;
const H: u32 = 1080;

/// An edge A (0,0) → B (10,0). Type X1 follows an arc up to y = 2 (through
/// (5, 2)), type Y2 the straight line (through (5, 0)). Robot R-1 is an X1, R-2 a Y2.
fn two_curves() -> LayoutLine {
    let node = |id: &str, x: f64| NodeRecord {
        id: id.into(),
        map: 0,
        x,
        y: 0.0,
    };
    let arc: Vec<[f64; 2]> = (0..=20)
        .map(|i| {
            let x = i as f64 * 0.5;
            // a parabola, 2 m high in the middle
            [x, 2.0 - 2.0 * ((x - 5.0) / 5.0).powi(2)]
        })
        .collect();
    LayoutLine {
        version: 1,
        maps: vec!["M".into()],
        nodes: vec![node("A", 0.0), node("B", 10.0)],
        edges: vec![EdgeRecord {
            id: "A-B".into(),
            start: 0,
            end: 1,
            length: 10.0,
            map_transition: false,
            paths: vec![
                EdgePath {
                    types: vec!["ACME/X1".into()],
                    points: arc,
                },
                EdgePath {
                    types: vec!["ACME/Y2".into()],
                    points: vec![[0.0, 0.0], [10.0, 0.0]],
                },
            ],
        }],
        geometry: vec![],
        robots: vec![
            LayoutRobot {
                id: "ACME/R-1".into(),
                robot_type: "ACME/X1".into(),
            },
            LayoutRobot {
                id: "ACME/R-2".into(),
                robot_type: "ACME/Y2".into(),
            },
        ],
    }
}

fn empty_snapshot() -> SnapshotLine {
    SnapshotLine {
        tick: 1,
        time: 0.05,
        robots: vec![],
        reservations: vec![],
        blocking: vec![],
        overlapping: vec![],
        deadlocks: vec![],
        lifts: vec![],
        doors: vec![],
    }
}

fn at(pixmap: &Pixmap, view: &View, x: f64, y: f64) -> Rgb {
    let (px, py) = view.to_px(x, y);
    let p = pixmap.pixel(px as u32, py as u32).unwrap();
    [p.red(), p.green(), p.blue()]
}

fn differing(a: &Pixmap, b: &Pixmap) -> usize {
    (0..a.height())
        .flat_map(|y| (0..a.width()).map(move |x| (x, y)))
        .filter(|&(x, y)| a.pixel(x, y) != b.pixel(x, y))
        .count()
}

#[test]
fn the_view_includes_a_curve_that_bulges_out_of_the_nodes() {
    let layout = two_curves();
    let view = View::fit(&layout, 0, W, H).unwrap();
    // the nodes are on y = 0, the arc reaches y = 2: both must be in the image
    let (_, top) = view.to_px(5.0, 2.0);
    assert!(
        top >= 0.05 * H as f32 - 0.5,
        "the arc is {top} px from the top"
    );
    // the middle of the box is y = 1, not y = 0
    assert!((view.centre.1 - 1.0).abs() < 1e-9, "{:?}", view.centre);
}

#[test]
fn all_curves_of_an_edge_are_dimmed_when_nothing_is_selected() {
    let layout = two_curves();
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let view = *r.view();
    let frame = r.frame(&empty_snapshot());
    assert_eq!(at(&frame, &view, 5.0, 2.0), palette::GRAPH_DIM, "the arc");
    assert_eq!(
        at(&frame, &view, 5.0, 0.0),
        palette::GRAPH_DIM,
        "the straight line"
    );
    assert_eq!(
        at(&frame, &view, 5.0, 1.0),
        palette::BACKGROUND,
        "between them"
    );
    // no selection through `frame_selected` is the same picture
    assert_eq!(
        differing(&frame, &r.frame_selected(&empty_snapshot(), None)),
        0
    );
}

#[test]
fn the_selected_robots_own_curve_is_strong() {
    let layout = two_curves();
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let view = *r.view();
    let snap = empty_snapshot();

    let x1 = r.frame_selected(&snap, Some("ACME/R-1"));
    assert_eq!(
        at(&x1, &view, 5.0, 2.0),
        palette::GRAPH_SELECTED,
        "R-1 follows the arc"
    );
    assert_eq!(at(&x1, &view, 5.0, 0.0), palette::GRAPH_DIM);

    let y2 = r.frame_selected(&snap, Some("ACME/R-2"));
    assert_eq!(
        at(&y2, &view, 5.0, 0.0),
        palette::GRAPH_SELECTED,
        "R-2 follows the line"
    );
    assert_eq!(at(&y2, &view, 5.0, 2.0), palette::GRAPH_DIM);
}

#[test]
fn an_unknown_robot_selects_nothing() {
    let layout = two_curves();
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let plain = r.frame(&empty_snapshot());
    for id in ["ACME/R-99", ""] {
        assert_eq!(
            differing(&plain, &r.frame_selected(&empty_snapshot(), Some(id))),
            0,
            "{id}"
        );
    }
    // a recording without the robots' types (an older one)
    let mut old = two_curves();
    old.robots.clear();
    let r = Renderer::new(&old, 0, W, H).unwrap();
    let plain = r.frame(&empty_snapshot());
    let selected = r.frame_selected(&empty_snapshot(), Some("ACME/R-1"));
    assert_eq!(differing(&plain, &selected), 0);
}

#[test]
fn nodes_stay_whole_under_the_strong_curve() {
    let layout = two_curves();
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let view = *r.view();
    let frame = r.frame_selected(&empty_snapshot(), Some("ACME/R-2"));
    // the strong straight line ends in the middle of the nodes; their white
    // centres are still there
    assert_eq!(at(&frame, &view, 0.0, 0.0), palette::NODE_CENTRE);
    assert_eq!(at(&frame, &view, 10.0, 0.0), palette::NODE_CENTRE);
}

#[test]
fn an_edge_with_one_curve_is_a_normal_edge_and_selection_changes_nothing() {
    let mut layout = two_curves();
    // both types follow the arc: one distinct curve
    let arc = layout.edges[0].paths[0].points.clone();
    layout.edges[0].paths[1].points = arc;
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let view = *r.view();
    let plain = r.frame(&empty_snapshot());
    assert_eq!(
        at(&plain, &view, 5.0, 2.0),
        palette::GRAPH,
        "normal, not dimmed"
    );
    let selected = r.frame_selected(&empty_snapshot(), Some("ACME/R-1"));
    assert_eq!(differing(&plain, &selected), 0);
}

#[test]
fn a_curved_edge_has_an_arrowhead_at_its_end_along_the_curve() {
    // one curve, straight and horizontal: the same arrowhead as a one-way edge
    let mut layout = two_curves();
    layout.edges[0].paths = vec![EdgePath {
        types: vec!["ACME/X1".into(), "ACME/Y2".into()],
        points: vec![[0.0, 0.0], [10.0, 0.0]],
    }];
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let view = *r.view();
    let frame = r.frame(&empty_snapshot());
    // just beside the line, where only the arrowhead reaches (tip at 9.875)
    assert_eq!(at(&frame, &view, 9.7, 0.055), palette::GRAPH);
    assert_eq!(at(&frame, &view, 5.0, 0.055), palette::BACKGROUND);
}

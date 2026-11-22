//! In a big hall drawn small, roads, nodes and arrowheads stay visible.

use botsim_export::palette;
use botsim_export::{Renderer, View};
use botsim_record::{EdgeRecord, LayoutLine, NodeRecord};
use tiny_skia::Pixmap;

/// Two nodes 200 m apart with a one-way edge between them: at 1920 pixels that is
/// 8.6 pixels per metre, so the road (0.08 m), a node (0.25 m) and an arrowhead
/// (0.35 m) would be about 0.7, 2 and 3 pixels.
fn hall() -> LayoutLine {
    let node = |id: &str, x: f64| NodeRecord {
        id: id.into(),
        map: 0,
        x,
        y: 0.0,
    };
    LayoutLine {
        version: 1,
        maps: vec!["M".into()],
        nodes: vec![node("A", 0.0), node("B", 200.0)],
        edges: vec![EdgeRecord {
            id: "A-B".into(),
            start: 0,
            end: 1,
            length: 200.0,
            map_transition: false,
            paths: vec![],
        }],
        geometry: vec![],
        robots: vec![],
    }
}

/// How many pixels of a column are not the background.
fn painted_in_column(p: &Pixmap, x: u32) -> usize {
    (0..p.height())
        .filter(|&y| {
            let c = p.pixel(x, y).unwrap();
            [c.red(), c.green(), c.blue()] != palette::BACKGROUND
        })
        .count()
}

#[test]
fn a_road_in_a_big_hall_is_at_least_two_pixels_wide() {
    let layout = hall();
    let r = Renderer::new(&layout, 0, 1920, 1080).unwrap();
    let view = *r.view();
    assert!(view.scale < 10.0, "a big hall: {} px/m", view.scale);
    let frame = r.empty_frame();
    let (mid, _) = view.to_px(100.0, 0.0);
    let painted = painted_in_column(&frame, mid as u32);
    assert!(
        (2..=4).contains(&painted),
        "the road is {painted} pixels wide"
    );
}

#[test]
fn a_node_in_a_big_hall_is_at_least_six_pixels_across() {
    let layout = hall();
    let r = Renderer::new(&layout, 0, 1920, 1080).unwrap();
    let view = *r.view();
    let frame = r.empty_frame();
    let (nx, ny) = view.to_px(200.0, 0.0);
    let at = |dx: f32, dy: f32| {
        let c = frame.pixel((nx + dx) as u32, (ny + dy) as u32).unwrap();
        [c.red(), c.green(), c.blue()]
    };
    // above the road, 2.5 pixels from the middle: the rim; 5 pixels: nothing
    assert_ne!(at(0.0, -2.5), palette::BACKGROUND, "the rim");
    assert_eq!(at(0.0, -5.0), palette::BACKGROUND);
    // the centre of the node is lighter
    assert_eq!(at(0.0, 0.0), palette::NODE_CENTRE);
}

#[test]
fn an_arrowhead_in_a_big_hall_is_wider_than_the_road() {
    let layout = hall();
    let r = Renderer::new(&layout, 0, 1920, 1080).unwrap();
    let view = *r.view();
    let frame = r.empty_frame();
    // a few pixels before the node's rim: the arrowhead is at least 7 pixels long
    let (nx, _) = view.to_px(200.0, 0.0);
    let painted = painted_in_column(&frame, (nx - 8.0) as u32);
    assert!(painted >= 4, "the arrowhead is {painted} pixels wide there");
}

#[test]
fn a_small_hall_is_drawn_as_before() {
    // the sizes in metres win when they are bigger: a 20 m hall at 1920 pixels
    let mut layout = hall();
    layout.nodes[1].x = 20.0;
    let r = Renderer::new(&layout, 0, 1920, 1080).unwrap();
    let view = View::fit(&layout, 0, 1920, 1080).unwrap();
    assert!(view.scale > 80.0);
    let (mid, _) = view.to_px(10.0, 0.0);
    let painted = painted_in_column(&r.empty_frame(), mid as u32);
    // 0.08 m at ~86 px/m is about 7 pixels
    assert!((6..=9).contains(&painted), "{painted}");
}

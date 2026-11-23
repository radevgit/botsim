use botsim_export::palette::{self, Rgb};
use botsim_export::{Renderer, View, ViewError};
use botsim_record::{
    EdgeRecord, GeometryFile, GeometryKind, LayoutLine, NodeRecord, RobotRecord, RouteRecord,
    ShapeRecord, SnapshotLine, StatusRecord,
};
use tiny_skia::Pixmap;

const W: u32 = 1920;
const H: u32 = 1080;

fn node(id: &str, map: usize, x: f64, y: f64) -> NodeRecord {
    NodeRecord {
        id: id.into(),
        map,
        x,
        y,
    }
}

fn edge(id: &str, start: usize, end: usize) -> EdgeRecord {
    EdgeRecord {
        id: id.into(),
        start,
        end,
        length: 10.0,
        map_transition: false,
        paths: vec![],
    }
}

/// The corridor A - B - C of the corridor_deadlock scenario, two-way edges, with
/// its geometry; a second, empty map "N".
fn corridor() -> LayoutLine {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/corridor_deadlock.geometry.json"
    );
    let geometry = GeometryFile::load(std::path::Path::new(path)).unwrap();
    LayoutLine {
        version: 1,
        maps: vec!["M".into(), "N".into()],
        nodes: vec![
            node("A", 0, 0.0, 0.0),
            node("B", 0, 10.0, 0.0),
            node("C", 0, 20.0, 0.0),
        ],
        edges: vec![
            edge("A-B", 0, 1),
            edge("B-A", 1, 0),
            edge("B-C", 1, 2),
            edge("C-B", 2, 1),
        ],
        geometry: Vec::new(),
        robots: vec![],
    }
    .with_geometry(&geometry)
    .unwrap()
}

fn robot(map: usize, x: f64, y: f64, heading: f64, status: StatusRecord) -> RobotRecord {
    RobotRecord {
        id: "ACME/R-1".into(),
        map,
        x,
        y,
        heading,
        shape: ShapeRecord::Rect([0.0, 0.0, 0.6, 0.4]),
        status,
        speed: 0.0,
        order: String::new(),
        last_node: None,
        released: RouteRecord::default(),
        planned: RouteRecord::default(),
    }
}

fn snapshot(robots: Vec<RobotRecord>) -> SnapshotLine {
    SnapshotLine {
        tick: 1,
        time: 0.05,
        robots,
        reservations: vec![],
        blocking: vec![],
        overlapping: vec![],
        deadlocks: vec![],
        lifts: vec![],
        doors: vec![],
    }
}

/// The colour of the pixel at a world point.
fn at(pixmap: &Pixmap, view: &View, x: f64, y: f64) -> Rgb {
    let (px, py) = view.to_px(x, y);
    let p = pixmap.pixel(px as u32, py as u32).unwrap();
    assert_eq!(p.alpha(), 255);
    [p.red(), p.green(), p.blue()]
}

#[test]
fn the_view_fits_the_floor_with_a_margin_and_one_scale() {
    let layout = corridor();
    let view = View::fit(&layout, 0, W, H).unwrap();
    // x -3 … 23 (26 m) is the limit, 90% of the width
    assert!((view.scale - 1728.0 / 26.0).abs() < 1e-9);
    assert_eq!(view.centre, (10.0, 0.0));
    assert_eq!(view.to_px(10.0, 0.0), (960.0, 540.0));
    // y points up in the world, down in the image
    assert!(view.to_px(10.0, 1.0).1 < view.to_px(10.0, 0.0).1);
    // everything is inside the image
    let (left, _) = view.to_px(-3.0, 0.0);
    let (right, _) = view.to_px(23.0, 0.0);
    assert!(left >= 96.0 - 0.01 && right <= 1824.0 + 0.01);
}

#[test]
fn a_pixel_maps_back_to_the_world_point() {
    let view = View::fit(&corridor(), 0, W, H).unwrap();
    for (x, y) in [(10.0, 0.0), (-3.0, 4.5), (23.0, -4.5), (0.37, 1.91)] {
        let (px, py) = view.to_px(x, y);
        let (wx, wy) = view.to_world(px, py);
        // a pixel is 1 / scale metres; f32 pixels lose a little
        assert!(
            (wx - x).abs() < 1e-3 && (wy - y).abs() < 1e-3,
            "{x},{y} -> {wx},{wy}"
        );
    }
    assert_eq!(view.to_world(960.0, 540.0), (10.0, 0.0), "the middle");
}

#[test]
fn zooming_keeps_the_point_under_the_pointer_and_a_drag_moves_the_picture() {
    let view = View::fit(&corridor(), 0, W, H).unwrap();
    let anchor = (500.0_f32, 300.0_f32);
    let (wx, wy) = view.to_world(anchor.0, anchor.1);
    let zoomed = view.zoomed(4.0, anchor);
    assert!((zoomed.scale - 4.0 * view.scale).abs() < 1e-9);
    let (px, py) = zoomed.to_px(wx, wy);
    assert!((px - anchor.0).abs() < 1e-3 && (py - anchor.1).abs() < 1e-3);
    // zooming out again is the same view
    let back = zoomed.zoomed(0.25, anchor);
    assert!((back.centre.0 - view.centre.0).abs() < 1e-9);
    assert!((back.centre.1 - view.centre.1).abs() < 1e-9);
    // a drag to the right and down moves what is shown with it
    let moved = zoomed.panned(30.0, -20.0);
    let (qx, qy) = moved.to_px(wx, wy);
    assert!((qx - (px + 30.0)).abs() < 1e-3 && (qy - (py - 20.0)).abs() < 1e-3);
}

#[test]
fn a_renderer_can_be_made_for_a_zoomed_view() {
    let layout = corridor();
    let view = View::fit(&layout, 0, W, H)
        .unwrap()
        .zoomed(5.0, (960.0, 540.0));
    let renderer = Renderer::with_view(&layout, 0, view).unwrap();
    assert_eq!(renderer.view(), &view);
    let frame = renderer.empty_frame();
    assert_eq!((frame.width(), frame.height()), (W, H));
    assert!(matches!(
        Renderer::with_view(&layout, 7, view),
        Err(ViewError::NoSuchMap(7))
    ));
}

#[test]
fn the_view_reports_what_cannot_be_shown() {
    let layout = corridor();
    assert_eq!(View::fit(&layout, 5, W, H), Err(ViewError::NoSuchMap(5)));
    assert_eq!(View::fit(&layout, 1, W, H), Err(ViewError::EmptyMap(1)));
    assert_eq!(View::fit(&layout, 0, 0, H), Err(ViewError::NoPixels));
}

#[test]
fn geometry_graph_and_robots_have_their_colours() {
    let layout = corridor();
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let view = *r.view();
    let frame = r.frame(&snapshot(vec![robot(
        0,
        10.0,
        0.0,
        0.0,
        StatusRecord::Driving,
    )]));
    assert_eq!((frame.width(), frame.height()), (W, H));

    assert_eq!(at(&frame, &view, 10.0, 7.0), palette::BACKGROUND);
    assert_eq!(
        at(&frame, &view, 10.0, 1.5),
        palette::kind(GeometryKind::Wall).line
    );
    assert_eq!(
        at(&frame, &view, 5.0, 3.5),
        palette::kind(GeometryKind::Rack).fill.unwrap()
    );
    assert_eq!(
        at(&frame, &view, 10.0, -3.5),
        palette::kind(GeometryKind::Shelf).fill.unwrap()
    );
    assert_eq!(
        at(&frame, &view, 0.7, 0.7),
        palette::kind(GeometryKind::Station).fill.unwrap()
    );
    // a track edge between nodes, a node centre
    assert_eq!(at(&frame, &view, 15.0, 0.0), palette::GRAPH);
    assert_eq!(at(&frame, &view, 0.0, 0.0), palette::NODE_CENTRE);
    // the robot: body in the status colour, heading line white, not beyond its length
    assert_eq!(
        at(&frame, &view, 9.7, 0.25),
        palette::status(StatusRecord::Driving)
    );
    assert_eq!(at(&frame, &view, 10.3, 0.0), [255, 255, 255]);
    assert_eq!(at(&frame, &view, 10.8, 0.0), palette::GRAPH);
}

#[test]
fn a_robot_turns_with_its_heading() {
    let layout = corridor();
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let view = *r.view();
    // heading up (+y): 1.2 m long in y, 0.8 m wide in x
    let frame = r.frame(&snapshot(vec![robot(
        0,
        10.0,
        0.0,
        std::f64::consts::FRAC_PI_2,
        StatusRecord::Obstructed,
    )]));
    let red = palette::status(StatusRecord::Obstructed);
    assert_eq!(at(&frame, &view, 10.3, 0.5), red);
    // beside the body (it is only 0.4 m to each side): the track edge shows
    assert_eq!(at(&frame, &view, 10.5, 0.0), palette::GRAPH);
    assert_eq!(
        at(&frame, &view, 10.0, 0.5),
        [255, 255, 255],
        "heading line up"
    );
}

#[test]
fn robots_on_other_floors_or_in_a_lift_are_not_drawn() {
    let layout = corridor();
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let frame = r.frame(&snapshot(vec![
        robot(1, 10.0, 0.0, 0.0, StatusRecord::Driving),
        robot(0, 10.0, 0.0, 0.0, StatusRecord::RidingLift),
    ]));
    assert_eq!(frame.data(), r.empty_frame().data());
    // and a robot that is drawn changes the picture
    let drawn = r.frame(&snapshot(vec![robot(
        0,
        10.0,
        0.0,
        0.0,
        StatusRecord::Idle,
    )]));
    assert_ne!(drawn.data(), r.empty_frame().data());
}

#[test]
fn a_one_way_edge_has_an_arrowhead_and_a_two_way_pair_has_none() {
    let mut one_way = LayoutLine {
        version: 1,
        maps: vec!["M".into()],
        nodes: vec![node("N1", 0, 0.0, 0.0), node("N2", 0, 10.0, 0.0)],
        edges: vec![edge("N1-N2", 0, 1)],
        geometry: vec![],
        robots: vec![],
    };
    let r = Renderer::new(&one_way, 0, W, H).unwrap();
    let a = r.empty_frame();
    // just beside the line, where only the arrowhead reaches
    assert_eq!(at(&a, r.view(), 9.7, 0.055), palette::GRAPH);

    one_way.edges.push(edge("N2-N1", 1, 0));
    let r = Renderer::new(&one_way, 0, W, H).unwrap();
    assert_eq!(
        at(&r.empty_frame(), r.view(), 9.7, 0.055),
        palette::BACKGROUND
    );
}

/// Two robots facing each other, 4 m apart, both waiting.
fn facing() -> Vec<RobotRecord> {
    vec![
        robot(0, 5.0, 0.0, 0.0, StatusRecord::WaitingForRelease),
        robot(
            0,
            9.0,
            0.0,
            std::f64::consts::PI,
            StatusRecord::WaitingForRelease,
        ),
    ]
}

#[test]
fn a_blocking_arrow_runs_from_the_waiter_to_the_blocker() {
    let layout = corridor();
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let view = *r.view();
    let mut snap = snapshot(facing());
    let without = r.frame(&snap);
    assert_eq!(
        at(&without, &view, 7.0, 0.0),
        palette::GRAPH,
        "only the track"
    );

    snap.blocking = vec![[0, 1]];
    let with = r.frame(&snap);
    assert_eq!(at(&with, &view, 7.0, 0.0), palette::BLOCKING);
    // the arrowhead ends at the blocker's edge (8.4) and widens backwards: 0.35 m
    // behind the tip it is 0.25 m wide, so 0.07 m off the line it is there at 8.15
    assert_eq!(at(&without, &view, 8.15, 0.07), palette::BACKGROUND);
    assert_eq!(at(&with, &view, 8.15, 0.07), palette::BLOCKING);
    assert_eq!(
        at(&with, &view, 8.35, 0.07),
        palette::BACKGROUND,
        "narrow near the tip"
    );
    // nothing is drawn over the bodies
    assert_ne!(at(&with, &view, 5.5, 0.25), palette::BLOCKING);
    assert_ne!(at(&with, &view, 8.5, 0.25), palette::BLOCKING);
}

#[test]
fn the_arrow_starts_and_ends_at_the_edges_of_turned_bodies() {
    let layout = corridor();
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let view = *r.view();
    // the waiter points up: towards the blocker its body is only 0.4 m deep
    let mut snap = snapshot(vec![
        robot(0, 5.0, 0.0, std::f64::consts::FRAC_PI_2, StatusRecord::Idle),
        robot(0, 9.0, 0.0, 0.0, StatusRecord::Idle),
    ]);
    snap.blocking = vec![[0, 1]];
    let frame = r.frame(&snap);
    assert_eq!(at(&frame, &view, 5.55, 0.0), palette::BLOCKING);
    assert_ne!(at(&frame, &view, 5.3, 0.0), palette::BLOCKING);
}

#[test]
fn no_arrow_when_the_bodies_touch_or_a_robot_is_not_on_the_floor() {
    let layout = corridor();
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let plain = |robots: Vec<RobotRecord>| r.frame(&snapshot(robots));
    let with_arrow = |robots: Vec<RobotRecord>, blocking: Vec<[usize; 2]>| {
        let mut s = snapshot(robots);
        s.blocking = blocking;
        r.frame(&s)
    };
    // 1 m apart: the 0.6 m long bodies overlap
    let close = vec![
        robot(0, 5.0, 0.0, 0.0, StatusRecord::Idle),
        robot(0, 6.0, 0.0, 0.0, StatusRecord::Idle),
    ];
    assert_eq!(
        with_arrow(close.clone(), vec![[0, 1]]).data(),
        plain(close).data()
    );
    // the blocker is on another floor; a number that is no robot
    let mut away = facing();
    away[1].map = 1;
    assert_eq!(
        with_arrow(away.clone(), vec![[0, 1]]).data(),
        plain(away).data()
    );
    assert_eq!(
        with_arrow(facing(), vec![[0, 7]]).data(),
        plain(facing()).data()
    );
}

#[test]
fn rings_mark_a_deadlock_and_an_overlap_and_both_show_when_they_meet() {
    let layout = corridor();
    let r = Renderer::new(&layout, 0, W, H).unwrap();
    let view = *r.view();
    let robots = vec![
        robot(0, 5.0, 0.0, 0.0, StatusRecord::WaitingForRelease),
        robot(0, 15.0, 0.0, 0.0, StatusRecord::WaitingForRelease),
    ];
    // the front of robot 0 is at x = 5.6; the deadlock ring is 0.15 … 0.27 m
    // beyond it, the overlap ring 0.32 … 0.44 m
    let (deadlock_x, overlap_x) = (5.8, 5.98);
    let plain = r.frame(&snapshot(robots.clone()));
    assert_eq!(at(&plain, &view, deadlock_x, 0.2), palette::BACKGROUND);
    assert_eq!(at(&plain, &view, overlap_x, 0.2), palette::BACKGROUND);

    let mut snap = snapshot(robots);
    snap.deadlocks = vec![vec![0, 1]];
    let frame = r.frame(&snap);
    assert_eq!(at(&frame, &view, deadlock_x, 0.2), palette::DEADLOCK);
    assert_eq!(
        at(&frame, &view, 15.8, 0.2),
        palette::DEADLOCK,
        "robot 1 too"
    );
    assert_eq!(at(&frame, &view, overlap_x, 0.2), palette::BACKGROUND);

    snap.deadlocks = vec![];
    snap.overlapping = vec![[0, 1]];
    let frame = r.frame(&snap);
    assert_eq!(at(&frame, &view, overlap_x, 0.2), palette::OVERLAP);
    assert_eq!(at(&frame, &view, deadlock_x, 0.2), palette::BACKGROUND);

    snap.deadlocks = vec![vec![0]];
    let frame = r.frame(&snap);
    assert_eq!(at(&frame, &view, deadlock_x, 0.2), palette::DEADLOCK);
    assert_eq!(at(&frame, &view, overlap_x, 0.2), palette::OVERLAP);
}

#[test]
fn the_same_input_gives_the_same_pixels_and_a_png() {
    let layout = corridor();
    let snap = snapshot(vec![
        robot(0, 3.0, 0.0, 0.0, StatusRecord::Driving),
        robot(
            0,
            17.0,
            0.0,
            std::f64::consts::PI,
            StatusRecord::WaitingForRelease,
        ),
    ]);
    let a = Renderer::new(&layout, 0, W, H).unwrap().frame(&snap);
    let b = Renderer::new(&layout, 0, W, H).unwrap().frame(&snap);
    assert_eq!(a.data(), b.data());

    let path = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("frame.png");
    a.save_png(&path).unwrap();
    let bytes = std::fs::read(&path).unwrap();
    assert_eq!(&bytes[..8], b"\x89PNG\r\n\x1a\n");
}

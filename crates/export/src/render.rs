//! Drawing one frame: geometry, track graph and robots of one floor.

use botsim_record::{
    EdgePath, EdgeRecord, GeometryKind, LayoutLine, Outline, RobotRecord, ShapeRecord,
    SnapshotLine, StatusRecord,
};
use tiny_skia::{FillRule, LineCap, LineJoin, Paint, Path, PathBuilder, Pixmap, Stroke, Transform};

use crate::clock::{Clock, NoFont};
use crate::palette::{self, Rgb};
use crate::view::{View, ViewError};

/// Width of a wall line, metres.
const WALL_WIDTH: f64 = 0.2;
/// Width of the outline of a polygon, metres.
const OUTLINE_WIDTH: f64 = 0.06;
/// Width of a track edge, metres.
const EDGE_WIDTH: f64 = 0.08;
/// Diameter of a node, metres; the white centre is half of it.
const NODE_DIAMETER: f64 = 0.25;
/// Arrowhead of a one-way edge, metres.
const ARROW_LENGTH: f64 = 0.35;
const ARROW_WIDTH: f64 = 0.25;
/// Outline of a robot, metres.
const ROBOT_OUTLINE: f64 = 0.04;
/// Lines are never thinner than this many pixels.
const MIN_LINE_PX: f32 = 1.0;
/// Roads, nodes and arrowheads are never smaller than this (pixels), so that they
/// stay visible when a big hall is drawn small. (In a small layout they are bigger
/// than this anyway.)
const MIN_ROAD_PX: f32 = 2.0;
const MIN_NODE_RADIUS_PX: f32 = 3.0;
const MIN_ARROW_LENGTH_PX: f64 = 7.0;
/// Width of a blocking arrow, metres.
const BLOCKING_WIDTH: f64 = 0.08;
/// Width of the ring around a robot in a deadlock or an overlap, metres.
const RING_WIDTH: f64 = 0.12;
/// Space between a robot's body and the inside of its deadlock ring, metres.
const DEADLOCK_RING_GAP: f64 = 0.15;
/// The overlap ring lies outside the deadlock ring, so a robot that is in both
/// shows both: the same gap, the ring width and a little more.
const OVERLAP_RING_GAP: f64 = DEADLOCK_RING_GAP + RING_WIDTH + 0.05;
/// The ring of a wait with an unknown blocker lies outside both, so all three show.
const WAIT_RING_GAP: f64 = OVERLAP_RING_GAP + RING_WIDTH + 0.05;

/// Draws the frames of one floor of a recording. The geometry and the track
/// graph never change, so they are drawn once; every frame starts from that
/// picture and adds the robots.
pub struct Renderer {
    layout: LayoutLine,
    view: View,
    map: usize,
    base: Pixmap,
    /// The font of the text in the frames (the clock, the name); `None` while no
    /// text is wanted.
    clock: Option<Clock>,
    show_clock: bool,
    show_brand: bool,
    /// Draw the waits instead of the blocking arrows (see [`Renderer::set_waits`]).
    show_waits: bool,
    ///Where the right edge of the name `botsim` is (pixels from the left): a few
    /// metres inside the right wall of the building.
    brand_right: f32,
    /// Where the left edge of the clock is (pixels from the left): a few metres
    /// inside the left wall of the building.
    clock_left: f32,
}

/// How far inside the right wall of the building the name ends, and inside the left
/// wall the clock begins (metres), at most 5% of the width of the image.
const BRAND_INSET_M: f64 = 3.0;

impl Renderer {
    /// A renderer for floor `map` (an index into the layout's maps), with the
    /// view fitted once to that floor.
    pub fn new(
        layout: &LayoutLine,
        map: usize,
        width: u32,
        height: u32,
    ) -> Result<Renderer, ViewError> {
        let view = View::fit(layout, map, width, height)?;
        Renderer::with_view(layout, map, view)
    }

    /// A renderer for floor `map` with a given view (the image has the size of the
    /// view): for a picture that is zoomed in on a part of the floor.
    pub fn with_view(layout: &LayoutLine, map: usize, view: View) -> Result<Renderer, ViewError> {
        if map >= layout.maps.len() {
            return Err(ViewError::NoSuchMap(map));
        }
        let (width, height) = (view.width, view.height);
        let mut base = Pixmap::new(width, height).ok_or(ViewError::NoPixels)?;
        base.fill(color(palette::BACKGROUND));
        draw_geometry(&mut base, layout, &view, map);
        draw_graph(&mut base, layout, &view, map);
        // the name ends a few metres inside the right wall of the building (and never
        // outside the image)
        let right_wall = View::content_bounds(layout, map).map_or(width as f64, |b| {
            let inset = (BRAND_INSET_M * view.scale).min(0.05 * width as f64);
            view.to_px(b.2, 0.0).0 as f64 - inset
        });
        // (zoomed in, the walls can be out of the image: the text stays in it)
        let brand_right = right_wall
            .min(width as f64 - 0.03 * height as f64)
            .max(width as f64 / 2.0) as f32;
        // the clock begins a few metres inside the left wall
        let clock_left = View::content_bounds(layout, map)
            .map_or(0.0, |b| {
                let inset = (BRAND_INSET_M * view.scale).min(0.05 * width as f64);
                view.to_px(b.0, 0.0).0 as f64 + inset
            })
            .clamp(0.0, width as f64 / 2.0) as f32;
        Ok(Renderer {
            layout: layout.clone(),
            view,
            map,
            base,
            clock: None,
            show_clock: false,
            show_brand: false,
            show_waits: false,
            brand_right,
            clock_left,
        })
    }

    /// The plate of the clock `(x, y, width, height)` in pixels for a time, if the
    /// clock is on (see [`Renderer::with_clock`]).
    pub fn clock_area(&self, seconds: f64) -> Option<(f32, f32, f32, f32)> {
        let text = self.clock.as_ref().filter(|_| self.show_clock)?;
        Some(text.area(self.clock_left, self.view.height, seconds))
    }

    /// The plate of the name `botsim` `(x, y, width, height)` in pixels, if the name
    /// is on (see [`Renderer::with_brand`]).
    pub fn brand_area(&self) -> Option<(f32, f32, f32, f32)> {
        let text = self.clock.as_ref().filter(|_| self.show_brand)?;
        Some(text.brand_area(self.brand_right, self.view.height))
    }

    /// Put the simulated time in a corner of every [`Renderer::frame`] (not of
    /// [`Renderer::empty_frame`]).
    /// Fails if the system has no usable font for the label.
    pub fn with_clock(mut self, on: bool) -> Result<Renderer, NoFont> {
        self.show_clock = on;
        self.load_font()?;
        Ok(self)
    }

    /// Put the name `botsim` at the bottom of every [`Renderer::frame`] (not of
    /// [`Renderer::empty_frame`]), a few metres inside the right wall of the
    /// building. Fails if the system has no usable font.
    pub fn with_brand(mut self, on: bool) -> Result<Renderer, NoFont> {
        self.show_brand = on;
        self.load_font()?;
        Ok(self)
    }

    /// The "waiting caused by others" overlay: when on, every [`Renderer::frame`] shows,
    /// **instead of** the red arrows of `blocking`, an arrow from each waiting robot to
    /// its blocker in the colour of the reason, and a ring in that colour round a
    /// waiting robot whose blocker is unknown. Off at first.
    pub fn set_waits(&mut self, on: bool) {
        self.show_waits = on;
    }

    /// The font is looked for once, and only if some text is wanted.
    fn load_font(&mut self) -> Result<(), NoFont> {
        if !(self.show_clock || self.show_brand) {
            self.clock = None;
        } else if self.clock.is_none() {
            self.clock = Some(Clock::new()?);
        }
        Ok(())
    }

    pub fn view(&self) -> &View {
        &self.view
    }

    pub fn layout(&self) -> &LayoutLine {
        &self.layout
    }

    /// The picture of a snapshot: the floor and the robots that are on it, then
    /// the rings of deadlocks and overlaps and the blocking arrows above them.
    pub fn frame(&self, snapshot: &SnapshotLine) -> Pixmap {
        self.frame_selected(snapshot, None)
    }

    /// Like [`Renderer::frame`], with a robot selected (by its id): on edges where
    /// the robot types follow different curves, the curve of the selected robot's
    /// type is drawn strong over the dimmed ones. A robot whose type is not in the
    /// layout line, or an id that is no robot, selects nothing.
    pub fn frame_selected(&self, snapshot: &SnapshotLine, selected: Option<&str>) -> Pixmap {
        let mut pixmap = self.base.clone();
        let robot_type = selected.and_then(|id| {
            self.layout
                .robots
                .iter()
                .find(|r| r.id == id)
                .map(|r| r.robot_type.as_str())
        });
        if let Some(robot_type) = robot_type {
            draw_selected_curves(&mut pixmap, &self.layout, &self.view, self.map, robot_type);
        }
        // a robot in a lift cabin is not on any floor (its pose is where it got in)
        let on_floor =
            |robot: &RobotRecord| robot.map == self.map && robot.status != StatusRecord::RidingLift;
        for robot in snapshot.robots.iter().filter(|r| on_floor(r)) {
            draw_robot(&mut pixmap, &self.view, robot);
        }
        // the lists of the snapshot refer to robots by position; one that is not
        // on this floor (or does not exist) is left out
        let robot_on_floor = |i: usize| snapshot.robots.get(i).filter(|r| on_floor(r));
        for &i in snapshot.deadlocks.iter().flatten() {
            if let Some(r) = robot_on_floor(i) {
                draw_ring(
                    &mut pixmap,
                    &self.view,
                    r,
                    DEADLOCK_RING_GAP,
                    palette::DEADLOCK,
                );
            }
        }
        for &i in snapshot.overlapping.iter().flatten() {
            if let Some(r) = robot_on_floor(i) {
                draw_ring(
                    &mut pixmap,
                    &self.view,
                    r,
                    OVERLAP_RING_GAP,
                    palette::OVERLAP,
                );
            }
        }
        if self.show_waits {
            for waiter in snapshot.robots.iter().filter(|r| on_floor(r)) {
                let Some(wait) = waiter.waiting else {
                    continue;
                };
                let colour = palette::wait(wait.reason);
                match wait.waiting_for {
                    None => draw_ring(&mut pixmap, &self.view, waiter, WAIT_RING_GAP, colour),
                    // (a blocker on another floor, or that does not exist: nothing)
                    Some(i) => {
                        if let Some(b) = robot_on_floor(i) {
                            draw_blocking(&mut pixmap, &self.view, waiter, b, colour);
                        }
                    }
                }
            }
        } else {
            for &[waiter, blocker] in &snapshot.blocking {
                if let (Some(w), Some(b)) = (robot_on_floor(waiter), robot_on_floor(blocker)) {
                    draw_blocking(&mut pixmap, &self.view, w, b, palette::BLOCKING);
                }
            }
        }
        if let Some(text) = &self.clock {
            if self.show_clock {
                text.draw(&mut pixmap, self.clock_left, snapshot.time);
            }
            if self.show_brand {
                text.draw_brand(&mut pixmap, self.brand_right);
            }
        }
        pixmap
    }

    /// The floor without robots.
    pub fn empty_frame(&self) -> Pixmap {
        self.base.clone()
    }
}

fn color(c: Rgb) -> tiny_skia::Color {
    tiny_skia::Color::from_rgba8(c[0], c[1], c[2], 255)
}

fn paint(c: Rgb) -> Paint<'static> {
    let mut p = Paint::default();
    p.set_color(color(c));
    p.anti_alias = true;
    p
}

fn stroke(width_px: f32) -> Stroke {
    Stroke {
        width: width_px.max(MIN_LINE_PX),
        line_cap: LineCap::Round,
        line_join: LineJoin::Round,
        ..Stroke::default()
    }
}

fn fill(pixmap: &mut Pixmap, path: &Path, c: Rgb) {
    pixmap.fill_path(
        path,
        &paint(c),
        FillRule::Winding,
        Transform::identity(),
        None,
    );
}

fn line(pixmap: &mut Pixmap, path: &Path, c: Rgb, width_px: f32) {
    pixmap.stroke_path(
        path,
        &paint(c),
        &stroke(width_px),
        Transform::identity(),
        None,
    );
}

/// An open or closed path through world points.
fn polyline(view: &View, points: &[(f64, f64)], closed: bool) -> Option<Path> {
    let mut pb = PathBuilder::new();
    for (i, &(x, y)) in points.iter().enumerate() {
        let (px, py) = view.to_px(x, y);
        if i == 0 {
            pb.move_to(px, py);
        } else {
            pb.line_to(px, py);
        }
    }
    if closed {
        pb.close();
    }
    pb.finish()
}

fn draw_geometry(pixmap: &mut Pixmap, layout: &LayoutLine, view: &View, map: usize) {
    for shape in layout.geometry.iter().filter(|s| s.map == map) {
        let style = palette::kind(shape.kind);
        let points: Vec<(f64, f64)> = shape.vertices.iter().map(|v| (v.x, v.y)).collect();
        match shape.shape {
            Outline::Polygon => {
                let Some(path) = polyline(view, &points, true) else {
                    continue;
                };
                if let Some(fill_colour) = style.fill {
                    fill(pixmap, &path, fill_colour);
                }
                line(pixmap, &path, style.line, view.px(OUTLINE_WIDTH));
            }
            Outline::Polyline => {
                let Some(path) = polyline(view, &points, false) else {
                    continue;
                };
                let width = if shape.kind == GeometryKind::Wall {
                    WALL_WIDTH
                } else {
                    OUTLINE_WIDTH
                };
                line(pixmap, &path, style.line, view.px(width));
            }
        }
    }
}

/// Track edges (a two-way pair as one line, a one-way edge with an arrowhead),
/// then the nodes. Edges between maps are not drawn.
fn draw_graph(pixmap: &mut Pixmap, layout: &LayoutLine, view: &View, map: usize) {
    draw_edges(pixmap, layout, view, map);
    draw_nodes(pixmap, layout, view, map);
}

/// The distinct curves of an edge (several robot types can follow the same one).
fn distinct_paths(edge: &EdgeRecord) -> Vec<&EdgePath> {
    let mut distinct: Vec<&EdgePath> = Vec::new();
    for path in &edge.paths {
        if !distinct.iter().any(|d| d.points == path.points) {
            distinct.push(path);
        }
    }
    distinct
}

/// A curve as a line with an arrowhead along its last piece.
fn draw_edge_path(pixmap: &mut Pixmap, view: &View, points: &[[f64; 2]], c: Rgb, width: f64) {
    let points: Vec<(f64, f64)> = points.iter().map(|p| (p[0], p[1])).collect();
    if let Some(path) = polyline(view, &points, false) {
        // a wider line (the selected curve) keeps its proportion at the minimum
        let min_px = MIN_ROAD_PX * (width / EDGE_WIDTH) as f32;
        line(pixmap, &path, c, view.px(width).max(min_px));
    }
    if let [.., a, b] = points[..] {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let length = dx.hypot(dy);
        if length > 1e-9 {
            let u = (dx / length, dy / length);
            let rim = node_radius(view);
            let tip = (b.0 - u.0 * rim, b.1 - u.1 * rim);
            draw_arrow_head_in(pixmap, view, tip, u, c);
        }
    }
}

/// The radius of a node in metres: at least [`MIN_NODE_RADIUS_PX`] pixels.
fn node_radius(view: &View) -> f64 {
    (NODE_DIAMETER / 2.0).max(MIN_NODE_RADIUS_PX as f64 / view.scale)
}

/// An edge with curves: the one curve, or all of them dimmed if the types of
/// robots follow different ones.
fn draw_curved_edge(pixmap: &mut Pixmap, view: &View, edge: &EdgeRecord) {
    let distinct = distinct_paths(edge);
    let colour = if distinct.len() == 1 {
        palette::GRAPH
    } else {
        palette::GRAPH_DIM
    };
    for path in distinct {
        draw_edge_path(pixmap, view, &path.points, colour, EDGE_WIDTH);
    }
}

/// On the edges where the types of robots follow different curves: the curve of
/// this robot type, strong, over the dimmed ones.
fn draw_selected_curves(
    pixmap: &mut Pixmap,
    layout: &LayoutLine,
    view: &View,
    map: usize,
    robot_type: &str,
) {
    let mut drawn = false;
    for edge in &layout.edges {
        let on_floor =
            !edge.map_transition && layout.nodes.get(edge.start).is_some_and(|n| n.map == map);
        if !on_floor || distinct_paths(edge).len() < 2 {
            continue;
        }
        if let Some(path) = edge
            .paths
            .iter()
            .find(|p| p.types.iter().any(|t| t == robot_type))
        {
            draw_edge_path(
                pixmap,
                view,
                &path.points,
                palette::GRAPH_SELECTED,
                EDGE_WIDTH * 1.5,
            );
            drawn = true;
        }
    }
    // the strong lines end in the middle of the nodes: draw the nodes again (only
    // then: drawing them twice changes the anti-aliased rim)
    if drawn {
        draw_nodes(pixmap, layout, view, map);
    }
}

fn draw_edges(pixmap: &mut Pixmap, layout: &LayoutLine, view: &View, map: usize) {
    for (i, e) in layout.edges.iter().enumerate() {
        // an edge that names a node the recording does not have is left out
        let (Some(a), Some(b)) = (layout.nodes.get(e.start), layout.nodes.get(e.end)) else {
            continue;
        };
        if e.map_transition || a.map != map || b.map != map {
            continue;
        }
        if !e.paths.is_empty() {
            draw_curved_edge(pixmap, view, e);
            continue;
        }
        let reverse = layout
            .edges
            .iter()
            .position(|r| r.start == e.end && r.end == e.start && !r.map_transition);
        match reverse {
            // a two-way pair: drawn once, by the edge that comes first
            Some(j) if j < i => continue,
            Some(_) => {}
            None => draw_arrow_head(pixmap, view, (a.x, a.y), (b.x, b.y)),
        }
        if let Some(path) = polyline(view, &[(a.x, a.y), (b.x, b.y)], false) {
            line(
                pixmap,
                &path,
                palette::GRAPH,
                view.px(EDGE_WIDTH).max(MIN_ROAD_PX),
            );
        }
    }
}

fn draw_nodes(pixmap: &mut Pixmap, layout: &LayoutLine, view: &View, map: usize) {
    let rim = node_radius(view);
    for n in layout.nodes.iter().filter(|n| n.map == map) {
        let (x, y) = view.to_px(n.x, n.y);
        for (radius, c) in [(rim, palette::GRAPH), (rim / 2.0, palette::NODE_CENTRE)] {
            if let Some(path) = PathBuilder::from_circle(x, y, view.px(radius).max(1.0)) {
                fill(pixmap, &path, c);
            }
        }
    }
}

/// A triangle that ends at the target node's rim and points along the edge.
fn draw_arrow_head(pixmap: &mut Pixmap, view: &View, from: (f64, f64), to: (f64, f64)) {
    let (dx, dy) = (to.0 - from.0, to.1 - from.1);
    let length = dx.hypot(dy);
    if length < 1e-9 {
        return;
    }
    let u = (dx / length, dy / length);
    let rim = node_radius(view);
    let tip = (to.0 - u.0 * rim, to.1 - u.1 * rim);
    draw_arrow_head_in(pixmap, view, tip, u, palette::GRAPH);
}

/// A filled triangle with its tip at `tip`, pointing along the unit vector `u`; at
/// least [`MIN_ARROW_LENGTH_PX`] pixels long, in the same proportion.
fn draw_arrow_head_in(pixmap: &mut Pixmap, view: &View, tip: (f64, f64), u: (f64, f64), c: Rgb) {
    let length = ARROW_LENGTH.max(MIN_ARROW_LENGTH_PX / view.scale);
    let width = ARROW_WIDTH * length / ARROW_LENGTH;
    let base = (tip.0 - u.0 * length, tip.1 - u.1 * length);
    let (nx, ny) = (-u.1 * width / 2.0, u.0 * width / 2.0);
    let points = [tip, (base.0 + nx, base.1 + ny), (base.0 - nx, base.1 - ny)];
    if let Some(path) = polyline(view, &points, true) {
        fill(pixmap, &path, c);
    }
}

/// A ring around the body, `gap` metres outside it.
fn draw_ring(pixmap: &mut Pixmap, view: &View, robot: &RobotRecord, gap: f64, c: Rgb) {
    if let Some(path) = body_path(view, robot, gap + RING_WIDTH / 2.0) {
        line(pixmap, &path, c, view.px(RING_WIDTH));
    }
}

/// An arrow from the waiting robot to the one that blocks it: from the edge of
/// one body to the edge of the other, in the colour `c`. Nothing if the bodies touch.
fn draw_blocking(
    pixmap: &mut Pixmap,
    view: &View,
    waiter: &RobotRecord,
    blocker: &RobotRecord,
    c: Rgb,
) {
    let (from, to) = (body_centre(waiter), body_centre(blocker));
    let (dx, dy) = (to.0 - from.0, to.1 - from.1);
    let distance = dx.hypot(dy);
    if distance < 1e-9 {
        return;
    }
    let u = (dx / distance, dy / distance);
    let start = body_reach(waiter, u);
    let end = distance - body_reach(blocker, (-u.0, -u.1));
    if end <= start {
        return;
    }
    let tail = (from.0 + u.0 * start, from.1 + u.1 * start);
    let tip = (from.0 + u.0 * end, from.1 + u.1 * end);
    if let Some(path) = polyline(view, &[tail, tip], false) {
        line(pixmap, &path, c, view.px(BLOCKING_WIDTH));
    }
    draw_arrow_head_in(pixmap, view, tip, u, c);
}

/// A point given in the robot frame (x forward, y left), in the world.
fn to_world(robot: &RobotRecord, lx: f64, ly: f64) -> (f64, f64) {
    let (sin, cos) = robot.heading.sin_cos();
    (robot.x + lx * cos - ly * sin, robot.y + lx * sin + ly * cos)
}

/// The outline of a robot's body, grown by `margin` metres on every side.
fn body_path(view: &View, robot: &RobotRecord, margin: f64) -> Option<Path> {
    match robot.shape {
        ShapeRecord::Rect([cx, cy, half_length, half_width]) => {
            let (l, w) = (half_length + margin, half_width + margin);
            let corners = [
                to_world(robot, cx + l, cy + w),
                to_world(robot, cx - l, cy + w),
                to_world(robot, cx - l, cy - w),
                to_world(robot, cx + l, cy - w),
            ];
            polyline(view, &corners, true)
        }
        ShapeRecord::Circle([cx, cy, radius]) => {
            let (x, y) = to_world(robot, cx, cy);
            let (px, py) = view.to_px(x, y);
            PathBuilder::from_circle(px, py, view.px(radius + margin).max(1.0))
        }
    }
}

/// The middle of the body, in the world.
fn body_centre(robot: &RobotRecord) -> (f64, f64) {
    let (cx, cy) = match robot.shape {
        ShapeRecord::Rect([cx, cy, ..]) | ShapeRecord::Circle([cx, cy, _]) => (cx, cy),
    };
    to_world(robot, cx, cy)
}

/// How far the body's edge is from its middle, looking along the world
/// direction `(ux, uy)` (a unit vector).
fn body_reach(robot: &RobotRecord, (ux, uy): (f64, f64)) -> f64 {
    match robot.shape {
        ShapeRecord::Circle([_, _, radius]) => radius,
        ShapeRecord::Rect([_, _, half_length, half_width]) => {
            // the direction in the robot frame
            let (sin, cos) = robot.heading.sin_cos();
            let (lx, ly) = ((ux * cos + uy * sin).abs(), (-ux * sin + uy * cos).abs());
            let along = if lx > 1e-12 {
                half_length / lx
            } else {
                f64::INFINITY
            };
            let across = if ly > 1e-12 {
                half_width / ly
            } else {
                f64::INFINITY
            };
            along.min(across)
        }
    }
}

fn draw_robot(pixmap: &mut Pixmap, view: &View, robot: &RobotRecord) {
    let world = |lx: f64, ly: f64| to_world(robot, lx, ly);
    let fill_colour = palette::status(robot.status);
    let outline = palette::darker(fill_colour);
    let outline_px = view.px(ROBOT_OUTLINE);

    if let Some(path) = body_path(view, robot, 0.0) {
        fill(pixmap, &path, fill_colour);
        line(pixmap, &path, outline, outline_px);
    }

    // the heading: a white line from the kinematic center to the front of the body
    let front = match robot.shape {
        ShapeRecord::Rect([cx, cy, half_length, _]) => (cx + half_length, cy),
        ShapeRecord::Circle([cx, cy, radius]) => (cx + radius, cy),
    };
    let (centre, tip) = (world(0.0, 0.0), world(front.0, front.1));
    if let Some(path) = polyline(view, &[centre, tip], false) {
        line(
            pixmap,
            &path,
            [0xFF, 0xFF, 0xFF],
            view.px(ROBOT_OUTLINE * 2.0),
        );
    }
}

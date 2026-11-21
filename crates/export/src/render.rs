//! Drawing one frame: geometry, track graph and robots of one floor.

use botsim_record::{
    GeometryKind, LayoutLine, Outline, RobotRecord, ShapeRecord, SnapshotLine, StatusRecord,
};
use tiny_skia::{FillRule, LineCap, LineJoin, Paint, Path, PathBuilder, Pixmap, Stroke, Transform};

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

/// Draws the frames of one floor of a recording. The geometry and the track
/// graph never change, so they are drawn once; every frame starts from that
/// picture and adds the robots.
pub struct Renderer<'a> {
    layout: &'a LayoutLine,
    view: View,
    map: usize,
    base: Pixmap,
}

impl<'a> Renderer<'a> {
    /// A renderer for floor `map` (an index into the layout's maps), with the
    /// view fitted once to that floor.
    pub fn new(
        layout: &'a LayoutLine,
        map: usize,
        width: u32,
        height: u32,
    ) -> Result<Renderer<'a>, ViewError> {
        let view = View::fit(layout, map, width, height)?;
        let mut base = Pixmap::new(width, height).ok_or(ViewError::NoPixels)?;
        base.fill(color(palette::BACKGROUND));
        draw_geometry(&mut base, layout, &view, map);
        draw_graph(&mut base, layout, &view, map);
        Ok(Renderer {
            layout,
            view,
            map,
            base,
        })
    }

    pub fn view(&self) -> &View {
        &self.view
    }

    pub fn layout(&self) -> &LayoutLine {
        self.layout
    }

    /// The picture of a snapshot: the floor and the robots that are on it.
    pub fn frame(&self, snapshot: &SnapshotLine) -> Pixmap {
        let mut pixmap = self.base.clone();
        for robot in &snapshot.robots {
            // a robot in a lift cabin is not on any floor (its pose is where it got in)
            if robot.map == self.map && robot.status != StatusRecord::RidingLift {
                draw_robot(&mut pixmap, &self.view, robot);
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
    for (i, e) in layout.edges.iter().enumerate() {
        let (a, b) = (&layout.nodes[e.start], &layout.nodes[e.end]);
        if e.map_transition || a.map != map || b.map != map {
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
            line(pixmap, &path, palette::GRAPH, view.px(EDGE_WIDTH));
        }
    }
    for n in layout.nodes.iter().filter(|n| n.map == map) {
        let (x, y) = view.to_px(n.x, n.y);
        for (radius, c) in [
            (NODE_DIAMETER / 2.0, palette::GRAPH),
            (NODE_DIAMETER / 4.0, palette::NODE_CENTRE),
        ] {
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
    let (ux, uy) = (dx / length, dy / length);
    let tip = (
        to.0 - ux * NODE_DIAMETER / 2.0,
        to.1 - uy * NODE_DIAMETER / 2.0,
    );
    let base = (tip.0 - ux * ARROW_LENGTH, tip.1 - uy * ARROW_LENGTH);
    let (nx, ny) = (-uy * ARROW_WIDTH / 2.0, ux * ARROW_WIDTH / 2.0);
    let points = [tip, (base.0 + nx, base.1 + ny), (base.0 - nx, base.1 - ny)];
    if let Some(path) = polyline(view, &points, true) {
        fill(pixmap, &path, palette::GRAPH);
    }
}

fn draw_robot(pixmap: &mut Pixmap, view: &View, robot: &RobotRecord) {
    let (sin, cos) = robot.heading.sin_cos();
    // robot frame (x forward, y left) to world
    let world = |lx: f64, ly: f64| (robot.x + lx * cos - ly * sin, robot.y + lx * sin + ly * cos);
    let fill_colour = palette::status(robot.status);
    let outline = palette::darker(fill_colour);
    let outline_px = view.px(ROBOT_OUTLINE);

    // the body, and the point at the front where the heading marker ends
    let (body, front) = match robot.shape {
        ShapeRecord::Rect([cx, cy, half_length, half_width]) => {
            let corners = [
                world(cx + half_length, cy + half_width),
                world(cx - half_length, cy + half_width),
                world(cx - half_length, cy - half_width),
                world(cx + half_length, cy - half_width),
            ];
            (polyline(view, &corners, true), (cx + half_length, cy))
        }
        ShapeRecord::Circle([cx, cy, radius]) => {
            let (x, y) = world(cx, cy);
            let (px, py) = view.to_px(x, y);
            (
                PathBuilder::from_circle(px, py, view.px(radius).max(1.0)),
                (cx + radius, cy),
            )
        }
    };
    if let Some(path) = body {
        fill(pixmap, &path, fill_colour);
        line(pixmap, &path, outline, outline_px);
    }

    // the heading: a white line from the kinematic center to the front
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

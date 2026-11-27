//! The outline of a robot swept along an edge (input of the conflict map).
//!
//! The edge path (`Graph::edge_path`: the NURBS trajectory sampled, or the straight
//! line) is sampled at a fixed distance, curves much more finely (so that no
//! corner moves more than [`CORNER_STEP`] between samples); at each sample the
//! body outline is placed at the robot's position, turned to the direction of the
//! path (the robot drives forwards). At a node the outline is swept over the turns
//! the robot can make there. An omnidirectional robot keeps a heading that the
//! layout does not fix, so it gets a circle round its kinematic center that
//! contains the outline in any heading (conservative: it can report conflicts
//! that do not exist). Not modelled yet: driving backwards, orientations other
//! than forwards.

use std::f64::consts::{PI, TAU};

use botsim_model::angle::angle_diff;
use botsim_model::graph::{Graph, Rotation};
use botsim_model::path::Path;
use botsim_model::safety::{FIELD_MARGIN, protective_field};
use botsim_model::spec::{Kinematics, Shape};
use togo::prelude::{
    Circle, OrientedRect, Point, circle, dist_circle_orect, dist_orect_orect, overlap_circle_orect,
    overlap_orect_orect, point,
};

use crate::types::TypeRef;

/// Default distance between two samples along a path (m).
pub const SAMPLE_STEP: f64 = 0.25;

/// (min x, min y, max x, max y).
pub type Bounds = (f64, f64, f64, f64);

/// The body of the robot at one sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Body {
    Rect(OrientedRect),
    Circle(Circle),
}

impl Body {
    /// (min x, min y, max x, max y) of the body.
    pub fn bounds(&self) -> Bounds {
        match self {
            Body::Rect(r) => {
                let c = r.corners();
                let xs = c.iter().map(|p| p.x);
                let ys = c.iter().map(|p| p.y);
                (
                    xs.clone().fold(f64::INFINITY, f64::min),
                    ys.clone().fold(f64::INFINITY, f64::min),
                    xs.fold(f64::NEG_INFINITY, f64::max),
                    ys.fold(f64::NEG_INFINITY, f64::max),
                )
            }
            Body::Circle(c) => (c.c.x - c.r, c.c.y - c.r, c.c.x + c.r, c.c.y + c.r),
        }
    }
}

/// Do two boxes come within `margin` of each other (touching counts)?
fn boxes_near(a: Bounds, b: Bounds, margin: f64) -> bool {
    !(a.2 + margin < b.0 || b.2 + margin < a.0 || a.3 + margin < b.1 || b.3 + margin < a.1)
}

/// Do two bodies overlap? Touching counts as overlap (togo's rule).
pub fn bodies_overlap(a: &Body, b: &Body) -> bool {
    match (a, b) {
        (Body::Rect(a), Body::Rect(b)) => overlap_orect_orect(a, b),
        (Body::Circle(c), Body::Rect(r)) | (Body::Rect(r), Body::Circle(c)) => {
            overlap_circle_orect(c, r)
        }
        (Body::Circle(a), Body::Circle(b)) => (a.c.x - b.c.x).hypot(a.c.y - b.c.y) <= a.r + b.r,
    }
}

/// The distance between two bodies (m); 0 if they overlap or touch.
pub fn body_distance(a: &Body, b: &Body) -> f64 {
    match (a, b) {
        (Body::Rect(a), Body::Rect(b)) => dist_orect_orect(a, b),
        (Body::Circle(c), Body::Rect(r)) | (Body::Rect(r), Body::Circle(c)) => {
            dist_circle_orect(c, r)
        }
        (Body::Circle(a), Body::Circle(b)) => {
            ((a.c.x - b.c.x).hypot(a.c.y - b.c.y) - a.r - b.r).max(0.0)
        }
    }
}

/// How deep two bodies overlap (m): the smallest distance one would have to be
/// moved to be clear of the other. `None` if they do not overlap (touching: 0).
pub fn overlap_depth(a: &Body, b: &Body) -> Option<f64> {
    let d = match (a, b) {
        (Body::Rect(a), Body::Rect(b)) => {
            let mut depth = f64::INFINITY;
            for axis in [a.dir, a.normal(), b.dir, b.normal()] {
                let project = |r: &OrientedRect| {
                    let p: Vec<f64> = r
                        .corners()
                        .iter()
                        .map(|c| c.x * axis.x + c.y * axis.y)
                        .collect();
                    (
                        p.iter().copied().fold(f64::INFINITY, f64::min),
                        p.iter().copied().fold(f64::NEG_INFINITY, f64::max),
                    )
                };
                let (a0, a1) = project(a);
                let (b0, b1) = project(b);
                // the way out along this axis: one body past the other, either side
                depth = depth.min((a1 - b0).min(b1 - a0));
            }
            depth
        }
        (Body::Circle(c), Body::Rect(r)) | (Body::Rect(r), Body::Circle(c)) => {
            let q = r.to_local(c.c);
            let (dx, dy) = (q.x.abs() - r.half_len, q.y.abs() - r.half_wid);
            if dx > 0.0 || dy > 0.0 {
                c.r - dx.max(0.0).hypot(dy.max(0.0))
            } else {
                c.r - dx.max(dy) // centre inside the rectangle
            }
        }
        (Body::Circle(a), Body::Circle(b)) => a.r + b.r - (a.c.x - b.c.x).hypot(a.c.y - b.c.y),
    };
    (d >= 0.0).then_some(d)
}

/// How two swept outlines come together: see [`Sweep::contact`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Contact {
    /// The deepest overlap of any two bodies (m); 0 if none overlaps (or they touch).
    pub depth: f64,
    /// The smallest distance between any two bodies (m); 0 if some overlap or touch.
    pub distance: f64,
}

/// A robot type's outline swept along one edge (or the bodies at one node).
#[derive(Debug, Clone)]
pub struct Sweep {
    pub bodies: Vec<Body>,
    /// Bounding box of all bodies: (min x, min y, max x, max y); for the broad phase.
    pub bounds: Bounds,
    /// Bounding box of each body.
    boxes: Vec<Bounds>,
}

impl Sweep {
    /// A sweep of the given bodies (empty: a sweep that touches nothing).
    pub fn from_bodies(bodies: Vec<Body>) -> Sweep {
        let boxes: Vec<Bounds> = bodies.iter().map(Body::bounds).collect();
        let bounds = boxes.iter().fold(
            (
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ),
            |b, c| (b.0.min(c.0), b.1.min(c.1), b.2.max(c.2), b.3.max(c.3)),
        );
        Sweep {
            bodies,
            bounds,
            boxes,
        }
    }

    /// The deepest overlap of any body of one with any body of the other (m);
    /// `None` if the swept outlines do not overlap.
    pub fn max_overlap(&self, other: &Sweep) -> Option<f64> {
        self.contact(other, 0.0).map(|c| c.depth)
    }

    /// Do the two swept outlines overlap anywhere? First the bounding boxes, then
    /// every body of one against every body of the other.
    pub fn overlaps(&self, other: &Sweep) -> bool {
        if !boxes_near(self.bounds, other.bounds, 0.0) {
            return false;
        }
        self.bodies.iter().zip(&self.boxes).any(|(x, bx)| {
            other
                .bodies
                .iter()
                .zip(&other.boxes)
                .any(|(y, by)| boxes_near(*bx, *by, 0.0) && bodies_overlap(x, y))
        })
    }

    /// How the two outlines come together, if any body of one overlaps, touches or
    /// is nearer than `clearance` (m) to a body of the other; `None` if the
    /// outlines stay `clearance` or more apart. With `clearance` 0 only overlap
    /// (touching too) is found.
    pub fn contact(&self, other: &Sweep, clearance: f64) -> Option<Contact> {
        let clearance = clearance.max(0.0);
        if !boxes_near(self.bounds, other.bounds, clearance) {
            return None;
        }
        let mut depth: Option<f64> = None;
        let mut distance = f64::INFINITY;
        for (x, bx) in self.bodies.iter().zip(&self.boxes) {
            for (y, by) in other.bodies.iter().zip(&other.boxes) {
                if !boxes_near(*bx, *by, clearance) {
                    continue;
                }
                if let Some(d) = overlap_depth(x, y) {
                    depth = Some(depth.map_or(d, |m: f64| m.max(d)));
                    distance = 0.0;
                } else if clearance > 0.0 && distance > 0.0 {
                    let gap = body_distance(x, y);
                    if gap < clearance {
                        distance = distance.min(gap);
                    }
                }
            }
        }
        // closer than the clearance counts, so a gap equal to it does not
        (depth.is_some() || distance < clearance).then_some(Contact {
            depth: depth.unwrap_or(0.0),
            distance: if depth.is_some() { 0.0 } else { distance },
        })
    }
}

/// The swept outline of `edge` for robot type `t`; `None` for a lift ride (the
/// two nodes are on different maps).
///
/// Straight edges are sampled every `step` (never more than the body length, so
/// nothing is skipped), curves also so finely that no corner of the outline moves
/// more than [`CORNER_STEP`] between neighbours, however short the curve. A
/// `step` of 0 or less sets no distance of its own: a straight edge gets its two
/// ends only.
pub fn sweep(graph: &Graph, edge: usize, t: &TypeRef, step: f64) -> Option<Sweep> {
    if graph.edges.get(edge)?.map_transition {
        return None;
    }
    let path = graph.edge_path(edge, &t.manufacturer, &t.series);
    let spacing = if step > 0.0 {
        step.min(t.length.max(CORNER_STEP))
    } else {
        f64::INFINITY
    };
    let outline = |x, y, heading| body_at(&t.shape, t.kinematics, x, y, heading);
    let bodies = samples(&path, spacing, |x, y, h| marks(&outline(x, y, h)))
        .into_iter()
        .map(|s| {
            let (x, y) = path.point_at(s);
            outline(x, y, path.tangent_at(s))
        })
        .collect();
    Some(Sweep::from_bodies(bodies))
}

/// On curves, and when a robot turns on the spot, outlines are placed so finely
/// that no corner moves more than this (m) between two neighbours.
pub const CORNER_STEP: f64 = 0.02;

/// How finely a curve is walked to place those poses (m). The edge path is a
/// polyline, so the direction jumps at its points: there a corner may move more
/// than [`CORNER_STEP`] between two poses this far apart.
const CURVE_WALK: f64 = 0.005;

/// The points of a body whose movement limits the distance between samples.
fn marks(body: &Body) -> [Point; 4] {
    match body {
        Body::Rect(r) => r.corners(),
        Body::Circle(c) => [c.c; 4],
    }
}

/// The distances along `path` at which an outline is placed: at most `spacing`
/// apart, and on a curve such that no mark of the outline (`marks_at`, from the
/// pose) moves more than [`CORNER_STEP`] where the direction changes.
fn samples(path: &Path, spacing: f64, marks_at: impl Fn(f64, f64, f64) -> [Point; 4]) -> Vec<f64> {
    let length = path.length();
    if !length.is_finite() {
        return vec![0.0];
    }
    if path.is_straight() {
        let n = ((length / spacing).ceil() as usize).max(1);
        return (0..=n).map(|i| length * i as f64 / n as f64).collect();
    }
    let at = |s: f64| {
        let (x, y) = path.point_at(s);
        let heading = path.tangent_at(s);
        (s, heading, marks_at(x, y, heading))
    };
    // Walk the curve in small steps; keep the last step before the outline would
    // have turned and moved too far (or the distance gets too long).
    let n = ((length / CURVE_WALK).ceil() as usize).clamp(1, 1_000_000);
    let mut kept = at(0.0);
    let mut previous = kept;
    let mut out = vec![0.0];
    for i in 1..=n {
        let next = at(length * i as f64 / n as f64);
        let turned = (next.1 - kept.1).sin().abs() > 1e-6 || (next.1 - kept.1).cos() < 0.0;
        let moved = kept
            .2
            .iter()
            .zip(&next.2)
            .map(|(a, b)| (a.x - b.x).hypot(a.y - b.y))
            .fold(0.0, f64::max);
        if ((turned && moved > CORNER_STEP) || next.0 - kept.0 > spacing) && previous.0 > kept.0 {
            kept = previous;
            out.push(kept.0);
        }
        previous = next;
    }
    out.push(length);
    out
}

/// The **standstill protective field** of a robot of type `t` swept along `edge`;
/// `None` for a lift ride.
///
/// The field is the one of the simulator (`botsim_model::safety::protective_field`
/// at speed 0): ahead of the body in the direction of travel, `FIELD_MARGIN` long
/// and `LATERAL_MARGIN` wider than the body on each side. An omnidirectional
/// robot gets the field of the circle that holds its body in any heading. A
/// robot standing on a node has no field (as in the simulator).
///
/// Sampled like [`sweep`], with the field's corners as the measure (never more
/// than the field length apart).
pub fn field_sweep(graph: &Graph, edge: usize, t: &TypeRef, step: f64) -> Option<Sweep> {
    if graph.edges.get(edge)?.map_transition {
        return None;
    }
    let path = graph.edge_path(edge, &t.manufacturer, &t.series);
    let longest = FIELD_MARGIN.min(t.length.max(CORNER_STEP));
    let spacing = if step > 0.0 {
        step.min(longest)
    } else {
        longest
    };
    let fields = samples(&path, spacing, |x, y, h| field_at(t, x, y, h).corners())
        .into_iter()
        .map(|s| {
            let (x, y) = path.point_at(s);
            Body::Rect(field_at(t, x, y, path.tangent_at(s)))
        })
        .collect();
    Some(Sweep::from_bodies(fields))
}

/// The standstill protective field with the kinematic center at (x, y), driving in
/// direction `heading`.
fn field_at(t: &TypeRef, x: f64, y: f64, heading: f64) -> OrientedRect {
    let shape = match (t.kinematics, t.shape) {
        // any heading: the field of the circle that holds the body
        (
            Kinematics::Omnidirectional,
            Shape::Rect {
                cx,
                cy,
                half_length,
                half_width,
            },
        ) => Shape::Circle {
            cx: 0.0,
            cy: 0.0,
            radius: (cx.abs() + half_length).hypot(cy.abs() + half_width),
        },
        (Kinematics::Omnidirectional, Shape::Circle { cx, cy, radius }) => Shape::Circle {
            cx: 0.0,
            cy: 0.0,
            radius: cx.hypot(cy) + radius,
        },
        (_, shape) => shape,
    };
    // speed 0: no braking distance, so the deceleration and the time step do not count
    protective_field(
        &shape,
        (x, y, heading),
        (heading.cos(), heading.sin()),
        0.0,
        1.0,
        0.0,
    )
}

/// The outline of a robot of type `t` swept at a node: its body in every heading
/// it can have there. These are the directions an edge it may drive arrives in or
/// leaves in and, for a robot that turns on the spot (differential), all headings
/// in between: for every edge in and every edge out that may follow it, the turn
/// in the direction the simulator takes (the shorter way if the layout allows it,
/// else the other; both for a U-turn), no corner moving more than
/// [`CORNER_STEP`] between neighbours. An omnidirectional robot has one body that
/// holds every heading. Empty if the type may not stand on the node or drives no
/// edge to or from it.
///
/// Not covered: a turn from a heading the layout does not give (a robot placed on
/// the node at the start).
pub fn node_bodies(graph: &Graph, node: usize, t: &TypeRef) -> Vec<Body> {
    let Some(n) = graph.nodes.get(node) else {
        return Vec::new();
    };
    if !graph.node_allows(node, &t.manufacturer, &t.series) {
        return Vec::new();
    }
    let usable = |e: usize| graph.edge_allows(e, &t.manufacturer, &t.series);
    let directions = |e: usize| graph.edge_directions(e, &t.manufacturer, &t.series);
    let rotation = |e: usize| graph.edge_rotation(e, &t.manufacturer, &t.series);
    // (heading, what the edge allows at this node)
    let mut arrivals: Vec<(f64, Option<Rotation>)> = Vec::new();
    let mut departures: Vec<(f64, Rotation)> = Vec::new();
    for (e, edge) in graph.edges.iter().enumerate() {
        if !usable(e) {
            continue;
        }
        if edge.map_transition {
            // out of a lift the robot has the heading it drove in with
            if edge.end == node {
                for (f, before) in graph.edges.iter().enumerate() {
                    if before.end == edge.start && !before.map_transition && usable(f) {
                        arrivals.push((directions(f).1, None));
                    }
                }
            }
            continue;
        }
        if edge.end == node {
            arrivals.push((directions(e).1, Some(rotation(e).1)));
        }
        if edge.start == node {
            departures.push((directions(e).0, rotation(e).0));
        }
    }
    let mut headings: Vec<f64> = arrivals
        .iter()
        .map(|a| a.0)
        .chain(departures.iter().map(|d| d.0))
        .collect();
    if t.kinematics == Kinematics::Differential {
        // the turns, as (from this heading, counter-clockwise by this much)
        let mut arcs: Vec<(f64, f64)> = Vec::new();
        for &(from, arrive) in &arrivals {
            for &(to, leave) in &departures {
                let allowed = arrive.map_or(leave, |r| r.and(leave));
                arcs.extend(turn_arcs(from, to, allowed));
            }
        }
        headings.extend(headings_in(&arcs, turning_radius(&t.shape)));
    }
    // the same heading (within 1 mrad) once
    let mut distinct: Vec<f64> = Vec::new();
    for h in headings {
        if !distinct
            .iter()
            .any(|d| (d - h).sin().abs() < 1e-3 && (d - h).cos() > 0.0)
        {
            distinct.push(h);
        }
    }
    if t.kinematics == Kinematics::Omnidirectional {
        distinct.truncate(1);
    }
    distinct
        .into_iter()
        .map(|h| body_at(&t.shape, t.kinematics, n.x, n.y, h))
        .collect()
}

/// The turn on the spot from heading `from` to heading `to` if `allowed` lets the
/// robot make it, as (start heading, counter-clockwise extent): the shorter way
/// if allowed, else the other one (the driver's rule). For a U-turn, where the
/// shorter way is a matter of rounding, every allowed direction.
fn turn_arcs(from: f64, to: f64, allowed: Rotation) -> Vec<(f64, f64)> {
    let diff = angle_diff(from, to);
    if !diff.is_finite() || diff.abs() <= 1e-9 {
        return Vec::new();
    }
    let ccw = (from, (to - from).rem_euclid(TAU));
    let cw = (to, (from - to).rem_euclid(TAU));
    if PI - diff.abs() < 1e-3 {
        return [(true, ccw), (false, cw)]
            .into_iter()
            .filter(|&(direction, _)| allowed.allows(direction))
            .map(|(_, arc)| arc)
            .collect();
    }
    let shortest_ccw = diff > 0.0;
    let direction = [shortest_ccw, !shortest_ccw]
        .into_iter()
        .find(|&d| allowed.allows(d));
    direction
        .map(|d| if d { ccw } else { cw })
        .into_iter()
        .collect()
}

/// The body of a robot of type `t` standing on a node in one heading (radians),
/// without the turns of [`node_bodies`]: e.g. as it arrived on an edge. `None`
/// for a node the graph does not have.
pub fn standing_body(graph: &Graph, node: usize, t: &TypeRef, heading: f64) -> Option<Body> {
    let n = graph.nodes.get(node)?;
    Some(body_at(&t.shape, t.kinematics, n.x, n.y, heading))
}

/// How far the farthest point of the body moves per radian when the robot turns
/// about its kinematic center (m); for a circle, only its centre moves.
fn turning_radius(shape: &Shape) -> f64 {
    match *shape {
        Shape::Rect {
            cx,
            cy,
            half_length,
            half_width,
        } => (cx.abs() + half_length).hypot(cy.abs() + half_width),
        Shape::Circle { cx, cy, .. } => cx.hypot(cy),
    }
}

/// The headings of an even grid, fine enough for [`CORNER_STEP`] at `radius`,
/// that lie inside one of the `arcs` (each heading once, however many turns pass
/// through it).
fn headings_in(arcs: &[(f64, f64)], radius: f64) -> Vec<f64> {
    if arcs.is_empty() || radius <= 0.0 || !radius.is_finite() {
        return Vec::new();
    }
    let n = ((TAU * radius / CORNER_STEP).ceil() as usize).clamp(8, 100_000);
    (0..n)
        .map(|i| TAU * i as f64 / n as f64)
        .filter(|h| {
            arcs.iter()
                .any(|&(start, extent)| (h - start).rem_euclid(TAU) <= extent)
        })
        .collect()
}

/// The body with its kinematic center at (x, y) and heading `heading`.
fn body_at(shape: &Shape, kinematics: Kinematics, x: f64, y: f64, heading: f64) -> Body {
    let (sin, cos) = heading.sin_cos();
    // robot frame (x forward, y left) -> map
    let to_map = |fx: f64, fy: f64| point(x + fx * cos - fy * sin, y + fx * sin + fy * cos);
    match *shape {
        Shape::Rect {
            cx,
            cy,
            half_length,
            half_width,
        } => {
            if kinematics == Kinematics::Omnidirectional {
                // any heading: a circle round the kinematic center that holds the outline
                let r = (cx.abs() + half_length).hypot(cy.abs() + half_width);
                Body::Circle(circle(point(x, y), r))
            } else {
                Body::Rect(OrientedRect::from_angle(
                    to_map(cx, cy),
                    heading,
                    half_length,
                    half_width,
                ))
            }
        }
        Shape::Circle { cx, cy, radius } => {
            if kinematics == Kinematics::Omnidirectional {
                Body::Circle(circle(point(x, y), cx.hypot(cy) + radius))
            } else {
                Body::Circle(circle(to_map(cx, cy), radius))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vdma_lif::Lif;

    /// Graph with edge E1 from (0,0) to (10,0).
    fn graph() -> Graph {
        let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}"#;
        let lif: Lif = serde_json::from_str(&format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
            "nodes":[{{"nodeId":"A","mapId":"M","nodePosition":{{"x":0,"y":0}},"mobileRobotTypeNodeProperties":[{ty}]}},
                     {{"nodeId":"B","mapId":"M","nodePosition":{{"x":10,"y":0}},"mobileRobotTypeNodeProperties":[{ty}]}}],
            "edges":[{{"edgeId":"E1","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[{ty}]}}]}}]}}]}}"#
        ))
        .expect("test LIF parses");
        Graph::from_lif(&lif).expect("graph")
    }

    fn near(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn straight_edge_sweeps_a_box() {
        // body 2 m x 1 m centered on the kinematic center, edge 10 m east:
        // swept box is x -1..11, y -0.5..0.5
        let t = TypeRef::test_rect("ACME/X1", Kinematics::Differential, 2.0, 1.0);
        let s = sweep(&graph(), 0, &t, 0.25).expect("sweep");
        assert_eq!(s.bodies.len(), 41); // 10 m / 0.25 m + 1
        let (x0, y0, x1, y1) = s.bounds;
        assert!(near(x0, -1.0) && near(x1, 11.0), "{:?}", s.bounds);
        assert!(near(y0, -0.5) && near(y1, 0.5), "{:?}", s.bounds);
    }

    #[test]
    fn omnidirectional_gets_a_circle_that_holds_the_outline() {
        let t = TypeRef::test_rect("ACME/X1", Kinematics::Omnidirectional, 2.0, 1.0);
        let s = sweep(&graph(), 0, &t, 1.0).expect("sweep");
        let Body::Circle(c) = s.bodies[0] else {
            panic!("expected a circle")
        };
        assert!(near(c.r, 1.0f64.hypot(0.5)));
    }

    /// Two edges: E1 from (0,0) to (10,0); E2 from (`x`,`y`) to (`x`+`dx`,`y`+`dy`).
    fn two_edges(x: f64, y: f64, dx: f64, dy: f64) -> Graph {
        let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}"#;
        let node = |id: &str, x: f64, y: f64| {
            format!(
                r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},"mobileRobotTypeNodeProperties":[{ty}]}}"#
            )
        };
        let edge = |id: &str, a: &str, b: &str| {
            format!(
                r#"{{"edgeId":"{id}","startNodeId":"{a}","endNodeId":"{b}","mobileRobotTypeEdgeProperties":[{ty}]}}"#
            )
        };
        let lif: Lif = serde_json::from_str(&format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
            "nodes":[{},{},{},{}],"edges":[{},{}]}}]}}]}}"#,
            node("A", 0.0, 0.0),
            node("B", 10.0, 0.0),
            node("C", x, y),
            node("D", x + dx, y + dy),
            edge("E1", "A", "B"),
            edge("E2", "C", "D")
        ))
        .expect("test LIF parses");
        Graph::from_lif(&lif).expect("graph")
    }

    fn overlap_of(g: &Graph, t: &TypeRef) -> bool {
        let a = sweep(g, 0, t, 0.25).expect("sweep");
        let b = sweep(g, 1, t, 0.25).expect("sweep");
        a.overlaps(&b)
    }

    #[test]
    fn crossing_parallel_near_and_far_edges() {
        let t = TypeRef::test_rect("ACME/X1", Kinematics::Differential, 2.0, 1.0);
        // crossing at (5,0)
        assert!(overlap_of(&two_edges(5.0, -5.0, 0.0, 10.0), &t));
        // parallel 0.9 m apart: bodies are 1 m wide -> overlap
        assert!(overlap_of(&two_edges(0.0, 0.9, 10.0, 0.0), &t));
        // parallel 1.2 m apart -> clear
        assert!(!overlap_of(&two_edges(0.0, 1.2, 10.0, 0.0), &t));
        // far away: bounding boxes reject
        assert!(!overlap_of(&two_edges(50.0, 50.0, 10.0, 0.0), &t));
        // end to end: the bodies at the two end nodes overlap (2 m long bodies, nodes 1 m apart)
        assert!(overlap_of(&two_edges(11.0, 0.0, 10.0, 0.0), &t));
        // end to end with the nodes 2.5 m apart: clear
        assert!(!overlap_of(&two_edges(12.5, 0.0, 10.0, 0.0), &t));
    }

    #[test]
    fn overlap_depth_of_parallel_edges_and_crossing() {
        let t = TypeRef::test_rect("ACME/X1", Kinematics::Differential, 2.0, 1.0);
        let depth = |g: &Graph| {
            let a = sweep(g, 0, &t, 0.25).expect("sweep");
            let b = sweep(g, 1, &t, 0.25).expect("sweep");
            a.max_overlap(&b)
        };
        // parallel 0.9 m apart, bodies 1 m wide: the bodies overlap by 0.1 m
        let d = depth(&two_edges(0.0, 0.9, 10.0, 0.0)).expect("overlap");
        assert!((d - 0.1).abs() < 1e-9, "{d}");
        // clear: none
        assert!(depth(&two_edges(0.0, 1.2, 10.0, 0.0)).is_none());
        // end to end, nodes 1 m apart, bodies 2 m long: the end bodies overlap by 1 m
        let d = depth(&two_edges(11.0, 0.0, 10.0, 0.0)).expect("overlap");
        assert!((d - 1.0).abs() < 1e-9, "{d}");
    }

    #[test]
    fn contact_finds_overlap_and_near_misses() {
        let t = TypeRef::test_rect("ACME/X1", Kinematics::Differential, 2.0, 1.0);
        let contact = |g: &Graph, clearance: f64| {
            let a = sweep(g, 0, &t, 0.25).expect("sweep");
            let b = sweep(g, 1, &t, 0.25).expect("sweep");
            a.contact(&b, clearance)
        };
        // parallel 1.05 m apart, bodies 1 m wide: 0.05 m gap
        let g = two_edges(0.0, 1.05, 10.0, 0.0);
        assert!(contact(&g, 0.0).is_none());
        assert!(contact(&g, 0.04).is_none());
        let c = contact(&g, 0.10).expect("within 10 cm");
        assert_eq!(c.depth, 0.0);
        assert!((c.distance - 0.05).abs() < 1e-9, "{c:?}");
        // a gap equal to the clearance is not "below" it
        assert!(contact(&two_edges(0.0, 1.10, 10.0, 0.0), 0.10 - 1e-6).is_none());
        // overlap: depth, distance 0, with or without clearance
        let g = two_edges(0.0, 0.9, 10.0, 0.0);
        for clearance in [0.0, 0.1] {
            let c = contact(&g, clearance).expect("overlap");
            assert!((c.depth - 0.1).abs() < 1e-9 && c.distance == 0.0, "{c:?}");
        }
        // far away: nothing
        assert!(contact(&two_edges(50.0, 50.0, 10.0, 0.0), 0.1).is_none());
    }

    #[test]
    fn overlap_depth_when_one_body_is_inside_the_other_on_an_axis() {
        // a 1 x 1 box centred in a 4 x 2 box: it has to move 1.5 m along x? no:
        // out along y by 1.5 (0.5 + 1.0), along x by 2.5 (0.5 + 2.0): 1.5
        let big = Body::Rect(OrientedRect::from_angle(point(0.0, 0.0), 0.0, 2.0, 1.0));
        let small = Body::Rect(OrientedRect::from_angle(point(0.0, 0.0), 0.0, 0.5, 0.5));
        let d = overlap_depth(&big, &small).expect("overlap");
        assert!((d - 1.5).abs() < 1e-12, "{d}");
    }

    #[test]
    fn overlap_depth_of_circles_and_rect() {
        let c = |x: f64, r: f64| Body::Circle(circle(point(x, 0.0), r));
        let d = overlap_depth(&c(0.0, 1.0), &c(1.5, 1.0)).expect("overlap");
        assert!((d - 0.5).abs() < 1e-12);
        assert!(overlap_depth(&c(0.0, 1.0), &c(3.0, 1.0)).is_none());
        // rectangle 2 x 1 at the origin; circle r = 0.5 centred at x = 1.25: 0.25 inside
        let r = Body::Rect(OrientedRect::from_angle(point(0.0, 0.0), 0.0, 1.0, 0.5));
        let d = overlap_depth(&c(1.25, 0.5), &r).expect("overlap");
        assert!((d - 0.25).abs() < 1e-12, "{d}");
        // distances
        assert!((body_distance(&c(0.0, 1.0), &c(3.0, 1.0)) - 1.0).abs() < 1e-12);
        // the rectangle reaches x = 1; the circle's near edge is at 3 - 0.5
        assert!((body_distance(&c(3.0, 0.5), &r) - 1.5).abs() < 1e-12);
        assert_eq!(body_distance(&c(1.25, 0.5), &r), 0.0);
    }

    #[test]
    fn omnidirectional_circles_overlap_more() {
        // 2 x 1 body: circle radius 1.118; parallel 2 m apart is clear for a
        // differential robot, but the circles (2 x 1.118 > 2) overlap
        let d = TypeRef::test_rect("ACME/X1", Kinematics::Differential, 2.0, 1.0);
        let o = TypeRef::test_rect("ACME/X1", Kinematics::Omnidirectional, 2.0, 1.0);
        let g = two_edges(0.0, 2.0, 10.0, 0.0);
        assert!(!overlap_of(&g, &d));
        assert!(overlap_of(&g, &o));
    }

    #[test]
    fn field_sweep_starts_at_the_body_front_and_is_wider() {
        // body 2 m x 1 m on a 10 m edge east: the fields cover x 1 .. 11.3 and are
        // 0.1 m wider on each side
        let t = TypeRef::test_rect("ACME/X1", Kinematics::Differential, 2.0, 1.0);
        let f = field_sweep(&graph(), 0, &t, 0.25).expect("sweep");
        let (x0, y0, x1, y1) = f.bounds;
        assert!(near(x0, 1.0) && near(x1, 11.3), "{:?}", f.bounds);
        assert!(near(y0, -0.6) && near(y1, 0.6), "{:?}", f.bounds);
        assert!(field_sweep(&graph(), 99, &t, 0.25).is_none());
        // a zero step falls back to the field length: no gaps, no panic
        let f = field_sweep(&graph(), 0, &t, 0.0).expect("sweep");
        assert!(f.bodies.len() >= 35);
    }

    /// Does any of the bodies reach into a small circle at (x, y)?
    fn reaches(bodies: &[Body], x: f64, y: f64) -> bool {
        let dot = Body::Circle(circle(point(x, y), 0.01));
        bodies.iter().any(|b| bodies_overlap(b, &dot))
    }

    /// Nodes A (-5, 0), N (0, 0), B (0, 5) and C (0.125, 0.125); edges `A-N`
    /// east, `N-B` north (with `extra` in its properties for the type) and the
    /// short left arc `N-C` of radius 0.125 m (0.196 m long).
    fn corner(extra: &str) -> Graph {
        let ty = r#""mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]"#;
        let node = |id: &str, x: f64, y: f64| {
            format!(
                r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},"mobileRobotTypeNodeProperties":[{{{ty}}}]}}"#
            )
        };
        let edge = |a: &str, b: &str, props: &str| {
            format!(
                r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}","mobileRobotTypeEdgeProperties":[{{{ty}{props}}}]}}"#
            )
        };
        let arc = r#","trajectory":{"degree":2,"knotVector":[0,0,0,1,1,1],"controlPoints":[{"x":0,"y":0},{"x":0.125,"y":0,"weight":0.7071067811865476},{"x":0.125,"y":0.125}]}"#;
        let lif: Lif = serde_json::from_str(&format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
            "nodes":[{},{},{},{}],"edges":[{},{},{}]}}]}}]}}"#,
            node("A", -5.0, 0.0),
            node("N", 0.0, 0.0),
            node("B", 0.0, 5.0),
            node("C", 0.125, 0.125),
            edge("A", "N", ""),
            edge("N", "B", extra),
            edge("N", "C", arc),
        ))
        .expect("test LIF parses");
        Graph::from_lif(&lif).expect("graph")
    }

    #[test]
    fn a_curve_shorter_than_the_step_is_sampled_in_between() {
        let t = TypeRef::test_rect("ACME/X1", Kinematics::Differential, 0.48, 0.24);
        let g = corner("");
        let arc = g.edge_idx("N-C").expect("arc");
        let s = sweep(&g, arc, &t, 0.25).expect("sweep");
        assert!(s.bodies.len() > 10, "{}", s.bodies.len());
        // the front right corner in mid turn reaches a place neither end does
        let ends = [s.bodies[0], *s.bodies.last().expect("end")];
        assert!(!reaches(&ends, 0.325, 0.11));
        assert!(reaches(&s.bodies, 0.325, 0.11));
        // however large the step is
        let s = sweep(&g, arc, &t, 100.0).expect("sweep");
        assert!(reaches(&s.bodies, 0.325, 0.11));
    }

    /// A point at `degrees` and `radius` from the node N.
    fn around(degrees: f64, radius: f64) -> (f64, f64) {
        let (s, c) = degrees.to_radians().sin_cos();
        (radius * c, radius * s)
    }

    #[test]
    fn a_node_is_swept_over_the_turn() {
        // 2 x 1 body: its corners are 1.118 m from the node
        let t = TypeRef::test_rect("ACME/X1", Kinematics::Differential, 2.0, 1.0);
        let g = corner("");
        let n = g.node_idx("N").expect("N");
        let bodies = node_bodies(&g, n, &t);
        // turning left from east (in) to north (out): a corner passes 40 degrees
        let (x, y) = around(40.0, 1.08);
        assert!(reaches(&bodies, x, y));
        // but no corner passes 135 degrees: the robot does not turn the long way
        let (x, y) = around(135.0, 1.08);
        assert!(!reaches(&bodies, x, y));
        // if the layout only allows the long way (clockwise), it does
        let g = corner(r#","rotationAtStartNodeAllowed":"CW""#);
        let (x, y) = around(135.0, 1.08);
        assert!(reaches(&node_bodies(&g, n, &t), x, y));
        // a car-like robot cannot turn on the spot
        let car = TypeRef::test_rect("ACME/X1", Kinematics::ThreeWheel, 2.0, 1.0);
        let (x, y) = around(40.0, 1.08);
        assert!(!reaches(&node_bodies(&corner(""), n, &car), x, y));
    }

    #[test]
    fn turn_directions_follow_the_driver() {
        let half = PI / 2.0;
        // the shorter way, else the other one, else none
        assert_eq!(turn_arcs(0.0, half, Rotation::Both), [(0.0, half)]);
        assert_eq!(turn_arcs(0.0, half, Rotation::Cw), [(half, 3.0 * half)]);
        assert!(turn_arcs(0.0, half, Rotation::None).is_empty());
        assert!(turn_arcs(1.0, 1.0, Rotation::Both).is_empty());
        // a U-turn may go either way
        assert_eq!(turn_arcs(0.0, PI, Rotation::Both).len(), 2);
        assert_eq!(turn_arcs(0.0, PI, Rotation::Ccw), [(0.0, PI)]);
    }

    #[test]
    fn unknown_edge_and_zero_step_do_not_panic() {
        let t = TypeRef::test_rect("ACME/X1", Kinematics::Differential, 2.0, 1.0);
        assert!(sweep(&graph(), 99, &t, 0.25).is_none());
        let s = sweep(&graph(), 0, &t, 0.0).expect("sweep");
        assert_eq!(s.bodies.len(), 2); // start and end only
    }
}

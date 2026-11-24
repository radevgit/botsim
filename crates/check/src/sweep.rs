//! The outline of a robot swept along an edge (input of the conflict map, check 5).
//!
//! The edge path (`Graph::edge_path`: the NURBS trajectory sampled, or the straight
//! line) is sampled at a fixed distance; at each sample the body outline is placed
//! at the robot's position, turned to the direction of the path (the robot drives
//! forwards). An omnidirectional robot keeps a heading that the layout does not
//! fix, so it gets a circle round its kinematic center that contains the outline
//! in any heading (conservative: it can report conflicts that do not exist).
//! Not modelled yet: driving backwards, orientations other than forwards.

use botsim_core::graph::Graph;
use botsim_core::spec::{Kinematics, Shape};
use togo::prelude::{
    Circle, OrientedRect, circle, overlap_circle_orect, overlap_orect_orect, point,
};

use crate::reach::TypeRef;

/// Default distance between two samples along a path (m).
pub const SAMPLE_STEP: f64 = 0.25;

/// The body of the robot at one sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Body {
    Rect(OrientedRect),
    Circle(Circle),
}

impl Body {
    /// (min x, min y, max x, max y) of the body.
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
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

/// A robot type's outline swept along one edge.
#[derive(Debug, Clone)]
pub struct Sweep {
    pub bodies: Vec<Body>,
    /// Bounding box of all bodies: (min x, min y, max x, max y); for the broad phase.
    pub bounds: (f64, f64, f64, f64),
}

impl Sweep {
    /// The deepest overlap of any body of one with any body of the other (m);
    /// `None` if the swept outlines do not overlap.
    pub fn max_overlap(&self, other: &Sweep) -> Option<f64> {
        if !self.overlaps(other) {
            return None;
        }
        let mut deepest: Option<f64> = None;
        for x in &self.bodies {
            for y in &other.bodies {
                if let Some(d) = overlap_depth(x, y) {
                    deepest = Some(deepest.map_or(d, |m: f64| m.max(d)));
                }
            }
        }
        deepest
    }

    /// Do the two swept outlines overlap anywhere? First the bounding boxes, then
    /// every body of one against every body of the other.
    pub fn overlaps(&self, other: &Sweep) -> bool {
        let (a, b) = (self.bounds, other.bounds);
        if a.2 < b.0 || b.2 < a.0 || a.3 < b.1 || b.3 < a.1 {
            return false;
        }
        self.bodies
            .iter()
            .any(|x| other.bodies.iter().any(|y| bodies_overlap(x, y)))
    }
}

/// The swept outline of `edge` for robot type `t`; `None` for a lift ride (the
/// two nodes are on different maps).
pub fn sweep(graph: &Graph, edge: usize, t: &TypeRef, step: f64) -> Option<Sweep> {
    if graph.edges.get(edge)?.map_transition {
        return None;
    }
    let path = graph.edge_path(edge, &t.manufacturer, &t.series);
    let length = path.length();
    let n = if step > 0.0 && length.is_finite() {
        ((length / step).ceil() as usize).max(1)
    } else {
        1
    };
    let mut bodies = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let s = length * i as f64 / n as f64;
        let (x, y) = path.point_at(s);
        let heading = path.tangent_at(s);
        bodies.push(body_at(&t.shape, t.kinematics, x, y, heading));
    }
    let mut b = (
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    );
    for body in &bodies {
        let (a, c, d, e) = body.bounds();
        b = (b.0.min(a), b.1.min(c), b.2.max(d), b.3.max(e));
    }
    Some(Sweep { bodies, bounds: b })
}

/// The bodies of a robot of type `t` standing on a node: one for each heading it
/// can have there, i.e. the direction an edge it may drive arrives in or leaves
/// in (an omnidirectional robot has one body that holds every heading). Empty if
/// the type may not stand on the node or drives no edge to or from it.
pub fn node_bodies(graph: &Graph, node: usize, t: &TypeRef) -> Vec<Body> {
    let Some(n) = graph.nodes.get(node) else {
        return Vec::new();
    };
    if !graph.node_allows(node, &t.manufacturer, &t.series) {
        return Vec::new();
    }
    let mut headings: Vec<f64> = Vec::new();
    for (e, edge) in graph.edges.iter().enumerate() {
        if edge.map_transition || !graph.edge_allows(e, &t.manufacturer, &t.series) {
            continue;
        }
        let (start, end) = graph.edge_directions(e, &t.manufacturer, &t.series);
        if edge.end == node {
            headings.push(end);
        }
        if edge.start == node {
            headings.push(start);
        }
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
    fn unknown_edge_and_zero_step_do_not_panic() {
        let t = TypeRef::test_rect("ACME/X1", Kinematics::Differential, 2.0, 1.0);
        assert!(sweep(&graph(), 99, &t, 0.25).is_none());
        let s = sweep(&graph(), 0, &t, 0.0).expect("sweep");
        assert_eq!(s.bodies.len(), 2); // start and end only
    }
}

//! Check 4: clearance. The robot outline swept along each edge against the
//! walls, racks and shelves of the geometry file (`botsim-record`'s
//! `GeometryFile`). Station outlines and `other` shapes are not obstacles: robots
//! drive to the stations, and the interaction nodes lie in or next to them.
//!
//! Walls (polylines) are tested segment by segment, racks and shelves (closed
//! polygons) as filled areas. A shape on a map that the layout does not have is
//! reported as info. A hit has a depth (how far the outline goes into the obstacle
//! at its worst sample); hits not deeper than 1 mm (touching) or than `min_overlap`
//! are not reported.

use std::collections::HashMap;

use botsim_model::graph::Graph;
use botsim_record::{GeometryFile, GeometryKind, Outline};
use vdma_lif::{Layout, Lif};

use crate::finding::{Finding, Level};
use crate::places::LayoutIndex;
use crate::reach::TypeRef;
use crate::sweep::{Body, sweep};

const CHECK: &str = "clearance";

/// Cell size of the grid used as broad phase (m).
const CELL: f64 = 5.0;

/// Touching or overlapping by less than this is not a hit (m).
const EPS: f64 = 1e-3;

type P = (f64, f64);

struct Obstacle {
    /// Index into `Graph::maps`.
    map: usize,
    id: String,
    verts: Vec<P>,
    /// A filled polygon (rack, shelf); false for one wall segment.
    closed: bool,
    bounds: (f64, f64, f64, f64),
}

/// One edge that hits an obstacle for one robot type.
struct Hit {
    edge: usize,
    robot: usize,
    obstacle: usize,
    /// How deep the outline goes into the obstacle at its worst sample (m).
    depth: f64,
}

/// `min_overlap` (m): hits less deep than this are not reported.
pub fn check(
    lif: &Lif,
    types: &[TypeRef],
    geometry: &GeometryFile,
    step: f64,
    min_overlap: f64,
) -> Vec<Finding> {
    // A layout that cannot be turned into a graph is reported by the reachability check.
    let Ok(graph) = Graph::from_lif(lif) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    let obstacles = obstacles(geometry, &graph, &mut out);
    let grid = Grid::new(&obstacles);

    let mut hits: Vec<Hit> = Vec::new();
    for (ti, t) in types.iter().enumerate() {
        for e in 0..graph.edges.len() {
            if !graph.edge_allows(e, &t.manufacturer, &t.series) {
                continue;
            }
            let Some(sw) = sweep(&graph, e, t, step) else {
                continue;
            };
            let map = graph.nodes[graph.edges[e].start].map;
            // the deepest overlap of any body with any obstacle
            let mut deepest: Option<(f64, usize)> = None;
            for o in grid.candidates(map, sw.bounds) {
                for b in &sw.bodies {
                    if let Some(d) = body_depth(b, &obstacles[o])
                        && deepest.is_none_or(|(m, _)| d > m)
                    {
                        deepest = Some((d, o));
                    }
                }
            }
            if let Some((depth, obstacle)) = deepest
                && depth > EPS
                && depth >= min_overlap
            {
                hits.push(Hit {
                    edge: e,
                    robot: ti,
                    obstacle,
                    depth,
                });
            }
        }
    }
    for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
        for (ti, t) in types.iter().enumerate() {
            report_layout(layout, &graph, &obstacles, &hits, ti, t, &mut out);
        }
    }
    out
}

/// The obstacles of the geometry file on maps of the layout; shapes on other
/// maps become one info finding.
fn obstacles(geometry: &GeometryFile, graph: &Graph, out: &mut Vec<Finding>) -> Vec<Obstacle> {
    let mut list = Vec::new();
    let mut unknown_maps: Vec<&str> = Vec::new();
    for (i, s) in geometry.shapes.iter().enumerate() {
        if !matches!(
            s.kind,
            GeometryKind::Wall | GeometryKind::Rack | GeometryKind::Shelf
        ) {
            continue;
        }
        let Some(map) = graph.maps.iter().position(|m| *m == s.map_id) else {
            if !unknown_maps.contains(&s.map_id.as_str()) {
                unknown_maps.push(&s.map_id);
            }
            continue;
        };
        let verts: Vec<P> = s.vertices.iter().map(|v| (v.x, v.y)).collect();
        let name = s.id.clone().unwrap_or_else(|| format!("#{}", i + 1));
        match s.shape {
            Outline::Polygon => list.push(make(map, name, verts, true)),
            Outline::Polyline => {
                for (k, w) in verts.windows(2).enumerate() {
                    list.push(make(map, format!("{name}[{k}]"), w.to_vec(), false));
                }
            }
        }
    }
    if !unknown_maps.is_empty() {
        out.push(Finding::new(
            Level::Info,
            CHECK,
            format!(
                "geometry shapes on maps that the layout does not have are ignored: {}",
                unknown_maps.join(", ")
            ),
        ));
    }
    list
}

fn make(map: usize, id: String, verts: Vec<P>, closed: bool) -> Obstacle {
    let xs = verts.iter().map(|p| p.0);
    let ys = verts.iter().map(|p| p.1);
    let bounds = (
        xs.clone().fold(f64::INFINITY, f64::min),
        ys.clone().fold(f64::INFINITY, f64::min),
        xs.fold(f64::NEG_INFINITY, f64::max),
        ys.fold(f64::NEG_INFINITY, f64::max),
    );
    Obstacle {
        map,
        id,
        verts,
        closed,
        bounds,
    }
}

/// Obstacles by grid cell, per map.
struct Grid {
    cells: HashMap<(usize, i64, i64), Vec<usize>>,
}

fn cell(v: f64) -> i64 {
    (v / CELL).floor() as i64
}

impl Grid {
    fn new(obstacles: &[Obstacle]) -> Grid {
        let mut cells: HashMap<(usize, i64, i64), Vec<usize>> = HashMap::new();
        for (i, o) in obstacles.iter().enumerate() {
            for cx in cell(o.bounds.0)..=cell(o.bounds.2) {
                for cy in cell(o.bounds.1)..=cell(o.bounds.3) {
                    cells.entry((o.map, cx, cy)).or_default().push(i);
                }
            }
        }
        Grid { cells }
    }

    /// Obstacles in the cells that the box touches (each once, sorted).
    fn candidates(&self, map: usize, b: (f64, f64, f64, f64)) -> Vec<usize> {
        let mut found: Vec<usize> = Vec::new();
        for cx in cell(b.0)..=cell(b.2) {
            for cy in cell(b.1)..=cell(b.3) {
                if let Some(v) = self.cells.get(&(map, cx, cy)) {
                    found.extend(v);
                }
            }
        }
        found.sort_unstable();
        found.dedup();
        found
    }
}

/// How deep the body goes into the obstacle (m); `None` if they do not overlap.
/// For a rectangle: the separating axis test with the axes of the rectangle and
/// of the obstacle's edges (exact for convex obstacles, such as racks; a wall
/// segment is a degenerate polygon). For a circle: its radius minus the distance
/// to the obstacle's boundary (plus that distance if the center is inside).
fn body_depth(body: &Body, o: &Obstacle) -> Option<f64> {
    match body {
        Body::Circle(c) => {
            let centre = (c.c.x, c.c.y);
            let to_boundary = segments(&o.verts, o.closed)
                .map(|(a, b)| dist_point_segment(centre, a, b))
                .fold(f64::INFINITY, f64::min);
            let d = if o.closed && inside(&o.verts, centre) {
                c.r + to_boundary
            } else {
                c.r - to_boundary
            };
            (d >= 0.0).then_some(d)
        }
        Body::Rect(r) => {
            let corners: Vec<P> = r.corners().iter().map(|p| (p.x, p.y)).collect();
            let mut axes: Vec<P> = vec![(r.dir.x, r.dir.y), (r.normal().x, r.normal().y)];
            for (a, b) in segments(&o.verts, o.closed) {
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let len = dx.hypot(dy);
                if len > 0.0 {
                    axes.push((-dy / len, dx / len));
                }
            }
            let mut depth = f64::INFINITY;
            for (ax, ay) in axes {
                let (a0, a1) = extent(&corners, (ax, ay));
                let (b0, b1) = extent(&o.verts, (ax, ay));
                // the way out along this axis: one body past the other, either side
                depth = depth.min((a1 - b0).min(b1 - a0));
                if depth < 0.0 {
                    return None;
                }
            }
            depth.is_finite().then_some(depth)
        }
    }
}

/// The interval of the projections of the points on the axis.
fn extent(points: &[P], axis: P) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for p in points {
        let v = p.0 * axis.0 + p.1 * axis.1;
        lo = lo.min(v);
        hi = hi.max(v);
    }
    (lo, hi)
}

fn segments(v: &[P], closed: bool) -> impl Iterator<Item = (P, P)> + '_ {
    let n = v.len();
    let count = if closed { n } else { n.saturating_sub(1) };
    (0..count).map(move |i| (v[i], v[(i + 1) % n]))
}

/// Is the point inside the closed polygon (even-odd rule)?
fn inside(poly: &[P], p: P) -> bool {
    let mut in_poly = false;
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0 {
            in_poly = !in_poly;
        }
    }
    in_poly
}

fn dist_point_segment(p: P, a: P, b: P) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 == 0.0 {
        0.0
    } else {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
    };
    (p.0 - a.0 - t * dx).hypot(p.1 - a.1 - t * dy)
}

fn report_layout(
    layout: &Layout,
    graph: &Graph,
    obstacles: &[Obstacle],
    hits: &[Hit],
    robot: usize,
    t: &TypeRef,
    out: &mut Vec<Finding>,
) {
    let mine: Vec<&Hit> = hits
        .iter()
        .filter(|h| h.robot == robot)
        .filter(|h| {
            layout
                .edges
                .iter()
                .any(|e| e.edge_id == graph.edges[h.edge].id)
        })
        .collect();
    let Some(deepest) = mine.iter().max_by(|a, b| a.depth.total_cmp(&b.depth)) else {
        return;
    };
    let ids: Vec<&str> = mine
        .iter()
        .map(|h| graph.edges[h.edge].id.as_str())
        .collect();
    let idx = LayoutIndex::new(layout);
    out.push(
        Finding::new(
            Level::Warning,
            CHECK,
            format!(
                "{} edge(s): the outline of this robot type, driving the edge, goes into a wall, rack or shelf; deepest: {} into {} by {:.2} m",
                mine.len(),
                graph.edges[deepest.edge].id,
                obstacles[deepest.obstacle].id,
                deepest.depth
            ),
        )
        .layout(&layout.layout_id)
        .robot(&t.name)
        .with_places(idx.edges(ids.iter().copied()))
        .with_ids(ids),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use botsim_model::spec::Kinematics;

    fn lif() -> Lif {
        let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}"#;
        serde_json::from_str(&format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
            "nodes":[{{"nodeId":"A","mapId":"M","nodePosition":{{"x":0,"y":0}},"mobileRobotTypeNodeProperties":[{ty}]}},
                     {{"nodeId":"B","mapId":"M","nodePosition":{{"x":10,"y":0}},"mobileRobotTypeNodeProperties":[{ty}]}}],
            "edges":[{{"edgeId":"E1","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[{ty}]}}]}}]}}]}}"#
        ))
        .expect("test LIF parses")
    }

    fn geometry(shapes: &str) -> GeometryFile {
        GeometryFile::from_json_str(&format!(r#"{{"version":1,"shapes":[{shapes}]}}"#))
            .expect("test geometry parses")
    }

    fn rect(id: &str, kind: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
        format!(
            r#"{{"id":"{id}","mapId":"M","kind":"{kind}","shape":"polygon","vertices":[{{"x":{x0},"y":{y0}}},{{"x":{x1},"y":{y0}}},{{"x":{x1},"y":{y1}}},{{"x":{x0},"y":{y1}}}]}}"#
        )
    }

    fn x1(kinematics: Kinematics) -> Vec<TypeRef> {
        // body 1.0 x 0.6: half width 0.3
        vec![TypeRef::test_rect("ACME/X1", kinematics, 1.0, 0.6)]
    }

    #[test]
    fn rack_beside_the_edge_hits_only_when_close() {
        let t = x1(Kinematics::Differential);
        // rack 0.2 m from the centreline: the body (0.3 half width) hits it
        let near = geometry(&rect("R1", "rack", 3.0, 0.2, 5.0, 1.0));
        let f = check(&lif(), &t, &near, 0.25, 0.0);
        assert_eq!(f.len(), 1, "{f:?}");
        assert_eq!(f[0].ids, ["E1"]);
        // the body (half width 0.3) goes 0.1 m into the rack; a tolerance of 0.2 m hides it
        assert!(f[0].text.contains("E1 into R1 by 0.10 m"), "{}", f[0].text);
        assert!(check(&lif(), &t, &near, 0.25, 0.2).is_empty());
        // 0.5 m away: clear
        let far = geometry(&rect("R1", "rack", 3.0, 0.5, 5.0, 1.0));
        assert!(check(&lif(), &t, &far, 0.25, 0.0).is_empty());
        // exactly touching (0.3 m): not a hit
        let touch = geometry(&rect("R1", "rack", 3.0, 0.3, 5.0, 1.0));
        assert!(check(&lif(), &t, &touch, 0.25, 0.0).is_empty());
    }

    #[test]
    fn stations_are_not_obstacles_and_walls_are_segments() {
        let t = x1(Kinematics::Differential);
        // a station right on the edge: ignored
        let st = geometry(&rect("S1", "station", 3.0, -1.0, 5.0, 1.0));
        assert!(check(&lif(), &t, &st, 0.25, 0.0).is_empty());
        // a wall across the edge
        let wall = geometry(
            r#"{"id":"W","mapId":"M","kind":"wall","shape":"polyline","vertices":[{"x":5.0,"y":-3.0},{"x":5.0,"y":3.0}]}"#,
        );
        let f = check(&lif(), &t, &wall, 0.25, 0.0);
        assert_eq!(f.len(), 1);
        // the wall crosses the middle of the body (half length 0.5): depth 0.5
        assert!(
            f[0].text.contains("E1 into W[0] by 0.50 m"),
            "{}",
            f[0].text
        );
    }

    #[test]
    fn omnidirectional_circle_and_unknown_map() {
        // circle radius hypot(0.5, 0.3) = 0.583: a rack 0.5 m away is hit by the circle
        let rack = geometry(&rect("R1", "rack", 3.0, 0.5, 5.0, 1.0));
        assert_eq!(
            check(&lif(), &x1(Kinematics::Omnidirectional), &rack, 0.25, 0.0).len(),
            1
        );
        // a shape on a map the layout does not have: one info finding, no hit
        let other = geometry(
            r#"{"id":"R9","mapId":"ELSEWHERE","kind":"rack","shape":"polygon","vertices":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}]}"#,
        );
        let f = check(&lif(), &x1(Kinematics::Differential), &other, 0.25, 0.0);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].level, Level::Info);
    }
}

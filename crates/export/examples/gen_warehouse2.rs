//! Generates the layout of "warehouse 2": `scenarios/warehouse2.lif.json`,
//! `scenarios/warehouse2.geometry.json`, and a preview picture
//! `data/warehouse2_layout.png` (layout and geometry, no robots).
//!
//! ```sh
//! cargo run --release -p botsim-export --example gen_warehouse2
//! ```
//!
//! A 240 m × 140 m hall, one floor:
//! - a one-way **ring road** with rounded corners round the hall;
//! - in the west a **grid** of aisles with irregular spacing (so the blocks of racking
//!   between them differ in size); rows run east and west in turns, columns north
//!   and south in turns;
//! - in the middle and the east a second grid, and a **boulevard** (one lane east, one
//!   west) through the middle of the hall;
//! - in the north a **labyrinth**: one long serpentine aisle of eight runs between
//!   shelves, with half-circle U-turns;
//! - in the north-east **charging and parking bays**: a lane with a stall on each side
//!   at three bends of sidings, joined to the roads by arcs;
//! - goods-in bays (west) and goods-out bays (east) on curved one-way sidings;
//! - narrow **lanes along the south and the north wall** (a place for small robots),
//!   with shelving along the walls.
//!
//! Every edge lists all the robot types for now (`ACME`: `X1` carrier, `Y2`
//! forklift, `T3` three-wheel, `O4` omnidirectional, `C5` small round robot); who
//! may use what comes in a later step.

use std::collections::HashMap;
use std::f64::consts::PI;
use std::path::Path;

use botsim_core::graph::Graph;
use botsim_export::Renderer;
use botsim_record::{
    GEOMETRY_VERSION, GeometryFile, GeometryKind, GeometryShape, LayoutLine, Outline, SnapshotLine,
    Vertex,
};
use serde_json::{Value, json};

type P = (f64, f64);

const MAP_ID: &str = "M";
const TYPES: [&str; 5] = ["X1", "Y2", "T3", "O4", "C5"];

/// The hall.
const W: f64 = 240.0;
const H: f64 = 140.0;
/// The ring road: its distance from the wall and the radius of its corners.
const RING: f64 = 10.0;
const CORNER: f64 = 10.0;

/// Lines of the west grid: columns (x) and rows (y), irregularly spaced.
const WEST_X: [f64; 4] = [30.0, 52.0, 70.0, 100.0];
const WEST_Y: [f64; 6] = [28.0, 46.0, 62.0, 80.0, 98.0, 114.0];
/// Lines of the east grid.
const EAST_X: [f64; 4] = [140.0, 160.0, 184.0, 206.0];
/// The rows of the east grid, and the direction of each (true: east).
const EAST_ROWS: [(f64, bool); 6] = [
    (28.0, true),
    (46.0, false),
    (58.0, true),
    (66.0, true),  // the boulevard, east lane
    (74.0, false), // the boulevard, west lane
    (84.0, false),
];

/// The radius of the arcs at the junctions (m): about what a forklift needs.
const TURN_RADIUS: f64 = 3.5;

/// Where the sidings of the charging and parking stalls begin (y); each is 8 m long,
/// with a stall on each side of the lane at x = 212.
const STALLS_Y: [f64; 3] = [92.0, 101.0, 110.0];

/// The second labyrinth, between the two grids (x = 100 to 140, y = 10 to 66):
/// six aisles that run north and south in turns (the first north), from `V_BOTTOM`
/// to `V_TOP`. The column at x = 140, which runs north, is the seventh road: the last
/// half circle lands on it. The roads are `V_STEP` apart (about 5.8 m, which leaves
/// about 2.6 m between the rows of shelving).
const V_AISLES: usize = 6;
const V_X0: f64 = 105.0;
const V_XCOL: f64 = 140.0;
const V_STEP: f64 = (V_XCOL - V_X0) / V_AISLES as f64;
/// The aisles start at the height where the 45 degree link from the ring corner
/// (100, 10) turns into the first one with an arc as wide as the half circles.
const V_BOTTOM: f64 = 15.0 + V_STEP / 2.0;
const V_TOP: f64 = 58.5;

/// The labyrinth in the north (x = 118 to 190): seven aisles that run east and west
/// in turns (the first east), from `H_Y0` up; the top ring road, which runs west, is
/// the eighth road: the last half circle lands on it. The roads are `H_STEP` apart
/// (about 5.7 m).
const H_AISLES: usize = 7;
const H_Y0: f64 = 90.0;
const H_STEP: f64 = (H - RING - H_Y0) / H_AISLES as f64;
const H_XA: f64 = 118.0;
const H_XB: f64 = 190.0;

/// The height of the road `k` of the labyrinth in the north (`H_AISLES` is the ring).
fn h_y(k: usize) -> f64 {
    H_Y0 + H_STEP * k as f64
}

/// The parts of `range` that are left when the `gaps` are taken out (parts shorter
/// than one bay are dropped).
fn segments(range: (f64, f64), mut gaps: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    gaps.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut parts = Vec::new();
    let mut from = range.0;
    for (g0, g1) in gaps {
        if g0 > from {
            parts.push((from, g0.min(range.1)));
        }
        from = from.max(g1);
    }
    if from < range.1 {
        parts.push((from, range.1));
    }
    parts.retain(|(a, b)| b - a >= 2.7);
    parts
}

/// The x of the road `k` of the vertical labyrinth (`V_AISLES` is the column).
fn v_x(k: usize) -> f64 {
    V_X0 + V_STEP * k as f64
}

/// Where the shortcut between vertical aisle `k` and the next one is (y).
fn v_shortcut_y(k: usize) -> f64 {
    [44.0, 32.0, 50.0, 26.0, 44.0, 32.0][k]
}

/// Where the shortcut between labyrinth aisle `k` and the next one is (x). The
/// half circle bulges east for an even `k` (the aisle runs east), west for an odd one.
fn shortcut_x(k: usize) -> f64 {
    if k.is_multiple_of(2) {
        [136.0, 154.0][(k / 2) % 2]
    } else {
        [160.0, 178.0][(k / 2) % 2]
    }
}

// ---------------------------------------------------------------- the graph

struct Edge {
    a: usize,
    b: usize,
    /// For a curve: the point where the tangents at the two ends meet, and the
    /// weight of that control point (a circular arc).
    curve: Option<(P, f64)>,
}

/// A station (LIF): a place where robots pick up and drop loads, with the points
/// where they stand to do it (its interaction nodes).
struct Station {
    id: String,
    name: String,
    kind: &'static str,
    points: Vec<P>,
}

#[derive(Default)]
struct Net {
    nodes: Vec<P>,
    index: HashMap<(i64, i64), usize>,
    edges: Vec<Edge>,
    stations: Vec<Station>,
}

impl Net {
    /// A point where robots stand for a station (made if the station is new).
    fn add_station(&mut self, id: &str, name: &str, kind: &'static str, point: P) {
        match self.stations.iter_mut().find(|s| s.id == id) {
            Some(station) => station.points.push(point),
            None => self.stations.push(Station {
                id: id.into(),
                name: name.into(),
                kind,
                points: vec![point],
            }),
        }
    }

    /// The node at a point, if there is one.
    fn find(&self, p: P) -> Option<usize> {
        let key = ((p.0 * 100.0).round() as i64, (p.1 * 100.0).round() as i64);
        self.index.get(&key).copied()
    }

    /// The node at a point (made if there is none; points 1 cm apart are one node).
    fn node(&mut self, p: P) -> usize {
        let key = ((p.0 * 100.0).round() as i64, (p.1 * 100.0).round() as i64);
        *self.index.entry(key).or_insert_with(|| {
            self.nodes.push(p);
            self.nodes.len() - 1
        })
    }

    fn line(&mut self, a: P, b: P) {
        let (a, b) = (self.node(a), self.node(b));
        self.edges.push(Edge { a, b, curve: None });
    }

    /// A circular arc from `a` to `b` whose tangents (of equal length) meet at
    /// `corner`.
    fn arc(&mut self, a: P, corner: P, b: P) {
        let unit = |from: P, to: P| {
            let (dx, dy) = (to.0 - from.0, to.1 - from.1);
            let length = dx.hypot(dy);
            (dx / length, dy / length)
        };
        let (d1, d2) = (unit(a, corner), unit(corner, b));
        let turn = (d1.0 * d2.0 + d1.1 * d2.1).clamp(-1.0, 1.0).acos();
        let (a, b) = (self.node(a), self.node(b));
        self.edges.push(Edge {
            a,
            b,
            curve: Some((corner, (turn / 2.0).cos())),
        });
    }

    /// An S-shaped link of two equal arcs from `a` to `b`, both driving in the
    /// direction `d` (a unit vector) at `a` and at `b`; `b` is ahead of `a` and to
    /// one side. (Two arcs that each turn by the same angle, one way and then back.)
    fn s_link(&mut self, a: P, b: P, d: P) {
        let left = (-d.1, d.0);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let along = dx * d.0 + dy * d.1;
        let across = dx * left.0 + dy * left.1;
        let turn = 2.0 * (across.abs() / along).atan();
        // the tangent length of each arc: half the distance along, over 2 cos^2(turn/2)
        let t = along / (4.0 * (turn / 2.0).cos().powi(2));
        let side = across.signum();
        let d2 = (
            turn.cos() * d.0 + side * turn.sin() * left.0,
            turn.cos() * d.1 + side * turn.sin() * left.1,
        );
        let k1 = (a.0 + t * d.0, a.1 + t * d.1);
        let m = (k1.0 + t * d2.0, k1.1 + t * d2.1);
        let k2 = (m.0 + t * d2.0, m.1 + t * d2.1);
        self.arc(a, k1, m);
        self.arc(m, k2, b);
    }

    /// A siding that bows out from a straight road: from `a` to `b` through the
    /// apex `m`, where the robot drives parallel to the road (two arcs).
    fn bow(&mut self, a: P, m: P, b: P) {
        // the apex is beside the middle of the road between a and b; the tangents at
        // a and b meet the line through the apex (parallel to the road) at the
        // distance that makes the two tangents of an arc equally long
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let length = dx.hypot(dy);
        let u = (dx / length, dy / length);
        let mid = ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
        let off = (m.0 - mid.0, m.1 - mid.1);
        let side = off.0.hypot(off.1);
        let half = length / 2.0;
        let d = (half * half - side * side) / (2.0 * half);
        let k1 = (a.0 + u.0 * d + off.0, a.1 + u.1 * d + off.1);
        let k2 = (b.0 - u.0 * d + off.0, b.1 - u.1 * d + off.1);
        self.arc(a, k1, m);
        self.arc(m, k2, b);
    }

    /// At every turn between two straight roads (a road that ends on another, a
    /// branch that leaves one) add a circular arc of the given radius between them,
    /// so that a robot that cannot turn on the spot (a forklift, a three-wheel robot)
    /// can drive the turn. Straight on and U-turns get none; neither do turns on
    /// roads too short for the arc (they stay corners). The roads are cut where the
    /// arcs begin and end, so the corner stays for the robots that can turn on the spot.
    fn add_turn_arcs(&mut self, radius: f64) {
        let mut ins: Vec<Vec<usize>> = vec![Vec::new(); self.nodes.len()];
        let mut outs: Vec<Vec<usize>> = vec![Vec::new(); self.nodes.len()];
        for e in self.edges.iter().filter(|e| e.curve.is_none()) {
            outs[e.a].push(e.b);
            ins[e.b].push(e.a);
        }
        let mut arcs: Vec<(P, P, P)> = Vec::new();
        for n in 0..self.nodes.len() {
            let p = self.nodes[n];
            for &a in &ins[n] {
                for &b in &outs[n] {
                    let (pa, pb) = (self.nodes[a], self.nodes[b]);
                    let (len_in, len_out) = (
                        (p.0 - pa.0).hypot(p.1 - pa.1),
                        (pb.0 - p.0).hypot(pb.1 - p.1),
                    );
                    if len_in < 1e-9 || len_out < 1e-9 {
                        continue;
                    }
                    let d_in = ((p.0 - pa.0) / len_in, (p.1 - pa.1) / len_in);
                    let d_out = ((pb.0 - p.0) / len_out, (pb.1 - p.1) / len_out);
                    let turn = (d_in.0 * d_out.0 + d_in.1 * d_out.1)
                        .clamp(-1.0, 1.0)
                        .acos();
                    // (a turn of more than 100 degrees would need a huge arc: it stays
                    // a corner)
                    if !(25f64.to_radians()..=100f64.to_radians()).contains(&turn) {
                        continue;
                    }
                    let t = radius * (turn / 2.0).tan();
                    if len_in < 2.0 * t + 0.5 || len_out < 2.0 * t + 0.5 {
                        continue;
                    }
                    arcs.push((
                        (p.0 - d_in.0 * t, p.1 - d_in.1 * t),
                        p,
                        (p.0 + d_out.0 * t, p.1 + d_out.1 * t),
                    ));
                }
            }
        }
        for (a, corner, b) in arcs {
            self.arc(a, corner, b);
        }
        self.split_at_nodes();
    }

    /// Cut every straight edge at the nodes that lie on it (where another road
    /// begins, ends or crosses), so that roads join at their crossings.
    fn split_at_nodes(&mut self) {
        let old = std::mem::take(&mut self.edges);
        for e in old {
            if e.curve.is_some() {
                self.edges.push(e);
                continue;
            }
            let (a, b) = (self.nodes[e.a], self.nodes[e.b]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let len2 = dx * dx + dy * dy;
            let mut on: Vec<(f64, usize)> = Vec::new();
            for (i, p) in self.nodes.iter().enumerate() {
                if i == e.a || i == e.b {
                    continue;
                }
                let t = ((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2;
                let off = ((p.0 - a.0) * dy - (p.1 - a.1) * dx).abs() / len2.sqrt();
                if t > 1e-9 && t < 1.0 - 1e-9 && off < 1e-6 {
                    on.push((t, i));
                }
            }
            on.sort_by(|x, y| x.0.total_cmp(&y.0));
            let mut from = e.a;
            for (_, i) in on {
                self.edges.push(Edge {
                    a: from,
                    b: i,
                    curve: None,
                });
                from = i;
            }
            self.edges.push(Edge {
                a: from,
                b: e.b,
                curve: None,
            });
        }
    }
}

fn id(n: usize) -> String {
    format!("N{n:03}")
}

/// The nodes that cannot be reached from node 0, or from which node 0 cannot be
/// reached (none when the net is strongly connected), with their position.
fn not_strongly_connected(net: &Net) -> Vec<(P, &'static str)> {
    let lost = |forward: bool| {
        let mut next = vec![Vec::new(); net.nodes.len()];
        for e in &net.edges {
            let (from, to) = if forward { (e.a, e.b) } else { (e.b, e.a) };
            next[from].push(to);
        }
        let mut seen = vec![false; net.nodes.len()];
        let mut todo = vec![0];
        seen[0] = true;
        while let Some(n) = todo.pop() {
            for &m in &next[n] {
                if !seen[m] {
                    seen[m] = true;
                    todo.push(m);
                }
            }
        }
        seen.iter()
            .enumerate()
            .filter(|(_, s)| !**s)
            .map(|(i, _)| net.nodes[i])
            .collect::<Vec<P>>()
    };
    let mut out: Vec<(P, &'static str)> = lost(true)
        .into_iter()
        .map(|p| (p, "cannot be reached from node 0"))
        .collect();
    out.extend(lost(false).into_iter().map(|p| (p, "cannot reach node 0")));
    out
}

/// A straight road from `a` to `b`, or from `b` to `a` if not `forward`.
fn road(net: &mut Net, a: P, b: P, forward: bool) {
    if forward {
        net.line(a, b);
    } else {
        net.line(b, a);
    }
}

/// A serpentine of `runs` aisles `step` apart between `xa` and `xb`, the first at
/// height `y`, going east; half circles join the ends. Returns where it starts
/// and where it ends (the end is at `xa` if `runs` is even).
fn serpentine(net: &mut Net, xa: f64, xb: f64, y: f64, step: f64, runs: usize) -> (P, P) {
    let radius = step / 2.0;
    for k in 0..runs {
        let yk = y + step * k as f64;
        let east = k.is_multiple_of(2);
        let (from, to) = if east { (xa, xb) } else { (xb, xa) };
        net.line((from, yk), (to, yk));
        if k + 1 < runs {
            let out = if east { xb + radius } else { xa - radius };
            net.arc((to, yk), (out, yk), (out, yk + radius));
            net.arc((out, yk + radius), (out, yk + step), (to, yk + step));
        }
    }
    let last = y + step * (runs - 1) as f64;
    let end = if runs.is_multiple_of(2) { xa } else { xb };
    ((xa, y), (end, last))
}

/// The vertical labyrinth: aisles joined by half circles at the ends, shortcuts
/// through gaps in the shelving, in from the ring road at the bottom left and out to
/// the boulevard at the top right, each by a link at 45 degrees that turns into the
/// aisle with an arc.
fn vertical_labyrinth(net: &mut Net) {
    let r = V_STEP / 2.0;
    // in: from the corner where the ring meets the column at x = 100, a link at 45
    // degrees and an arc into the first aisle
    let t = r; // the tangent length of the 45 degree arc
    let d = t * std::f64::consts::FRAC_1_SQRT_2;
    let x0 = v_x(0);
    net.line((100.0, 10.0), (x0 - d, V_BOTTOM - t - d));
    net.arc(
        (x0 - d, V_BOTTOM - t - d),
        (x0, V_BOTTOM - t),
        (x0, V_BOTTOM),
    );
    for k in 0..V_AISLES {
        let x = v_x(k);
        if k.is_multiple_of(2) {
            net.line((x, V_BOTTOM), (x, V_TOP));
        } else {
            net.line((x, V_TOP), (x, V_BOTTOM));
        }
        // the half circle to the next road (the next aisle; for the last aisle the
        // column), at the end the robot has reached
        if k.is_multiple_of(2) {
            net.arc((x, V_TOP), (x, V_TOP + r), (x + r, V_TOP + r));
            net.arc(
                (x + r, V_TOP + r),
                (x + 2.0 * r, V_TOP + r),
                (x + 2.0 * r, V_TOP),
            );
        } else {
            net.arc((x, V_BOTTOM), (x, V_BOTTOM - r), (x + r, V_BOTTOM - r));
            net.arc(
                (x + r, V_BOTTOM - r),
                (x + 2.0 * r, V_BOTTOM - r),
                (x + 2.0 * r, V_BOTTOM),
            );
        }
        // the shortcut half way, through a gap in the shelving
        let y = v_shortcut_y(k);
        let up = if k.is_multiple_of(2) { r } else { -r };
        net.arc((x, y), (x, y + up), (x + r, y + up));
        net.arc((x + r, y + up), (x + 2.0 * r, y + up), (x + 2.0 * r, y));
    }
    // shortcuts along the outside: the half circles at the top (after the aisles 0,
    // 2, 4) and the half circles at the bottom (after 1, 3, 5) are joined by straight
    // roads from apex to apex, where the robot drives straight east: it skips two
    // aisles and meets no corner sharper than the half circle itself
    for k in (0..V_AISLES - 2).step_by(2) {
        net.line((v_x(k) + r, V_TOP + r), (v_x(k + 2) + r, V_TOP + r));
    }
    for k in (1..V_AISLES - 1).step_by(2) {
        net.line((v_x(k) + r, V_BOTTOM - r), (v_x(k + 2) + r, V_BOTTOM - r));
    }
    // the two roads along the outside go on to the roads that enter and leave the
    // labyrinth, so that a robot can pass it without winding through it:
    // - the top road goes on east and leaves to the node where the column at x = 140
    //   meets the boulevard, by an arc and a link at 45 degrees;
    let y_top = V_TOP + r;
    let corner_x = V_XCOL - (66.0 - y_top); // where y = y_top meets the link
    net.line((v_x(V_AISLES - 2) + r, y_top), (corner_x - t, y_top));
    net.arc(
        (corner_x - t, y_top),
        (corner_x, y_top),
        (corner_x + d, y_top + d),
    );
    net.line((corner_x + d, y_top + d), (V_XCOL, 66.0));
    // - the bottom road starts from the entry link (it leaves it by an arc that turns
    //   45 degrees to the east); its other end is the last half circle, which lands
    //   on the column
    net.arc(
        (x0 - d, V_BOTTOM - t - d),
        (x0, V_BOTTOM - t),
        (x0 + t, V_BOTTOM - t),
    );
    net.line((x0 + t, V_BOTTOM - t), (v_x(1) + r, V_BOTTOM - r));
}

/// A ring road round `c`: eight arcs of 45 degrees, counter-clockwise if `ccw`.
fn ring(net: &mut Net, c: P, r: f64, ccw: bool) {
    let point = |deg: f64| {
        (
            c.0 + r * deg.to_radians().cos(),
            c.1 + r * deg.to_radians().sin(),
        )
    };
    let d = r / (PI / 8.0).cos();
    let corner = |deg: f64| {
        (
            c.0 + d * (deg + 22.5).to_radians().cos(),
            c.1 + d * (deg + 22.5).to_radians().sin(),
        )
    };
    for k in 0..8 {
        let deg = 45.0 * k as f64;
        if ccw {
            net.arc(point(deg), corner(deg), point(deg + 45.0));
        } else {
            net.arc(point(deg + 45.0), corner(deg), point(deg));
        }
    }
}

/// Which way the roads of the west grid run (east?) at height `y`: the rows run east
/// and west in turns; the ring road at the bottom runs east, at the top west.
fn west_row_runs_east(y: f64) -> bool {
    if (y - RING).abs() < 1e-9 {
        return true;
    }
    if (y - (H - RING)).abs() < 1e-9 {
        return false;
    }
    WEST_Y
        .iter()
        .position(|&row| (row - y).abs() < 1e-9)
        .is_some_and(|i| i.is_multiple_of(2))
}

/// The cells of the west grid are between these lines (the ring at the sides).
const WEST_BOUNDS_X: [f64; 5] = [RING, WEST_X[0], WEST_X[1], WEST_X[2], WEST_X[3]];
const WEST_BOUNDS_Y: [f64; 8] = [
    RING,
    WEST_Y[0],
    WEST_Y[1],
    WEST_Y[2],
    WEST_Y[3],
    WEST_Y[4],
    WEST_Y[5],
    H - RING,
];

/// The cells of the east grid: between these columns, and between these rows (the
/// strip between the two lanes of the boulevard has no racking).
const EAST_BOUNDS_X: [f64; 5] = [EAST_X[0], EAST_X[1], EAST_X[2], EAST_X[3], W - RING];
const EAST_BOUNDS_Y: [(f64, f64); 5] = [
    (RING, 28.0),
    (28.0, 46.0),
    (46.0, 58.0),
    (58.0, 66.0),
    (74.0, 84.0),
];

/// Which way the roads of the east grid run (east?) at height `y`.
fn east_row_runs_east(y: f64) -> bool {
    if (y - RING).abs() < 1e-9 {
        return true;
    }
    EAST_ROWS
        .iter()
        .find(|(row, _)| (row - y).abs() < 1e-9)
        .is_none_or(|&(_, east)| east)
}

/// A stop in front of a rack block: a siding 8 m long beside the road in front of
/// the block, bowing 1.8 m towards the rack face. A robot leaves the road, stands at
/// the apex (the stop) with its side to the rack, picks or drops, and rejoins the
/// road, so it does not block the aisle while it works.
struct Pocket {
    /// The station: a rack block (its two sides are two stops of one station).
    station: String,
    a: P,
    b: P,
    apex: P,
}

fn cell_pockets(
    out: &mut Vec<Pocket>,
    station: &str,
    (xl, xr): (f64, f64),
    (yb, yt): (f64, f64),
    runs_east: fn(f64) -> bool,
) {
    let cx = (xl + xr) / 2.0;
    for (y, bulge) in [(yb, 1.8), (yt, -1.8)] {
        let ring = (y - RING).abs() < 1e-9 || (y - (H - RING)).abs() < 1e-9;
        let (lo, hi) = (cx - 4.0, cx + 4.0);
        // not in the corners of the ring road
        if ring && (lo < RING + CORNER + 1.0 || hi > W - RING - CORNER - 1.0) {
            continue;
        }
        let (a, b) = if runs_east(y) {
            ((lo, y), (hi, y))
        } else {
            ((hi, y), (lo, y))
        };
        out.push(Pocket {
            station: station.into(),
            a,
            b,
            apex: (cx, y + bulge),
        });
    }
}

/// The stops in front of the rack blocks of both grids: one or two per block (on the
/// road below it and on the road above it).
fn rack_pockets() -> Vec<Pocket> {
    let mut out = Vec::new();
    for (i, x) in WEST_BOUNDS_X.windows(2).enumerate() {
        for (j, y) in WEST_BOUNDS_Y.windows(2).enumerate() {
            if !is_staging(i, j) {
                let name = format!("RW{i}{j}");
                cell_pockets(
                    &mut out,
                    &name,
                    (x[0], x[1]),
                    (y[0], y[1]),
                    west_row_runs_east,
                );
            }
        }
    }
    for (i, x) in EAST_BOUNDS_X.windows(2).enumerate() {
        for (j, y) in EAST_BOUNDS_Y.iter().enumerate() {
            let name = format!("RE{i}{j}");
            cell_pockets(&mut out, &name, (x[0], x[1]), *y, east_row_runs_east);
        }
    }
    out
}

/// Is the cell `(i, j)` of the west grid a staging cell (a roundabout instead of
/// racking)?
fn is_staging(i: usize, j: usize) -> bool {
    (i * 3 + j * 5) % 11 == 4
}

/// A roundabout in a cell between two roads that run the opposite way (the road
/// below east, the one above west, or the other way round). It is tangent to both:
/// where the ring road runs the same way as the road, an S-shaped link of two arcs
/// joins the road to it, so a robot enters and leaves it with no sharp turn. Each
/// road has a link in to the ring and a link out of it. The links start where the
/// arcs of the junctions at the sides of the cell end.
fn roundabout_in_cell(net: &mut Net, (xl, xr): (f64, f64), (yb, yt): (f64, f64)) {
    let (cx, cy) = ((xl + xr) / 2.0, (yb + yt) / 2.0);
    let rho = (yt - yb) / 2.0 - 2.5;
    let reach = (xr - xl) / 2.0 - TURN_RADIUS;
    let east = west_row_runs_east(yb);
    let db = if east { 1.0 } else { -1.0 };
    ring(net, (cx, cy), rho, east);
    let (south, north) = ((cx, cy - rho), (cx, cy + rho));
    net.s_link((cx - db * reach, yb), south, (db, 0.0));
    net.s_link(south, (cx + db * reach, yb), (db, 0.0));
    net.s_link((cx + db * reach, yt), north, (-db, 0.0));
    net.s_link(north, (cx - db * reach, yt), (-db, 0.0));
}

fn build_net() -> Net {
    let mut net = Net::default();
    let (x0, x1, y0, y1) = (RING, W - RING, RING, H - RING);
    let r = CORNER;

    // the ring road, counter-clockwise: the bottom east, the right side north, the
    // top west, the left side south; the corners are arcs
    net.arc((x1 - r, y0), (x1, y0), (x1, y0 + r));
    net.arc((x1, y1 - r), (x1, y1), (x1 - r, y1));
    net.arc((x0 + r, y1), (x0, y1), (x0, y1 - r));
    net.arc((x0, y0 + r), (x0, y0), (x0 + r, y0));
    net.line((x0 + r, y0), (x1 - r, y0));
    net.line((x1, y0 + r), (x1, y1 - r));
    net.line((x1 - r, y1), (x0 + r, y1));
    net.line((x0, y1 - r), (x0, y0 + r));

    // the west grid: rows from the ring to the column at x = 100, east and west in
    // turns; columns from the bottom ring to the top ring, north and south in turns
    for (i, &y) in WEST_Y.iter().enumerate() {
        road(&mut net, (x0, y), (100.0, y), i % 2 == 0);
    }
    for (i, &x) in WEST_X.iter().enumerate() {
        road(&mut net, (x, y1), (x, y0), i % 2 == 1);
        for &y in &WEST_Y {
            net.node((x, y));
        }
    }
    // the staging cells of the west grid are roundabouts
    for (i, x) in WEST_BOUNDS_X.windows(2).enumerate() {
        for (j, y) in WEST_BOUNDS_Y.windows(2).enumerate() {
            if is_staging(i, j) {
                roundabout_in_cell(&mut net, (x[0], x[1]), (y[0], y[1]));
            }
        }
    }

    // the east grid and the boulevard: rows from x = 140 (the boulevard and the top
    // row from x = 100) to the ring; columns from the bottom ring up to the top row
    for &(y, east) in &EAST_ROWS {
        let from = if y == 66.0 || y == 74.0 || y == 84.0 {
            100.0
        } else {
            140.0
        };
        road(&mut net, (from, y), (x1, y), east);
    }
    for (i, &x) in EAST_X.iter().enumerate() {
        road(&mut net, (x, 84.0), (x, y0), i % 2 == 1);
        for &(y, _) in &EAST_ROWS {
            net.node((x, y));
        }
    }

    // the labyrinth in the north: seven aisles, in from the column at x = 100 at the
    // bottom; the last half circle lands on the top ring road, which is the way out
    let (start, _) = serpentine(&mut net, H_XA, H_XB, H_Y0, H_STEP, H_AISLES);
    net.line((100.0, start.1), start);
    let r = H_STEP / 2.0;
    let y_last = h_y(H_AISLES - 1);
    net.arc((H_XB, y_last), (H_XB + r, y_last), (H_XB + r, y_last + r));
    net.arc(
        (H_XB + r, y_last + r),
        (H_XB + r, y_last + H_STEP),
        (H_XB, y_last + H_STEP),
    );
    // shortcuts: a half circle through a gap in the shelving between every two
    // roads, so that a robot can go over to the next one half way (it keeps its
    // direction there: the next road runs the other way)
    for k in 0..H_AISLES {
        let y = h_y(k);
        let xs = shortcut_x(k);
        let out = if k.is_multiple_of(2) { xs + r } else { xs - r };
        net.arc((xs, y), (out, y), (out, y + r));
        net.arc((out, y + r), (out, y + H_STEP), (xs, y + H_STEP));
    }

    // shortcuts along the outside of the labyrinth in the north: the half circles at
    // one end are joined by straight roads that leave each half circle at its apex,
    // where the robot drives straight north, and join the next half circle there: a
    // robot skips two aisles and meets no corner sharper than the half circle itself
    // (the east road ends in the last half circle, the way out)
    let east_apex = |k: usize| (H_XB + r, h_y(k) + r);
    let west_apex = |k: usize| (H_XA - r, h_y(k) + r);
    for k in (0..H_AISLES - 1).step_by(2) {
        net.line(east_apex(k), east_apex(k + 2));
    }
    for k in (1..H_AISLES - 2).step_by(2) {
        net.line(west_apex(k), west_apex(k + 2));
    }

    vertical_labyrinth(&mut net);

    // charging and parking bays in the north-east: a lane north from the row at
    // y = 84 to the top ring road, in by an arc from the row (which runs west) and
    // out by an arc onto the ring (which also runs west), so that a robot that cannot
    // turn on the spot can drive it; a stall on each side at every bend of a siding
    let (lane, r) = (212.0, 6.0);
    net.arc((lane + r, 84.0), (lane, 84.0), (lane, 84.0 + r));
    net.line((lane, 84.0 + r), (lane, y1 - r));
    net.arc((lane, y1 - r), (lane, y1), (lane - r, y1));
    for y in STALLS_Y {
        net.bow((lane, y), (lane - 3.0, y + 4.0), (lane, y + 8.0));
        net.bow((lane, y), (lane + 3.0, y + 4.0), (lane, y + 8.0));
        // the stalls are one station: a robot that is sent to park draws one of them
        net.add_station(
            "PARK",
            "Parking and charging",
            "parking",
            (lane - 3.0, y + 4.0),
        );
        net.add_station(
            "PARK",
            "Parking and charging",
            "parking",
            (lane + 3.0, y + 4.0),
        );
    }

    // bays on curved one-way sidings that rejoin the ring further on: west (the left
    // side runs south) from `a` down to `a - 10`, east (it runs north) up to `a + 10`
    for (n, a) in [111.0, 95.0, 77.0, 59.0, 43.0].into_iter().enumerate() {
        net.bow((x0, a), (x0 - 3.0, a - 5.0), (x0, a - 10.0));
        let name = format!("Goods in {}", n + 1);
        net.add_station(
            &format!("GOODS-IN-{}", n + 1),
            &name,
            "bay",
            (x0 - 3.0, a - 5.0),
        );
    }
    // (the upper two are 8 m long: the straight part of the ring ends at y = 120)
    for (n, (a, length)) in [(33.0, 10.0), (100.0, 8.0), (110.0, 8.0)]
        .into_iter()
        .enumerate()
    {
        net.bow((x1, a), (x1 + 3.0, a + length / 2.0), (x1, a + length));
        let name = format!("Goods out {}", n + 1);
        let apex = (x1 + 3.0, a + length / 2.0);
        net.add_station(&format!("GOODS-OUT-{}", n + 1), &name, "bay", apex);
    }

    // a stop in front of every rack block, in a pocket off the road (see `rack_pockets`)
    for p in rack_pockets() {
        net.bow(p.a, p.apex, p.b);
        net.add_station(&p.station, &format!("Rack {}", p.station), "rack", p.apex);
    }

    // narrow lanes along the south wall (east) and the north wall (west)
    net.line((40.0, y0), (40.0, 4.0));
    net.line((40.0, 4.0), (200.0, 4.0));
    net.line((200.0, 4.0), (200.0, y0));
    net.line((200.0, y1), (200.0, H - 4.0));
    net.line((200.0, H - 4.0), (40.0, H - 4.0));
    net.line((40.0, H - 4.0), (40.0, y1));

    net.split_at_nodes();
    // every turn between straight roads gets an arc of 3.5 m radius
    net.add_turn_arcs(TURN_RADIUS);
    net
}

fn lif(net: &Net) -> Value {
    let types: Vec<Value> = TYPES
        .iter()
        .map(|s| json!({"manufacturer": "ACME", "seriesName": s}))
        .collect();
    // the stations, with the nodes where robots stand to interact with them
    let stations: Vec<Value> = net
        .stations
        .iter()
        .map(|s| {
            let nodes: Vec<String> = s
                .points
                .iter()
                .filter_map(|&p| net.find(p))
                .map(id)
                .collect();
            json!({
                "stationId": s.id,
                "stationName": s.name,
                "stationType": s.kind,
                "interactionNodeIds": nodes,
                "stationPosition": {"x": s.points[0].0, "y": s.points[0].1},
            })
        })
        .collect();
    let nodes: Vec<Value> = net
        .nodes
        .iter()
        .enumerate()
        .map(|(i, p)| {
            json!({
                "nodeId": id(i),
                "mapId": MAP_ID,
                "nodePosition": {"x": p.0, "y": p.1},
                "mobileRobotTypeNodeProperties": [{"mobileRobotTypes": types}],
            })
        })
        .collect();
    let edges: Vec<Value> = net
        .edges
        .iter()
        .map(|e| {
            let mut props = json!({
                "mobileRobotTypes": types,
                "orientationType": "TANGENTIAL",
                "mobileRobotOrientations": [0.0],
            });
            if let Some((corner, weight)) = e.curve {
                let (a, b) = (net.nodes[e.a], net.nodes[e.b]);
                props["trajectory"] = json!({
                    "degree": 2,
                    "controlPoints": [
                        {"x": a.0, "y": a.1},
                        {"x": corner.0, "y": corner.1, "weight": weight},
                        {"x": b.0, "y": b.1},
                    ],
                });
            }
            json!({
                "edgeId": format!("{}-{}", id(e.a), id(e.b)),
                "startNodeId": id(e.a),
                "endNodeId": id(e.b),
                "mobileRobotTypeEdgeProperties": [props],
            })
        })
        .collect();
    json!({
        "metaInformation": {
            "lifId": "warehouse2",
            "creator": "radevgit",
            "exportTimestamp": "2026-11-22T10:00:00.000Z",
            "lifVersion": "2.0.0",
        },
        "origins": [{
            "originId": "O1",
            "layouts": [{
                "layoutId": "L1",
                "layoutVersion": "1",
                "nodes": nodes,
                "edges": edges,
                "stations": stations,
            }],
        }],
    })
}

// ------------------------------------------------------------- the geometry

fn mm(v: f64) -> f64 {
    (v * 1000.0).round() / 1000.0
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Vertex> {
    [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        .map(|(x, y)| Vertex { x: mm(x), y: mm(y) })
        .to_vec()
}

/// A rectangle given by its corners `(x0, y0, x1, y1)`.
type R = (f64, f64, f64, f64);

#[derive(Default)]
struct Shapes(Vec<GeometryShape>);

impl Shapes {
    fn add(&mut self, id: String, kind: GeometryKind, outline: Outline, vertices: Vec<Vertex>) {
        self.0.push(GeometryShape {
            id: Some(id),
            map_id: MAP_ID.into(),
            kind,
            shape: outline,
            vertices,
        });
    }

    fn polygon(&mut self, id: String, kind: GeometryKind, r: R) {
        self.add(id, kind, Outline::Polygon, rect(r.0, r.1, r.2, r.3));
    }

    /// A row of bays (an upright frame between two bays is the gap): along x from
    /// `a` to `b`, across from `c` to `d` (or the other way round if not `along_x`);
    /// the bays are centred in the length.
    fn row(
        &mut self,
        name: &str,
        kind: GeometryKind,
        along_x: bool,
        ab: (f64, f64),
        cd: (f64, f64),
    ) {
        const BAY: f64 = 2.7;
        const GAP: f64 = 0.1;
        let (a, b) = ab;
        let count = (((b - a) + GAP) / (BAY + GAP)).floor().max(1.0) as usize;
        let used = count as f64 * BAY + (count - 1) as f64 * GAP;
        let start = a + ((b - a) - used) / 2.0;
        for i in 0..count {
            let p = start + i as f64 * (BAY + GAP);
            let r = if along_x {
                (p, cd.0, p + BAY, cd.1)
            } else {
                (cd.0, p, cd.1, p + BAY)
            };
            self.polygon(format!("{name}-{i}"), kind, r);
        }
    }

    /// The shelving of the vertical labyrinth: a row between every two aisles, in two
    /// parts with a gap where the shortcut between the two aisles goes through.
    fn vertical_labyrinth(&mut self) {
        let r = V_STEP / 2.0;
        for k in 0..V_AISLES {
            let x = v_x(k);
            let y = v_shortcut_y(k);
            // the half circle of the shortcut bulges north from an even aisle, south
            // from an odd one
            let shortcut = if k.is_multiple_of(2) {
                (y - 1.0, y + r + 1.5)
            } else {
                (y - r - 1.5, y + 1.0)
            };
            let gaps = vec![shortcut];
            let across = (x + 1.3, x + V_STEP - 1.3);
            for (i, part) in segments((V_BOTTOM + 3.0, V_TOP - 3.0), gaps)
                .into_iter()
                .enumerate()
            {
                self.row(
                    &format!("shelf-v{k}-{i}"),
                    GeometryKind::Shelf,
                    false,
                    part,
                    across,
                );
            }
        }
    }

    /// A block of pallet racking: back-to-back pairs of rows (1.1 m deep each, with
    /// 1.2 m between the pairs); the rows run along the longer side.
    fn rack_block(&mut self, name: &str, r: R) {
        const DEPTH: f64 = 1.1;
        const PAIR: f64 = 2.0 * DEPTH + 0.1;
        const PITCH: f64 = PAIR + 1.2;
        let along_x = (r.2 - r.0) >= (r.3 - r.1);
        let (along, cross) = if along_x {
            ((r.0, r.2), (r.1, r.3))
        } else {
            ((r.1, r.3), (r.0, r.2))
        };
        if cross.1 - cross.0 < PAIR || along.1 - along.0 < 2.7 {
            return;
        }
        let pairs = (((cross.1 - cross.0) + 1.2) / PITCH).floor().max(1.0) as usize;
        let used = pairs as f64 * PITCH - 1.2;
        let start = cross.0 + ((cross.1 - cross.0) - used) / 2.0;
        for p in 0..pairs {
            let c = start + p as f64 * PITCH;
            for (side, lo_hi) in [(c, c + DEPTH), (c + DEPTH + 0.1, c + PAIR)]
                .into_iter()
                .enumerate()
            {
                self.row(
                    &format!("{name}-{p}-{side}"),
                    GeometryKind::Rack,
                    along_x,
                    along,
                    lo_hi,
                );
            }
        }
    }
}

/// The cells between consecutive lines: `xs` and `ys` are the lines (with the
/// walls of the grid as the first and last), each cell is shrunk by `inset`.
fn cells(xs: &[f64], ys: &[f64], inset: f64) -> Vec<(usize, usize, R)> {
    let mut out = Vec::new();
    for (i, x) in xs.windows(2).enumerate() {
        for (j, y) in ys.windows(2).enumerate() {
            out.push((
                i,
                j,
                (x[0] + inset, y[0] + inset, x[1] - inset, y[1] - inset),
            ));
        }
    }
    out
}

fn build_geometry() -> GeometryFile {
    let mut s = Shapes::default();
    let inset = 2.0;

    // the walls
    let wall = [(0.0, 0.0), (W, 0.0), (W, H), (0.0, H), (0.0, 0.0)]
        .map(|(x, y)| Vertex { x: mm(x), y: mm(y) });
    s.add(
        "wall".into(),
        GeometryKind::Wall,
        Outline::Polyline,
        wall.to_vec(),
    );

    // bays: stations at the apex of each siding
    for (name, x, ys) in [
        ("W", RING - 3.0, vec![106.0, 90.0, 72.0, 54.0, 38.0]),
        ("E", W - RING + 3.0, vec![38.0, 104.0, 114.0]),
    ] {
        for (i, y) in ys.iter().enumerate() {
            s.polygon(
                format!("bay-{name}{i}"),
                GeometryKind::Station,
                (x - 1.6, y - 1.6, x + 1.6, y + 1.6),
            );
        }
    }

    // the west grid: its cells; the staging cells are roundabouts with a stack of
    // pallets in the middle
    for (i, j, r) in cells(&WEST_BOUNDS_X, &WEST_BOUNDS_Y, inset) {
        let name = format!("w{i}{j}");
        if is_staging(i, j) {
            let (cx, cy) = ((r.0 + r.2) / 2.0, (r.1 + r.3) / 2.0);
            s.rack_block(
                &format!("island-{name}"),
                (cx - 3.0, cy - 3.0, cx + 3.0, cy + 3.0),
            );
        } else {
            s.rack_block(&format!("rack-{name}"), r);
        }
    }

    // the stops in front of the rack blocks: a small station square at each
    for (k, p) in rack_pockets().iter().enumerate() {
        s.polygon(
            format!("stop-{}-{k}", p.station),
            GeometryKind::Station,
            (
                p.apex.0 - 0.7,
                p.apex.1 - 0.7,
                p.apex.0 + 0.7,
                p.apex.1 + 0.7,
            ),
        );
    }

    // the middle: a big block south of the boulevard, a small one north of it (the
    // lane at y = 84 is above), and one beside the labyrinth's entry
    s.vertical_labyrinth();
    s.rack_block("rack-m1", (102.0, 74.0 + inset, 138.0, 84.0 - inset));
    s.rack_block("rack-m2", (102.0, 94.0, 114.0, 121.0));

    // the east grid
    let exs = [EAST_X[0], EAST_X[1], EAST_X[2], EAST_X[3], W - RING];
    let eys = [RING, 28.0, 46.0, 58.0, 66.0];
    for (i, j, r) in cells(&exs, &eys, inset) {
        s.rack_block(&format!("rack-e{i}{j}"), r);
    }
    for (i, _, r) in cells(&exs, &[74.0, 84.0], inset) {
        s.rack_block(&format!("rack-n{i}"), r);
    }

    // the labyrinth: a row of shelving between every two aisles
    // (with a gap where the shortcut between the two aisles goes through)
    let r = H_STEP / 2.0;
    for k in 0..H_AISLES {
        let y = h_y(k);
        let xs = shortcut_x(k);
        let shortcut = if k.is_multiple_of(2) {
            (xs - 1.0, xs + r + 1.5)
        } else {
            (xs - r - 1.5, xs + 1.0)
        };
        let gaps = vec![shortcut];
        for (i, part) in segments((122.0, 186.0), gaps).into_iter().enumerate() {
            s.row(
                &format!("shelf-l{k}-{i}"),
                GeometryKind::Shelf,
                true,
                part,
                (y + 1.3, y + H_STEP - 1.3),
            );
        }
    }

    // the charging and parking area: a marked floor (grey) round the lane at x = 212,
    // and a stall (a long station, along the lane) at the apex of each siding, one on
    // each side
    s.polygon(
        "parking".into(),
        GeometryKind::Other,
        (200.0, 88.0, 224.0, 122.0),
    );
    for (i, y) in STALLS_Y.iter().enumerate() {
        for (side, x) in [("w", 212.0 - 3.0), ("e", 212.0 + 3.0)] {
            s.polygon(
                format!("charge-{side}{i}"),
                GeometryKind::Station,
                (x - 1.2, y + 4.0 - 3.0, x + 1.2, y + 4.0 + 3.0),
            );
        }
    }

    // racking beside the labyrinth, and over it along the top ring road
    s.rack_block("rack-q0", (196.0, 90.0, 200.0, 124.0));
    s.rack_block("rack-q1", (150.0, 128.0 - 1.0, 196.0, 128.0));

    // shelving along the south and the north wall, outside the narrow lanes
    s.row(
        "shelf-s",
        GeometryKind::Shelf,
        true,
        (30.0, 210.0),
        (0.3, 1.8),
    );
    s.row(
        "shelf-n",
        GeometryKind::Shelf,
        true,
        (30.0, 210.0),
        (H - 1.8, H - 0.3),
    );

    // the corners of the ring road are arcs: nothing may stick out past the arc
    // (and a little before it, for the robots) into the corner
    let centres = [
        (RING + CORNER, RING + CORNER),
        (W - RING - CORNER, RING + CORNER),
        (W - RING - CORNER, H - RING - CORNER),
        (RING + CORNER, H - RING - CORNER),
    ];
    let clearance = 2.0;
    let in_corner = |v: &Vertex| {
        centres.iter().any(|&(cx, cy)| {
            let (dx, dy) = (v.x - cx, v.y - cy);
            let outwards_x = if cx < W / 2.0 { dx < 0.0 } else { dx > 0.0 };
            let outwards_y = if cy < H / 2.0 { dy < 0.0 } else { dy > 0.0 };
            outwards_x && outwards_y && dx.hypot(dy) > CORNER - clearance
        })
    };
    s.0.retain(|shape| {
        matches!(shape.kind, GeometryKind::Wall | GeometryKind::Station)
            || !shape.vertices.iter().any(in_corner)
    });

    GeometryFile {
        version: GEOMETRY_VERSION,
        shapes: s.0,
    }
}

// --------------------------------------------------------------- the output

fn write(path: &Path, value: &Value) {
    let mut text = serde_json::to_string_pretty(value).expect("serializes");
    text.push('\n');
    std::fs::write(path, text).unwrap_or_else(|e| panic!("cannot write {}: {e}", path.display()));
}

fn main() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let net = build_net();
    let lost = not_strongly_connected(&net);
    assert!(lost.is_empty(), "not strongly connected: {lost:?}");
    let lif_json = lif(&net);
    let geometry = build_geometry();
    geometry.validate().expect("the geometry is valid");

    write(&root.join("scenarios/warehouse2.lif.json"), &lif_json);
    let geometry_json = serde_json::to_value(&geometry).expect("serializes");
    write(
        &root.join("scenarios/warehouse2.geometry.json"),
        &geometry_json,
    );

    // the preview: the layout and the geometry, drawn the way the exporter does
    let lif: vdma_lif::Lif = serde_json::from_value(lif_json).expect("a valid LIF");
    let graph = Graph::from_lif(&lif).expect("a valid graph");
    let layout = LayoutLine::from_graph(&graph)
        .with_geometry(&geometry)
        .expect("the geometry fits the layout");
    let renderer = Renderer::new(&layout, 0, 1920, 1080)
        .expect("a view")
        .with_brand(true)
        .expect("a font");
    let empty = SnapshotLine {
        tick: 0,
        time: 0.0,
        robots: vec![],
        reservations: vec![],
        blocking: vec![],
        overlapping: vec![],
        deadlocks: vec![],
        lifts: vec![],
        doors: vec![],
    };
    let data = root.join("data");
    std::fs::create_dir_all(&data).expect("a data directory");
    let png = data.join("warehouse2_layout.png");
    renderer
        .frame(&empty)
        .save_png(&png)
        .unwrap_or_else(|e| panic!("cannot write {}: {e}", png.display()));

    println!(
        "{} nodes, {} edges ({} curved), {} stations, {} shapes; picture: {}",
        net.nodes.len(),
        net.edges.len(),
        net.edges.iter().filter(|e| e.curve.is_some()).count(),
        net.stations.len(),
        geometry.shapes.len(),
        png.display()
    );
}

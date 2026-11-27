//! Generates the layout of "warehouse 4": `scenarios/warehouse4.lif.json`,
//! `scenarios/warehouse4.geometry.json`, and a preview picture
//! `data/warehouse4_layout.png` (layout and geometry, no robots).
//!
//! Warehouse 4 is warehouse 2 with **one loop round every block of racks in the west**: a
//! rectangle close to the racks with an arc of 3 m at each corner, no straight line through
//! the streets. Neighbouring loops turn in opposite directions (like gears), so the two sides
//! that face each other in a street run the same way; a robot changes from one loop to the
//! next by a lane change (an S-curve) along the street, never at a corner.
//!
//! ```sh
//! cargo run --release -p botsim-export --example gen_warehouse4
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

use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;
use std::path::Path;

use botsim_export::Renderer;
use botsim_model::graph::Graph;
use botsim_model::spec::Kinematics;
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
/// (12 m, it was 10: the ring road at the bottom and the top, where the queues form, was too
/// close to the road along the wall; the bridges between them are now 8 m wide, user 2026-11-24.)
const RING: f64 = 12.0;
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

/// Where the sidings of the goods-in bays on the west side begin (y); each is 8 m
/// long and runs south (the left side of the ring road runs south).
/// (With a street across the west part the fourth was at 40, not 66.)
const WEST_BAYS: [f64; 5] = [120.0, 100.0, 87.0, 66.0, 52.0];

/// The row of cells of the west part (an index into `NET_Y`, 3: y = 62 to 80) that is a street
/// across the hall instead of loops and racks; none (the user did not want it).
const STREET_ROW: usize = 99;
/// The goods-in bays on the left side of the ring (outside it), each `(from, apex, to)`: not
/// where a lane change joins the ring (the lane changes start at y = 41.2, 75.2 and 109.2).
const GOODS_IN_BAYS: [(P, P, P); 5] = [
    (
        (RING, WEST_BAYS[0]),
        (RING - 3.0, WEST_BAYS[0] - 4.0),
        (RING, WEST_BAYS[0] - 8.0),
    ),
    (
        (RING, WEST_BAYS[1]),
        (RING - 3.0, WEST_BAYS[1] - 4.0),
        (RING, WEST_BAYS[1] - 8.0),
    ),
    (
        (RING, WEST_BAYS[2]),
        (RING - 3.0, WEST_BAYS[2] - 4.0),
        (RING, WEST_BAYS[2] - 8.0),
    ),
    (
        (RING, WEST_BAYS[3]),
        (RING - 3.0, WEST_BAYS[3] - 4.0),
        (RING, WEST_BAYS[3] - 8.0),
    ),
    (
        (RING, WEST_BAYS[4]),
        (RING - 3.0, WEST_BAYS[4] - 4.0),
        (RING, WEST_BAYS[4] - 8.0),
    ),
];
/// The sidings of the goods-out bays on the east side: where they begin (y) and how
/// long they are (they run north).
/// (the bay at y = 33 is gone: the second east road is where it was)
const EAST_BAYS: [(f64, f64); 2] = [(80.0, 8.0), (92.0, 8.0)];

/// The second road beside the east ring road: its distance from the ring (m, outside) and its
/// first and last y. Both run north.
const EAST_ROAD_DX: f64 = 3.6;
/// (from the south wall road to the north wall road: with them it is the outer loop round the
/// hall, the ring road is the inner one)
const EAST_ROAD_Y: (f64, f64) = (4.0, H - 4.0);
/// Its bridges (10 m S-curves, driving north): `(y on the start side, true: from the ring to
/// the road)`. The ends on the ring are on a straight piece of it, not on the arcs of the rows
/// of the east grid (rows at y = 28, 46, 58, 66, 74, 84; a row that runs east turns north on the
/// ring, with an arc of 3.5 m after the junction, one that runs west before it).
const EAST_BRIDGES: [(f64, bool); 4] = [(26.0, true), (40.0, false), (62.5, true), (104.0, true)];
// (the last one leaves the ring for the road, so that nobody merges from the road into the top
// ring, which is where the jams were: the road goes on round the north wall to x = 215)

/// Where the narrow lanes along the south and north wall branch off the ring road (x).
/// (the east one is next to the big corner arc of the ring road: 3 m before it starts)
const LANES_X: [f64; 2] = [40.0, 215.0];

/// Are there turn arcs (for the three-wheel robots) at the crossings of two roads? Without
/// them nobody drives past the corner node of a crossing at the same time as a robot on
/// an arc, but a three-wheel robot cannot turn at a crossing.
const CROSSING_ARCS: bool = true;

/// The longest straight edge (m): longer roads get nodes in between, so that more
/// than one robot can be on a road (the baseline gives an edge to one robot).
const MAX_EDGE: f64 = 6.0;

/// How far the stop of a pocket (the loading lane beside a road) is from the road (m). The racks
/// are `POCKET_OFFSET + 1.0` from the road: a forklift's side (0.7 m) and some room.
const POCKET_OFFSET: f64 = 1.8;

/// The same on the narrow roads (small robots only: the biggest, X1, is 1.2 m long, so 2.5
/// lengths is 3 m).
const NARROW_MAX_EDGE: f64 = 2.95;

/// The radii (m) a turn arc may have, biggest first; an arc gets the biggest one that the
/// roads have room for, and `TURN_RADIUS` if none. A 90 degree arc passes r (sqrt(2) - 1) from
/// the corner node: 2.5 m for 6 m (1.45 m for `TURN_RADIUS`).
const ARC_RADII: [f64; 3] = [3.0, 3.0, 3.0];

/// The shortest tangent (m) of a turn arc: the arc starts and ends at least this far
/// from the corner node (a forklift is 2.4 m long).
const MIN_TANGENT: f64 = 3.0;

/// A node put in a long straight edge (not a narrow one) is at least this far (m) from the ends
/// and the middle of any curve that is not its own: a robot that waits at the node would be
/// in the way of a robot on the curve (the baseline found three such places at 2 to 3 m).
const CURVE_CLEAR: f64 = 3.5;

/// A node put in a long straight edge is at least this far (m) from the corner of any arc.
const SPLIT_CLEAR: f64 = MIN_TANGENT;

/// The loops' corners are 1.8 m from the ring and from the column copy, and two of them (the
/// two loops above and below a street) are 3.6 m apart: no node fits in the 8.4 m round them,
/// so a piece may be this much (m) longer than the limit to jump over them (before, the edges
/// beside the loops stayed 10 to 34 m long: one robot at a time on each).
const SPLIT_EXTRA: f64 = 5.0;

/// The smallest arc (m) at a corner of a narrow road, where the roads are too short for
/// the arcs of `TURN_RADIUS`; only the small robots drive there.
const NARROW_RADIUS: f64 = 1.8;

/// The radius of the arcs at the junctions (m): about what a forklift needs.
const TURN_RADIUS: f64 = 3.0;

/// The charging stalls: sidings (a bow 8 m long, 3 m inwards, the station at its apex) off the
/// wall roads, which carry little traffic: `(x where it begins, true: on the south road, which
/// runs east, false: on the north road, which runs west)`. Not on a road that others use to get
/// somewhere (the lane at x = 212 was one, and the robots that charged there stood in the queue
/// of the jobs), and clear of the bridges and the links at x = 215 and of the shelving stops.
const CHARGER_SIDINGS: [(f64, bool); 6] = [
    (94.0, true),
    (137.0, true),
    (178.0, true),
    (190.0, true),
    (194.0, false),
    (206.0, false),
];
/// How far a siding of `CHARGER_SIDINGS` runs along the road, and how far it bulges (m).
const SIDING_LENGTH: f64 = 8.0;
const SIDING_BULGE: f64 = 3.0;

/// The apex of a siding of `CHARGER_SIDINGS` (the stall).
fn siding_apex((x, south): (f64, bool)) -> P {
    if south {
        (x + SIDING_LENGTH / 2.0, 4.0 + SIDING_BULGE)
    } else {
        (x - SIDING_LENGTH / 2.0, H - 4.0 - SIDING_BULGE)
    }
}

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
const V_BOTTOM: f64 = 17.0 + V_STEP / 2.0;
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

/// The parts of the shelving row north of road `k` of the labyrinth in the north
/// (x ranges): the row without the gap where the shortcut goes through.
fn h_row_parts(k: usize) -> Vec<(f64, f64)> {
    let r = H_STEP / 2.0;
    let xs = shortcut_x(k);
    let shortcut = if k.is_multiple_of(2) {
        (xs - 1.0, xs + r + 1.5)
    } else {
        (xs - r - 1.5, xs + 1.0)
    };
    segments((122.0, 186.0), vec![shortcut])
}

/// The parts of the shelving row east of aisle `k` of the vertical labyrinth (y ranges).
fn v_row_parts(k: usize) -> Vec<(f64, f64)> {
    let r = V_STEP / 2.0;
    let y = v_shortcut_y(k);
    // the half circle of the shortcut bulges north from an even aisle, south from an
    // odd one
    let shortcut = if k.is_multiple_of(2) {
        (y - 1.0, y + r + 1.5)
    } else {
        (y - r - 1.5, y + 1.0)
    };
    segments((V_BOTTOM + 3.0, V_TOP - 3.0), vec![shortcut])
}

/// About this far apart (m) the stops of the aisles of the north labyrinth are.
const NORTH_STOP_SPACING: f64 = 40.0;

/// The stops in the aisles of both labyrinths (station, point): in the middle of every
/// part of the shelving, on the aisle.
fn aisle_stops() -> Vec<(String, P)> {
    let mut out = Vec::new();
    for k in 0..H_AISLES {
        // the shortcuts that end on this aisle: its own (to the next aisle) and the one from the
        // aisle before; the stop is not within 3.5 m of either (as for the vertical aisles)
        let mut avoid = vec![shortcut_x(k)];
        if k >= 1 {
            avoid.push(shortcut_x(k - 1));
        }
        // a stop for about every `NORTH_STOP_SPACING` m of a part (each one is a station of its
        // own, so that the jobs draw them as often as the stations elsewhere), in the middle of
        // its piece, moved off the shortcuts
        let mut j = 0;
        for (pa, pb) in h_row_parts(k) {
            let n = (((pb - pa) / NORTH_STOP_SPACING).ceil() as usize).max(1);
            for i in 0..n {
                let a = pa + (pb - pa) * i as f64 / n as f64;
                let b = pa + (pb - pa) * (i + 1) as f64 / n as f64;
                let mid = (a + b) / 2.0;
                let clear = |x: f64| avoid.iter().all(|s| (x - s).abs() >= 3.5);
                let x = (0..=((b - a) * 5.0) as usize)
                    .flat_map(|i| [mid - i as f64 * 0.1, mid + i as f64 * 0.1])
                    .find(|&x| x >= a + 1.0 && x <= b - 1.0 && clear(x))
                    .unwrap_or(mid);
                out.push((format!("LN{k}-{j}"), (x, h_y(k))));
                j += 1;
            }
        }
    }
    for k in 0..V_AISLES {
        // the shortcuts that end on this aisle: its own (to the next aisle) and the one from
        // the aisle before; the stop is not within 3.5 m of either (a robot waiting there
        // would stand beside the half circle)
        let mut avoid = vec![v_shortcut_y(k)];
        if k >= 1 {
            avoid.push(v_shortcut_y(k - 1));
        }
        for (a, b) in v_row_parts(k) {
            let mid = (a + b) / 2.0;
            let clear = |y: f64| avoid.iter().all(|s| (y - s).abs() >= 3.5);
            // the point nearest the middle that is clear, 0.1 m steps, inside the part
            let y = (0..=((b - a) * 5.0) as usize)
                .flat_map(|i| [mid - i as f64 * 0.1, mid + i as f64 * 0.1])
                .find(|&y| y >= a + 1.0 && y <= b - 1.0 && clear(y))
                .unwrap_or(mid);
            out.push((format!("LV{k}"), (v_x(k), y)));
        }
    }
    out
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
    curve: Option<Curve>,
}

/// The shape of a curved edge.
#[derive(Clone, Copy)]
enum Curve {
    /// A circular arc: the point where the tangents at the two ends meet, and the
    /// weight of that control point.
    Arc(P, f64),
    /// An S-shaped cubic from one end to the other (both ends in the same direction):
    /// its two inner control points.
    Cubic(P, P),
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
    /// Corner nodes where every turn got an arc and at most one road goes straight on:
    /// the corner node is not needed (see `remove_corner_nodes`).
    corners: Vec<usize>,
    /// Pieces of road (from, to) between two loading lanes that replace the road: the
    /// road piece itself is taken out (`remove_shared_roads`).
    shared_roads: Vec<(P, P)>,
    /// The corner nodes of the crossings of the double net, in cm: nobody turns there (the
    /// layout forbids turning on the spot, no turn arc is made); the turns are the arcs made
    /// by `double_net`.
    no_rotation: HashSet<(i64, i64)>,
    /// Every turn arc made: where it starts, its corner, where it ends.
    arc_log: Vec<(P, P, P)>,
    /// The arcs (start and end, in cm) at a junction that keeps its corner node: only
    /// the robots that cannot turn on the spot use them; the others turn at the corner,
    /// so that they hold the node, and the arc is not next to a robot that waits on it.
    crossing_arcs: HashSet<(i64, i64, i64, i64)>,
}

/// A point in whole centimetres.
fn cm(p: P) -> (i64, i64) {
    ((p.0 * 100.0).round() as i64, (p.1 * 100.0).round() as i64)
}

/// The key of an arc edge in `Net::crossing_arcs`.
fn arc_key(start: P, end: P) -> (i64, i64, i64, i64) {
    let (a, b) = (cm(start), cm(end));
    (a.0, a.1, b.0, b.1)
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
            curve: Some(Curve::Arc(corner, (turn / 2.0).cos())),
        });
    }

    /// An S-shaped link from `a` to `b`, driving in the direction `d` (a unit vector)
    /// at `a` and at `b`; `b` is ahead of `a` and to one side. One edge (a cubic
    /// curve), so that there is no node in the middle: the nodes of a siding are far
    /// enough apart (a forklift is 2.4 m long) for the robots not to overlap.
    fn s_link(&mut self, a: P, b: P, d: P) {
        let along = (b.0 - a.0) * d.0 + (b.1 - a.1) * d.1;
        let s = 0.5 * along;
        let c1 = (a.0 + d.0 * s, a.1 + d.1 * s);
        let c2 = (b.0 - d.0 * s, b.1 - d.1 * s);
        let (a, b) = (self.node(a), self.node(b));
        self.edges.push(Edge {
            a,
            b,
            curve: Some(Curve::Cubic(c1, c2)),
        });
    }

    /// Take out what leads nowhere: nodes where nothing comes in or nothing goes out, with
    /// their edges, again and again; and the stations that have no node left.
    fn prune_dead_ends(&mut self) {
        loop {
            let mut ins = vec![0usize; self.nodes.len()];
            let mut outs = vec![0usize; self.nodes.len()];
            for e in &self.edges {
                outs[e.a] += 1;
                ins[e.b] += 1;
            }
            let before = self.edges.len();
            self.edges
                .retain(|e| ins[e.a] > 0 && outs[e.b] > 0 && ins[e.b] > 0 && outs[e.a] > 0);
            if self.edges.len() == before {
                break;
            }
        }
        self.compact();
        let kept: Vec<Station> = std::mem::take(&mut self.stations)
            .into_iter()
            .filter(|s| s.points.iter().any(|&p| self.find(p).is_some()))
            .collect();
        self.stations = kept;
    }

    /// Take out the pieces of road that two loading lanes replace (one on each side of
    /// the road, see `shared_roads`): between two rows of racks there are then two lanes
    /// that run the same way, not the road and a lane on each side.
    fn remove_shared_roads(&mut self) {
        for (a, b) in std::mem::take(&mut self.shared_roads) {
            if let (Some(ia), Some(ib)) = (self.find(a), self.find(b)) {
                self.edges
                    .retain(|e| !(e.curve.is_none() && e.a == ia && e.b == ib));
            }
        }
    }

    /// A loading lane beside a straight road: from `a` it leaves the road (an S-curve),
    /// runs through the `stops` (straight, parallel to the road) and rejoins the road at
    /// `b` (an S-curve). With one stop it is a `bow`.
    fn loading_lane(&mut self, a: P, stops: &[P], b: P) {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let length = dx.hypot(dy);
        let u = (dx / length, dy / length);
        self.s_link(a, stops[0], u);
        for pair in stops.windows(2) {
            self.line(pair[0], pair[1]);
        }
        self.s_link(stops[stops.len() - 1], b, u);
    }

    /// A siding that bows out from a straight road: from `a` to `b` through the
    /// apex `m`, where the robot drives parallel to the road. Two S-shaped links, so
    /// that the siding leaves the road and rejoins it in the direction of the road:
    /// there is no corner, and a robot that cannot turn on the spot can drive it.
    fn bow(&mut self, a: P, m: P, b: P) {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let length = dx.hypot(dy);
        let u = (dx / length, dy / length);
        self.s_link(a, m, u);
        self.s_link(m, b, u);
    }

    /// Cut every straight edge longer than `max` into equal pieces with a node
    /// between them. The baseline gives an edge to one robot at a time, so a long edge
    /// is one robot long: with shorter edges the robots can follow each other.
    ///
    /// No new node is put within `MIN_TANGENT` of the corner of an arc (also one
    /// whose corner node was taken out): a robot that waits on it would be next to the
    /// arc.
    fn split_long_edges(&mut self, max: f64) {
        // the corner of every arc there is (the arcs of the layout, the turn arcs, also
        // those whose corner node was taken out)
        let corners: Vec<P> = self
            .edges
            .iter()
            .filter_map(|e| match e.curve {
                Some(Curve::Arc(corner, _)) => Some(corner),
                _ => None,
            })
            .collect();
        // the curves of the layout: their two ends and the middle (the nodes they join, and
        // three points), so that a new node is not put 2 to 3 m from one that is not its own
        // (a robot that waits there is in the way of the one on the curve)
        let curves: Vec<(usize, usize, [P; 3])> = self
            .edges
            .iter()
            .filter_map(|e| {
                let (a, b) = (self.nodes[e.a], self.nodes[e.b]);
                let mid = match e.curve? {
                    Curve::Arc(c, _) => {
                        ((a.0 + 2.0 * c.0 + b.0) / 4.0, (a.1 + 2.0 * c.1 + b.1) / 4.0)
                    }
                    Curve::Cubic(c1, c2) => (
                        (a.0 + 3.0 * c1.0 + 3.0 * c2.0 + b.0) / 8.0,
                        (a.1 + 3.0 * c1.1 + 3.0 * c2.1 + b.1) / 8.0,
                    ),
                };
                Some((e.a, e.b, [a, mid, b]))
            })
            .collect();
        let edges = std::mem::take(&mut self.edges);
        for e in edges {
            let (a, b) = (self.nodes[e.a], self.nodes[e.b]);
            let length = (b.0 - a.0).hypot(b.1 - a.1);
            // (on a narrow road the robots are small: 2.5 lengths of the biggest of them, 3 m)
            let is_narrow = narrow(((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0));
            let max = if is_narrow { NARROW_MAX_EDGE } else { max };
            // the shortest piece: the longest robot there plus its protective field
            let shortest = if is_narrow { 1.6 } else { 2.8 };
            if e.curve.is_some() || length <= max {
                self.edges.push(e);
                continue;
            }
            // from the start: the next node as far along as `max` allows, where no arc has its
            // corner within `MIN_TANGENT` (0.25 m steps); no piece is shorter than `shortest`
            let point = |s: f64| {
                (
                    a.0 + (b.0 - a.0) * s / length,
                    a.1 + (b.1 - a.1) * s / length,
                )
            };
            // (on a narrow road the robots are small, 1.2 m at most: 1.5 m from a corner is enough)
            let clear = if is_narrow { 1.5 } else { SPLIT_CLEAR };
            let allowed = |s: f64| {
                let at = point(s);
                !corners
                    .iter()
                    .any(|c| (c.0 - at.0).hypot(c.1 - at.1) < clear)
                    && (is_narrow
                        || !curves.iter().any(|(ca, cb, pts)| {
                            ![ca, cb].contains(&&e.a)
                                && ![ca, cb].contains(&&e.b)
                                && pts
                                    .iter()
                                    .any(|c| (c.0 - at.0).hypot(c.1 - at.1) < CURVE_CLEAR)
                        }))
            };
            let mut from = e.a;
            let mut at = 0.0;
            while length - at > max {
                let far = (at + max).min(length - shortest);
                let mut s = (far * 4.0).floor() / 4.0;
                while s >= at + shortest && !allowed(s) {
                    s -= 0.25;
                }
                if s < at + shortest {
                    // no node fits within `max` (the corners of the loops beside the road): the
                    // next allowed place up to `SPLIT_EXTRA` further
                    let far = (at + max + SPLIT_EXTRA).min(length - shortest);
                    s = (far * 4.0).floor() / 4.0;
                    while s > at + max && !allowed(s) {
                        s -= 0.25;
                    }
                    if s <= at + max {
                        break; // no room for a node: the edge stays long here
                    }
                }
                let to = self.node(point(s));
                self.edges.push(Edge {
                    a: from,
                    b: to,
                    curve: None,
                });
                from = to;
                at = s;
            }
            self.edges.push(Edge {
                a: from,
                b: e.b,
                curve: None,
            });
        }
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
        // does a turn (that gets an arc) happen at this node? A road piece needs room
        // for the arc at its far end only if it does
        let turns_at = |n: usize| {
            let p = self.nodes[n];
            if self.no_rotation.contains(&cm(p)) {
                return false;
            }
            ins[n].iter().any(|&a| {
                outs[n].iter().any(|&b| {
                    let (pa, pb) = (self.nodes[a], self.nodes[b]);
                    let (li, lo) = (
                        (p.0 - pa.0).hypot(p.1 - pa.1),
                        (pb.0 - p.0).hypot(pb.1 - p.1),
                    );
                    if li < 1e-9 || lo < 1e-9 {
                        return false;
                    }
                    let c = ((p.0 - pa.0) * (pb.0 - p.0) + (p.1 - pa.1) * (pb.1 - p.1)) / (li * lo);
                    (25f64.to_radians()..=100f64.to_radians()).contains(&c.clamp(-1.0, 1.0).acos())
                })
            })
        };
        let turning: Vec<bool> = (0..self.nodes.len()).map(turns_at).collect();
        let mut arcs: Vec<(P, P, P)> = Vec::new();
        // (corner node, the node a road comes from, the node it goes on to) for every arc
        let mut arced: HashSet<(usize, usize, usize)> = HashSet::new();
        for n in 0..self.nodes.len() {
            let p = self.nodes[n];
            // (no turn arc at a crossing of the double net: see `no_rotation`)
            if self.no_rotation.contains(&cm(p)) {
                continue;
            }
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
                    // (a shallow turn gets a bigger radius: the arc starts at least
                    // `MIN_TANGENT` from the corner, so that a robot that waits at the
                    // node where the arc starts is not in the junction)
                    let mut t = (radius * (turn / 2.0).tan()).max(MIN_TANGENT);
                    // the biggest arc the roads have room for (`ARC_RADII`, biggest first):
                    // it passes further from the corner node, where robots wait
                    for r in ARC_RADII {
                        let wide = (r * (turn / 2.0).tan()).max(MIN_TANGENT);
                        let far = |on: bool| if on { wide } else { 0.0 };
                        if len_in >= wide + far(turning[a]) + 0.5
                            && len_out >= wide + far(turning[b]) + 0.5
                        {
                            t = wide;
                            break;
                        }
                    }
                    // (room for this arc, and for one of about the same size at the
                    // far end if there is a turn there)
                    let far = |on: bool| if on { t } else { 0.0 };
                    if len_in < t + far(turning[a]) + 0.5 || len_out < t + far(turning[b]) + 0.5 {
                        // on a narrow road (used by the small robots only) the arc may
                        // be smaller, down to `NARROW_RADIUS`, to fit between two turns
                        let room = |len: f64, turns: bool| {
                            // (no 0.5 m piece left between two arcs: the arcs meet, or the
                            // arc starts at the node: pieces of 0.2-0.6 m put nodes next
                            // to the corners of the arcs)
                            if turns { len / 2.0 } else { len }
                        };
                        let fit = room(len_in, turning[a]).min(room(len_out, turning[b]));
                        // (where a wall lane joins the ring road the arc may be smaller too:
                        // the lane is only 6 m long and has an arc at its other end)
                        // (and at the corners of the road: the road is 6 m from the ring)
                        let lane_joint = LANES_X.iter().any(|&x| (p.0 - x).abs() < 0.01)
                            && [RING, 4.0, H - 4.0, H - RING]
                                .iter()
                                .any(|&y| (p.1 - y).abs() < 0.01);
                        // (and where a row leaves the column at x = 100, which has a lane
                        // change a few metres away: not less than 2 m, a forklift needs 1.7 m)
                        let min_radius = if narrow(p) || lane_joint {
                            NARROW_RADIUS
                        } else if (p.0 - WEST_X[3]).abs() < 0.01 {
                            2.0
                        } else {
                            continue;
                        };
                        if fit / (turn / 2.0).tan() < min_radius {
                            continue;
                        }
                        t = fit;
                    }
                    arced.insert((n, a, b));
                    arcs.push((
                        (p.0 - d_in.0 * t, p.1 - d_in.1 * t),
                        p,
                        (p.0 + d_out.0 * t, p.1 + d_out.1 * t),
                    ));
                }
            }
        }
        // the nodes whose corner is not needed: every pair of a road in and a road
        // out is either straight on (at most one) or has an arc
        self.corners.clear();
        for n in 0..self.nodes.len() {
            let p = self.nodes[n];
            let (mut straight, mut missing) = (0, 0);
            for &a in &ins[n] {
                for &b in &outs[n] {
                    let (pa, pb) = (self.nodes[a], self.nodes[b]);
                    let (li, lo) = (
                        (p.0 - pa.0).hypot(p.1 - pa.1),
                        (pb.0 - p.0).hypot(pb.1 - p.1),
                    );
                    let cos =
                        ((p.0 - pa.0) * (pb.0 - p.0) + (p.1 - pa.1) * (pb.1 - p.1)) / (li * lo);
                    if cos > 0.9999 {
                        straight += 1;
                    } else if !arced.contains(&(n, a, b)) {
                        missing += 1;
                    }
                }
            }
            if straight <= 1 && missing == 0 && arced.iter().any(|&(m, _, _)| m == n) {
                self.corners.push(n);
            }
        }
        self.arc_log = arcs.clone();
        for (a, corner, b) in arcs {
            self.arc(a, corner, b);
        }
        self.split_at_nodes();
    }

    /// Take out the corner node of every junction in `corners`: the road that goes
    /// straight on becomes one edge past the corner and the corner paths go; only the
    /// arcs are left for the turns. A robot that waits for the next node then waits
    /// before the junction, not on the corner next to the arcs. (A crossing of two
    /// roads keeps its node: it is where they exclude each other.)
    fn remove_corner_nodes(&mut self) {
        let mut gone = vec![false; self.edges.len()];
        for &j in &self.corners {
            let touching: Vec<usize> = (0..self.edges.len())
                .filter(|&i| !gone[i] && (self.edges[i].a == j || self.edges[i].b == j))
                .collect();
            // only straight roads meet at the corner
            if touching.iter().any(|&i| self.edges[i].curve.is_some()) {
                continue;
            }
            let dir = |i: usize| {
                let e = &self.edges[i];
                let (a, b) = (self.nodes[e.a], self.nodes[e.b]);
                let l = (b.0 - a.0).hypot(b.1 - a.1);
                ((b.0 - a.0) / l, (b.1 - a.1) / l)
            };
            let ins: Vec<usize> = touching
                .iter()
                .copied()
                .filter(|&i| self.edges[i].b == j)
                .collect();
            let outs: Vec<usize> = touching
                .iter()
                .copied()
                .filter(|&i| self.edges[i].a == j)
                .collect();
            for &i in &touching {
                gone[i] = true;
            }
            // (the new edge goes into the list at once: the corner at its far end, if it
            // is one too, then finds it there, and its corner is taken out as well)
            let mut new_edges: Vec<Edge> = Vec::new();
            for &i in &ins {
                for &o in &outs {
                    let (di, d_o) = (dir(i), dir(o));
                    if di.0 * d_o.0 + di.1 * d_o.1 > 0.9999 {
                        new_edges.push(Edge {
                            a: self.edges[i].a,
                            b: self.edges[o].b,
                            curve: None,
                        });
                    }
                }
            }
            for e in new_edges {
                self.edges.push(e);
                gone.push(false);
            }
        }
        let old = std::mem::take(&mut self.edges);
        self.edges = old
            .into_iter()
            .enumerate()
            .filter(|(i, _)| !gone[*i])
            .map(|(_, e)| e)
            .collect();
        self.compact();
        // the arcs at the junctions that kept their corner node
        let kept: Vec<(P, P)> = self
            .arc_log
            .iter()
            .filter(|&&(_, corner, _)| self.find(corner).is_some())
            .map(|&(start, _, end)| (start, end))
            .collect();
        self.crossing_arcs = kept.into_iter().map(|(s, e)| arc_key(s, e)).collect();
    }

    /// Drop the nodes that no edge uses, and renumber the rest.
    fn compact(&mut self) {
        let mut used = vec![false; self.nodes.len()];
        for e in &self.edges {
            used[e.a] = true;
            used[e.b] = true;
        }
        let mut new_index = vec![usize::MAX; self.nodes.len()];
        let mut nodes = Vec::new();
        for (i, p) in self.nodes.iter().enumerate() {
            if used[i] {
                new_index[i] = nodes.len();
                nodes.push(*p);
            }
        }
        for e in &mut self.edges {
            e.a = new_index[e.a];
            e.b = new_index[e.b];
        }
        self.index = nodes
            .iter()
            .enumerate()
            .map(|(i, p)| {
                (
                    ((p.0 * 100.0).round() as i64, (p.1 * 100.0).round() as i64),
                    i,
                )
            })
            .collect();
        self.nodes = nodes;
        self.corners.clear();
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

/// The robot types that cannot use the narrow roads: the forklift `Y2` (too big for
/// the aisles) and the three-wheel robot `T3` (the half circles of the labyrinths
/// are tighter than it can turn).
const WIDE: [&str; 2] = ["Y2", "T3"];

/// The middle of an edge (for a curve the point of the arc half way).
fn edge_middle(net: &Net, e: &Edge) -> P {
    let (a, b) = (net.nodes[e.a], net.nodes[e.b]);
    match e.curve {
        // a rational quadratic Bezier at t = 1/2
        Some(Curve::Arc(c, w)) => (
            (a.0 + 2.0 * w * c.0 + b.0) / (2.0 + 2.0 * w),
            (a.1 + 2.0 * w * c.1 + b.1) / (2.0 + 2.0 * w),
        ),
        // a cubic Bezier at t = 1/2
        Some(Curve::Cubic(c1, c2)) => (
            (a.0 + 3.0 * c1.0 + 3.0 * c2.0 + b.0) / 8.0,
            (a.1 + 3.0 * c1.1 + 3.0 * c2.1 + b.1) / 8.0,
        ),
        None => ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0),
    }
}

/// Is this point on a narrow road: in one of the two labyrinths or in a lane along
/// the south or north wall?
fn narrow(p: P) -> bool {
    // (the top ring road at y = H - RING is the way out of the north labyrinth: wide)
    // (warehouse 3: the serpentines start further east, the outer lines of the double net at
    // y = 10 and y = 130 are streets, not wall lanes; the wall lanes are at y = 4 and 136)
    // (from x = 101: the way in from the column at x = 100 along y = 90 is the labyrinth's, for
    // the small robots only; the wide robots would drive in and not come out)
    // (the arc from the top ring road down to the column at x = 100, at y = 127..130, is wide)
    let north = ((100.5..117.0).contains(&p.0) && (88.0..127.0).contains(&p.1))
        || ((117.0..196.0).contains(&p.0) && (88.0..H - RING - 0.3).contains(&p.1));
    let middle = (100.5..139.5).contains(&p.0) && (14.0..66.0).contains(&p.1);
    // (only the shelving stops beside the roads along the walls, at y = 2.5 and H - 2.5, and
    // their links: the roads themselves, at y = 4, are for all robots)
    let lane = p.1 < 3.7 || p.1 > H - 3.7;
    north || middle || lane
}

/// The robot types that may drive an edge.
fn edge_types(net: &Net, e: &Edge) -> Vec<Value> {
    // an arc at a crossing is for the three-wheel robots only
    let crossing = net
        .crossing_arcs
        .contains(&arc_key(net.nodes[e.a], net.nodes[e.b]));
    // (an arc that starts or ends on a narrow road is narrow: nobody wide gets to its end)
    let narrow = narrow(edge_middle(net, e))
        || (crossing && (narrow(net.nodes[e.a]) || narrow(net.nodes[e.b])));
    TYPES
        .iter()
        .filter(|t| !(narrow && WIDE.contains(t)))
        .filter(|t| !crossing || **t == "T3")
        .map(|s| json!({"manufacturer": "ACME", "seriesName": s}))
        .collect()
}

/// The fleet: serial prefix, factsheet, seconds to pick or drop, number of robots,
/// and whether it may use the narrow roads (the order is the order in the scenario).
const FLEET: [(&str, &str, f64, usize); 5] = [
    ("Y", "forklift.factsheet.json", 8.0, 24),
    ("X", "diff_robot.factsheet.json", 5.0, 24),
    ("T", "three_wheel.factsheet.json", 6.0, 11),
    ("O", "omni_robot.factsheet.json", 4.0, 19),
    ("C", "small_robot.factsheet.json", 3.0, 12),
];

/// Write a battery for every robot type and chargers into the scenario (see `battery_wh` in
/// the scenario crate and `briefs/charging_ready.md`)? Off: a run without batteries, as before.
const WITH_BATTERY: bool = true;
/// The batteries of the types of `FLEET` (same order): capacity (Wh), energy per metre driven
/// (Wh), power standing (W), power working (W). Placeholders, not measured values: chosen so that
/// a robot that drives about half of the time lasts about three hours (the first values, 1.0 to
/// 0.25 Wh/m, emptied a battery in 15 minutes: 1.0 Wh/m at 1.2 m/s is 4.3 kW).
const BATTERY: [(f64, f64, f64, f64); 5] = [
    (2000.0, 0.25, 50.0, 150.0),
    (800.0, 0.1, 30.0, 80.0),
    (1500.0, 0.2, 40.0, 100.0),
    (600.0, 0.08, 30.0, 60.0),
    (300.0, 0.04, 20.0, 40.0),
];
/// The power of every charger (W) and the charge (percent) below which a free robot goes to charge.
const CHARGER_POWER_W: f64 = 1500.0;
const CHARGE_LOW: f64 = 25.0;

/// The robots start at least this far apart (m).
const START_GAP: f64 = 6.0;

/// Where each group of the fleet starts (node numbers): at least `START_GAP` apart,
/// not at a station, spread over the hall. A robot starts heading east; the
/// three-wheel robots cannot turn on the spot, so they start only where every road
/// leaving the node runs east. The forklifts and the three-wheel robots do not
/// start on the narrow roads.
fn start_nodes(net: &Net) -> Vec<Vec<usize>> {
    // the stops in pockets, bays and stalls (off the road) and the stops on the
    // aisles of the labyrinths (on the road)
    let nodes_of = |on_aisle: bool| -> HashSet<usize> {
        net.stations
            .iter()
            .filter(|s| (s.id.starts_with("LN") || s.id.starts_with("LV")) == on_aisle)
            .flat_map(|s| s.points.iter().filter_map(|&p| net.find(p)))
            .collect()
    };
    let (pockets, aisle_stops) = (nodes_of(false), nodes_of(true));
    // the direction in which each road leaves each node
    let mut leaving: Vec<Vec<f64>> = vec![Vec::new(); net.nodes.len()];
    for e in &net.edges {
        let a = net.nodes[e.a];
        let towards = match e.curve {
            Some(Curve::Arc(c, _)) | Some(Curve::Cubic(c, _)) => c,
            None => net.nodes[e.b],
        };
        leaving[e.a].push((towards.1 - a.1).atan2(towards.0 - a.0));
    }
    // a fixed shuffle, so that the robots are spread and the result does not change
    let mut order: Vec<usize> = (0..net.nodes.len()).collect();
    order.sort_by_key(|&n| (n as u64).wrapping_mul(2_654_435_761) % 4_294_967_296);

    let mut taken: Vec<P> = Vec::new();
    let mut result = vec![Vec::new(); FLEET.len()];
    // the three-wheel robots first: they have the fewest places
    for k in [2, 0, 1, 3, 4] {
        let (prefix, _, _, count) = FLEET[k];
        let wide = prefix == "Y" || prefix == "T";
        // first the places off the road (so that a robot that waits for its first job
        // does not block a road), then the roads
        for (pass, &n) in [true, false]
            .iter()
            .flat_map(|&pass| order.iter().map(move |n| (pass, n)))
        {
            if result[k].len() == count {
                break;
            }
            let p = net.nodes[n];
            let east = leaving[n].iter().all(|a| a.abs() < 0.44);
            // (not on a corner of a crossing of the double net: a robot starts heading east
            // and may not turn there, so it could not leave if the line runs another way)
            let fits = !leaving[n].is_empty()
                && !net.no_rotation.contains(&cm(p))
                && pockets.contains(&n) == pass
                && !aisle_stops.contains(&n)
                && !(wide && narrow(p))
                && (prefix != "T" || east)
                && taken
                    .iter()
                    .all(|q| (q.0 - p.0).hypot(q.1 - p.1) >= START_GAP);
            if fits {
                taken.push(p);
                result[k].push(n);
            }
        }
        assert_eq!(
            result[k].len(),
            count,
            "not enough start nodes for {prefix}"
        );
    }
    result
}

/// The scenario file `warehouse3.toml`: the fleet, jobs between all the stations but
/// the parking, and the commands that make the video.
fn scenario(net: &Net) -> String {
    let starts = start_nodes(net);
    let robots: usize = FLEET.iter().map(|f| f.3).sum();
    let mut s = String::new();
    let _ = write!(
        s,
        "# The big warehouse (240 m x 140 m): {robots} robots of five types, jobs between\n\
         # shelf, rack and bay stations. THIS FILE IS GENERATED (with the layout and the\n\
         # geometry) by\n\
         #   cargo run --release -p botsim-export --example gen_warehouse4\n\
         # so change the fleet there (the node numbers change with the layout).\n\
         #\n\
         # Robot types (see the factsheets): X1 carrier, Y2 forklift, T3 three-wheel,\n\
         # O4 omnidirectional, C5 small. The forklift and the three-wheel robot may not use\n\
         # the narrow roads (the labyrinths and the lanes along the walls); the others may.\n\
         # The three-wheel robots cannot turn on the spot: they start where the road runs east.\n\
         #\n\
         # A 10 minute video (1920x1080, 30 fps, 18000 frames) from the repository root; the\n\
         # run is 6000 simulated seconds, shown at speed 10\n\
         name = \"warehouse4\"\n\
         seed = 29\n\
         duration = 6000.0\n\
         time_step = 0.05\n\
         start_time = 2026-11-22T10:00:00Z\n\
         lif = \"warehouse4.lif.json\"\n\
         deadlock_timeout = 20.0\n\
         # route with load: each other robot on an edge (or at the node it ends at) makes the\n\
         # edge 5 m longer for the route, so that the empty parallel lane is preferred\n\
         route_load_weight = 5.0\n"
    );
    if WITH_BATTERY {
        let _ = write!(
            s,
            "# chargers: the six stalls in the north-east (stations of type charger)\n\
             charger_power_w = {CHARGER_POWER_W:?}\ncharge_low = {CHARGE_LOW:?}\n"
        );
    }
    for (k, (prefix, factsheet, action, _)) in FLEET.iter().enumerate() {
        let _ = write!(
            s,
            "\n[[robot]]\nmanufacturer = \"ACME\"\nserial_prefix = \"{prefix}\"\n\
             factsheet = \"{factsheet}\"\nstart_nodes = [\n"
        );
        for row in starts[k].chunks(8) {
            let line: Vec<String> = row.iter().map(|&n| format!("\"{}\"", id(n))).collect();
            let _ = writeln!(s, "    {},", line.join(", "));
        }
        let _ = write!(
            s,
            "]\n# each robot faces along the road that leaves its start node\nstart_heading = \"road\"\naction_time = 1.0\n"
        );
        if WITH_BATTERY {
            let (wh, per_m, idle, work) = BATTERY[k];
            let _ = write!(
                s,
                "battery_wh = {wh:?}\ndrive_wh_per_m = {per_m:?}\nidle_w = {idle:?}\nwork_w = {work:?}\n\
                 start_charge = [50.0, 100.0]\n"
            );
        }
        let _ = write!(
            s,
            "\n[robot.action_times]\npick = {action:?}\ndrop = {action:?}\n"
        );
    }
    let stations: Vec<String> = net
        .stations
        .iter()
        .filter(|st| st.id != "PARK")
        .map(|st| format!("\"{}\"", st.id))
        .collect();
    let _ = write!(
        s,
        "\n[jobs]\nrate_per_hour = {:?}\n# every station but the parking\nstations = [\n",
        robots as f64 * 10.0
    );
    for row in stations.chunks(8) {
        let _ = writeln!(s, "    {},", row.join(", "));
    }
    s.push_str(
        "]\n\n[disturbances]\nenabled = true\nspeed_variation = 0.05\naction_variation = 0.2\n\
         stops_per_hour = 12.0\nstop_seconds_min = 5.0\nstop_seconds_max = 15.0\n",
    );
    s
}

/// What a three-wheel robot (it cannot turn on the spot: the next road must start in
/// the direction the last one ended) can reach from each of its start nodes,
/// starting east: the number of nodes and stations, and where the roads do not join
/// smoothly (the places where it gets stuck).
fn three_wheel_report(graph: &Graph, net: &Net, starts: &[usize]) {
    let (m, s) = ("ACME", "T3");
    let kin = Kinematics::ThreeWheel;
    let mut reached_edges: HashSet<usize> = HashSet::new();
    let mut todo: Vec<usize> = Vec::new();
    for &n in starts {
        let Some(idx) = graph.node_idx(&id(n)) else {
            continue;
        };
        for &e in graph.out_edges(idx) {
            if graph.edges[e].props_for(m, s).is_some()
                && graph.can_start(0.0, None, e, kin, m, s)
                && reached_edges.insert(e)
            {
                todo.push(e);
            }
        }
    }
    let mut stuck: HashMap<usize, usize> = HashMap::new();
    while let Some(e) = todo.pop() {
        let end = graph.edges[e].end;
        let mut any = false;
        for &next in graph.out_edges(end) {
            if graph.edges[next].props_for(m, s).is_none() {
                continue;
            }
            if graph.can_follow(e, next, kin, m, s) {
                any = true;
                if reached_edges.insert(next) {
                    todo.push(next);
                }
            }
        }
        if !any {
            *stuck.entry(end).or_default() += 1;
        }
    }
    let nodes: HashSet<usize> = reached_edges
        .iter()
        .flat_map(|&e| [graph.edges[e].start, graph.edges[e].end])
        .collect();
    let served = net
        .stations
        .iter()
        .filter(|st| {
            st.points
                .iter()
                .filter_map(|&p| net.find(p))
                .filter_map(|n| graph.node_idx(&id(n)))
                .any(|n| nodes.contains(&n))
        })
        .count();
    // where it can also come back from: the biggest strongly connected set of edges
    // (an edge leads on to another edge only if the robot can follow it)
    let edges: Vec<usize> = reached_edges.iter().copied().collect();
    let pos: HashMap<usize, usize> = edges.iter().enumerate().map(|(i, &e)| (e, i)).collect();
    let mut succ: Vec<Vec<usize>> = vec![Vec::new(); edges.len()];
    let mut pred: Vec<Vec<usize>> = vec![Vec::new(); edges.len()];
    for (i, &e) in edges.iter().enumerate() {
        for &next in graph.out_edges(graph.edges[e].end) {
            if let Some(&j) = pos.get(&next)
                && graph.can_follow(e, next, kin, m, s)
            {
                succ[i].push(j);
                pred[j].push(i);
            }
        }
    }
    // Kosaraju, without recursion
    let mut order: Vec<usize> = Vec::new();
    let mut seen = vec![false; edges.len()];
    for root in 0..edges.len() {
        if seen[root] {
            continue;
        }
        seen[root] = true;
        let mut stack = vec![(root, 0usize)];
        while let Some((v, k)) = stack.pop() {
            if k < succ[v].len() {
                stack.push((v, k + 1));
                let w = succ[v][k];
                if !seen[w] {
                    seen[w] = true;
                    stack.push((w, 0));
                }
            } else {
                order.push(v);
            }
        }
    }
    let mut component = vec![usize::MAX; edges.len()];
    let mut sizes: Vec<usize> = Vec::new();
    for &root in order.iter().rev() {
        if component[root] != usize::MAX {
            continue;
        }
        let id = sizes.len();
        sizes.push(0);
        let mut stack = vec![root];
        component[root] = id;
        while let Some(v) = stack.pop() {
            sizes[id] += 1;
            for &w in &pred[v] {
                if component[w] == usize::MAX {
                    component[w] = id;
                    stack.push(w);
                }
            }
        }
    }
    let main = (0..sizes.len()).max_by_key(|&c| sizes[c]).unwrap_or(0);
    let main_nodes: HashSet<usize> = (0..edges.len())
        .filter(|&i| component[i] == main)
        .flat_map(|i| [graph.edges[edges[i]].start, graph.edges[edges[i]].end])
        .collect();
    let back = net
        .stations
        .iter()
        .filter(|st| {
            st.points
                .iter()
                .filter_map(|&p| net.find(p))
                .filter_map(|n| graph.node_idx(&id(n)))
                .any(|n| main_nodes.contains(&n))
        })
        .count();
    println!(
        "three-wheel robots can also come back from {} of {} edges (biggest loop: {} edges, {} stations in it)",
        sizes.get(main).copied().unwrap_or(0),
        edges.len(),
        sizes.get(main).copied().unwrap_or(0),
        back
    );
    let outside: Vec<&str> = net
        .stations
        .iter()
        .filter(|st| {
            !st.points
                .iter()
                .filter_map(|&p| net.find(p))
                .filter_map(|n| graph.node_idx(&id(n)))
                .any(|n| main_nodes.contains(&n))
        })
        .map(|st| st.id.as_str())
        .collect();
    println!("stations outside the loop of the three-wheel robots: {outside:?}");
    let mut dead: Vec<(usize, usize)> = stuck.into_iter().collect();
    dead.sort_by_key(|&(n, c)| (std::cmp::Reverse(c), n));
    println!(
        "three-wheel robots: {} of {} edges, {} of {} stations reachable; stuck at {} nodes, e.g. {:?}",
        reached_edges.len(),
        graph.edges.len(),
        served,
        net.stations.len(),
        dead.len(),
        dead.iter()
            .take(6)
            .map(|&(n, _)| {
                let node = &graph.nodes[n];
                (node.id.clone(), node.x.round(), node.y.round())
            })
            .collect::<Vec<_>>()
    );
}

/// The stations that a robot type can serve: those with an interaction node that it
/// can reach from node 0 and from which it can come back to node 0.
fn stations_served(net: &Net, wide: bool) -> (usize, usize) {
    let allowed: Vec<&Edge> = net
        .edges
        .iter()
        .filter(|e| !(wide && narrow(edge_middle(net, e))))
        .collect();
    let reach = |forward: bool| {
        let mut next = vec![Vec::new(); net.nodes.len()];
        for e in &allowed {
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
        seen
    };
    let (out, back) = (reach(true), reach(false));
    let served = net
        .stations
        .iter()
        .filter(|s| {
            s.points
                .iter()
                .filter_map(|&p| net.find(p))
                .any(|n| out[n] && back[n])
        })
        .count();
    (served, net.stations.len())
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

/// The loop round the racks of a cell of the west part: a rectangle `DOUBLE_HALF` inside the
/// cell, an arc of `LOOP_R` at each corner. Clockwise (the left side north) if `cw`.
#[derive(Clone, Copy)]
struct Loop {
    i: usize,
    j: usize,
    x0: f64,
    x1: f64,
    y0: f64,
    y1: f64,
    cw: bool,
}

/// The radius (m) of the arcs at the corners of the loops (a forklift needs 1.7 m for its
/// full speed). 2.0 m (it was 3.0): the sides are 2 m longer, so that a lane change (an
/// S-curve of 8 m) ends before the corner arc and not on its start (user, 2026-11-24).
const LOOP_R: f64 = 2.0;

/// The road the loops of column `i` of the west part have on their right, row `j`: for the last
/// column the second road beside the column at x = 100 where there is one (`COLUMN_COPY`),
/// else the column itself.
fn column_x(i: usize, j: usize) -> f64 {
    if i == NET_X.len() - 2 && COLUMN_COPY_ROWS.contains(&j) {
        COLUMN_COPY_X
    } else if i == NET_X.len() - 2 && j == 0 {
        // the bottom row: the loop is as far from the column as at the copy (3.6 m), a large
        // robot that turns between the column and the ring road needs the room
        NET_X[i + 1] - DOUBLE_HALF
    } else {
        NET_X[i + 1]
    }
}

/// The second road beside the column at x = 100: 3.6 m west of it (the loops are 1.8 m west of
/// it), from y = 112 to y = 40 (rows 2 to 5 of the loops), with bridges to the column.
const COLUMN_COPY_X: f64 = 96.4;
const COLUMN_COPY_ROWS: std::ops::RangeInclusive<usize> = 1..=6;
/// The bridges between the column and its copy (10 m long S-curves, driving south):
/// `(y on the column, true: from the column to the copy)`; the end on the column is not near the
/// junctions of the column (at y = 130, 90, 84, 74, 66, 37, 10).
/// The third number is the length of the bridge along the road (m): the one at 62 is shorter,
/// so that its end on the copy (y = 55.2, a node already: the end of a lane change of the loops)
/// is 3.5 m from the corners of the loops beside the copy, not 2.5 m (a robot waiting at the
/// old end, y = 48, was in the way of the one on the loop's corner arc).
const COLUMN_BRIDGES: [(f64, bool, f64); 4] = [
    (122.0, true, 10.0),
    (98.0, false, 10.0),
    (62.0, true, 6.8),
    (28.0, false, 10.0),
];

/// The loops of all the cells of the west part. Neighbours turn in opposite directions (a
/// chessboard), so that the two sides that face each other in a street run the same way.
fn loops() -> Vec<Loop> {
    let h = DOUBLE_HALF;
    let mut out = Vec::new();
    for (i, x) in NET_X.windows(2).enumerate() {
        for (j, y) in NET_Y.windows(2).enumerate() {
            if j == STREET_ROW {
                continue;
            }
            out.push(Loop {
                i,
                j,
                x0: x[0] + h,
                x1: column_x(i, j) - h,
                y0: y[0] + h,
                y1: y[1] - h,
                cw: (i + j).is_multiple_of(2),
            });
        }
    }
    out
}

/// The four straight sides of a loop `(start, end)` in the direction a robot drives: the left,
/// the top, the right, the bottom side (between the arcs at the corners).
fn loop_sides(l: &Loop) -> [(P, P); 4] {
    let r = LOOP_R;
    if l.cw {
        [
            ((l.x0, l.y0 + r), (l.x0, l.y1 - r)),
            ((l.x0 + r, l.y1), (l.x1 - r, l.y1)),
            ((l.x1, l.y1 - r), (l.x1, l.y0 + r)),
            ((l.x1 - r, l.y0), (l.x0 + r, l.y0)),
        ]
    } else {
        [
            ((l.x0, l.y1 - r), (l.x0, l.y0 + r)),
            ((l.x1 - r, l.y1), (l.x0 + r, l.y1)),
            ((l.x1, l.y0 + r), (l.x1, l.y1 - r)),
            ((l.x0 + r, l.y0), (l.x1 - r, l.y0)),
        ]
    }
}

/// How far a lane change goes along the street (m) between two lanes `gap` apart (an S-curve
/// of two arcs of about 3 m needs 8 m for 3.6 m, 6 m for 1.8 m); a change that would end less
/// than 2.8 m (a forklift is 2.4 m long and has a field) before the end of the side goes to the
/// end of it. If the side is a little too short for the full S-curve the change gets shorter
/// (down to 6 m, radius about 1.7 m for 3.6 m) so that 2.8 m of the side are left after it:
/// pieces of 2.4 m (it was 2.4) were too short for a forklift and its field.
fn change_along(gap: f64, length: f64) -> f64 {
    let base: f64 = if gap > 3.0 { 8.0 } else { 6.0 };
    let along = base.min(length - 2.8);
    if along >= 6.0 { along } else { length }
}

/// The lane changes of the loops: `(from, to, direction)` for `Net::s_link`. Between two
/// neighbouring loops, along their facing sides (which run the same way): one change (from
/// the loop of an even sum `i + j` to the other, or the other way round), two (one each way,
/// at the two ends of the side) if the side is long enough. The ring on the left and the
/// column at x = 100 on the right (the ring at the bottom and the top) are a lane like any
/// other: a change from it into the loop beside it, and one out of the loop, where they run
/// the same way.
fn lane_changes() -> Vec<(P, P, P)> {
    let all = loops();
    let find = |i: usize, j: usize| all.iter().find(|l| l.i == i && l.j == j).copied();
    let mut out = Vec::new();
    // `a` and `b` are the starts of two lanes in the driving direction `d`, `length` m of
    // straight side, `gap` apart; `first`: the change from a to b is the one if there is one
    let mut changes = |a: P, b: P, d: P, length: f64, gap: f64, first_a_to_b: bool, both: bool| {
        let along = change_along(gap, length);
        let at = |p: P, t: f64| (p.0 + d.0 * t, p.1 + d.1 * t);
        if both && length >= 2.0 * along + 2.4 {
            out.push((at(a, 0.0), at(b, along), d));
            out.push((at(b, length - along), at(a, length), d));
        } else if first_a_to_b {
            out.push((at(a, 0.0), at(b, along), d));
        } else {
            out.push((at(b, 0.0), at(a, along), d));
        }
    };
    for l in &all {
        // the loop to the east: its left side faces the right side of this one
        if let Some(m) = find(l.i + 1, l.j) {
            let (right, left) = (loop_sides(l)[2], loop_sides(&m)[0]);
            let d = if l.cw { (0.0, -1.0) } else { (0.0, 1.0) };
            let length = (l.y1 - l.y0) - 2.0 * LOOP_R;
            // (one change: east along the even rows, west along the odd ones)
            changes(
                right.0,
                left.0,
                d,
                length,
                m.x0 - l.x1,
                l.j.is_multiple_of(2),
                true,
            );
        }
        // the loop to the north: its bottom side faces the top side of this one
        if let Some(m) = find(l.i, l.j + 1) {
            let (top, bottom) = (loop_sides(l)[1], loop_sides(&m)[3]);
            let d = if l.cw { (1.0, 0.0) } else { (-1.0, 0.0) };
            let length = (l.x1 - l.x0) - 2.0 * LOOP_R;
            // (one change: north along the even columns, south along the odd ones; the
            // corner cell (0, 0) would have two ways out and none in: its change goes south)
            changes(
                top.0,
                bottom.0,
                d,
                length,
                m.y0 - l.y1,
                l.i.is_multiple_of(2) != (l.i == 0 && (l.j == 0 || find(l.i, l.j - 1).is_none())),
                true,
            );
        }
    }
    // the ring and the column: the lane is on the line of the cell's side, `DOUBLE_HALF` away
    let h = DOUBLE_HALF;
    let (last_i, last_j) = (NET_X.len() - 2, NET_Y.len() - 2);
    for l in &all {
        let sides = loop_sides(l);
        // the left side faces the ring (runs south): same way if the loop is counter-clockwise
        if l.i == 0 && !l.cw {
            let length = (l.y1 - l.y0) - 2.0 * LOOP_R;
            changes(
                (l.x0 - h, sides[0].0.1),
                sides[0].0,
                (0.0, -1.0),
                length,
                h,
                true,
                true,
            );
        }
        // the right side faces the column at x = 100 (runs south): same way if clockwise
        if l.i == last_i && l.cw {
            let length = (l.y1 - l.y0) - 2.0 * LOOP_R;
            // (the lane: the copy of the column, or the column itself; the loop of the bottom row
            // is further from it, see `column_x`)
            let lane_x = if l.j == 0 {
                NET_X[last_i + 1]
            } else {
                l.x1 + h
            };
            changes(
                (lane_x, sides[2].0.1),
                sides[2].0,
                (0.0, -1.0),
                length,
                lane_x - l.x1,
                false,
                true,
            );
        }
        // the bottom faces the ring road at the bottom (runs east): same way if
        // counter-clockwise
        if l.j == 0 && !l.cw {
            let length = (l.x1 - l.x0) - 2.0 * LOOP_R;
            changes(
                (sides[3].0.0, l.y0 - h),
                sides[3].0,
                (1.0, 0.0),
                length,
                h,
                true,
                true,
            );
        }
        // the top faces the ring road at the top (runs west): counter-clockwise
        if l.j == last_j && !l.cw {
            let length = (l.x1 - l.x0) - 2.0 * LOOP_R;
            changes(
                (sides[1].0.0, l.y1 + h),
                sides[1].0,
                (-1.0, 0.0),
                length,
                h,
                true,
                true,
            );
        }
    }
    out
}

/// The loops of the west part with their lane changes.
fn loop_net(net: &mut Net) {
    let r = LOOP_R;
    for l in loops() {
        let sides = loop_sides(&l);
        for (a, b) in sides {
            net.line(a, b);
        }
        // the arcs at the corners, in the direction of the loop
        let corners = [
            ((l.x0 + r, l.y0), (l.x0, l.y0), (l.x0, l.y0 + r)),
            ((l.x0, l.y1 - r), (l.x0, l.y1), (l.x0 + r, l.y1)),
            ((l.x1 - r, l.y1), (l.x1, l.y1), (l.x1, l.y1 - r)),
            ((l.x1, l.y0 + r), (l.x1, l.y0), (l.x1 - r, l.y0)),
        ];
        for (a, c, b) in corners {
            if l.cw {
                net.arc(a, c, b);
            } else {
                net.arc(b, c, a);
            }
        }
    }
    for (from, to, d) in lane_changes() {
        net.s_link(from, to, d);
    }
}

/// Where the stops of a loop are: on each of its four sides the point farthest from the
/// nodes that are on the side (its ends and the lane changes), if that is 2.6 m or more
/// from them.
fn loop_stops(l: &Loop) -> Vec<P> {
    let changes = lane_changes();
    let mut out = Vec::new();
    for (a, b) in loop_sides(l) {
        let length = (b.0 - a.0).hypot(b.1 - a.1);
        let d = ((b.0 - a.0) / length, (b.1 - a.1) / length);
        // the along-coordinates of the nodes on this side (its ends, the ends of lane changes)
        let mut nodes = vec![0.0, length];
        for &(from, to, _) in &changes {
            for p in [from, to] {
                let off = (p.0 - a.0) * d.1 - (p.1 - a.1) * d.0;
                let t = (p.0 - a.0) * d.0 + (p.1 - a.1) * d.1;
                if off.abs() < 0.05 && t > 0.05 && t < length - 0.05 {
                    nodes.push(t);
                }
            }
        }
        let steps = (length * 4.0) as usize;
        let best = (0..=steps)
            .map(|k| k as f64 * 0.25)
            .map(|t| {
                let gap = nodes
                    .iter()
                    .map(|n| (n - t).abs())
                    .fold(f64::INFINITY, f64::min);
                (t, gap)
            })
            .max_by(|x, y| x.1.total_cmp(&y.1));
        if let Some((t, gap)) = best
            && gap >= 2.6
        {
            out.push((a.0 + d.0 * t, a.1 + d.1 * t));
        }
    }
    out
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
    // (in warehouse 3 the entry is a link from the right edge of the double net, half way
    // between two crossings, to the first aisle: the robots that use the serpentine can turn
    // on the spot; the part of the aisle and of the outside road below it is not reached
    // and goes)
    let _ = (t, d);
    net.line((RIGHT_EDGE_X, 37.0), (x0, 37.0));
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
    // - (the top road used to go on east and leave to the node where the column at x = 140
    //   meets the boulevard, by an arc and a link at 45 degrees; taken out: it put six
    //   roads and the arcs of the crossing into one point, and the labyrinth has its way
    //   out in the last half circle)
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
        .is_some_and(|i| !lane_east(i))
}

/// The direction of road number `i` of a grid, counted from the ring: the roads alternate
/// (east, west, east, ...); for the columns north, south, north, ... (`lane_east` is then
/// "north"). Each road has two lanes that run the same way.
fn lane_east(i: usize) -> bool {
    i.is_multiple_of(2)
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
    /// The station: a rack block, a row of shelving between two aisles or the
    /// shelving along a wall (its stops are the interaction nodes of one station).
    station: String,
    /// What the station is called, and what it is (`rack`, `shelf`).
    label: &'static str,
    kind: &'static str,
    /// Where the loading lane leaves the road and where it rejoins it.
    a: P,
    b: P,
    /// The stops along the lane (parallel to the road, 1.8 m beside it), in the order the
    /// robots drive; at least one. Any of them serves the whole station.
    stops: Vec<P>,
    /// Is this a station? (A lane along the vertical side of a block is only a way round
    /// the block: no station.)
    has_station: bool,
    /// The stops are on a lane of the double net (the lane is there already): only the
    /// stops are made, no loading lane.
    on_lane: bool,
}

/// Half the distance between the two lines of a street (m): the lines run along the racks.
const DOUBLE_HALF: f64 = 1.8;

/// The streets of the double net (west part of the hall): the vertical ones at these x
/// (the left one is the outer street), the horizontal ones at these y (the bottom and the
/// top one are outer streets). East of the last vertical street the streets are single lines
/// again, up to the column at x = 100.
const NET_X: [f64; 5] = [RING, WEST_X[0], WEST_X[1], WEST_X[2], WEST_X[3]];
const NET_Y: [f64; 8] = [
    RING,
    WEST_Y[0],
    WEST_Y[1],
    WEST_Y[2],
    WEST_Y[3],
    WEST_Y[4],
    WEST_Y[5],
    H - RING,
];

/// Where the single lines of the streets begin (x), east of the double net: the two lines of
/// a street run into one (or one into two) by S-curves from the last vertical street to here.
const FUNNEL_X: f64 = 120.0;

/// Where the streets of the middle of the hall start at the right edge of the double net: the
/// outer line of the last vertical street, x = 100 + `DOUBLE_HALF` (the boulevard, the north
/// serpentine and the vertical serpentine branch off it).
const RIGHT_EDGE_X: f64 = WEST_X[3];

/// Are the two loading lanes between two rows of racks the road (joined at both ends and by
/// crossovers), instead of a road with a lane on each side (three ways side by side)?
const SHARED_LANES: bool = true;

/// The shortest distance between two stops (m) for a crossover between the lanes: the
/// lanes are 3.6 m apart, a shorter S-curve would be too sharp.
const CROSSOVER_MIN_GAP: f64 = 5.5;

/// How far from the junctions at the ends of a cell a loading lane begins and ends (m): the
/// turn arc of the junction (3 m) and a road of 2.8 m after it, a forklift and its field
/// (at 5 m the road between the arc and the lane was 2 m long). Less than 3.5 m and the arc
/// does not fit.
const LANE_MARGIN: f64 = 5.8;

/// The distance between the stops of a loading lane (m): a forklift is 2.4 m long.
const STOP_GAP: f64 = 4.0;

/// The bridges between the ring road and the road along the south wall: `(x, true)` leaves the
/// ring at x for the road (12 m long), `(x, false)` leaves the road for the ring.
const BRIDGES_SOUTH: [(f64, bool); 6] = [
    // (45: one more in the long gap between the wall road's start at x = 40 and the bridge at
    // 80, to take traffic off the ring)
    (45.0, true),
    // (80: from the ring to the road; it was from the road to the ring, landing at x = 92, just
    // before the column at x = 100, where 143 of 249 robots waited)
    (80.0, true),
    (108.0, true),
    (124.0, false),
    (146.0, true),
    (162.0, false),
];

/// The same along the north wall (the roads run west): from x down to x - 12.
const BRIDGES_NORTH: [(f64, bool); 5] = [
    (182.0, false),
    (140.0, true),
    (124.0, false),
    (100.0, true),
    (84.0, false),
];

/// Where along the shelving of the lanes at the south and north walls the stops are (x).
const WALL_STOPS_X: [f64; 3] = [70.0, 110.0, 150.0];

/// The stops of the shelving along the walls: the lanes along the south and the north
/// wall have three stops each, 0.4 m from the shelving (no niche needed). (The
/// aisles of the labyrinths have stops on the aisle itself: see `build_net`.)
fn shelf_pockets() -> Vec<Pocket> {
    let mut out = Vec::new();
    // the lanes along the south wall (they run east) and the north wall (west)
    // (the stop is 1.5 m from the lane: at 1.8 m a small robot that turns into the pocket
    // dips into the shelving, which is 0.3 to 1.8 m from the wall)
    for (station, y, bulge, east) in [("WALL-S", 4.0, -1.5, true), ("WALL-N", H - 4.0, 1.5, false)]
    {
        for xc in WALL_STOPS_X {
            let (a, b) = if east {
                ((xc - 4.0, y), (xc + 4.0, y))
            } else {
                ((xc + 4.0, y), (xc - 4.0, y))
            };
            out.push(Pocket {
                station: station.into(),
                label: "Shelf",
                kind: "shelf",
                a,
                b,
                stops: vec![(xc, y + bulge)],
                has_station: true,
                on_lane: false,
            });
        }
    }
    out
}

/// All the stops in front of racks and shelves.
fn all_pockets() -> Vec<Pocket> {
    let mut all = rack_pockets();
    all.extend(shelf_pockets());
    all
}

fn cell_pockets(
    out: &mut Vec<Pocket>,
    station: &str,
    (xl, xr): (f64, f64),
    (yb, yt): (f64, f64),
    runs_east: fn(f64) -> bool,
) {
    let cx = (xl + xr) / 2.0;
    // the loading lane runs along the rack face between the two junctions (5 m clear of
    // each, for the arcs); in a short cell it is only the one stop in the middle
    let (lo, hi) = if xr - xl - 2.0 * LANE_MARGIN >= 8.0 {
        (xl + LANE_MARGIN, xr - LANE_MARGIN)
    } else {
        // (a short cell: the lane keeps `LANE_MARGIN` from the junctions, the S-curves
        // get shorter, see `cell_lanes_vertical`)
        let half = ((xr - xl) / 2.0 - LANE_MARGIN).clamp(3.0, 4.0);
        (cx - half, cx + half)
    };
    // the stops, STOP_GAP apart, evenly between the two S-curves (4 m long each)
    let room = hi - lo - 8.0;
    let n = (room / STOP_GAP).floor() as usize + 1;
    let xs: Vec<f64> = if n == 1 {
        vec![cx]
    } else {
        (0..n)
            .map(|k| lo + 4.0 + room * k as f64 / (n - 1) as f64)
            .collect()
    };
    for (y, bulge) in [(yb, POCKET_OFFSET), (yt, -POCKET_OFFSET)] {
        let ring = (y - RING).abs() < 1e-9 || (y - (H - RING)).abs() < 1e-9;
        // not in the corners of the ring road, and not where a lane along the wall
        // branches off it (4 m clear of the branch, for the arc)
        let at_lane = LANES_X.iter().any(|&x| x > lo - 4.0 && x < hi + 4.0);
        if ring && (lo < RING + CORNER + 1.0 || hi > W - RING - CORNER - 1.0 || at_lane) {
            continue;
        }
        let east = runs_east(y);
        let (a, b) = if east {
            ((lo, y), (hi, y))
        } else {
            ((hi, y), (lo, y))
        };
        let mut stops: Vec<P> = xs.iter().map(|&x| (x, y + bulge)).collect();
        if !east {
            stops.reverse();
        }
        out.push(Pocket {
            station: station.into(),
            label: "Rack",
            kind: "rack",
            a,
            b,
            stops,
            has_station: true,
            on_lane: false,
        });
    }
}

/// The stops of a cell of the double net: on the two lanes along its long sides (the lane of
/// the street below it, the lane of the street above it), every `STOP_GAP` between the
/// arcs of the crossings (8 m from the middle of each street). Any stop serves the cell.
fn cell_loop_stops(out: &mut Vec<Pocket>, station: &str, i: usize, j: usize) {
    let Some(l) = loops().into_iter().find(|l| l.i == i && l.j == j) else {
        return;
    };
    let stops = loop_stops(&l);
    if stops.is_empty() {
        return;
    }
    out.push(Pocket {
        station: station.into(),
        label: "Rack",
        kind: "rack",
        a: stops[0],
        b: stops[stops.len() - 1],
        stops,
        has_station: true,
        on_lane: true,
    });
}

/// Which way the columns of the east grid run (north?) at `x`: the ring on the right runs
/// north.
fn east_col_runs_north(x: f64) -> bool {
    if (x - (W - RING)).abs() < 1e-9 {
        return true;
    }
    EAST_X
        .iter()
        .position(|&col| (col - x).abs() < 1e-9)
        .is_some_and(lane_east)
}

/// The lanes along the vertical sides of a cell: a way round the block (no station). One
/// lane on each side of a column (not along the ring), 1.8 m from its middle line; the
/// lane in the cell on the other side of the column is its twin (see `SHARED_LANES`).
fn cell_lanes_vertical(
    out: &mut Vec<Pocket>,
    (xl, xr): (f64, f64),
    (yb, yt): (f64, f64),
    runs_north: fn(f64) -> bool,
) {
    // only where the cell is tall enough for a lane between the two junctions
    if yt - yb < 14.0 {
        return;
    }
    let cy = (yb + yt) / 2.0;
    let (lo, hi) = if yt - yb - 2.0 * LANE_MARGIN >= 8.0 {
        (yb + LANE_MARGIN, yt - LANE_MARGIN)
    } else {
        // (a short cell: the lane keeps `LANE_MARGIN` from the junctions and its S-curves
        // get shorter, 3.2 m for the 18 m cells: radius 1.9 m, a forklift needs 1.7 m)
        let half = ((yt - yb) / 2.0 - LANE_MARGIN).clamp(3.0, 4.0);
        (cy - half, cy + half)
    };
    for (x, bulge) in [(xl, POCKET_OFFSET), (xr, -POCKET_OFFSET)] {
        // the ring is the way round on its side
        if (x - RING).abs() < 1e-9 || (x - (W - RING)).abs() < 1e-9 {
            continue;
        }
        // (not in the bottom cell beside the column at x = 140: the half circle of the middle
        // labyrinth lands there, 3 m from the lane's first node, where a waiting robot is in its
        // way: the lane is only a way past anyway)
        if (x - V_XCOL).abs() < 1e-9 && (yb - RING).abs() < 1e-9 {
            continue;
        }
        // (beside the column at x = 140 the lane starts where the shortcut arcs of the middle
        // labyrinth land, 4 m from the middle of the cell: these keep the long S-curves)
        let (lo, hi) = if (x - V_XCOL).abs() < 1e-9 {
            (cy - 4.0, cy + 4.0)
        } else {
            (lo, hi)
        };
        let north = runs_north(x);
        let (a, b) = if north {
            ((x, lo), (x, hi))
        } else {
            ((x, hi), (x, lo))
        };
        out.push(Pocket {
            station: String::new(),
            label: "",
            kind: "",
            a,
            b,
            stops: vec![(x + bulge, cy)],
            has_station: false,
            on_lane: false,
        });
    }
}

/// The stops in front of the rack blocks of both grids: one or two per block (on the
/// road below it and on the road above it).
fn rack_pockets() -> Vec<Pocket> {
    let mut out = Vec::new();
    for (i, x) in WEST_BOUNDS_X.windows(2).enumerate() {
        for (j, y) in WEST_BOUNDS_Y.windows(2).enumerate() {
            let name = format!("RW{i}{j}");
            if i < NET_X.len() - 1 {
                // a cell with a loop: stops on its sides
                cell_loop_stops(&mut out, &name, i, j);
            } else {
                // the cell east of the double net: single roads, pockets (from the funnels
                // to the column)
                cell_pockets(
                    &mut out,
                    &name,
                    (FUNNEL_X, x[1]),
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
            cell_lanes_vertical(&mut out, (x[0], x[1]), *y, east_col_runs_north);
        }
    }
    out
}

/// Is the cell `(i, j)` of the west grid a staging cell (a roundabout instead of
/// racking)?
fn is_staging(_i: usize, _j: usize) -> bool {
    // (none in warehouse 3: every cell of the double net is racking)
    false
}

fn build_net() -> Net {
    let mut net = Net::default();
    let (x0, x1, y0, y1) = (RING, W - RING, RING, H - RING);
    let r = CORNER;

    // the ring road, counter-clockwise: the bottom east, the right side north, the
    // top west, the left side south; the corners are arcs
    net.arc((x1 - r, y0), (x1, y0), (x1, y0 + r));
    net.arc((x1, y1 - r), (x1, y1), (x1 - r, y1));
    // (the corners on the left have a smaller arc: the loops are close to the ring there)
    let rl = LOOP_R;
    net.arc((x0, y0 + rl), (x0, y0), (x0 + rl, y0));
    net.arc((x0 + rl, y1), (x0, y1), (x0, y1 - rl));
    net.line((x0 + rl, y0), (x1 - r, y0));
    net.line((x1, y0 + r), (x1, y1 - r));
    net.line((x1 - r, y1), (x0 + rl, y1));
    net.line((x0, y1 - rl), (x0, y0 + rl));

    // the west part: a loop round the racks of every cell, with lane changes between
    // neighbouring loops, and to the ring and the column at x = 100 (it runs south from the
    // top ring to the bottom one; the boulevard and the entries of the labyrinths branch off it)
    loop_net(&mut net);
    net.line((WEST_X[3], y1), (WEST_X[3], y0));
    // the copy of the column, beside it on the west, with bridges (S-curves of 10 m: the road is
    // 3.6 m from the column) from the column to the copy and back. The copy goes from y = 112
    // (a bridge from the column at 122) to y = 40 (a bridge back to the column at 30)
    net.line((COLUMN_COPY_X, 112.0), (COLUMN_COPY_X, 38.0));
    for (y, to_copy, len) in COLUMN_BRIDGES {
        let (from, to) = if to_copy {
            ((WEST_X[3], y), (COLUMN_COPY_X, y - len))
        } else {
            ((COLUMN_COPY_X, y + len), (WEST_X[3], y))
        };
        net.s_link(from, to, (0.0, -1.0));
    }
    // (a street across the west part in line with the boulevard was tried here: `STREET_ROW`;
    // the user wanted duplicates of the ring roads at the top and the bottom instead)
    if STREET_ROW < NET_Y.len() - 1 {
        for &(y, east) in &EAST_ROWS {
            if y == 66.0 || y == 74.0 {
                road(&mut net, (x0, y), (RIGHT_EDGE_X, y), east);
            }
        }
    }

    // the east grid and the boulevard: rows from x = 140 (the boulevard and the top
    // row from x = 100) to the ring; columns from the bottom ring up to the top row
    for &(y, east) in &EAST_ROWS {
        let from = if y == 66.0 || y == 74.0 || y == 84.0 {
            RIGHT_EDGE_X
        } else {
            140.0
        };
        road(&mut net, (from, y), (x1, y), east);
    }
    for (i, &x) in EAST_X.iter().enumerate() {
        road(&mut net, (x, 84.0), (x, y0), !lane_east(i));
        for &(y, _) in &EAST_ROWS {
            net.node((x, y));
        }
    }

    // the labyrinth in the north: seven aisles, in from the column at x = 100 at the
    // bottom; the last half circle lands on the top ring road, which is the way out
    let (start, _) = serpentine(&mut net, H_XA, H_XB, H_Y0, H_STEP, H_AISLES);
    net.line((RIGHT_EDGE_X, start.1), start);
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

    // stops in the aisles of both labyrinths, in the middle of every part of the
    // shelving (never at a shortcut): the robot stands on the aisle, with shelving
    // 1.3 m away on both sides, so the aisle is blocked while it works
    for (station, p) in aisle_stops() {
        net.node(p);
        net.add_station(&station, &format!("Shelf {station}"), "shelf", p);
    }

    // charging and parking bays in the north-east: a lane north from the row at
    // y = 84 to the top ring road, in by an arc from the row (which runs west) and
    // out by an arc onto the ring (which also runs west), so that a robot that cannot
    // turn on the spot can drive it; a stall on each side at every bend of a siding
    let (lane, r) = (212.0, 6.0);
    net.arc((lane + r, 84.0), (lane, 84.0), (lane, 84.0 + r));
    net.line((lane, 84.0 + r), (lane, y1 - r));
    net.arc((lane, y1 - r), (lane, y1), (lane - r, y1));
    // (the stalls were here, beside the lane: now sidings off the wall roads, see below)
    for siding in CHARGER_SIDINGS {
        let (x, south) = siding;
        let y = if south { 4.0 } else { H - 4.0 };
        let dx = if south { SIDING_LENGTH } else { -SIDING_LENGTH };
        net.bow((x, y), siding_apex(siding), (x + dx, y));
        // the stalls are one station: a robot that is sent to park draws one of them
        // (type `charger`: with `charger_power_w` in the scenario a robot charges here; the jobs
        // leave the station out)
        net.add_station("PARK", "Charging", "charger", siding_apex(siding));
    }

    // bays on curved one-way sidings that rejoin the ring further on: west (the left
    // side runs south) from `a` down to `a - 8`, east (it runs north) up to `a + 8`;
    // both ends are at least 4 m from the roads that meet the ring, so that the arcs
    // of 3.5 m fit there
    for (n, (from, apex, to)) in GOODS_IN_BAYS.into_iter().enumerate() {
        net.bow(from, apex, to);
        let name = format!("Goods in {}", n + 1);
        net.add_station(&format!("GOODS-IN-{}", n + 1), &name, "bay", apex);
    }
    for (n, (a, length)) in EAST_BAYS.into_iter().enumerate() {
        // (the sidings branch off the second east road, which runs as far as they do)
        let xr = x1 + EAST_ROAD_DX;
        net.bow((xr, a), (xr + 3.0, a + length / 2.0), (xr, a + length));
        let name = format!("Goods out {}", n + 1);
        let apex = (xr + 3.0, a + length / 2.0);
        net.add_station(&format!("GOODS-OUT-{}", n + 1), &name, "bay", apex);
    }

    // a stop in front of every rack block and every row of shelving, in a pocket off
    // the road (see `rack_pockets` and `shelf_pockets`)
    let pockets = all_pockets();
    for p in &pockets {
        if p.on_lane {
            // the stops are nodes on a line of the double net
            for &stop in &p.stops {
                net.node(stop);
            }
        } else {
            net.loading_lane(p.a, &p.stops, p.b);
        }
        if p.has_station {
            let name = format!("{} {}", p.label, p.station);
            for &stop in &p.stops {
                net.add_station(&p.station, &name, p.kind, stop);
            }
        }
    }
    // where the blocks on both sides of a road have a lane, the two lanes are the road: no
    // road piece between them, and crossovers from one lane to the other (a zigzag, so that no
    // two cross) wherever the stops are far enough apart for a gentle S-curve
    if SHARED_LANES {
        // the road piece, whichever way it is drawn, and on which side of the drive direction
        // a lane is (the sign of the cross product)
        let key = |p: &Pocket| {
            let (a, b) = (cm(p.a), cm(p.b));
            (a.0.min(b.0), a.1.min(b.1), a.0.max(b.0), a.1.max(b.1))
        };
        let side = |p: &Pocket| {
            (p.b.0 - p.a.0) * (p.stops[0].1 - p.a.1) - (p.b.1 - p.a.1) * (p.stops[0].0 - p.a.0)
        };
        for p in &pockets {
            let Some(q) = pockets
                .iter()
                .find(|q| key(q) == key(p) && side(q) * side(p) < 0.0)
            else {
                continue;
            };
            // handle each pair once: from the lane on the left of the drive direction
            if side(p) < 0.0 || p.stops.len() != q.stops.len() {
                continue;
            }
            net.shared_roads.push((p.a, p.b));
            let (dx, dy) = (p.b.0 - p.a.0, p.b.1 - p.a.1);
            let length = dx.hypot(dy);
            let u = (dx / length, dy / length);
            let n = p.stops.len();
            let gap = (p.stops[1.min(n - 1)].0 - p.stops[0].0)
                .hypot(p.stops[1.min(n - 1)].1 - p.stops[0].1);
            if n >= 2 && gap >= CROSSOVER_MIN_GAP {
                for k in 0..n - 1 {
                    let (from, to) = if k % 2 == 0 {
                        (q.stops[k], p.stops[k + 1])
                    } else {
                        (p.stops[k], q.stops[k + 1])
                    };
                    net.s_link(from, to, u);
                }
            }
        }
    }

    // a second road beside the ring road along the south wall (east, like the ring there) and
    // along the north wall (west): the same for all robots, with a bridge now and then (an
    // S-curve of 12 m) from one road to the other and back. The robots going across the hall
    // are not all on the one lane of the ring any more. (The shelving stops along the wall
    // are pockets off this road, narrow: small robots only.)
    let [lw, le] = LANES_X;
    net.line((lw, y0), (lw, 4.0));
    net.line((lw, 4.0), (le, 4.0));
    // (the east link at the south goes from the ring to the road, as at the north: nobody merges
    // from the outer road into the ring just before its corner, where 91 of 153 robots waited)
    net.line((le, y0), (le, 4.0));
    net.line((le, y1), (le, H - 4.0));
    net.line((le, H - 4.0), (lw, H - 4.0));
    net.line((lw, H - 4.0), (lw, y1));
    // bridges, `(x where it starts, true: from the ring to the road)`; the end of the bridge on
    // the road is not within 7 m of a shelving stop (`WALL_STOPS_X`)
    let bridge = 12.0;
    for (x, to_road) in BRIDGES_SOUTH {
        let (from, to) = if to_road {
            ((x, y0), (x + bridge, 4.0))
        } else {
            ((x, 4.0), (x + bridge, y0))
        };
        net.s_link(from, to, (1.0, 0.0));
    }
    // (the north road and ring run west: the bridges go from x down to x - 12)
    for (x, to_road) in BRIDGES_NORTH {
        let (from, to) = if to_road {
            ((x, y1), (x - bridge, H - 4.0))
        } else {
            ((x, H - 4.0), (x - bridge, y1))
        };
        net.s_link(from, to, (-1.0, 0.0));
    }

    // a second road beside the ring road on the east (outside it, 3.6 m from it, both run north),
    // with bridges (S-curves of 10 m) from one to the other and back, as for the column at x = 100:
    // the east ring road was the only way for everything that goes north or south on that side
    // the wall roads go on to it: south road east, up the east road, north road west
    let xe = x1 + EAST_ROAD_DX;
    net.line((le, 4.0), (xe, 4.0));
    net.line((xe, EAST_ROAD_Y.0), (xe, EAST_ROAD_Y.1));
    net.line((xe, H - 4.0), (le, H - 4.0));
    for (y, to_road) in EAST_BRIDGES {
        let (from, to) = if to_road {
            ((x1, y), (x1 + EAST_ROAD_DX, y + 10.0))
        } else {
            ((x1 + EAST_ROAD_DX, y), (x1, y + 10.0))
        };
        net.s_link(from, to, (0.0, 1.0));
    }

    net.split_at_nodes();
    net.remove_shared_roads();
    // every turn between straight roads gets an arc of 3.5 m radius
    net.add_turn_arcs(TURN_RADIUS);
    net.remove_corner_nodes();
    // and no edge is longer than `MAX_EDGE`
    net.split_long_edges(MAX_EDGE);
    // what leads nowhere (the ends of the lines at the edge of the net, arcs that end beyond
    // it) goes
    net.prune_dead_ends();
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
        .filter(|e| {
            CROSSING_ARCS
                || !net
                    .crossing_arcs
                    .contains(&arc_key(net.nodes[e.a], net.nodes[e.b]))
        })
        .map(|e| {
            let mut props = json!({
                "mobileRobotTypes": edge_types(net, e),
                "orientationType": "TANGENTIAL",
                "mobileRobotOrientations": [0.0],
            });
            let (a, b) = (net.nodes[e.a], net.nodes[e.b]);
            // nobody turns on the spot at a corner of a crossing of the double net
            if net.no_rotation.contains(&cm(b)) {
                props["rotationAtEndNodeAllowed"] = json!("NONE");
            }
            if net.no_rotation.contains(&cm(a)) {
                props["rotationAtStartNodeAllowed"] = json!("NONE");
            }
            match e.curve {
                Some(Curve::Arc(corner, weight)) => {
                    props["trajectory"] = json!({
                        "degree": 2,
                        "controlPoints": [
                            {"x": a.0, "y": a.1},
                            {"x": corner.0, "y": corner.1, "weight": weight},
                            {"x": b.0, "y": b.1},
                        ],
                    });
                }
                Some(Curve::Cubic(c1, c2)) => {
                    props["trajectory"] = json!({
                        "degree": 3,
                        "controlPoints": [
                            {"x": a.0, "y": a.1},
                            {"x": c1.0, "y": c1.1},
                            {"x": c2.0, "y": c2.1},
                            {"x": b.0, "y": b.1},
                        ],
                    });
                }
                None => {}
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
            "lifId": "warehouse4",
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
        // (racks are stretched to fill the length: the stops at the ends of a block are then
        // close to a rack, not 1.2 m or more from the nearest one)
        let bay = if matches!(kind, GeometryKind::Rack) && (b - a) + GAP >= BAY + GAP {
            ((b - a) + GAP) / count as f64 - GAP
        } else {
            BAY
        };
        let used = count as f64 * bay + (count - 1) as f64 * GAP;
        let start = a + ((b - a) - used) / 2.0;
        for i in 0..count {
            let p = start + i as f64 * (bay + GAP);
            let r = if along_x {
                (p, cd.0, p + bay, cd.1)
            } else {
                (cd.0, p, cd.1, p + bay)
            };
            self.polygon(format!("{name}-{i}"), kind, r);
        }
    }

    /// The shelving of the vertical labyrinth: a row between every two aisles, in two
    /// parts with a gap where the shortcut between the two aisles goes through.
    fn vertical_labyrinth(&mut self) {
        for k in 0..V_AISLES {
            let x = v_x(k);
            let across = (x + 1.3, x + V_STEP - 1.3);
            for (i, part) in v_row_parts(k).into_iter().enumerate() {
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
        // (with two or more pairs the aisles between them are widened to fill the block: the
        // first and the last pair are then at the edge, next to the lane)
        let pitch = if pairs >= 2 {
            ((cross.1 - cross.0) + 1.2) / pairs as f64
        } else {
            PITCH
        };
        let used = pairs as f64 * pitch - 1.2;
        let start = cross.0 + ((cross.1 - cross.0) - used) / 2.0;
        for p in 0..pairs {
            let c = start + p as f64 * pitch;
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
    // racks of the east grid and the middle: this far from the road. A forklift (1.4 m wide)
    // at a stop in a pocket stands 1.8 m beside the road, so its side reaches 2.5 m (a bit
    // more on the S-curve into the pocket): 2.0 m put it into the racks
    let inset = POCKET_OFFSET + 1.0;

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
    for (i, (_, apex, _)) in GOODS_IN_BAYS.iter().enumerate() {
        s.polygon(
            format!("bay-W{i}"),
            GeometryKind::Station,
            (apex.0 - 1.6, apex.1 - 1.6, apex.0 + 1.6, apex.1 + 1.6),
        );
    }
    for (name, x, ys) in [(
        "E",
        W - RING + EAST_ROAD_DX + 3.0,
        EAST_BAYS.map(|(a, l)| a + l / 2.0).to_vec(),
    )] {
        for (i, y) in ys.iter().enumerate() {
            s.polygon(
                format!("bay-{name}{i}"),
                GeometryKind::Station,
                (x - 1.6, y - 1.6, x + 1.6, y + 1.6),
            );
        }
    }

    // the west grid: its cells; the racks start `DOUBLE_HALF` + 0.7 m from the middle of a
    // street: the lanes are `DOUBLE_HALF` from it, a forklift is 1.4 m wide (0.7 on each side)
    // and the big arcs of the crossings pass the corners of the blocks
    // (warehouse 4: 3.2 m, so that the arcs at the corners of the loops clear the corners of the
    // racks)
    let west_inset = DOUBLE_HALF + 1.4;
    for (i, j, r) in cells(&WEST_BOUNDS_X, &WEST_BOUNDS_Y, west_inset) {
        if j == STREET_ROW {
            continue;
        }
        // (the loops beside the copy of the column are narrower: so are the racks)
        let r = (r.0, r.1, r.2 - (NET_X[i + 1] - column_x(i, j)), r.3);
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

    // the stops in front of the rack blocks and the shelving: a small station square
    // at each
    let stops: Vec<(String, P)> = all_pockets()
        .into_iter()
        .filter(|p| p.has_station)
        .flat_map(|p| p.stops.into_iter().map(move |s| (p.station.clone(), s)))
        .collect();
    for (k, (station, stop)) in stops.iter().enumerate() {
        s.polygon(
            format!("stop-{station}-{k}"),
            GeometryKind::Station,
            (stop.0 - 0.7, stop.1 - 0.7, stop.0 + 0.7, stop.1 + 0.7),
        );
    }

    // the stops in the labyrinth aisles: a station square on the aisle
    for (k, (station, p)) in aisle_stops().iter().enumerate() {
        s.polygon(
            format!("stop-{station}-{k}"),
            GeometryKind::Station,
            (p.0 - 0.7, p.1 - 0.7, p.0 + 0.7, p.1 + 0.7),
        );
    }

    // the middle: a big block south of the boulevard, a small one north of it (the
    // lane at y = 84 is above), and one beside the labyrinth's entry
    s.vertical_labyrinth();
    s.rack_block("rack-m1", (102.5, 74.0 + inset, 138.0, 84.0 - inset));
    s.rack_block("rack-m2", (102.5, 94.0, 114.0, 121.0));

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
    for k in 0..H_AISLES {
        let y = h_y(k);
        let across = (y + 1.3, y + H_STEP - 1.3);
        for (i, part) in h_row_parts(k).into_iter().enumerate() {
            s.row(
                &format!("shelf-l{k}-{i}"),
                GeometryKind::Shelf,
                true,
                part,
                across,
            );
        }
    }

    // the charging stalls: a station (along the road) at the apex of each siding
    for (i, siding) in CHARGER_SIDINGS.into_iter().enumerate() {
        let (x, y) = siding_apex(siding);
        s.polygon(
            format!("charge-{i}"),
            GeometryKind::Station,
            (x - 2.4, y - 1.2, x + 2.4, y + 1.2),
        );
    }

    // racking beside the labyrinth, and over it along the top ring road
    s.rack_block("rack-q0", (196.0, 90.0, 200.0, 124.0));
    s.rack_block("rack-q1", (150.0, 128.0 - 1.0, 196.0, 128.0));

    // shelving along the south and the north wall, outside the narrow lanes (the stops
    // are 0.4 m from it: no niche needed)
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
    // (only the two on the right: on the left the outer streets are lines of the double net)
    let centres = [
        (W - RING - CORNER, RING + CORNER),
        (W - RING - CORNER, H - RING - CORNER),
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

/// The groups of roads for the table of the use of the roads (the viewer's "Usage" and
/// `usage_report`): boxes `[x0, y0, x1, y1]`, an edge belongs to the first group with its middle
/// in a box.
fn groups_json() -> Value {
    let (w, h, ring) = (W, H, RING);
    let band = |y: f64, half: f64| [0.0, y - half, w, y + half];
    // the bridges of the column: the middle of each is half way between its two ends
    let column_bridges: Vec<Value> = COLUMN_BRIDGES
        .iter()
        .map(|&(y, to_copy, len)| {
            let mid = if to_copy {
                y - len / 2.0
            } else {
                y + len / 2.0
            };
            json!([COLUMN_COPY_X + 1.0, mid - 1.5, WEST_X[3] - 1.0, mid + 1.5])
        })
        .collect();
    json!([
        {"name": "wall road", "boxes": [band(4.0, 0.3), band(h - 4.0, 0.3)]},
        {"name": "bridges wall-ring", "boxes": [[0.0, 4.4, w, ring - 0.4], [0.0, h - ring + 0.4, w, h - 4.4]]},
        {"name": "ring bottom/top", "boxes": [band(ring, 0.3), band(h - ring, 0.3)]},
        {"name": "column x=100", "boxes": [[WEST_X[3] - 0.3, ring, WEST_X[3] + 0.3, h - ring]]},
        {"name": "column copy", "boxes": [[COLUMN_COPY_X - 0.3, 38.0, COLUMN_COPY_X + 0.3, 112.0]]},
        {"name": "bridges column", "boxes": column_bridges},
        {"name": "west loops", "boxes": [[ring + 0.5, ring + 0.5, WEST_X[3] - 1.0, h - ring - 0.5]]},
        {"name": "boulevard", "boxes": [[140.0, 65.7, w - ring, 66.3], [140.0, 73.7, w - ring, 74.3], [140.0, 83.7, w - ring, 84.3]]},
        {"name": "north serpentine", "boxes": [[WEST_X[3] + 0.5, 88.0, 196.0, h - ring - 1.0]]},
        {"name": "middle serpentine", "boxes": [[WEST_X[3] + 0.5, 14.0, 140.0, 66.0]]},
        {"name": "east road", "boxes": [[w - ring + EAST_ROAD_DX - 0.3, EAST_ROAD_Y.0, w - ring + EAST_ROAD_DX + 0.3, EAST_ROAD_Y.1]]},
        {"name": "bridges east", "boxes": [[w - ring + 0.4, 26.0, w - ring + EAST_ROAD_DX - 0.4, 114.0]]},
        {"name": "east grid", "boxes": [[140.0, ring, w - ring + 6.0, h - ring]]},
    ])
}

fn write(path: &Path, value: &Value) {
    let mut text = serde_json::to_string_pretty(value).expect("serializes");
    text.push('\n');
    std::fs::write(path, text).unwrap_or_else(|e| panic!("cannot write {}: {e}", path.display()));
}

/// The stops (station, position) that have no rack or shelf within `reach` metres:
/// a robot there could not put a load anywhere.
fn stops_without_shelving(geometry: &GeometryFile, reach: f64) -> Vec<(String, P)> {
    // the boxes of the racks and shelves (all are rectangles along the axes)
    let boxes: Vec<(f64, f64, f64, f64)> = geometry
        .shapes
        .iter()
        .filter(|s| matches!(s.kind, GeometryKind::Rack | GeometryKind::Shelf))
        .map(|s| {
            let xs = s.vertices.iter().map(|v| v.x);
            let ys = s.vertices.iter().map(|v| v.y);
            (
                xs.clone().fold(f64::INFINITY, f64::min),
                ys.clone().fold(f64::INFINITY, f64::min),
                xs.fold(f64::NEG_INFINITY, f64::max),
                ys.fold(f64::NEG_INFINITY, f64::max),
            )
        })
        .collect();
    let distance = |p: P| {
        boxes
            .iter()
            .map(|b| {
                let dx = (b.0 - p.0).max(0.0).max(p.0 - b.2);
                let dy = (b.1 - p.1).max(0.0).max(p.1 - b.3);
                dx.hypot(dy)
            })
            .fold(f64::INFINITY, f64::min)
    };
    all_pockets()
        .into_iter()
        .filter(|p| p.has_station)
        .flat_map(|p| p.stops.into_iter().map(move |s| (p.station.clone(), s)))
        .filter(|&(_, stop)| distance(stop) > reach)
        .collect()
}

fn main() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let net = build_net();
    let lost = not_strongly_connected(&net);
    // (for now only reported: the picture is wanted to look at the design)
    if !lost.is_empty() {
        println!(
            "NOT strongly connected: {} nodes with a problem",
            lost.len()
        );
    }
    let (all, wide) = (stations_served(&net, false), stations_served(&net, true));
    println!(
        "stations a small robot can serve: {} of {}; a forklift or three-wheel robot: {}",
        all.0, all.1, wide.0
    );
    let lif_json = lif(&net);
    let geometry = build_geometry();
    geometry.validate().expect("the geometry is valid");
    // every stop must have racking or shelving within reach
    // (warehouse 4: 3 m, the rack blocks of the west are 3.2 m from the cell sides)
    let far = stops_without_shelving(&geometry, 3.0);
    if !far.is_empty() {
        println!("stops with no shelving close: {far:?}");
    }

    std::fs::write(root.join("scenarios/warehouse4.toml"), scenario(&net))
        .expect("write the scenario");
    write(&root.join("scenarios/warehouse4.lif.json"), &lif_json);
    let geometry_json = serde_json::to_value(&geometry).expect("serializes");
    write(
        &root.join("scenarios/warehouse4.geometry.json"),
        &geometry_json,
    );
    write(
        &root.join("scenarios/warehouse4.groups.json"),
        &groups_json(),
    );

    // the preview: the layout and the geometry, drawn the way the exporter does
    let lif: vdma_lif::Lif = serde_json::from_value(lif_json).expect("a valid LIF");
    let graph = Graph::from_lif(&lif).expect("a valid graph");
    three_wheel_report(&graph, &net, &start_nodes(&net)[2]);
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
    let png = data.join("warehouse4_layout.png");
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

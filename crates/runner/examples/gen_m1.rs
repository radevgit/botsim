//! Generates `scenarios/m1.lif.json`, the layout of measurement scenario M1
//! (narrow two-way corridors, mixed sizes, delays).
//!
//! ```sh
//! cargo run -p botsim --example gen_m1
//! ```
//!
//! ```text
//!  y=26  outer lane  ◄──────────────────────────────────  (one-way west)
//!  y=23  inner lane  ──────┬──┬──┬──┬──┬──┬──┬──┬───────►  (one-way east)
//!  y≈20        parking     P  │  P  │     │  P  │  P     parking
//!                             │     │     │     │
//!                     4 corridors, two-way, 20 m long, 6 m apart
//!                             │     │     │     │
//!  y≈6         parking     D  │  D  │     │  D  │  D     parking
//!  y=3   inner lane  ◄─────┴──┴──┴──┴──┴──┴──┴──┴────────  (one-way west)
//!  y=0   outer lane  ──────────────────────────────────►  (one-way east)
//! ```
//!
//! - **Columns** every 3 m from x = -1 to 41; both lanes of both aisles have a node
//!   in every column.
//! - **Aisles**: two one-way lanes in opposite directions, joined by one-way
//!   crossovers at both ends (a ring) and at the station columns, in turns, so that
//!   a robot can turn back without driving to the end.
//! - **Corridors** at x = 11, 17, 23, 29: one edge each way between the inner lanes.
//!   A robot holds both end nodes while it drives through, so one robot at a time.
//! - **Stations** at x = 8, 14, 26, 32, pick on top, drop at the bottom: one-way
//!   sidings, 3.25 m off the inner lane, entered from the node in the station's
//!   column and left to the next node downstream. (A dead-end spur lets a robot at
//!   the station and one on the lane node block each other for good.)
//! - **Parking**: 12 dead-end lay-bys of 3.25 m, three at each end of each aisle.
//! - **Why 3.25 m**: at 3 m a forklift on a station's entry node was within the
//!   0.10 m clearance of one leaving the station (neither could go on), and a
//!   parked forklift within it of one leaving the station next to it.
//!
//! Every node and edge may be used by all three robot types.

use std::collections::HashMap;
use std::path::Path;

use serde_json::{Value, json};

const MAP_ID: &str = "M";
/// The x of column 0 and the distance between columns, m.
const X0: f64 = -1.0;
const STEP: f64 = 3.0;
const COLUMNS: usize = 15;
const CORRIDORS: [usize; 4] = [4, 6, 8, 10];
const STATIONS: [usize; 4] = [3, 5, 9, 11];
const PARKING: [usize; 6] = [0, 1, 2, 12, 13, 14];
/// `(manufacturer, series)` of the fleet: carrier, forklift, small robot.
const TYPES: [(&str, &str); 3] = [("ACME", "X1"), ("ACME", "Y2"), ("ACME", "C5")];

/// One of the two aisles.
struct Aisle {
    name: &'static str,
    /// y of the outer lane, the inner lane, and the stations and parking lay-bys.
    outer: f64,
    inner: f64,
    spur: f64,
    /// The inner lane runs east (the outer one the other way).
    inner_east: bool,
    /// The columns of the crossovers that lead from the inner to the outer lane,
    /// and of those that lead back.
    out: [usize; 3],
    back: [usize; 3],
    /// What the stations of this aisle are, and how their ids start.
    station: &'static str,
    letter: char,
}

/// The two aisles are the same turned by half a turn.
const AISLES: [Aisle; 2] = [
    Aisle {
        name: "B",
        outer: 0.0,
        inner: 3.0,
        spur: 6.25,
        inner_east: false,
        out: [0, 3, 9],
        back: [5, 11, 14],
        station: "drop",
        letter: 'D',
    },
    Aisle {
        name: "T",
        outer: 26.0,
        inner: 23.0,
        spur: 19.75,
        inner_east: true,
        out: [14, 11, 5],
        back: [9, 3, 0],
        station: "pick",
        letter: 'P',
    },
];

#[derive(Default)]
struct Layout {
    /// `(id, x, y)`.
    nodes: Vec<(String, f64, f64)>,
    /// `(start, end)` node ids.
    edges: Vec<(String, String)>,
    /// `(id, type, node)`.
    stations: Vec<(String, &'static str, String)>,
}

impl Layout {
    fn node(&mut self, id: &str, column: usize, y: f64) {
        let x = X0 + column as f64 * STEP;
        self.nodes.push((id.to_string(), x, y));
    }

    fn one_way(&mut self, a: &str, b: &str) {
        self.edges.push((a.to_string(), b.to_string()));
    }

    fn two_way(&mut self, a: &str, b: &str) {
        self.one_way(a, b);
        self.one_way(b, a);
    }
}

/// A node of a lane: aisle, `I`nner or `O`uter, column.
fn lane(aisle: &str, side: char, column: usize) -> String {
    format!("{aisle}{side}{column:02}")
}

fn layout() -> Layout {
    let mut l = Layout::default();
    for a in &AISLES {
        for c in 0..COLUMNS {
            l.node(&lane(a.name, 'I', c), c, a.inner);
            l.node(&lane(a.name, 'O', c), c, a.outer);
        }
        for c in 0..COLUMNS - 1 {
            for (side, east) in [('I', a.inner_east), ('O', !a.inner_east)] {
                let (west, other) = (lane(a.name, side, c), lane(a.name, side, c + 1));
                if east {
                    l.one_way(&west, &other);
                } else {
                    l.one_way(&other, &west);
                }
            }
        }
        for c in a.out {
            l.one_way(&lane(a.name, 'I', c), &lane(a.name, 'O', c));
        }
        for c in a.back {
            l.one_way(&lane(a.name, 'O', c), &lane(a.name, 'I', c));
        }
        // stations: P1 .. P4 on top, D1 .. D4 at the bottom, west to east
        for (k, c) in STATIONS.into_iter().enumerate() {
            let id = format!("{}{}", a.letter, k + 1);
            l.node(&id, c, a.spur);
            let downstream = if a.inner_east { c + 1 } else { c - 1 };
            l.one_way(&lane(a.name, 'I', c), &id);
            l.one_way(&id, &lane(a.name, 'I', downstream));
            l.stations.push((id.clone(), a.station, id));
        }
        for (k, c) in PARKING.into_iter().enumerate() {
            let id = format!("K{}{}", a.name, k + 1);
            l.node(&id, c, a.spur);
            l.two_way(&lane(a.name, 'I', c), &id);
            l.stations.push((id.clone(), "parking", id));
        }
    }
    for c in CORRIDORS {
        l.two_way(&lane("B", 'I', c), &lane("T", 'I', c));
    }
    l
}

/// Every node can be reached from every other, along the edges.
fn is_strongly_connected(l: &Layout) -> bool {
    let reach = |forward: bool| {
        let mut next: HashMap<&str, Vec<&str>> = HashMap::new();
        for (a, b) in &l.edges {
            let (from, to) = if forward { (a, b) } else { (b, a) };
            next.entry(from.as_str()).or_default().push(to.as_str());
        }
        let start = l.nodes[0].0.as_str();
        let mut seen = vec![start];
        let mut todo = vec![start];
        while let Some(n) = todo.pop() {
            for &m in next.get(n).into_iter().flatten() {
                if !seen.contains(&m) {
                    seen.push(m);
                    todo.push(m);
                }
            }
        }
        seen.len()
    };
    reach(true) == l.nodes.len() && reach(false) == l.nodes.len()
}

fn lif(l: &Layout) -> Value {
    let types: Vec<Value> = TYPES
        .iter()
        .map(|(m, s)| json!({"manufacturer": m, "seriesName": s}))
        .collect();
    let nodes: Vec<Value> = l
        .nodes
        .iter()
        .map(|(id, x, y)| {
            json!({
                "nodeId": id,
                "mapId": MAP_ID,
                "nodePosition": {"x": x, "y": y},
                "mobileRobotTypeNodeProperties": [{"mobileRobotTypes": types}],
            })
        })
        .collect();
    let edges: Vec<Value> = l
        .edges
        .iter()
        .map(|(a, b)| {
            json!({
                "edgeId": format!("{a}-{b}"),
                "startNodeId": a,
                "endNodeId": b,
                "mobileRobotTypeEdgeProperties": [{
                    "mobileRobotTypes": types,
                    "orientationType": "TANGENTIAL",
                    "mobileRobotOrientations": [0.0],
                }],
            })
        })
        .collect();
    let stations: Vec<Value> = l
        .stations
        .iter()
        .map(|(id, kind, node)| {
            json!({"stationId": id, "stationType": kind, "interactionNodeIds": [node]})
        })
        .collect();
    json!({
        "metaInformation": {
            "lifId": "m1",
            "creator": "radevgit",
            "exportTimestamp": "2026-11-25T10:00:00.000Z",
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let l = layout();
    if !is_strongly_connected(&l) {
        return Err("not strongly connected".into());
    }
    let mut text = serde_json::to_string_pretty(&lif(&l))?;
    text.push('\n');
    let file = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/m1.lif.json");
    std::fs::write(&file, text)?;
    println!(
        "{} nodes, {} edges, {} stations",
        l.nodes.len(),
        l.edges.len(),
        l.stations.len()
    );
    Ok(())
}

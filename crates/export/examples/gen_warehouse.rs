//! Generates `scenarios/warehouse.lif.json` and `scenarios/warehouse.geometry.json`
//! (the bigger example, see `scenarios/warehouse.toml`).
//!
//! ```sh
//! cargo run -p botsim-export --example gen_warehouse
//! ```
//!
//! A one-way "Manhattan" grid of aisles, `COLS` × `ROWS` nodes, `SPACING` metres
//! apart: rows run east / west in turns (the bottom row east), columns run south /
//! north in turns (the left column south), so the outer aisles form a ring and
//! every node can be reached from every other (the example checks it). Goods-in
//! bays (`W1` ..) and goods-out bays (`E1` ..) are one-way sidings outside the
//! ring, so a robot that waits in a bay does not block the ring. (A dead-end spur
//! would let two robots meet head on: one in the bay wants the ring node, the one
//! on the ring node wants the bay.) Rack and shelf blocks fill the cells between
//! the aisles.

use std::collections::HashMap;
use std::path::Path;

use botsim_record::{GEOMETRY_VERSION, GeometryFile, GeometryKind, GeometryShape, Outline, Vertex};
use serde_json::{Value, json};

/// Both even, so that the outer aisles form a ring.
const COLS: usize = 8;
const ROWS: usize = 6;
const SPACING: f64 = 8.0;
/// The rows that have a goods-in bay (west) and a goods-out bay (east).
const BAYS: [usize; 4] = [1, 2, 3, 4];
/// How far a bay is from the ring, metres.
const SPUR: f64 = 8.0;
/// Space between an aisle and a rack block, metres.
const INSET: f64 = 1.6;
const MAP_ID: &str = "M";

fn node_id(r: usize, c: usize) -> String {
    format!("R{r}C{c}")
}

struct Node {
    id: String,
    x: f64,
    y: f64,
}

/// `(start, end)` node ids.
type Edge = (String, String);

fn layout() -> (Vec<Node>, Vec<Edge>) {
    let mut nodes = Vec::new();
    let mut edges: Vec<Edge> = Vec::new();
    for r in 0..ROWS {
        for c in 0..COLS {
            nodes.push(Node {
                id: node_id(r, c),
                x: c as f64 * SPACING,
                y: r as f64 * SPACING,
            });
        }
    }
    // rows: the bottom row east, then alternating
    for r in 0..ROWS {
        for c in 0..COLS - 1 {
            let (a, b) = (node_id(r, c), node_id(r, c + 1));
            edges.push(if r % 2 == 0 { (a, b) } else { (b, a) });
        }
    }
    // columns: the left column south, then alternating (a is below b)
    for c in 0..COLS {
        for r in 0..ROWS - 1 {
            let (a, b) = (node_id(r, c), node_id(r + 1, c));
            edges.push(if c % 2 == 0 { (b, a) } else { (a, b) });
        }
    }
    // bays on one-way sidings: leave the ring at row r, rejoin it at the next row
    // in the direction of the ring (west: the left column runs south, so one row
    // lower; east: the right column runs north, so one row higher)
    for r in BAYS {
        let y = r as f64 * SPACING;
        let sides = [
            ("W", 0, -SPUR, r - 1),
            ("E", COLS - 1, (COLS - 1) as f64 * SPACING + SPUR, r + 1),
        ];
        for (name, c, x, rejoin) in sides {
            let bay = format!("{name}{r}");
            nodes.push(Node {
                id: bay.clone(),
                x,
                y,
            });
            edges.push((node_id(r, c), bay.clone()));
            edges.push((bay, node_id(rejoin, c)));
        }
    }
    (nodes, edges)
}

/// Every node can be reached from every other, along the edges.
fn is_strongly_connected(nodes: &[Node], edges: &[Edge]) -> bool {
    let reach = |forward: bool| {
        let mut next: HashMap<&str, Vec<&str>> = HashMap::new();
        for (a, b) in edges {
            let (from, to) = if forward { (a, b) } else { (b, a) };
            next.entry(from.as_str()).or_default().push(to.as_str());
        }
        let start = nodes[0].id.as_str();
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
    reach(true) == nodes.len() && reach(false) == nodes.len()
}

fn lif(nodes: &[Node], edges: &[Edge]) -> Value {
    let robot_type = json!({"manufacturer": "ACME", "seriesName": "X1"});
    let nodes: Vec<Value> = nodes
        .iter()
        .map(|n| {
            json!({
                "nodeId": n.id,
                "mapId": MAP_ID,
                "nodePosition": {"x": n.x, "y": n.y},
                "mobileRobotTypeNodeProperties": [{"mobileRobotTypes": [robot_type]}],
            })
        })
        .collect();
    let edges: Vec<Value> = edges
        .iter()
        .map(|(a, b)| {
            json!({
                "edgeId": format!("{a}-{b}"),
                "startNodeId": a,
                "endNodeId": b,
                "mobileRobotTypeEdgeProperties": [{
                    "mobileRobotTypes": [robot_type],
                    "orientationType": "TANGENTIAL",
                    "mobileRobotOrientations": [0.0],
                }],
            })
        })
        .collect();
    json!({
        "metaInformation": {
            "lifId": "warehouse",
            "creator": "radevgit",
            "exportTimestamp": "2026-11-21T10:00:00.000Z",
            "lifVersion": "2.0.0",
        },
        "origins": [{
            "originId": "O1",
            "layouts": [{
                "layoutId": "L1",
                "layoutVersion": "1",
                "nodes": nodes,
                "edges": edges,
            }],
        }],
    })
}

/// Millimetres, so that 8.0 - 1.6 is 6.4 and not 6.3999999999999995.
fn mm(v: f64) -> f64 {
    (v * 1000.0).round() / 1000.0
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Vertex> {
    [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        .map(|(x, y)| Vertex { x: mm(x), y: mm(y) })
        .to_vec()
}

fn shape(id: String, kind: GeometryKind, outline: Outline, vertices: Vec<Vertex>) -> GeometryShape {
    GeometryShape {
        id: Some(id),
        map_id: MAP_ID.into(),
        kind,
        shape: outline,
        vertices,
    }
}

fn geometry() -> GeometryFile {
    let east_x = (COLS - 1) as f64 * SPACING + SPUR;
    let mut shapes = Vec::new();

    // the outer wall
    let (x0, y0) = (-SPUR - 5.0, -5.0);
    let (x1, y1) = (east_x + 5.0, (ROWS - 1) as f64 * SPACING + 5.0);
    let corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)];
    let wall = corners.map(|(x, y)| Vertex { x: mm(x), y: mm(y) });
    shapes.push(shape(
        "wall".into(),
        GeometryKind::Wall,
        Outline::Polyline,
        wall.to_vec(),
    ));

    // the bays
    for r in BAYS {
        let y = r as f64 * SPACING;
        for (name, x) in [("W", -SPUR), ("E", east_x)] {
            shapes.push(shape(
                format!("bay-{name}{r}"),
                GeometryKind::Station,
                Outline::Polygon,
                rect(x - 1.6, y - 1.6, x + 1.6, y + 1.6),
            ));
        }
    }

    // rack and shelf blocks in the cells between the aisles; the two middle
    // columns of cells are shelves
    for r in 0..ROWS - 1 {
        for c in 0..COLS - 1 {
            let middle = (COLS / 2 - 1..=COLS / 2).contains(&c);
            let (kind, name) = if middle {
                (GeometryKind::Shelf, "shelf")
            } else {
                (GeometryKind::Rack, "rack")
            };
            shapes.push(shape(
                format!("{name}-{r}-{c}"),
                kind,
                Outline::Polygon,
                rect(
                    c as f64 * SPACING + INSET,
                    r as f64 * SPACING + INSET,
                    (c + 1) as f64 * SPACING - INSET,
                    (r + 1) as f64 * SPACING - INSET,
                ),
            ));
        }
    }
    let file = GeometryFile {
        version: GEOMETRY_VERSION,
        shapes,
    };
    file.validate().expect("the generated geometry is valid");
    file
}

fn write(dir: &Path, name: &str, value: &Value) {
    let mut text = serde_json::to_string_pretty(value).expect("serializes");
    text.push('\n');
    std::fs::write(dir.join(name), text).unwrap_or_else(|e| panic!("cannot write {name}: {e}"));
}

fn main() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    let (nodes, edges) = layout();
    assert!(
        is_strongly_connected(&nodes, &edges),
        "not strongly connected"
    );
    let geometry = geometry();
    write(&dir, "warehouse.lif.json", &lif(&nodes, &edges));
    let geometry_json = serde_json::to_value(&geometry).expect("serializes");
    write(&dir, "warehouse.geometry.json", &geometry_json);
    println!(
        "{} nodes, {} edges, {} shapes",
        nodes.len(),
        edges.len(),
        geometry.shapes.len()
    );
}

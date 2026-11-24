//! Check: stops with no shelving close (`docs/graph_problems.md`, G9).
//!
//! An interaction node of a station of type `rack` or `shelf` must have a rack or a
//! shelf of the geometry file within [`REACH`] metres (on the same map). Other station
//! types (bays, parking) need no shelving.

use std::collections::HashMap;

use botsim_core::graph::Graph;
use botsim_record::{GeometryFile, GeometryKind, Outline};
use vdma_lif::{Layout, Lif};

use crate::finding::{Finding, Level};
use crate::places::LayoutIndex;

const CHECK: &str = "shelving";

/// How close shelving must be to an interaction node (m).
pub const REACH: f64 = 2.5;

type P = (f64, f64);

/// A station node with no shelving within reach.
struct Far {
    station: String,
    node: String,
    /// Distance to the nearest rack or shelf on the map (m); `None` if there is none.
    nearest: Option<f64>,
}

pub fn check(lif: &Lif, geometry: &GeometryFile) -> Vec<Finding> {
    // A layout that cannot be turned into a graph is reported by the reachability check.
    let Ok(graph) = Graph::from_lif(lif) else {
        return Vec::new();
    };
    // racks and shelves by map name
    let mut shelving: HashMap<&str, Vec<Vec<P>>> = HashMap::new();
    for s in &geometry.shapes {
        if matches!(s.kind, GeometryKind::Rack | GeometryKind::Shelf) {
            let verts: Vec<P> = s.vertices.iter().map(|v| (v.x, v.y)).collect();
            let closed = s.shape == Outline::Polygon;
            shelving
                .entry(&s.map_id)
                .or_default()
                .push(if closed { close(verts) } else { verts });
        }
    }
    let mut out = Vec::new();
    for layout in lif.origins.iter().flat_map(|o| &o.layouts) {
        let mut far: Vec<Far> = Vec::new();
        for s in layout.stations.iter().flatten() {
            let ty = s.station_type.as_deref().unwrap_or("").to_ascii_lowercase();
            if ty != "rack" && ty != "shelf" {
                continue;
            }
            for id in &s.interaction_node_ids {
                let Some(n) = graph.node_idx(id) else {
                    continue;
                };
                let node = &graph.nodes[n];
                let map = &graph.maps[node.map];
                let nearest = shelving
                    .get(map.as_str())
                    .map(|list| {
                        list.iter()
                            .map(|poly| distance((node.x, node.y), poly))
                            .fold(f64::INFINITY, f64::min)
                    })
                    .filter(|d| d.is_finite());
                if nearest.is_none_or(|d| d > REACH) {
                    far.push(Far {
                        station: s.station_id.clone(),
                        node: id.clone(),
                        nearest,
                    });
                }
            }
        }
        report_layout(layout, &far, &mut out);
    }
    out
}

/// The polygon as a closed ring (first vertex repeated).
fn close(mut v: Vec<P>) -> Vec<P> {
    if let Some(&first) = v.first() {
        v.push(first);
    }
    v
}

/// Distance from the point to the polygon or polyline (0 inside a closed ring).
fn distance(p: P, poly: &[P]) -> f64 {
    let closed = poly.len() > 3 && poly.first() == poly.last();
    if closed && inside(poly, p) {
        return 0.0;
    }
    poly.windows(2)
        .map(|w| dist_point_segment(p, w[0], w[1]))
        .fold(f64::INFINITY, f64::min)
}

fn inside(poly: &[P], p: P) -> bool {
    let mut in_poly = false;
    for w in poly.windows(2) {
        let (a, b) = (w[0], w[1]);
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

fn report_layout(layout: &Layout, far: &[Far], out: &mut Vec<Finding>) {
    if far.is_empty() {
        return;
    }
    let mut stations: Vec<&str> = Vec::new();
    for f in far {
        if !stations.contains(&f.station.as_str()) {
            stations.push(&f.station);
        }
    }
    let first = &far[0];
    let nearest = match first.nearest {
        Some(d) => format!("the nearest rack or shelf is {d:.1} m away"),
        None => "there is no rack or shelf on its map".to_string(),
    };
    let idx = LayoutIndex::new(layout);
    let mut places = idx.nodes(far.iter().map(|f| f.node.as_str()));
    places.extend(idx.stations(stations.iter().copied()));
    let mut ids: Vec<&str> = far.iter().map(|f| f.node.as_str()).collect();
    ids.extend(stations);
    out.push(
        Finding::new(
            Level::Warning,
            CHECK,
            format!(
                "{} interaction node(s) of rack or shelf stations have no rack or shelf within {REACH} m; first: node {} of station {}: {nearest}",
                far.len(),
                first.node,
                first.station
            ),
        )
        .layout(&layout.layout_id)
        .with_places(places)
        .with_ids(ids),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lif(stations: &str) -> Lif {
        let ty = r#"{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}"#;
        serde_json::from_str(&format!(
            r#"{{"metaInformation":{{"lifId":"x","creator":"t","exportTimestamp":"2026-01-01T00:00:00Z","lifVersion":"1.0.0"}},
            "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
            "nodes":[{{"nodeId":"A","mapId":"M","nodePosition":{{"x":0,"y":0}},"mobileRobotTypeNodeProperties":[{ty}]}},
                     {{"nodeId":"B","mapId":"M","nodePosition":{{"x":10,"y":0}},"mobileRobotTypeNodeProperties":[{ty}]}}],
            "edges":[{{"edgeId":"E1","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[{ty}]}}],
            "stations":[{stations}]}}]}}]}}"#
        ))
        .expect("test LIF parses")
    }

    fn geometry() -> GeometryFile {
        // a rack from (8,1) to (9,2): 2.2 m from node B (10,0)... distance from B is hypot(1,1) = 1.41
        GeometryFile::from_json_str(
            r#"{"version":1,"shapes":[{"id":"R","mapId":"M","kind":"rack","shape":"polygon",
            "vertices":[{"x":8.0,"y":1.0},{"x":9.0,"y":1.0},{"x":9.0,"y":2.0},{"x":8.0,"y":2.0}]}]}"#,
        )
        .expect("geometry parses")
    }

    #[test]
    fn stops_far_from_shelving() {
        // station S1 (rack) at node B: rack 1.4 m away: fine. S2 (rack) at node A: 8 m: far.
        // S3 (bay) at node A: no shelving needed.
        let l = lif(
            r#"{"stationId":"S1","stationType":"rack","interactionNodeIds":["B"]},
               {"stationId":"S2","stationType":"rack","interactionNodeIds":["A"]},
               {"stationId":"S3","stationType":"bay","interactionNodeIds":["A"]}"#,
        );
        let f = check(&l, &geometry());
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].level, Level::Warning);
        assert_eq!(f[0].ids, ["A", "S2"]);
        assert!(f[0].text.contains("node A of station S2"), "{}", f[0].text);
        assert!(f[0].text.contains("is 8.1 m away"), "{}", f[0].text);
        assert_eq!(f[0].places.len(), 2);
    }

    #[test]
    fn no_shelving_on_the_map_and_no_stations() {
        let l = lif(r#"{"stationId":"S2","stationType":"shelf","interactionNodeIds":["A"]}"#);
        let empty = GeometryFile::from_json_str(r#"{"version":1,"shapes":[]}"#).expect("parses");
        let f = check(&l, &empty);
        assert_eq!(f.len(), 1);
        assert!(f[0].text.contains("no rack or shelf on its map"), "{}", f[0].text);
        assert!(check(&lif(""), &geometry()).is_empty());
    }
}

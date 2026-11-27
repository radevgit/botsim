//! Line 1 of a recording: the layout.

use botsim_model::graph::Graph;
use serde::{Deserialize, Serialize};

use crate::{FORMAT_VERSION, GeometryError, GeometryFile, StaticShape};

/// Line 1: maps, nodes, edges (with their curves), the robots' types and the
/// static geometry (walls etc.).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename = "layout")]
pub struct LayoutLine {
    pub version: u32,
    /// Map ids (VDA 5050 `mapId`); a map is referred to by its position.
    pub maps: Vec<String>,
    pub nodes: Vec<NodeRecord>,
    pub edges: Vec<EdgeRecord>,
    /// Static geometry (walls, racks, ...), drawn under the track graph in
    /// this order; empty if there is none.
    #[serde(default)]
    pub geometry: Vec<StaticShape>,
    /// The type of each robot, to pick the curve a robot follows on an edge
    /// ([`EdgeRecord::paths`]); empty if not recorded.
    #[serde(default)]
    pub robots: Vec<LayoutRobot>,
}

/// A robot and its type.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LayoutRobot {
    /// `"manufacturer/serialNumber"`, as in [`crate::RobotRecord::id`].
    pub id: String,
    /// `"manufacturer/seriesName"`, as in [`EdgePath::types`].
    #[serde(rename = "type")]
    pub robot_type: String,
}

/// The curve that robots of some types follow on an edge.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EdgePath {
    /// The robot types, each `"manufacturer/seriesName"`.
    pub types: Vec<String>,
    /// Points `[x, y]` (metres, in the map of the edge's nodes) from the start
    /// node to the end node, joined by straight pieces. Two points: a straight
    /// line.
    pub points: Vec<[f64; 2]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NodeRecord {
    pub id: String,
    /// Index into [`LayoutLine::maps`].
    pub map: usize,
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EdgeRecord {
    pub id: String,
    /// Index into [`LayoutLine::nodes`].
    pub start: usize,
    pub end: usize,
    pub length: f64,
    pub map_transition: bool,
    /// The curves of the edge, one per group of robot types that follow the same
    /// one. Empty: every type drives the straight line from the start node to the
    /// end node. If some type has a curve, all types of the edge are listed (those
    /// without a curve with their straight line).
    #[serde(default)]
    pub paths: Vec<EdgePath>,
}

/// Points of a curve at most this far apart (metres).
const SAMPLE_STEP: f64 = 0.2;

/// The curves of one edge, as the recording keeps them.
fn edge_paths(graph: &Graph, index: usize) -> Vec<EdgePath> {
    let edge = &graph.edges[index];
    if edge.map_transition || edge.types.iter().all(|t| t.trajectory.is_none()) {
        return Vec::new();
    }
    edge.types
        .iter()
        .filter_map(|props| {
            let (manufacturer, series) = props.robot_types.first()?;
            let path = graph.edge_path(index, manufacturer, series);
            // a straight line needs its two ends only
            let steps = if path.is_straight() {
                1
            } else {
                (path.length() / SAMPLE_STEP).ceil().max(1.0) as usize
            };
            let points = (0..=steps)
                .map(|i| {
                    let (x, y) = path.point_at(path.length() * i as f64 / steps as f64);
                    [x, y]
                })
                .collect();
            let types = props
                .robot_types
                .iter()
                .map(|(m, s)| format!("{m}/{s}"))
                .collect();
            Some(EdgePath { types, points })
        })
        .collect()
}

impl LayoutLine {
    /// The layout line for a graph (current format version).
    pub fn from_graph(graph: &Graph) -> LayoutLine {
        LayoutLine {
            version: FORMAT_VERSION,
            maps: graph.maps.clone(),
            nodes: graph
                .nodes
                .iter()
                .map(|n| NodeRecord {
                    id: n.id.clone(),
                    map: n.map,
                    x: n.x,
                    y: n.y,
                })
                .collect(),
            edges: graph
                .edges
                .iter()
                .enumerate()
                .map(|(i, e)| EdgeRecord {
                    id: e.id.clone(),
                    start: e.start,
                    end: e.end,
                    length: e.length,
                    map_transition: e.map_transition,
                    paths: edge_paths(graph, i),
                })
                .collect(),
            geometry: Vec::new(),
            robots: Vec::new(),
        }
    }

    /// The same line with the types of the robots.
    pub fn with_robots(mut self, robots: Vec<LayoutRobot>) -> LayoutLine {
        self.robots = robots;
        self
    }

    /// The same line with the static geometry of a file; fails if a shape is on
    /// a map that the layout does not have.
    pub fn with_geometry(mut self, file: &GeometryFile) -> Result<LayoutLine, GeometryError> {
        self.geometry = file.resolve(&self.maps)?;
        Ok(self)
    }
}

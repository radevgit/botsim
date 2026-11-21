//! Line 1 of a recording: the layout.

use botsim_core::graph::Graph;
use serde::{Deserialize, Serialize};

use crate::{FORMAT_VERSION, GeometryError, GeometryFile, StaticShape};

/// Line 1: maps, nodes and edges. Static geometry (walls etc.) will be added as
/// a `geometry` field later.
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
                .map(|e| EdgeRecord {
                    id: e.id.clone(),
                    start: e.start,
                    end: e.end,
                    length: e.length,
                    map_transition: e.map_transition,
                })
                .collect(),
            geometry: Vec::new(),
        }
    }

    /// The same line with the static geometry of a file; fails if a shape is on
    /// a map that the layout does not have.
    pub fn with_geometry(mut self, file: &GeometryFile) -> Result<LayoutLine, GeometryError> {
        self.geometry = file.resolve(&self.maps)?;
        Ok(self)
    }
}

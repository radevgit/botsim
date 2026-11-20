//! Track graph built from one LIF layout: nodes with positions and map (floor),
//! directed straight edges with lengths. Geometry only; per-robot-type edge rules
//! (speed, orientation, rotation) are added in later steps.

use std::collections::HashMap;
use std::fmt;

use vdma_lif::{Layout, Lif};

/// Index of a node in [`Graph::nodes`].
pub type NodeIdx = usize;
/// Index of an edge in [`Graph::edges`].
pub type EdgeIdx = usize;
/// Index of a map (floor) in [`Graph::maps`]; the VDA 5050 `mapId` as a small number.
pub type MapIdx = usize;

#[derive(Debug, Clone, PartialEq)]
pub struct GraphNode {
    pub id: String,
    /// The map (floor) this node is on. x and y are in that map's coordinates.
    pub map: MapIdx,
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct GraphEdge {
    pub id: String,
    pub start: NodeIdx,
    pub end: NodeIdx,
    /// Straight-line length in metres. 0.0 for a map transition (see below).
    pub length: f64,
    /// Start and end node are on different maps (e.g. a lift ride). Coordinates of
    /// different maps cannot be compared, so there is no length; how such an
    /// edge is travelled comes with the elevator peripheral.
    pub map_transition: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum GraphError {
    DuplicateNode(String),
    DuplicateEdge(String),
    UnknownNode { edge: String, node: String },
}

impl fmt::Display for GraphError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GraphError::DuplicateNode(id) => write!(f, "duplicate nodeId '{id}'"),
            GraphError::DuplicateEdge(id) => write!(f, "duplicate edgeId '{id}'"),
            GraphError::UnknownNode { edge, node } => {
                write!(f, "edge '{edge}' refers to unknown node '{node}'")
            }
        }
    }
}

impl std::error::Error for GraphError {}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Graph {
    pub nodes: Vec<GraphNode>,
    pub edges: Vec<GraphEdge>,
    /// Map ids (VDA 5050 `mapId`), in order of first appearance.
    pub maps: Vec<String>,
    map_by_id: HashMap<String, MapIdx>,
    node_by_id: HashMap<String, NodeIdx>,
    edge_by_id: HashMap<String, EdgeIdx>,
    /// Outgoing edges per node, in layout order.
    out_edges: Vec<Vec<EdgeIdx>>,
}

impl Graph {
    /// Build the graph from a single LIF layout.
    pub fn from_layout(layout: &Layout) -> Result<Graph, GraphError> {
        Graph::from_layouts(&[layout])
    }

    /// Build one graph from all layouts of all origins of a LIF file (e.g. one
    /// layout per level). Node and edge ids must be unique across the file; an
    /// edge may end in another layout (a lift ride), it then becomes a map
    /// transition if the two nodes are on different maps.
    pub fn from_lif(lif: &Lif) -> Result<Graph, GraphError> {
        let layouts: Vec<&Layout> = lif.origins.iter().flat_map(|o| &o.layouts).collect();
        Graph::from_layouts(&layouts)
    }

    /// Build one graph from several layouts: all nodes first, then all edges,
    /// so that edges can refer to nodes of other layouts.
    pub fn from_layouts(layouts: &[&Layout]) -> Result<Graph, GraphError> {
        let mut g = Graph::default();

        for n in layouts.iter().flat_map(|l| &l.nodes) {
            let idx = g.nodes.len();
            if g.node_by_id.insert(n.node_id.clone(), idx).is_some() {
                return Err(GraphError::DuplicateNode(n.node_id.clone()));
            }
            let map = match g.map_by_id.get(&n.map_id) {
                Some(&m) => m,
                None => {
                    g.maps.push(n.map_id.clone());
                    g.map_by_id.insert(n.map_id.clone(), g.maps.len() - 1);
                    g.maps.len() - 1
                }
            };
            g.nodes.push(GraphNode {
                id: n.node_id.clone(),
                map,
                x: n.node_position.x,
                y: n.node_position.y,
            });
            g.out_edges.push(Vec::new());
        }

        for e in layouts.iter().flat_map(|l| &l.edges) {
            let lookup = |node: &String| {
                g.node_by_id
                    .get(node)
                    .copied()
                    .ok_or_else(|| GraphError::UnknownNode {
                        edge: e.edge_id.clone(),
                        node: node.clone(),
                    })
            };
            let start = lookup(&e.start_node_id)?;
            let end = lookup(&e.end_node_id)?;

            let idx = g.edges.len();
            if g.edge_by_id.insert(e.edge_id.clone(), idx).is_some() {
                return Err(GraphError::DuplicateEdge(e.edge_id.clone()));
            }
            let (a, b) = (&g.nodes[start], &g.nodes[end]);
            let map_transition = a.map != b.map;
            let length = if map_transition {
                0.0
            } else {
                (b.x - a.x).hypot(b.y - a.y)
            };
            g.edges.push(GraphEdge {
                id: e.edge_id.clone(),
                start,
                end,
                length,
                map_transition,
            });
            g.out_edges[start].push(idx);
        }

        Ok(g)
    }

    pub fn node_idx(&self, id: &str) -> Option<NodeIdx> {
        self.node_by_id.get(id).copied()
    }

    pub fn map_idx(&self, id: &str) -> Option<MapIdx> {
        self.map_by_id.get(id).copied()
    }

    pub fn edge_idx(&self, id: &str) -> Option<EdgeIdx> {
        self.edge_by_id.get(id).copied()
    }

    pub fn out_edges(&self, node: NodeIdx) -> &[EdgeIdx] {
        &self.out_edges[node]
    }
}

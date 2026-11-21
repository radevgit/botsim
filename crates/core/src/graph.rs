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

/// A robot type as the LIF names it and the factsheet states it:
/// (`manufacturer`, `seriesName`).
pub type RobotTypeKey = (String, String);

/// Is the type (`manufacturer`, `series`) in the list?
fn lists_type(list: &[RobotTypeKey], manufacturer: &str, series: &str) -> bool {
    list.iter().any(|(m, s)| m == manufacturer && s == series)
}

#[derive(Debug, Clone, PartialEq)]
pub struct GraphNode {
    pub id: String,
    /// The map (floor) this node is on. x and y are in that map's coordinates.
    pub map: MapIdx,
    pub x: f64,
    pub y: f64,
    /// The robot types that may use the node (LIF `mobileRobotTypeNodeProperties`);
    /// a type that is not listed may not.
    pub robot_types: Vec<RobotTypeKey>,
}

/// What an edge is like for some robot types (one LIF
/// `mobileRobotTypeEdgeProperties` entry).
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeTypeProps {
    pub robot_types: Vec<RobotTypeKey>,
    /// Permitted maximum speed in m/s (> 0); no limit if `None`.
    pub maximum_speed: Option<f64>,
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
    /// The properties of the edge per robot type. Only the listed types may use
    /// the edge (LIF: "one element per allowed robot type").
    pub types: Vec<EdgeTypeProps>,
}

impl GraphEdge {
    /// The properties of this edge for a robot type; `None` if the type may not
    /// use the edge.
    pub fn props_for(&self, manufacturer: &str, series: &str) -> Option<&EdgeTypeProps> {
        self.types
            .iter()
            .find(|p| lists_type(&p.robot_types, manufacturer, series))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum GraphError {
    DuplicateNode(String),
    DuplicateEdge(String),
    UnknownNode {
        edge: String,
        node: String,
    },
    /// `maximumSpeed` must be above 0 (a robot could never drive the edge).
    BadMaximumSpeed {
        edge: String,
    },
}

impl fmt::Display for GraphError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GraphError::DuplicateNode(id) => write!(f, "duplicate nodeId '{id}'"),
            GraphError::DuplicateEdge(id) => write!(f, "duplicate edgeId '{id}'"),
            GraphError::UnknownNode { edge, node } => {
                write!(f, "edge '{edge}' refers to unknown node '{node}'")
            }
            GraphError::BadMaximumSpeed { edge } => write!(
                f,
                "edge '{edge}': maximumSpeed must be above 0 (a robot could never drive it)"
            ),
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
                robot_types: n
                    .mobile_robot_type_node_properties
                    .iter()
                    .flat_map(|p| &p.mobile_robot_types)
                    .map(|t| (t.manufacturer.clone(), t.series_name.clone()))
                    .collect(),
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
            let mut types = Vec::new();
            for p in &e.mobile_robot_type_edge_properties {
                if p.maximum_speed.is_some_and(|v| !(v.is_finite() && v > 0.0)) {
                    return Err(GraphError::BadMaximumSpeed {
                        edge: e.edge_id.clone(),
                    });
                }
                types.push(EdgeTypeProps {
                    robot_types: p
                        .mobile_robot_types
                        .iter()
                        .map(|t| (t.manufacturer.clone(), t.series_name.clone()))
                        .collect(),
                    maximum_speed: p.maximum_speed,
                });
            }
            g.edges.push(GraphEdge {
                id: e.edge_id.clone(),
                start,
                end,
                length,
                map_transition,
                types,
            });
            g.out_edges[start].push(idx);
        }

        Ok(g)
    }

    /// May a robot of this type stand on the node?
    pub fn node_allows(&self, node: NodeIdx, manufacturer: &str, series: &str) -> bool {
        lists_type(&self.nodes[node].robot_types, manufacturer, series)
    }

    /// May a robot of this type drive the edge? The edge must list the type, and
    /// the type must be allowed on both end nodes.
    pub fn edge_allows(&self, edge: EdgeIdx, manufacturer: &str, series: &str) -> bool {
        let e = &self.edges[edge];
        e.props_for(manufacturer, series).is_some()
            && self.node_allows(e.start, manufacturer, series)
            && self.node_allows(e.end, manufacturer, series)
    }

    /// The speed limit (m/s) of the edge for this type, if it has one.
    pub fn edge_max_speed(&self, edge: EdgeIdx, manufacturer: &str, series: &str) -> Option<f64> {
        self.edges[edge]
            .props_for(manufacturer, series)
            .and_then(|p| p.maximum_speed)
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

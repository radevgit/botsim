//! Track graph built from one LIF layout: nodes with positions and map (floor),
//! directed edges with lengths, and what the layout says per robot type (who may
//! use a node or edge; an edge's speed limit, orientation, rotation rules and
//! curved trajectory).

use std::collections::HashMap;
use std::fmt;

use vdma_lif::{Layout, Lif};

use crate::angle::angle_diff;
use crate::path::{Nurbs, Path};
use crate::spec::Kinematics;

/// How far (rad) a car-like robot's heading may be off the direction an edge starts
/// in and still count as aligned (S70): about 1 degree.
pub const THREE_WHEEL_TOLERANCE: f64 = 0.02;

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

/// Which ways a robot may turn on the spot at a node (LIF `rotationAtStartNodeAllowed`
/// / `rotationAtEndNodeAllowed`, S64).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rotation {
    None,
    /// Counter-clockwise (positive) only.
    Ccw,
    /// Clockwise (negative) only.
    Cw,
    Both,
}

impl Rotation {
    fn bits(self) -> u8 {
        match self {
            Rotation::None => 0,
            Rotation::Ccw => 1,
            Rotation::Cw => 2,
            Rotation::Both => 3,
        }
    }

    /// What two rules together allow: both must allow it (LIF §8.3.13.1: the
    /// rotation at the end node of the edge a robot comes in on and at the start
    /// node of the edge it leaves on "are combined as a boolean and").
    pub fn and(self, other: Rotation) -> Rotation {
        match self.bits() & other.bits() {
            0 => Rotation::None,
            1 => Rotation::Ccw,
            2 => Rotation::Cw,
            _ => Rotation::Both,
        }
    }

    /// May the robot turn counter-clockwise (`ccw`) or clockwise?
    pub fn allows(self, ccw: bool) -> bool {
        self.bits() & if ccw { 1 } else { 2 } != 0
    }
}

impl From<vdma_lif::edge::RotationAllowed> for Rotation {
    fn from(r: vdma_lif::edge::RotationAllowed) -> Rotation {
        use vdma_lif::edge::RotationAllowed as R;
        match r {
            R::None => Rotation::None,
            R::Ccw => Rotation::Ccw,
            R::Cw => Rotation::Cw,
            R::Both => Rotation::Both,
        }
    }
}

/// What an edge is like for some robot types (one LIF
/// `mobileRobotTypeEdgeProperties` entry).
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeTypeProps {
    pub robot_types: Vec<RobotTypeKey>,
    /// Permitted maximum speed in m/s (> 0); no limit if `None`.
    pub maximum_speed: Option<f64>,
    /// Rotation allowed at the start node of the edge (LIF default: both ways).
    pub rotation_at_start: Rotation,
    /// Rotation allowed at the end node of the edge (LIF default: both ways).
    pub rotation_at_end: Rotation,
    /// The curve the robot follows between the two nodes (LIF `trajectory`, S69);
    /// a straight line if `None`.
    pub trajectory: Option<Nurbs>,
    /// The orientation a robot should have on the edge: the first of the LIF's
    /// `mobileRobotOrientations`, read as `orientationType` says (default
    /// `TANGENTIAL`); `None` if the LIF lists none.
    pub orientation: Option<EdgeOrientation>,
}

/// An orientation (radians) of a robot on an edge, as the LIF gives it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EdgeOrientation {
    /// Relative to the direction of the edge (0 forwards, π backwards).
    Tangential(f64),
    /// Relative to the map.
    Global(f64),
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

/// How far (m) a trajectory may end from the node it should end at.
const TRAJECTORY_TOLERANCE: f64 = 0.05;

/// A LIF trajectory as a curve; it must start at the start node and end at the end
/// node of its edge (the control points include both ends).
fn read_trajectory(
    t: &vdma_lif::edge::Trajectory,
    edge: &str,
    start: (f64, f64),
    end: (f64, f64),
) -> Result<Nurbs, GraphError> {
    let bad = |reason: String| GraphError::BadTrajectory {
        edge: edge.to_string(),
        reason,
    };
    let points: Vec<(f64, f64, Option<f64>)> = t
        .control_points
        .iter()
        .map(|p| (p.x, p.y, p.weight))
        .collect();
    let curve = Nurbs::new(
        t.degree.unwrap_or(1) as usize,
        t.knot_vector.clone(),
        &points,
    )
    .map_err(|e| bad(e.to_string()))?;
    let (a, b) = curve.ends();
    let off = |p: (f64, f64), q: (f64, f64)| (p.0 - q.0).hypot(p.1 - q.1);
    if off(a, start) > TRAJECTORY_TOLERANCE || off(b, end) > TRAJECTORY_TOLERANCE {
        return Err(bad(
            "it does not run from the start node to the end node of the edge".to_string(),
        ));
    }
    Ok(curve)
}

/// A station (LIF): a logical place where a robot interacts with the
/// environment, reached at one of its interaction nodes (S60).
#[derive(Debug, Clone, PartialEq)]
pub struct GraphStation {
    pub id: String,
    pub name: Option<String>,
    pub station_type: Option<String>,
    /// The nodes where the interaction takes place (at least one).
    pub interaction_nodes: Vec<NodeIdx>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum GraphError {
    DuplicateNode(String),
    DuplicateEdge(String),
    UnknownNode {
        edge: String,
        node: String,
    },
    DuplicateStation(String),
    /// A station lists a node that is not in the layout, or no node at all.
    BadStation {
        station: String,
        node: Option<String>,
    },
    /// `maximumSpeed` must be above 0 (a robot could never drive the edge).
    BadMaximumSpeed {
        edge: String,
    },
    /// A trajectory is not a usable curve, or does not run from the start node to
    /// the end node of its edge.
    BadTrajectory {
        edge: String,
        reason: String,
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
            GraphError::BadTrajectory { edge, reason } => {
                write!(f, "edge '{edge}': trajectory: {reason}")
            }
            GraphError::DuplicateStation(id) => write!(f, "duplicate stationId '{id}'"),
            GraphError::BadStation {
                station,
                node: Some(n),
            } => write!(f, "station '{station}' refers to unknown node '{n}'"),
            GraphError::BadStation {
                station,
                node: None,
            } => write!(f, "station '{station}' has no interaction node"),
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
    /// The stations of all layouts, in file order (S60).
    pub stations: Vec<GraphStation>,
    station_by_id: HashMap<String, usize>,
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
                    orientation: p
                        .mobile_robot_orientations
                        .as_ref()
                        .and_then(|o| o.first().copied())
                        .map(|a| match p.orientation_type {
                            Some(vdma_lif::edge::OrientationType::Global) => {
                                EdgeOrientation::Global(a)
                            }
                            _ => EdgeOrientation::Tangential(a),
                        }),
                    rotation_at_start: p
                        .rotation_at_start_node_allowed
                        .map_or(Rotation::Both, Rotation::from),
                    rotation_at_end: p
                        .rotation_at_end_node_allowed
                        .map_or(Rotation::Both, Rotation::from),
                    trajectory: match (&p.trajectory, map_transition) {
                        (Some(t), false) => {
                            Some(read_trajectory(t, &e.edge_id, (a.x, a.y), (b.x, b.y))?)
                        }
                        _ => None,
                    },
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

        // stations come last: they refer to nodes of any layout
        for s in layouts.iter().flat_map(|l| l.stations.iter().flatten()) {
            let idx = g.stations.len();
            if g.station_by_id.insert(s.station_id.clone(), idx).is_some() {
                return Err(GraphError::DuplicateStation(s.station_id.clone()));
            }
            if s.interaction_node_ids.is_empty() {
                return Err(GraphError::BadStation {
                    station: s.station_id.clone(),
                    node: None,
                });
            }
            let mut interaction_nodes = Vec::new();
            for n in &s.interaction_node_ids {
                let node = g
                    .node_by_id
                    .get(n)
                    .copied()
                    .ok_or_else(|| GraphError::BadStation {
                        station: s.station_id.clone(),
                        node: Some(n.clone()),
                    })?;
                if !interaction_nodes.contains(&node) {
                    interaction_nodes.push(node);
                }
            }
            g.stations.push(GraphStation {
                id: s.station_id.clone(),
                name: s.station_name.clone(),
                station_type: s.station_type.clone(),
                interaction_nodes,
            });
        }

        Ok(g)
    }

    /// The index of the station with this `stationId`.
    pub fn station_idx(&self, id: &str) -> Option<usize> {
        self.station_by_id.get(id).copied()
    }

    /// May a robot of this type stand on the node?
    pub fn node_allows(&self, node: NodeIdx, manufacturer: &str, series: &str) -> bool {
        lists_type(&self.nodes[node].robot_types, manufacturer, series)
    }

    /// The orientation the LIF gives robots of this type on the edge (see
    /// [`EdgeTypeProps::orientation`]); `None` if it gives none or the type may not
    /// use the edge.
    pub fn edge_orientation(
        &self,
        edge: EdgeIdx,
        manufacturer: &str,
        series: &str,
    ) -> Option<EdgeOrientation> {
        self.edges[edge]
            .props_for(manufacturer, series)?
            .orientation
    }

    /// May a robot of this type drive the edge? The edge must list the type, and
    /// the type must be allowed on both end nodes.
    pub fn edge_allows(&self, edge: EdgeIdx, manufacturer: &str, series: &str) -> bool {
        let e = &self.edges[edge];
        e.props_for(manufacturer, series).is_some()
            && self.node_allows(e.start, manufacturer, series)
            && self.node_allows(e.end, manufacturer, series)
    }

    /// The rotation this type may do at the start and at the end node of the edge
    /// (both ways if the layout says nothing, or the type may not use the edge).
    pub fn edge_rotation(
        &self,
        edge: EdgeIdx,
        manufacturer: &str,
        series: &str,
    ) -> (Rotation, Rotation) {
        self.edges[edge]
            .props_for(manufacturer, series)
            .map_or((Rotation::Both, Rotation::Both), |p| {
                (p.rotation_at_start, p.rotation_at_end)
            })
    }

    /// The path a robot of this type follows on the edge: its trajectory sampled,
    /// or the straight line between the nodes (S69). Lift rides have none.
    pub fn edge_path(&self, edge: EdgeIdx, manufacturer: &str, series: &str) -> Path {
        let e = &self.edges[edge];
        match e
            .props_for(manufacturer, series)
            .and_then(|p| p.trajectory.as_ref())
        {
            Some(curve) => curve.to_path(),
            None => {
                let (a, b) = (&self.nodes[e.start], &self.nodes[e.end]);
                Path::straight((a.x, a.y), (b.x, b.y))
            }
        }
    }

    /// The direction the edge starts in and the direction it ends in (radians), for
    /// this type: from the curve's control points, or the straight line. Cheap (no
    /// sampling), unlike [`Graph::edge_path`].
    pub fn edge_directions(&self, edge: EdgeIdx, manufacturer: &str, series: &str) -> (f64, f64) {
        let e = &self.edges[edge];
        match e
            .props_for(manufacturer, series)
            .and_then(|p| p.trajectory.as_ref())
        {
            Some(curve) => curve.end_tangents(),
            None => {
                let (a, b) = (&self.nodes[e.start], &self.nodes[e.end]);
                let dir = (b.y - a.y).atan2(b.x - a.x);
                (dir, dir)
            }
        }
    }

    /// Can a robot that has just driven edge `prev` (no orientation given: forwards)
    /// drive edge `next`, which starts where `prev` ends (S71)? It must turn if the
    /// direction `prev` ends in differs from the one `next` starts in:
    /// - an omnidirectional robot keeps its heading and needs no turn;
    /// - a three-wheel robot cannot turn on the spot: the directions must agree;
    /// - a differential robot turns on the spot if the layout allows the turn (the
    ///   rotation rules at the end of `prev` and at the start of `next`, combined).
    ///
    /// The same rules the driver applies when it takes an order. Lift rides always
    /// pass.
    pub fn can_follow(
        &self,
        prev: EdgeIdx,
        next: EdgeIdx,
        kinematics: Kinematics,
        manufacturer: &str,
        series: &str,
    ) -> bool {
        if self.edges[prev].map_transition || self.edges[next].map_transition {
            return true;
        }
        let heading = self.edge_directions(prev, manufacturer, series).1;
        let (_, end_rotation) = self.edge_rotation(prev, manufacturer, series);
        self.can_start(
            heading,
            Some(end_rotation),
            next,
            kinematics,
            manufacturer,
            series,
        )
    }

    /// Can a robot that heads `heading` (radians) at the start node of `edge` drive
    /// it forwards (S71)? `end_rotation` is what the edge it came in on allows at
    /// this node (`None` if unknown, e.g. it stands there). See [`Graph::can_follow`].
    pub fn can_start(
        &self,
        heading: f64,
        end_rotation: Option<Rotation>,
        edge: EdgeIdx,
        kinematics: Kinematics,
        manufacturer: &str,
        series: &str,
    ) -> bool {
        if self.edges[edge].map_transition || kinematics == Kinematics::Omnidirectional {
            return true;
        }
        let (start_dir, _) = self.edge_directions(edge, manufacturer, series);
        let diff = angle_diff(heading, start_dir);
        if kinematics == Kinematics::ThreeWheel {
            return diff.abs() <= THREE_WHEEL_TOLERANCE;
        }
        if diff.abs() <= 1e-9 {
            return true;
        }
        let (start_rotation, _) = self.edge_rotation(edge, manufacturer, series);
        let allowed = end_rotation.map_or(start_rotation, |r| r.and(start_rotation));
        allowed.allows(true) || allowed.allows(false)
    }

    /// The distance a robot of this type drives on the edge (S74): the length of
    /// the curve for an edge with a trajectory, else the straight distance between
    /// the nodes ([`GraphEdge::length`]). Samples the curve, so compute it once.
    pub fn edge_travel_length(&self, edge: EdgeIdx, manufacturer: &str, series: &str) -> f64 {
        let e = &self.edges[edge];
        match e
            .props_for(manufacturer, series)
            .and_then(|p| p.trajectory.as_ref())
        {
            Some(curve) => curve.to_path().length(),
            None => e.length,
        }
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

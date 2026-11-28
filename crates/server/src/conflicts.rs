//! The conflicts of a layout for the fleet, as the traffic rules are given them
//! ([`TrafficPolicy::conflicts`](crate::policy::TrafficPolicy::conflicts) and the two
//! calls after it): by layout ids, per pair of robot types. They are made by
//! `botsim-space` from the layout and the factsheets.

/// The conflicting edge pairs of one pair of robot types, by edge id: the first edge
/// is driven by a robot of `type_a`, the second by one of `type_b` (types as
/// `manufacturer/seriesName`). Both orders of two different types are listed by the
/// checker; a type with itself once. Each pair counts both ways.
#[derive(Debug, Clone, PartialEq)]
pub struct ConflictInput {
    pub type_a: String,
    pub type_b: String,
    pub edges: Vec<(String, String)>,
}

/// The node-edge conflicts of one pair of robot types, by id: a robot of `type_node`
/// standing on a node against an edge (that does not touch the node) driven by a
/// robot of `type_edge`.
#[derive(Debug, Clone, PartialEq)]
pub struct NodeConflictInput {
    pub type_node: String,
    pub type_edge: String,
    /// (node id, edge id)
    pub pairs: Vec<(String, String)>,
}

/// The node-node conflicts of one pair of robot types, by id: two different nodes
/// too close for a robot of `type_a` on the first and one of `type_b` on the second,
/// standing or turning there. Each pair counts both ways.
#[derive(Debug, Clone, PartialEq)]
pub struct NodeNodeConflictInput {
    pub type_a: String,
    pub type_b: String,
    /// (node id for `type_a`, node id for `type_b`)
    pub pairs: Vec<(String, String)>,
}

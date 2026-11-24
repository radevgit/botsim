//! Swept-area reservation: edges whose swept outlines overlap.
//!
//! Two edges that cross, merge or run close have no node in common, so the node
//! and edge reservations do not stop two robots from using them at the same time.
//! A [`ConflictTable`] lists, per pair of robot types, the pairs of edges whose swept
//! outlines overlap (made by the layout checker, `botsim-check`, from the factsheets);
//! the controller releases a robot onto an edge only if no other robot holds an edge
//! that conflicts with it.

use std::collections::HashMap;

use botsim_core::graph::{EdgeIdx, Graph};

use crate::reservations::Reservations;

/// The conflicting edge pairs of one pair of robot types, by edge id: the first edge
/// is driven by a robot of `type_a`, the second by one of `type_b` (types as
/// `manufacturer/seriesName`). Both orders of two different types are listed by the
/// checker; a type with itself once. The controller reads each pair both ways.
#[derive(Debug, Clone, PartialEq)]
pub struct ConflictInput {
    pub type_a: String,
    pub type_b: String,
    pub edges: Vec<(String, String)>,
}

/// The conflicts by edge index: `by_type[a][b][e]` are the edges a robot of type `b`
/// may not hold while a robot of type `a` is released onto `e`.
#[derive(Debug, Default)]
pub struct ConflictTable {
    by_type: Vec<Vec<HashMap<EdgeIdx, Vec<EdgeIdx>>>>,
    /// Number of conflicting edge pairs, for reports.
    pairs: usize,
}

impl ConflictTable {
    /// Read the input against the layout. `types` are the robot types of the
    /// controller as `manufacturer/seriesName`, in the order of their index. Edges and
    /// types the layout or the fleet do not have are left out.
    pub fn new(graph: &Graph, types: &[String], input: &[ConflictInput]) -> ConflictTable {
        let mut by_type = vec![vec![HashMap::new(); types.len()]; types.len()];
        let mut pairs = 0;
        let type_idx = |name: &str| types.iter().position(|t| t == name);
        let ids: HashMap<&str, EdgeIdx> = graph
            .edges
            .iter()
            .enumerate()
            .map(|(i, e)| (e.id.as_str(), i))
            .collect();
        for pair in input {
            let (Some(a), Some(b)) = (type_idx(&pair.type_a), type_idx(&pair.type_b)) else {
                continue;
            };
            for (ea, eb) in &pair.edges {
                let (Some(&x), Some(&y)) = (ids.get(ea.as_str()), ids.get(eb.as_str())) else {
                    continue;
                };
                // x is driven by a robot of type a, y by one of type b: a robot of
                // type a on x is blocked by type b on y, and the other way round
                by_type[a][b].entry(x).or_insert_with(Vec::new).push(y);
                by_type[b][a].entry(y).or_insert_with(Vec::new).push(x);
                pairs += 1;
            }
        }
        for row in &mut by_type {
            for map in row {
                for list in map.values_mut() {
                    list.sort_unstable();
                    list.dedup();
                }
            }
        }
        ConflictTable { by_type, pairs }
    }

    /// Number of conflicting edge pairs read (both orders of two different types
    /// count once each).
    pub fn pairs(&self) -> usize {
        self.pairs
    }

    /// Is a robot of type `robot_type` that is `robot` kept off edge `e` because
    /// another robot holds an edge that conflicts with it? `type_of[r]` is the type
    /// index of robot `r`.
    pub fn blocks(
        &self,
        reservations: &Reservations,
        type_of: &[usize],
        robot: usize,
        e: EdgeIdx,
    ) -> bool {
        self.blocker(reservations, type_of, robot, e).is_some()
    }

    /// The robot that keeps `robot` off edge `e`, with the conflicting edge it holds;
    /// `None` if no robot does.
    pub fn blocker(
        &self,
        reservations: &Reservations,
        type_of: &[usize],
        robot: usize,
        e: EdgeIdx,
    ) -> Option<(usize, EdgeIdx)> {
        let row = self.by_type.get(type_of[robot])?;
        row.iter().enumerate().find_map(|(other_type, map)| {
            map.get(&e)?.iter().find_map(|&f| {
                let h = reservations.edge_holder(f)?;
                (h != robot && type_of[h] == other_type).then_some((h, f))
            })
        })
    }
}

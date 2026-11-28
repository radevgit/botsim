//! A transport job: from a pickup node to a drop node.

use crate::graph::NodeIdx;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Job {
    pub id: u64,
    pub pickup: NodeIdx,
    pub drop: NodeIdx,
    /// The tick the job appeared.
    pub created_tick: u64,
}

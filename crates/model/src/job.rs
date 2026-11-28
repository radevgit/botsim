//! A transport job: from a pickup node to a drop node.

use crate::graph::NodeIdx;
use crate::time::Millis;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Job {
    pub id: u64,
    pub pickup: NodeIdx,
    pub drop: NodeIdx,
    /// When the job appeared.
    pub created: Millis,
}

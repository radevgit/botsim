//! What a controller has reserved, as it publishes it to a watcher of the run.

use crate::graph::{EdgeIdx, NodeIdx};

/// The nodes and edges a controller has reserved for a robot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Reservation {
    /// Index of the robot in the order the robots were added.
    pub robot: usize,
    pub nodes: Vec<NodeIdx>,
    pub edges: Vec<EdgeIdx>,
}

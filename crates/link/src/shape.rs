//! Why an order or the route of a state report cannot be made.

use std::fmt;

/// Parts that do not fit together.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShapeError {
    /// More legs or steps ahead are released (`released`) than there are (`there`).
    TooManyReleased { released: u32, there: usize },
    /// An action is at node `node`, but the last node of the order is `last`.
    ActionBeyondRoute { node: u32, last: u32 },
    /// A state report has steps ahead but no last node to count them from.
    AheadWithoutLastNode,
    /// A `sequenceId` would not fit in 32 bits.
    SequenceTooLarge,
}

impl fmt::Display for ShapeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ShapeError::TooManyReleased { released, there } => {
                write!(f, "{released} released, but there are only {there}")
            }
            ShapeError::ActionBeyondRoute { node, last } => {
                write!(f, "an action at node {node}, but the last node is {last}")
            }
            ShapeError::AheadWithoutLastNode => {
                write!(f, "steps ahead without a last node")
            }
            ShapeError::SequenceTooLarge => {
                write!(f, "a sequenceId does not fit in 32 bits")
            }
        }
    }
}

impl std::error::Error for ShapeError {}

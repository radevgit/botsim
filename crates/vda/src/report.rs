//! What a translation reports: why a message was refused, or which parts of it
//! were left out or changed.

use std::fmt;
use std::sync::Arc;

use botsim_link::shape::ShapeError;
use botsim_model::errors::ErrorType;
use botsim_model::robot::{RobotId, RobotNo};

/// A translated message, and what of the original it does not say as it was.
#[derive(Debug, Clone, PartialEq)]
pub struct Translated<T> {
    pub message: T,
    /// The parts that were left out or changed; empty if the message says all of
    /// the original that is carried at all.
    pub issues: Vec<Issue>,
}

/// One thing that could not be translated as it was.
///
/// As the `Err` of a translation it says why the whole message was refused; in
/// [`Translated::issues`] it names a part that was left out or changed.
#[derive(Debug, Clone, PartialEq)]
pub struct Issue {
    pub problem: Problem,
    /// The VDA 5050 error type with which a robot refuses this; `None` where the
    /// standard has no error for it.
    pub error: Option<ErrorType>,
    /// The `errorReferences` that go with [`error`](Issue::error), key and value,
    /// for example `("orderId", "job-5")`.
    pub references: Vec<(Arc<str>, Arc<str>)>,
    /// For an entry of a list that was left out or changed: its place in the list of
    /// the original message. The field named by the problem says which list.
    pub entry: Option<usize>,
}

/// A name or number of the layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Item {
    Node,
    Edge,
    Map,
}

/// What is wrong. `field` is the VDA 5050 name of the field.
#[derive(Debug, Clone, PartialEq)]
pub enum Problem {
    /// The robot of an incoming message is not in the table.
    UnknownRobot(RobotId),
    /// The robot number was never in the table.
    UnknownRobotNo(RobotNo),
    /// The number of a robot that is gone.
    OldRobotNo(RobotNo),
    /// The major version of an incoming message is not 3.
    Version(String),
    /// A time that is not an ISO 8601 UTC timestamp, or that cannot be written as
    /// one (the years 0 to 9999).
    BadTime { field: &'static str },
    /// A name that the layout does not have.
    UnknownName { item: Item, name: String },
    /// A number that is not in the layout.
    OutsideLayout { item: Item, number: usize },
    /// A number that is not finite.
    NotFinite { field: &'static str },
    /// A number outside the range the message allows.
    OutOfRange { field: &'static str },
    /// An angle outside the range the message allows; it was brought into −π..π.
    AngleNormalized { field: &'static str, from: f64 },
    /// An id that is empty.
    UnusableId { field: &'static str },
    /// A value that would be read as something else, for example an error type
    /// given as "other" with the name of a predefined one.
    ReadsBackDifferently { field: &'static str },
    /// A zone with fewer than 3 vertices.
    TooFewVertices { zone: String, there: usize },
    /// An order without nodes, or whose edge count is not the node count minus one.
    WrongCounts { nodes: usize, edges: usize },
    /// A `sequenceId` that is not what its place requires. `id` is the `nodeId` or
    /// `edgeId`.
    BadSequenceId {
        field: &'static str,
        id: String,
        expected: u64,
        found: u32,
    },
    /// The first node of an order is not released.
    FirstNodeNotReleased,
    /// An edge of an order is released but one of its nodes is not.
    EdgeReleasedWithoutNode { edge: String },
    /// Something of an order is released after an unreleased edge.
    ReleasedAfterHorizon { id: String },
    /// A state has nodes or edges ahead but no `lastNodeId`.
    AheadWithoutLastNode,
    /// The `nodeStates` and `edgeStates` of a state do not pair up.
    UnpairedRoute { nodes: usize, edges: usize },
    /// The `released` flags of a state's route do not fit together; `id` is the
    /// `nodeId` where it shows.
    ReleaseMismatch { id: String },
    /// Parts that `link` does not accept together.
    Shape(ShapeError),
}

impl Issue {
    pub(crate) fn new(problem: Problem) -> Issue {
        Issue {
            problem,
            error: None,
            references: Vec::new(),
            entry: None,
        }
    }

    /// With the error type and one reference.
    pub(crate) fn with_error(mut self, error: ErrorType, key: &str, value: &str) -> Issue {
        self.error = Some(error);
        self.references = vec![(key.into(), value.into())];
        self
    }

    pub(crate) fn at(mut self, entry: usize) -> Issue {
        self.entry = Some(entry);
        self
    }
}

impl fmt::Display for Item {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Item::Node => "node",
            Item::Edge => "edge",
            Item::Map => "map",
        })
    }
}

impl fmt::Display for Problem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Problem::UnknownRobot(id) => write!(
                f,
                "unknown robot '{}' / '{}'",
                id.manufacturer, id.serial_number
            ),
            Problem::UnknownRobotNo(no) => write!(
                f,
                "unknown robot number {} (generation {})",
                no.slot(),
                no.generation()
            ),
            Problem::OldRobotNo(no) => write!(
                f,
                "robot number {} (generation {}) is of a robot that is gone",
                no.slot(),
                no.generation()
            ),
            Problem::Version(v) => write!(f, "version '{v}' is not a version 3"),
            Problem::BadTime { field } => {
                write!(
                    f,
                    "{field} is not an ISO 8601 UTC time of the years 0 to 9999"
                )
            }
            Problem::UnknownName { item, name } => {
                write!(f, "{item} '{name}' is not in the layout")
            }
            Problem::OutsideLayout { item, number } => {
                write!(f, "{item} number {number} is not in the layout")
            }
            Problem::NotFinite { field } => write!(f, "{field} is not a finite number"),
            Problem::OutOfRange { field } => write!(f, "{field} is out of range"),
            Problem::AngleNormalized { field, from } => {
                write!(f, "{field} {from} was brought into -pi..pi")
            }
            Problem::UnusableId { field } => write!(f, "{field} is not a usable id"),
            Problem::ReadsBackDifferently { field } => {
                write!(f, "{field} would be read as something else")
            }
            Problem::TooFewVertices { zone, there } => {
                write!(f, "zone '{zone}' needs at least 3 vertices, got {there}")
            }
            Problem::WrongCounts { nodes, edges } => write!(
                f,
                "order needs at least one node and nodes - 1 edges, got {nodes} nodes and {edges} edges"
            ),
            Problem::BadSequenceId {
                field,
                id,
                expected,
                found,
            } => write!(f, "{field} of '{id}' is {found}, expected {expected}"),
            Problem::FirstNodeNotReleased => write!(f, "the first node must be released"),
            Problem::EdgeReleasedWithoutNode { edge } => {
                write!(f, "edge '{edge}' is released but one of its nodes is not")
            }
            Problem::ReleasedAfterHorizon { id } => {
                write!(f, "'{id}' is released after an unreleased edge or node")
            }
            Problem::AheadWithoutLastNode => {
                write!(f, "nodes or edges ahead without a lastNodeId")
            }
            Problem::UnpairedRoute { nodes, edges } => write!(
                f,
                "{nodes} nodeStates and {edges} edgeStates do not pair up"
            ),
            Problem::ReleaseMismatch { id } => {
                write!(f, "the released flags do not fit together at node '{id}'")
            }
            Problem::Shape(e) => e.fmt(f),
        }
    }
}

impl fmt::Display for Issue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.problem.fmt(f)?;
        if let Some(entry) = self.entry {
            write!(f, " (entry {entry})")?;
        }
        if let Some(error) = self.error {
            write!(f, " [{}]", error.as_str())?;
        }
        Ok(())
    }
}

impl std::error::Error for Issue {}

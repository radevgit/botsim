//! Recording of a botsim run as JSON Lines: line 1 is the layout
//! ([`LayoutLine`]), then one snapshot per line.
//!
//! The line types here are plain serde data, separate from the types of
//! `botsim-core` (core knows no file format). Maps, nodes and edges are numbered
//! by their position in the lists, as in the observer port.

mod geometry;
mod layout;
mod reader;
mod snapshot;
mod writer;

pub use geometry::{
    GEOMETRY_VERSION, GeometryError, GeometryFile, GeometryKind, GeometryShape, Outline,
    StaticShape, Vertex,
};
pub use layout::{EdgePath, EdgeRecord, LayoutLine, LayoutRobot, NodeRecord};
pub use reader::{ReadError, Reader, Recording, read_recording};
pub use snapshot::{
    CallRecord, DoorRecord, DoorStateRecord, LiftRecord, ReservationRecord, RobotRecord,
    RouteRecord, ShapeRecord, SnapshotLine, StatusRecord,
};
pub use writer::{RecordHandle, Recorder};

/// The version written in the layout line. Raise it when the format changes in a
/// way an old reader cannot handle.
pub const FORMAT_VERSION: u32 = 1;

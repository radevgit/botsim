//! Recording of a botsim run as JSON Lines: line 1 is the layout
//! ([`LayoutLine`]), then one snapshot per line.
//!
//! The line types here are plain serde data, separate from the types of
//! `botsim-core` (core knows no file format). Maps, nodes and edges are numbered
//! by their position in the lists, as in the observer port.

mod layout;
mod snapshot;

pub use layout::{EdgeRecord, LayoutLine, NodeRecord};
pub use snapshot::{
    CallRecord, DoorRecord, DoorStateRecord, LiftRecord, ReservationRecord, RobotRecord,
    RouteRecord, ShapeRecord, SnapshotLine, StatusRecord,
};

/// The version written in the layout line. Raise it when the format changes in a
/// way an old reader cannot handle.
pub const FORMAT_VERSION: u32 = 1;

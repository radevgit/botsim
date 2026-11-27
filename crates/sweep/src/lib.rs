//! Swept robot bodies along LIF edges, and the conflict map of a layout for a mixed fleet.
//!
//! For each robot type the outline of its body is swept along each edge it may drive
//! ([`sweep::sweep`]) and placed on each node it may stand on ([`sweep::node_bodies`]).
//! The [conflict map](conflict::ConflictMap) lists, per pair of robot types, which
//! pairs of edges (and which node against which edge) come nearer than the
//! [clearance](conflict::MapOptions::clearance) or overlap. It is plain data with
//! JSON output; the library knows nothing about findings or scenarios.
//!
//! ```no_run
//! use botsim_sweep::{MapOptions, TypeRef, compute};
//! # fn run(lif: &vdma_lif::Lif, types: &[TypeRef]) {
//! let map = compute(lif, types, &MapOptions::default());
//! # }
//! ```
//!
//! [`TypeRef`] is made by the caller from a factsheet (the checker has the code).
//! Used by `botsim-check` (findings) and the runner (reservations of the baseline).

pub mod conflict;
pub mod places;
pub mod sweep;
pub mod types;

pub use conflict::{
    CLEARANCE, Conflict, ConflictMap, MIN_OVERLAP, MapOptions, NodeConflict, NodeNodeConflict,
    NodeNodeTypePair, NodeTypePair, TypePair, compute, to_json,
};
pub use places::{LayoutIndex, Place, PlaceKind};
pub use sweep::{
    Body, CORNER_STEP, Contact, SAMPLE_STEP, Sweep, field_sweep, node_bodies, standing_body, sweep,
};
pub use types::TypeRef;

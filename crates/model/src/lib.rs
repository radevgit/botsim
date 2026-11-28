//! What the botsim crates share about a site and its robots: the layout graph,
//! robot types, the geometry of paths, headings and protective fields, and the
//! VDA 5050 error types and timestamps.
//!
//! Depends on no other botsim crate.

pub mod angle;
pub mod errors;
pub mod graph;
pub mod orientation;
pub mod path;
pub mod robot;
pub mod safety;
pub mod spec;
pub mod time;

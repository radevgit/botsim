//! What the botsim crates share about a site and its robots: the layout graph,
//! robot types, zones, the geometry of paths, bodies, headings and protective
//! fields, transport jobs, waiting and reservations, and the VDA 5050 version,
//! error types and timestamps.
//!
//! Depends on no other botsim crate.

pub mod angle;
pub mod collision;
pub mod errors;
pub mod graph;
pub mod job;
pub mod orientation;
pub mod path;
pub mod protocol;
pub mod reservation;
pub mod robot;
pub mod safety;
pub mod spec;
pub mod time;
pub mod waiting;
pub mod zones;

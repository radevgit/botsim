//! What the botsim crates share about a site and its robots: the layout graph,
//! robot types, zones, the geometry of paths, bodies, headings and protective
//! fields, transport jobs, waiting and reservations, the VDA 5050 version,
//! error types and timestamps, and the hash by which two runs are compared
//! ([`checksum`]).
//!
//! Depends on no other botsim crate.

pub mod angle;
pub mod checksum;
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

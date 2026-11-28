//! The command line of botsim: it connects the fleet server and the simulator
//! ([`connect`]; for tests also through VDA 5050, [`vda_route`]), runs a scenario
//! and writes the JSON summary. The scenario file types live in the
//! `botsim-scenario` crate (shared with the checker) and are re-exported here as
//! `scenario`. The server and the simulator know nothing about files.

pub use botsim_scenario as scenario;
pub mod connect;
pub mod summary;
pub mod traffic_setup;
pub mod vda_route;
pub mod waiting_summary;

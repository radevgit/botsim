//! Headless simulation core for botsim.
//!
//! Knows nothing about files, Rerun or MQTT; those live in adapters.

pub mod actions;
pub mod drive;
pub mod errors;
pub mod graph;
pub mod motion;
pub mod order_route;
pub mod orientation;
pub mod robot;
pub mod spec;
pub mod state_report;
pub mod world;

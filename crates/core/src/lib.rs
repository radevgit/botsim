//! Headless simulation core for botsim.
//!
//! Knows nothing about files, Rerun or MQTT; those live in adapters.

pub mod actions;
pub mod collision;
pub mod deadlock;
pub mod disturbances;
pub mod door;
pub mod drive;
pub mod errors;
pub mod graph;
pub mod jobs;
pub mod lift;
pub mod metrics;
pub mod motion;
pub mod observer;
pub mod order_route;
pub mod orientation;
pub mod path;
pub mod robot;
pub mod safety;
pub mod spec;
pub mod state_report;
pub mod world;
pub mod zone_requests;
pub mod zones;

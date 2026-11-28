//! Headless simulation core for botsim.
//!
//! Knows nothing about files, Rerun or MQTT; those live in adapters.

pub mod actions;
pub mod battery;
pub mod deadlock;
pub mod disturbances;
pub mod door;
pub mod drive;
pub mod errors;
pub mod jobs;
pub mod lift;
pub mod motion;
pub mod observer;
pub mod order_route;
pub mod robot;
pub mod robot_zones;
pub mod state_report;
pub mod waiting;
pub mod world;
pub mod zone_requests;

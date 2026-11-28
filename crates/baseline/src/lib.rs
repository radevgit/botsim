//! The bundled baseline controller: nearest free robot, first come first
//! served, node-by-node release (see the brief). It knows no simulator: it is
//! given the site and, cycle by cycle, the robots' state messages as data, and
//! gives back the messages to send (see [`cycle`]).

pub mod baseline_policy;
pub mod conflicts;
pub mod controller;
pub mod cycle;
pub mod doors;
pub mod lifts;
pub mod metrics;
pub mod node_grid;
pub mod order_builder;
pub mod policy;
pub mod reservations;
pub mod zone_grants;

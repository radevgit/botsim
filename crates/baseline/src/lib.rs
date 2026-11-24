//! The bundled baseline controller: nearest free robot, first come first
//! served, node-by-node release (see the brief). It talks to the simulator only
//! through VDA 5050 messages, like any other controller.

pub mod conflicts;
pub mod controller;
pub mod doors;
pub mod lifts;
pub mod node_grid;
pub mod order_builder;
pub mod reservations;
pub mod route;
pub mod zone_grants;

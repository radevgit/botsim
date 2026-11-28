//! The botsim fleet server. Its control loop, the [`controller`], carries out the
//! tasks a [dispatcher](botsim_dispatcher) gives out, and asks the traffic rules in
//! its slot ([`policy::TrafficPolicy`]) who may go now. The rules are the caller's
//! choice: the traffic module (through [`traffic_policy::TrafficAdapter`]),
//! `botsim-baseline`'s first come, first served rules, or others.
//!
//! The server knows no simulator and no scenario file: it is given the site and,
//! cycle by cycle, the reports of the robots and lifts as data, and gives back the
//! messages to send (see [`cycle`]), all of them [`botsim_link`]'s.

pub mod checksum;
pub mod conflicts;
pub mod controller;
pub mod cycle;
pub mod doors;
pub mod lifts;
pub mod metrics;
pub mod node_grid;
pub mod order_builder;
pub mod policy;
pub mod traffic_policy;

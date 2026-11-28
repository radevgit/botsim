//! The baseline's traffic rules for the botsim fleet server: first come, first
//! served, node-by-node release (see [`baseline_policy`]). They are the first
//! rules behind the server's slot ([`botsim_server::policy`]) and the reference
//! other rules are measured against.
//!
//! The control loop that asks them is [`botsim_server::controller`];
//! [`controller()`] makes one with these rules.

pub mod baseline_policy;
pub mod conflicts;
pub mod reservations;
pub mod zone_grants;

use botsim_server::controller::Controller;
use botsim_server::cycle::Config;

use crate::baseline_policy::BaselinePolicy;

/// A controller for the fleet of `config` with the baseline's traffic rules and
/// dispatcher.
pub fn controller(config: &Config) -> Controller<BaselinePolicy> {
    Controller::with_policy(config, BaselinePolicy::new(&config.graph))
}

//! Adapters around the simulator core: the CLI and the JSON summary. The scenario
//! file types live in the `botsim-scenario` crate (shared with the checker) and
//! are re-exported here as `scenario`. The core itself knows nothing about files.

pub use botsim_scenario as scenario;
pub mod summary;

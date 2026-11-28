//! Route finding on a layout graph: the cheapest routes for robots of a type,
//! routes through the waiting nodes of lift entrances, and the route cost with the
//! load and waiting penalties. Everything it needs comes as plain data; it knows
//! nothing of a controller or of a simulation.

pub mod lift;
pub mod load;
pub mod route;

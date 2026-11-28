//! The messages between the botsim fleet server and its robots, doors and lifts.
//! Those of robots are compact data with VDA 5050 3.0.1 meaning.
//!
//! These types are used inside the program and are never sent to a robot; the
//! translation to and from VDA 5050 messages is not here but in `botsim-vda`.
//!
//! - Layout nodes, edges and maps are the numbers of the layout graph
//!   ([`botsim_model::graph`]); a robot is a
//!   [`RobotNo`](botsim_model::robot::RobotNo). Names are looked up elsewhere.
//! - A message of a robot has a [`Header`](message::Header), which names the robot;
//!   one of a door or lift has a [`DeviceHeader`](device::DeviceHeader), which names
//!   the device. Every message is complete in itself.
//! - The types promise **shape** only, for example that no more of an order is
//!   released than there is. Whether a number lies in the layout, a float is finite
//!   or an id is usable is for whoever reads a message to check.
//! - To and from robots: only what VDA 5050 can express, and of that what botsim
//!   writes or reads. Doors and lifts are outside VDA 5050.
//!
//! The fleet server's controller (`botsim-server`) and the simulator
//! (`botsim-core`) speak them; `botsim-bridge` passes them between the two.
//!
//! Depends on no botsim crate but `botsim-model`.

pub mod device;
pub mod ids;
pub mod instant;
pub mod message;
pub mod order;
pub mod response;
pub mod shape;
pub mod state;
pub mod zone_set;

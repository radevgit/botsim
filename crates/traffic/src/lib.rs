//! Decides the passing order of robots at every shared spot, from their full routes,
//! so that they cannot collide or deadlock.
//!
//! The module never plans paths and never talks to a robot. Built so far: the
//! [spots], RELEASE zones among them, the [groups](mod@groups) of conflicts that
//! share one order, the order [graph], how [routes](traffic) enter it, are
//! changed, cut short, cancelled and followed by a new order, first come, first
//! served node by node, the release query, the answers to zone requests and the
//! holder query.

#[cfg(test)]
mod brute_force;
pub mod graph;
pub mod groups;
#[cfg(test)]
mod scenarios;
pub mod spots;
#[cfg(test)]
mod testing;
pub mod traffic;

pub use graph::{BadProgress, Before, OrderGraph, Pass, Refused};
pub use groups::{Group, Route, UnknownName, groups};
pub use spots::{Piece, SetupError, Spots};
pub use traffic::{Cancel, NotAdded, Release, Report, Reroute, Traffic, ZoneAnswer};

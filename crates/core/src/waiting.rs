//! Who a waiting robot waits for, and why (brief U12, "Definition"; user decisions
//! S157, S158). The world decides it once per step, see
//! [`crate::world::World::waits`]; the deadlock detector uses it.
//!
//! A robot **waits** when it has an order ahead of it and stands still without
//! running an action, or its protective field stops or brakes it:
//!
//! 1. **Safety stop:** another robot is in its protective field.
//! 2. **Release**, **zone**, **lift**, **door:** the controller holds it back and
//!    says why and who causes it ([`crate::world::World::publish_held_back`]).
//!
//! If several robots cause a wait, the **nearest** is named (the smallest gap
//! between the bodies; at equal gaps the one added first). If the controller names
//! nobody, the blocker is unknown (`None`).
//!
//! Without a controller that publishes, the world has its own two rules, both a
//! release wait: the robot waits for the robot that stands on its next node, else
//! for the one that has that node released ahead of it; otherwise it has no wait.
//!
//! The world adds the waits up, step by step, for the "waiting caused by others"
//! metric ([`crate::world::World::wait_totals`]), leaving out the steps in which the
//! waiting robot is in its own stop (a random stop or an empty battery, S170).

use crate::graph::{EdgeIdx, NodeIdx};

/// Why a robot waits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum WaitReason {
    /// The next piece of its route is not released to it.
    Release,
    /// A RELEASE zone request is not granted.
    Zone,
    /// Another robot owns the lift its route needs.
    Lift,
    /// Another robot owns the door its route needs.
    Door,
    /// Another robot is in its protective field.
    SafetyStop,
}

/// What the controller says about a robot it holds back.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HeldBack {
    pub reason: WaitReason,
    /// The robots that cause it (indices in the order they were added to the
    /// world); empty if none can be named.
    pub blockers: Vec<usize>,
}

/// Where a waiting robot stands (S168): on a node (within 0.1 m of it), else on the
/// edge it is driving.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Place {
    Node(NodeIdx),
    Edge(EdgeIdx),
}

/// How long one robot has waited so far for one blocker, for one reason, in one
/// place.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WaitTotal {
    /// The waiting robot (index in the order the robots were added).
    pub waiter: usize,
    /// The robot it waited for; `None` if unknown.
    pub blocker: Option<usize>,
    pub reason: WaitReason,
    pub place: Place,
    /// Steps.
    pub ticks: u64,
}

/// One robot's wait in one step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Wait {
    pub reason: WaitReason,
    /// The robot it waits for; `None` if unknown.
    pub blocker: Option<usize>,
}

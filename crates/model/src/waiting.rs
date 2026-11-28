//! Waiting: when a robot counts as waiting, and what a controller says about a
//! robot it holds back. The controller reads the first from state messages; the
//! simulator (`botsim_core::waiting`) uses both to decide who waits for whom.
//!
//! **Waiting** (user decision S32): the time a robot **stands still while it has a
//! job** and is **not busy with an action** (`pick`, `drop`, …). That is the time
//! traffic control costs.

use vda5050::state::{ActionStatus, State};

/// Is this robot waiting, judged from its state message? Standing still means
/// not driving and not turning on the spot (`driving` covers both); any running
/// action makes it busy. Whether the robot has a job is for the caller to know.
pub fn is_waiting(state: &State) -> bool {
    let acting = state
        .action_states
        .iter()
        .any(|a| a.action_status == ActionStatus::Running);
    stands_waiting(state.driving, state.safety_state.field_violation, acting)
}

/// The rule of [`is_waiting`], on the three things it reads.
pub fn stands_waiting(driving: bool, field_violation: bool, acting: bool) -> bool {
    // a robot stopped by an obstacle or a disturbance (protective field
    // violation) is not waiting for the controller
    !driving && !field_violation && !acting
}

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

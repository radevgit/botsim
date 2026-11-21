//! Actions as timed waits, with VDA 5050 blocking types (§6.2.2).
//!
//! When the robot reaches a point with actions (here: a node), they are queued in
//! the order of the `actions` array. The queue is processed like this:
//! - `NONE` / `SOFT` actions are collected and run in parallel;
//! - a `SINGLE` / `HARD` action starts only when nothing else is running, and runs
//!   alone; later actions wait until it is finished;
//! - driving is not allowed while a `SOFT` or `HARD` action is unfinished.
//!
//! An action takes a fixed time (from [`ActionTimes`]); it has no effect on the
//! world. Time is quantised to the step: an action that finishes in a step lets
//! the next one start at once (no leftover time carried over).
//!
//! Not yet: failing actions, paused actions, instant actions, edge actions.

use std::collections::HashMap;

use vda5050::common::BlockingType;
use vda5050::state::ActionStatus;

use crate::jobs::Rng;

/// Remaining time at or below this counts as done. Subtracting a step ten times
/// (10 × 0.1) from 1.0 leaves about 1e-16, not 0; without a tolerance the action
/// would finish one step late.
pub const TIME_EPSILON: f64 = 1e-9;

/// How long actions take, in seconds: a default and per-action-type overrides.
#[derive(Debug, Clone, PartialEq)]
pub struct ActionTimes {
    pub default: f64,
    pub by_type: HashMap<String, f64>,
    /// Random extra time per action (S38), if disturbances are on.
    pub noise: Option<DurationNoise>,
}

/// Each action takes its base time times `1 + u × variation`, with `u` drawn
/// uniformly from [0, 1): a variation of 0.2 adds up to 20 %.
#[derive(Debug, Clone, PartialEq)]
pub struct DurationNoise {
    pub rng: Rng,
    pub variation: f64,
}

impl ActionTimes {
    pub fn new(default: f64) -> ActionTimes {
        ActionTimes {
            default,
            by_type: HashMap::new(),
            noise: None,
        }
    }

    /// Turn on random extra time; `rng` is the robot's own stream.
    pub fn with_noise(mut self, rng: Rng, variation: f64) -> ActionTimes {
        self.noise = Some(DurationNoise { rng, variation });
        self
    }

    /// The times for the next order of a robot. Without noise this is a plain
    /// copy. With noise every order gets its own random stream, taken from this
    /// one, so the durations of an order do not depend on what earlier orders did.
    pub fn for_next_order(&mut self) -> ActionTimes {
        let mut t = self.clone();
        if let (Some(master), Some(order)) = (self.noise.as_mut(), t.noise.as_mut()) {
            order.rng = Rng::new(master.rng.next_u64());
        }
        t
    }

    pub fn with(mut self, action_type: &str, seconds: f64) -> ActionTimes {
        self.by_type.insert(action_type.to_string(), seconds);
        self
    }

    pub fn duration(&self, action_type: &str) -> f64 {
        self.by_type
            .get(action_type)
            .copied()
            .unwrap_or(self.default)
    }
}

/// An action as the order describes it, before a duration is chosen.
#[derive(Debug, Clone, PartialEq)]
pub struct ActionSpec {
    pub action_id: String,
    pub action_type: String,
    pub blocking: BlockingType,
}

impl ActionTimes {
    /// Decide the duration of an action (with random extra time if noise is on;
    /// each call draws one number).
    pub fn plan(&mut self, a: &ActionSpec) -> PlannedAction {
        let base = self.duration(&a.action_type);
        let duration = match self.noise.as_mut() {
            Some(n) => base * (1.0 + n.rng.next_f64() * n.variation),
            None => base,
        };
        PlannedAction {
            action_id: a.action_id.clone(),
            action_type: a.action_type.clone(),
            blocking: a.blocking,
            duration,
        }
    }
}

/// An action to be executed, with its duration already decided.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedAction {
    pub action_id: String,
    pub action_type: String,
    pub blocking: BlockingType,
    /// Seconds, ≥ 0.
    pub duration: f64,
}

#[derive(Debug, Clone, PartialEq)]
struct Item {
    action: PlannedAction,
    status: ActionStatus,
    /// Seconds left while running.
    remaining: f64,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ActionQueue {
    items: Vec<Item>,
}

impl ActionQueue {
    pub fn new() -> ActionQueue {
        ActionQueue::default()
    }

    /// Queue actions (in the given order) and start whatever may start now.
    pub fn push(&mut self, actions: impl IntoIterator<Item = PlannedAction>) {
        for a in actions {
            let remaining = a.duration.max(0.0);
            self.items.push(Item {
                action: a,
                status: ActionStatus::Waiting,
                remaining,
            });
        }
        self.settle();
    }

    /// Let `dt` seconds pass.
    pub fn step(&mut self, dt: f64) {
        for it in &mut self.items {
            if it.status == ActionStatus::Running {
                it.remaining -= dt;
            }
        }
        self.settle();
    }

    /// Finish what is done and start what may start, until nothing changes.
    fn settle(&mut self) {
        loop {
            let mut changed = false;

            for it in &mut self.items {
                if it.status == ActionStatus::Running && it.remaining <= TIME_EPSILON {
                    it.status = ActionStatus::Finished;
                    changed = true;
                }
            }

            // Start waiting actions in queue order; the first one that may not
            // start holds back everything behind it.
            let mut running = self
                .items
                .iter()
                .filter(|i| i.status == ActionStatus::Running)
                .count();
            let mut exclusive_running = self
                .items
                .iter()
                .any(|i| i.status == ActionStatus::Running && is_exclusive(i.action.blocking));
            for it in &mut self.items {
                if it.status != ActionStatus::Waiting {
                    continue;
                }
                let exclusive = is_exclusive(it.action.blocking);
                let may_start = if exclusive {
                    running == 0
                } else {
                    !exclusive_running
                };
                if !may_start {
                    break;
                }
                it.status = ActionStatus::Running;
                running += 1;
                exclusive_running |= exclusive;
                changed = true;
                if exclusive {
                    break; // nothing may start next to it
                }
            }

            if !changed {
                return;
            }
        }
    }

    /// Cancel everything waiting or running (all actions are treated as
    /// cancellable): they become `FAILED`, as `cancelOrder` requires.
    pub fn cancel_all(&mut self) {
        for it in &mut self.items {
            if matches!(it.status, ActionStatus::Waiting | ActionStatus::Running) {
                it.status = ActionStatus::Failed;
            }
        }
    }

    /// List actions that were cancelled before they were ever triggered (`FAILED`).
    pub fn push_failed(&mut self, actions: impl IntoIterator<Item = PlannedAction>) {
        for a in actions {
            self.items.push(Item {
                action: a,
                status: ActionStatus::Failed,
                remaining: 0.0,
            });
        }
    }

    /// Driving is allowed unless a `SOFT` or `HARD` action is unfinished.
    pub fn driving_allowed(&self) -> bool {
        !self.items.iter().any(|i| {
            !is_done(i.status)
                && matches!(i.action.blocking, BlockingType::Soft | BlockingType::Hard)
        })
    }

    /// No unfinished action left (finished and failed ones are done).
    pub fn is_idle(&self) -> bool {
        self.items.iter().all(|i| is_done(i.status))
    }

    /// Id, type and status of every action ever queued, in queue order (for the
    /// state message; finished ones stay listed).
    pub fn states(&self) -> impl Iterator<Item = (&str, &str, ActionStatus)> {
        self.items.iter().map(|i| {
            (
                i.action.action_id.as_str(),
                i.action.action_type.as_str(),
                i.status,
            )
        })
    }
}

fn is_done(s: ActionStatus) -> bool {
    matches!(s, ActionStatus::Finished | ActionStatus::Failed)
}

/// `SINGLE` and `HARD` allow no other action next to them.
fn is_exclusive(b: BlockingType) -> bool {
    matches!(b, BlockingType::Single | BlockingType::Hard)
}

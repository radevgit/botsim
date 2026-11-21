//! Deadlock detection (user decision S35). The simulator only observes and
//! reports; resolving a deadlock is the controller's job.
//!
//! **Wait-for rule** (the world builds it from the robots' state messages): robot
//! A *waits for* robot B if A stands still with an order still ahead of it, is
//! not busy with an action, and B stands on A's next node. A **deadlock** is a
//! cycle of such waits (A waits for B, B waits for A, or a longer ring) that
//! lasts at least the timeout (default 30 s).
//!
//! Limits: it sees only robots standing on the next node. A cycle through a node
//! that is reserved but empty is not seen. A robot that waits for a robot that
//! is not itself waiting (e.g. one standing idle for ever) is not a deadlock
//! by this rule.

use std::collections::HashMap;

/// The default time a cycle must last to count as a deadlock.
pub const DEFAULT_TIMEOUT_SECONDS: f64 = 30.0;

/// One deadlock. Robots are indices in the order they were added to the world.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeadlockRecord {
    /// The robots in the cycle, in ascending order.
    pub robots: Vec<usize>,
    /// The tick the cycle was first seen.
    pub first_seen_tick: u64,
    /// The tick it had lasted for the timeout and was reported.
    pub detected_tick: u64,
    /// The tick the cycle was gone again; `None` while it lasts.
    pub ended_tick: Option<u64>,
}

#[derive(Debug, Clone)]
pub struct DeadlockDetector {
    timeout_ticks: u64,
    /// Cycles seen now, with the tick they were first seen and, once reported,
    /// the index of their record.
    seen: HashMap<Vec<usize>, (u64, Option<usize>)>,
    records: Vec<DeadlockRecord>,
}

impl DeadlockDetector {
    /// `timeout_seconds` > 0, `dt` the time step.
    pub fn new(timeout_seconds: f64, dt: f64) -> DeadlockDetector {
        DeadlockDetector {
            timeout_ticks: (timeout_seconds / dt).ceil().max(1.0) as u64,
            seen: HashMap::new(),
            records: Vec::new(),
        }
    }

    pub fn records(&self) -> &[DeadlockRecord] {
        &self.records
    }

    /// Look at the world after `tick`. `waits_for[i]` is the robot that robot
    /// `i` waits for, if any.
    pub fn observe(&mut self, tick: u64, waits_for: &[Option<usize>]) {
        let now = cycles(waits_for);

        // cycles that are gone
        let gone: Vec<Vec<usize>> = self
            .seen
            .keys()
            .filter(|k| !now.contains(k))
            .cloned()
            .collect();
        for key in gone {
            if let Some((_, Some(r))) = self.seen.remove(&key) {
                self.records[r].ended_tick = Some(tick);
            }
        }

        // cycles that are new, or that have lasted long enough
        for key in now {
            let entry = self.seen.entry(key.clone()).or_insert((tick, None));
            if entry.1.is_none() && tick - entry.0 >= self.timeout_ticks {
                self.records.push(DeadlockRecord {
                    robots: key,
                    first_seen_tick: entry.0,
                    detected_tick: tick,
                    ended_tick: None,
                });
                entry.1 = Some(self.records.len() - 1);
            }
        }
    }
}

/// All cycles of the "waits for" relation, each as its sorted robot indices.
/// Every robot waits for at most one other, so a walk along the links either
/// ends or runs into a cycle.
fn cycles(waits_for: &[Option<usize>]) -> Vec<Vec<usize>> {
    #[derive(Clone, Copy, PartialEq)]
    enum Mark {
        New,
        OnWalk(usize),
        Done,
    }
    let n = waits_for.len();
    let mut mark = vec![Mark::New; n];
    let mut found = Vec::new();

    for start in 0..n {
        if mark[start] != Mark::New {
            continue;
        }
        let mut walk = Vec::new();
        let mut at = Some(start);
        while let Some(r) = at {
            match mark[r] {
                Mark::Done => break,
                Mark::OnWalk(pos) => {
                    let mut c: Vec<usize> = walk[pos..].to_vec();
                    c.sort_unstable();
                    found.push(c);
                    break;
                }
                Mark::New => {
                    mark[r] = Mark::OnWalk(walk.len());
                    walk.push(r);
                    at = waits_for.get(r).copied().flatten();
                }
            }
        }
        for r in walk {
            mark[r] = Mark::Done;
        }
    }
    found.sort();
    found
}

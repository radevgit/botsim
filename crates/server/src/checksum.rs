//! Two numbers per cycle that say whether two runs of the controller went the same
//! way: one of what came out ([`of_outbox`]), one of the state it is left in
//! ([`Controller::state_checksum`](crate::controller::Controller::state_checksum)).
//! Both are [`Checksum`]s, so they can be compared only between runs of **one
//! build on one kind of machine**; nothing computes them unless asked.
//!
//! **Messages.** Everything in the [`Outbox`]: each message with all it carries, in
//! their order, why each robot is held back, and the reservations.
//!
//! **State.** Everything the controller keeps that can change from cycle to cycle:
//! - its clock, the newest report of every robot (all of it) and which are new;
//! - of every robot the work in hand (job, route, how far it is released, what is
//!   not yet confirmed, a cancel under way), the sequence numbers of its messages,
//!   what was sent of the zones, and its times;
//! - the jobs done and refused, the count of new routes, the route weights and the
//!   waiting averages of the nodes;
//! - who owns which lift and door, and the sequence numbers of their commands;
//! - what the dispatcher and the traffic rules keep
//!   ([`Dispatcher::checksum`](botsim_dispatcher::Dispatcher::checksum),
//!   [`TrafficPolicy::checksum`](crate::policy::TrafficPolicy::checksum); one of
//!   your own that does not say is seen only by what it makes the controller send).
//!
//! **Not in it:** what is fixed by the setup (layout, fleet, routing and conflict
//! tables). So a number is the same although the input differed if the difference
//! changed nothing that is kept or sent: a report older than the one kept, a field
//! of a lift state that nothing reads.
//!
//! **Cost.** One pass over that state; it grows with the fleet, the length of the
//! routes, the jobs that wait for a robot and the orders the traffic rules keep,
//! not with the jobs already done.

use std::hash::{Hash, Hasher};

use botsim_model::checksum::Checksum;

use crate::cycle::Outbox;

/// The checksum of what a cycle gave back (see the module text).
pub fn of_outbox(outbox: &Outbox) -> u64 {
    let mut sum = Checksum::new();
    outbox.hash(&mut sum);
    sum.finish()
}

/// A list that only grows, with a running checksum of its entries, so that hashing
/// it costs the same however long it is.
#[derive(Debug)]
pub(crate) struct History<T> {
    entries: Vec<T>,
    digest: Checksum,
}

impl<T> Default for History<T> {
    fn default() -> Self {
        History {
            entries: Vec::new(),
            digest: Checksum::new(),
        }
    }
}

impl<T: Hash> History<T> {
    pub(crate) fn push(&mut self, entry: T) {
        entry.hash(&mut self.digest);
        self.entries.push(entry);
    }

    pub(crate) fn entries(&self) -> &[T] {
        &self.entries
    }
}

impl<T> Hash for History<T> {
    fn hash<H: Hasher>(&self, h: &mut H) {
        // (the entries are in the digest)
        let History { entries, digest } = self;
        entries.len().hash(h);
        digest.finish().hash(h);
    }
}

//! The baseline's answers to RELEASE zone requests (VDA 5050 §6.4.3).
//!
//! **Rule: exit before enter.** A zone is granted to **one robot at a time**:
//! - first come first served: requests are answered in the order they were first
//!   seen; a robot that has to wait gets QUEUED once, then GRANTED when it is its
//!   turn;
//! - a zone is *not free* while another robot holds a grant for it (from the
//!   grant until the robot has left and its request is gone) or another robot
//!   stands inside it, even without a grant;
//! - a later request for another zone is not held back by an earlier one that has
//!   to wait.
//!
//! The grant has no lease (no expiry) and is never revoked: not yet.
//! Why one robot at a time and the zone covering the lift approach: see the
//! design rule in `docs/rules.md` §7d.

use std::collections::HashSet;

use botsim_link::ids::RequestId;
use botsim_link::response::Grant;
use botsim_link::state::{ZoneRequest, ZoneRequestType};
use botsim_model::zones::Zones;
use botsim_server::policy::ZoneAnswer;

/// What the controller knows of one robot (from its state report).
#[derive(Debug, Clone)]
pub struct RobotView {
    pub requests: Vec<ZoneRequest>,
    /// Map index and position, if the robot reported one.
    pub position: Option<(usize, f64, f64)>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct Entry {
    robot: usize,
    request: RequestId,
    zone: usize,
}

#[derive(Debug, Clone)]
pub struct ZoneGrants {
    zones: Zones,
    /// Waiting for a grant, first seen first.
    queue: Vec<Entry>,
    granted: Vec<Entry>,
    /// Requests that were told QUEUED already.
    told_queued: HashSet<(usize, RequestId)>,
    /// `(waiting robot, robot that keeps a zone from it)`, as of the last decision.
    kept_from: Vec<(usize, usize)>,
}

impl ZoneGrants {
    pub fn new(zones: Zones) -> ZoneGrants {
        ZoneGrants {
            zones,
            queue: Vec::new(),
            granted: Vec::new(),
            told_queued: HashSet::new(),
            kept_from: Vec::new(),
        }
    }

    /// The robots that kept a zone from `robot` at the last decision: those with a
    /// grant for a zone it waits for, and those standing inside it; sorted, each
    /// once.
    pub fn blockers(&self, robot: usize) -> Vec<usize> {
        let mut found: Vec<usize> = self
            .kept_from
            .iter()
            .filter(|(r, _)| *r == robot)
            .map(|&(_, b)| b)
            .collect();
        found.sort_unstable();
        found.dedup();
        found
    }

    /// Who holds which zone now: `(robot, zone index)`.
    pub fn granted(&self) -> Vec<(usize, usize)> {
        self.granted.iter().map(|g| (g.robot, g.zone)).collect()
    }

    /// Look at all robots and return the answers to send now.
    pub fn decide(&mut self, robots: &[RobotView]) -> Vec<ZoneAnswer> {
        // the live ACCESS requests: (robot, request id, zone index)
        let mut live: Vec<Entry> = Vec::new();
        for (i, r) in robots.iter().enumerate() {
            for q in &r.requests {
                if q.kind != ZoneRequestType::Access
                    || q.zone_set.as_str() != self.zones.zone_set_id
                {
                    continue;
                }
                let zones = &self.zones.zones;
                if let Some(z) = zones.iter().position(|z| z.zone_id == q.zone.as_str()) {
                    live.push(Entry {
                        robot: i,
                        request: q.id.clone(),
                        zone: z,
                    });
                }
            }
        }

        // what a robot no longer has is forgotten: it has left the zone or gave up
        self.queue.retain(|e| live.contains(e));
        self.granted.retain(|e| live.contains(e));
        self.told_queued
            .retain(|(r, id)| live.iter().any(|e| e.robot == *r && e.request == *id));

        // new requests join the queue, in the order they are seen
        for e in &live {
            if !self.queue.contains(e) && !self.granted.contains(e) {
                self.queue.push(e.clone());
            }
        }

        // robots standing in a zone, with or without a grant
        let inside: Vec<(usize, Vec<usize>)> = robots
            .iter()
            .enumerate()
            .map(|(i, r)| {
                let zones = match r.position {
                    Some((map, x, y)) if map == self.zones.map => self.zones.at(x, y),
                    _ => Vec::new(),
                };
                (i, zones)
            })
            .collect();

        // answer in queue order
        let mut answers = Vec::new();
        let waiting = std::mem::take(&mut self.queue);
        self.kept_from.clear();
        for e in waiting {
            let free = !self
                .granted
                .iter()
                .any(|g| g.zone == e.zone && g.robot != e.robot)
                && !inside
                    .iter()
                    .any(|(r, zs)| *r != e.robot && zs.contains(&e.zone));
            if free {
                answers.push(ZoneAnswer {
                    robot: e.robot,
                    request_id: e.request.clone(),
                    grant: Grant::Granted,
                });
                self.granted.push(e);
            } else {
                let holders = self
                    .granted
                    .iter()
                    .filter(|g| g.zone == e.zone)
                    .map(|g| g.robot);
                let standing = inside
                    .iter()
                    .filter(|(_, zs)| zs.contains(&e.zone))
                    .map(|(r, _)| *r);
                self.kept_from.extend(
                    holders
                        .chain(standing)
                        .filter(|&r| r != e.robot)
                        .map(|r| (e.robot, r)),
                );
                if self.told_queued.insert((e.robot, e.request.clone())) {
                    answers.push(ZoneAnswer {
                        robot: e.robot,
                        request_id: e.request.clone(),
                        grant: Grant::Queued,
                    });
                }
                self.queue.push(e);
            }
        }
        answers
    }
}

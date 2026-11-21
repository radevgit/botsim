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

use botsim_core::zones::Zones;
use vda5050::responses::GrantType;
use vda5050::state::{ZoneRequest, ZoneRequestType};

/// What the controller knows of one robot (from its state message).
#[derive(Debug, Clone)]
pub struct RobotView {
    pub requests: Vec<ZoneRequest>,
    /// Map index and position, if the robot reported one.
    pub position: Option<(usize, f64, f64)>,
}

/// One answer to send.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Answer {
    pub robot: usize,
    pub request_id: String,
    pub grant: GrantType,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct Entry {
    robot: usize,
    request: String,
    zone: usize,
}

#[derive(Debug, Clone)]
pub struct ZoneGrants {
    zones: Zones,
    /// Waiting for a grant, first seen first.
    queue: Vec<Entry>,
    granted: Vec<Entry>,
    /// Requests that were told QUEUED already.
    told_queued: HashSet<(usize, String)>,
}

impl ZoneGrants {
    pub fn new(zones: Zones) -> ZoneGrants {
        ZoneGrants {
            zones,
            queue: Vec::new(),
            granted: Vec::new(),
            told_queued: HashSet::new(),
        }
    }

    /// Who holds which zone now: `(robot, zone index)`.
    pub fn granted(&self) -> Vec<(usize, usize)> {
        self.granted.iter().map(|g| (g.robot, g.zone)).collect()
    }

    /// Look at all robots and return the answers to send now.
    pub fn decide(&mut self, robots: &[RobotView]) -> Vec<Answer> {
        // the live ACCESS requests: (robot, request id, zone index)
        let mut live: Vec<Entry> = Vec::new();
        for (i, r) in robots.iter().enumerate() {
            for q in &r.requests {
                if q.request_type != ZoneRequestType::Access
                    || q.zone_set_id != self.zones.zone_set_id
                {
                    continue;
                }
                if let Some(z) = self.zones.zones.iter().position(|z| z.zone_id == q.zone_id) {
                    live.push(Entry {
                        robot: i,
                        request: q.request_id.clone(),
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
        for e in waiting {
            let free = !self
                .granted
                .iter()
                .any(|g| g.zone == e.zone && g.robot != e.robot)
                && !inside
                    .iter()
                    .any(|(r, zs)| *r != e.robot && zs.contains(&e.zone));
            if free {
                answers.push(Answer {
                    robot: e.robot,
                    request_id: e.request.clone(),
                    grant: GrantType::Granted,
                });
                self.granted.push(e);
            } else {
                if self.told_queued.insert((e.robot, e.request.clone())) {
                    answers.push(Answer {
                        robot: e.robot,
                        request_id: e.request.clone(),
                        grant: GrantType::Queued,
                    });
                }
                self.queue.push(e);
            }
        }
        answers
    }
}

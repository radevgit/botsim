//! The robot side of RELEASE zone requests (VDA 5050 §6.4.3): **the robot states
//! its requests, takes the controller's answers, and does not enter a zone
//! before it is granted** (it waits at the start node of the entering edge,
//! S42).
//!
//! - Before an edge that **enters** a RELEASE zone, the robot adds a `zoneRequest`
//!   (`requestType` ACCESS, `requestStatus` REQUESTED) to its state. A request is
//!   needed even when the edge is released.
//! - **When** (S41; the standard leaves it to the robot, so it is a setting per
//!   robot type, [`ZoneRequestTiming`]):
//!   - `OnRelease` (default): when the entering edge is **released** (in the
//!     base). Only robots that are really coming ask; the risk is that the robot
//!     reaches the zone before the answer.
//!   - `OnHorizon`: as soon as the entering edge is **in the order**, also in the
//!     horizon. The controller sees demand early (e.g. to call a lift), but the
//!     order may still change, so requests can go stale.
//! - A request stays while an edge ahead still enters the zone (by the timing
//!   rule) or while the robot is **inside** the zone; once the robot has left it,
//!   or no edge ahead enters it any more, the request is removed.
//! - `requestId`s are unique per robot (`req-1`, `req-2`, …).
//! - An edge **enters** a zone if it starts outside it and reaches it
//!   ([`Zones::release_zones_entered`]); a straight edge is used.
//! - All zone sets count as enabled from the start (no `enableZoneSet` yet).
//!
//! - **Answers** ([`ZoneRequester::respond`]): GRANTED → status GRANTED and the
//!   robot may enter; QUEUED → status QUEUED, still waiting; REJECTED → the
//!   request is removed (the robot states a new one while it still wants the
//!   zone). The robot does not start an edge that enters a zone unless the
//!   request for it is GRANTED; it may turn towards the edge first.
//! - **Lease**: a GRANTED answer can carry a `leaseExpiry`; a later GRANTED answer
//!   with a new one extends it. When it runs out, or the controller answers
//!   REVOKED: **before the robot has entered** the zone the request is removed
//!   and the robot does not enter; **inside** the zone it has *lost its release*:
//!   status EXPIRED or REVOKED, and the zone's `releaseLossBehavior` applies
//!   (STOP if the zone has none):
//!   - `STOP`: the robot stops driving; error `ZONE_RELEASE_LOST` (CRITICAL) until
//!     it is granted again or has left;
//!   - `CONTINUE`: it drives on along its order; error `ZONE_RELEASE_LOSS_HANDLING`
//!     (WARNING) until it has left the zone or the order has ended inside it;
//!   - `EVACUATE`: the robot's own evacuation behaviour. **Simplification:** ours
//!     is the same as CONTINUE (it drives on along its order); it stops where the
//!     order ends. Error `ZONE_RELEASE_LOSS_HANDLING` as for CONTINUE.
//!
//!   The request stays in the state (status REVOKED or EXPIRED) until the zone has
//!   been left or the release is granted again. A robot that brakes after losing
//!   the release needs its braking distance; if it rolls out of the zone while
//!   braking, the loss is over (it has left) and it drives on.
//!
//! Not yet: `COORDINATED_REPLANNING`, retry delays after a rejection (the robot
//! asks again at once).

use std::collections::{HashMap, HashSet};

use vda5050::responses::GrantType;
use vda5050::state::{RequestStatus, ZoneRequest, ZoneRequestType};
use vda5050::zone_set::{ZoneKind, ZoneReleaseLossBehavior};

use crate::errors::{ErrorType, RobotError};
use crate::order_route::PlanEdge;
use crate::zones::Zones;
use botsim_model::graph::Graph;

/// When a robot states the request for a zone it is about to enter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ZoneRequestTiming {
    /// When the entering edge is released (S41 default).
    #[default]
    OnRelease,
    /// When the entering edge is in the order, even as horizon.
    OnHorizon,
}

#[derive(Debug, Clone, PartialEq)]
struct Request {
    id: String,
    zone: usize,
    status: RequestStatus,
    /// Until when the grant is valid (milliseconds since 1970); `None` = for ever.
    lease_ms: Option<i64>,
}

#[derive(Debug, Clone)]
pub struct ZoneRequester {
    zones: Zones,
    timing: ZoneRequestTiming,
    /// Edge id → the RELEASE zones that edge enters (only edges of the order).
    entering: HashMap<String, Vec<usize>>,
    /// (edge sequenceId, zone) pairs the robot has already entered.
    entered: HashSet<(u32, usize)>,
    requests: Vec<Request>,
    next_id: u32,
    /// Zones the robot was inside at the last update.
    inside: Vec<usize>,
    /// Did the order have edges ahead at the last update?
    has_ahead: bool,
}

impl ZoneRequester {
    pub fn new(zones: Zones, timing: ZoneRequestTiming) -> ZoneRequester {
        ZoneRequester {
            zones,
            timing,
            entering: HashMap::new(),
            entered: HashSet::new(),
            requests: Vec::new(),
            next_id: 1,
            inside: Vec::new(),
            has_ahead: false,
        }
    }

    pub fn set_timing(&mut self, timing: ZoneRequestTiming) {
        self.timing = timing;
    }

    pub fn timing(&self) -> ZoneRequestTiming {
        self.timing
    }

    /// A new order: forget the edges of the old one.
    pub fn forget_edges(&mut self) {
        self.entering.clear();
        self.entered.clear();
    }

    /// Learn which zones the edges of an order (or update) enter.
    pub fn learn_edges(&mut self, graph: &Graph, edges: &[PlanEdge]) {
        for e in edges {
            let Some(idx) = graph.edge_idx(&e.edge_id) else {
                continue;
            };
            let edge = &graph.edges[idx];
            let (a, b) = (&graph.nodes[edge.start], &graph.nodes[edge.end]);
            let zones = if a.map == self.zones.map && b.map == self.zones.map {
                self.zones.release_zones_entered((a.x, a.y), (b.x, b.y))
            } else {
                Vec::new()
            };
            self.entering.insert(e.edge_id.clone(), zones);
        }
    }

    /// Bring the requests up to date. `ahead` are the edges not yet left
    /// (the one being driven included), `pose` is where the robot is, `now_ms`
    /// the time (milliseconds since 1970) for lease expiry.
    pub fn update(&mut self, ahead: &[PlanEdge], pose_map: usize, x: f64, y: f64, now_ms: i64) {
        let inside_now: Vec<usize> = if pose_map == self.zones.map {
            self.zones.at(x, y)
        } else {
            Vec::new()
        };
        self.inside = inside_now.clone();
        self.has_ahead = !ahead.is_empty();

        // leases that have run out: inside the zone this is a release loss (status
        // EXPIRED), before it the robot must not enter and drops the request
        let zones_inside = inside_now.clone();
        self.requests.retain_mut(|r| {
            let expired =
                r.status == RequestStatus::Granted && r.lease_ms.is_some_and(|l| now_ms >= l);
            if !expired {
                return true;
            }
            if zones_inside.contains(&r.zone) {
                r.status = RequestStatus::Expired;
                true
            } else {
                false
            }
        });

        // The edge being driven (the first one ahead): a zone it enters that the
        // robot is inside now has been entered; the edge must not ask for it again
        // while the robot is still on that edge after leaving the zone.
        if let Some(first) = ahead.first() {
            for &z in self.entering.get(&first.edge_id).into_iter().flatten() {
                if inside_now.contains(&z) {
                    self.entered.insert((first.sequence_id, z));
                }
            }
        }

        // zones some edge ahead still enters, by the timing rule
        let mut wanted: Vec<usize> = Vec::new();
        for e in ahead {
            if self.timing == ZoneRequestTiming::OnRelease && !e.released {
                continue;
            }
            for &z in self.entering.get(&e.edge_id).into_iter().flatten() {
                if !wanted.contains(&z) && !self.entered.contains(&(e.sequence_id, z)) {
                    wanted.push(z);
                }
            }
        }

        // remove what is not needed any more
        self.requests
            .retain(|r| wanted.contains(&r.zone) || inside_now.contains(&r.zone));

        // state new requests (a zone the robot is already inside needs none)
        for z in wanted {
            if inside_now.contains(&z) || self.requests.iter().any(|r| r.zone == z) {
                continue;
            }
            self.requests.push(Request {
                id: format!("req-{}", self.next_id),
                zone: z,
                status: RequestStatus::Requested,
                lease_ms: None,
            });
            self.next_id += 1;
        }
    }

    /// The controller's answer to a request (an answer to a request the robot
    /// does not have is ignored). `lease_ms` is the lease expiry of a GRANTED
    /// answer (milliseconds since 1970); a new GRANTED answer with another expiry
    /// extends the lease.
    /// - GRANTED: the robot may enter (also after a release loss: the release is
    ///   granted again).
    /// - QUEUED: only an acknowledgement; the robot goes on waiting. It changes
    ///   nothing for a request that is already granted, revoked or expired.
    /// - REJECTED: the robot must not enter and **removes the request**. While it
    ///   still wants the zone it states a new request (new id) at its next update.
    /// - REVOKED: before the robot has entered, it removes the request; inside the
    ///   zone it has lost its release: status REVOKED and the zone's
    ///   `releaseLossBehavior` applies ([`ZoneRequester::halt`],
    ///   [`ZoneRequester::errors`]).
    pub fn respond(&mut self, request_id: &str, grant: GrantType, lease_ms: Option<i64>) {
        let Some(pos) = self.requests.iter().position(|r| r.id == request_id) else {
            return;
        };
        let inside = self.inside.contains(&self.requests[pos].zone);
        let r = &mut self.requests[pos];
        match grant {
            GrantType::Granted => {
                r.status = RequestStatus::Granted;
                r.lease_ms = lease_ms;
            }
            GrantType::Queued => {
                if r.status == RequestStatus::Requested {
                    r.status = RequestStatus::Queued;
                }
            }
            GrantType::Rejected => {
                if r.status != RequestStatus::Granted {
                    self.requests.remove(pos);
                }
            }
            GrantType::Revoked => {
                if inside {
                    r.status = RequestStatus::Revoked;
                } else {
                    self.requests.remove(pos);
                }
            }
        }
    }

    /// What the robot does after losing a release inside a RELEASE zone (the
    /// zone's `releaseLossBehavior`; STOP if the zone has none).
    fn loss_behavior(&self, zone: usize) -> ZoneReleaseLossBehavior {
        match &self.zones.zones[zone].kind {
            ZoneKind::Release {
                release_loss_behavior: Some(b),
            } => *b,
            _ => ZoneReleaseLossBehavior::Stop,
        }
    }

    fn lost(&self) -> impl Iterator<Item = &Request> {
        self.requests
            .iter()
            .filter(|r| matches!(r.status, RequestStatus::Revoked | RequestStatus::Expired))
    }

    /// Must the robot stand still? Yes while it has lost the release of a zone
    /// it is in and the zone says `STOP` (or says nothing). `CONTINUE` and
    /// `EVACUATE` let it drive on.
    pub fn halt(&self) -> bool {
        self.lost()
            .any(|r| self.loss_behavior(r.zone) == ZoneReleaseLossBehavior::Stop)
    }

    /// The errors of a release loss, for the state: `ZONE_RELEASE_LOST` (CRITICAL)
    /// for STOP; `ZONE_RELEASE_LOSS_HANDLING` (WARNING) for CONTINUE and
    /// EVACUATE, the latter only while the order still has edges ahead (it ends
    /// when the order has ended inside the zone). References: `zoneId`, `requestId`.
    pub fn errors(&self) -> Vec<RobotError> {
        self.lost()
            .filter_map(|r| {
                let error_type = match self.loss_behavior(r.zone) {
                    ZoneReleaseLossBehavior::Stop => ErrorType::ZoneReleaseLost,
                    _ if self.has_ahead => ErrorType::ZoneReleaseLossHandling,
                    _ => return None,
                };
                Some(RobotError {
                    error_type,
                    references: vec![
                        ("zoneId".into(), self.zones.zones[r.zone].zone_id.clone()),
                        ("requestId".into(), r.id.clone()),
                    ],
                    description: Some("the release of the zone was lost".into()),
                })
            })
            .collect()
    }

    /// The edges (by `sequenceId`) among `ahead` that the robot must not start:
    /// they enter a zone, by the timing rule, that is not granted.
    pub fn blocked_edges(&self, ahead: &[PlanEdge]) -> HashSet<u32> {
        let mut blocked = HashSet::new();
        for e in ahead {
            for &z in self.entering.get(&e.edge_id).into_iter().flatten() {
                if self.entered.contains(&(e.sequence_id, z)) {
                    continue;
                }
                let granted = self
                    .requests
                    .iter()
                    .any(|r| r.zone == z && r.status == RequestStatus::Granted);
                if !granted {
                    blocked.insert(e.sequence_id);
                }
            }
        }
        blocked
    }

    /// The `zoneRequests` of the state message.
    pub fn to_state(&self) -> Vec<ZoneRequest> {
        self.requests
            .iter()
            .map(|r| ZoneRequest {
                request_id: r.id.clone(),
                request_type: ZoneRequestType::Access,
                zone_id: self.zones.zones[r.zone].zone_id.clone(),
                zone_set_id: self.zones.zone_set_id.clone(),
                request_status: r.status,
                trajectory: None,
            })
            .collect()
    }
}

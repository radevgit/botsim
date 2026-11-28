//! The "waiting caused by others" figures of the summary (brief U12; S167 to S170):
//! robot-seconds robots waited because of another robot, added up from
//! [`botsim_sim::world::World::wait_totals`]. The reasons and who is named are in
//! [`botsim_sim::waiting`].

use std::cmp::Reverse;
use std::collections::BTreeMap;

use botsim_model::waiting::WaitReason;
use botsim_sim::waiting::Place;
use botsim_sim::world::World;
use serde::Serialize;

/// How many pairs and places are listed (S169).
pub const TOP: usize = 10;

/// All times in robot-seconds.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct WaitingSummary {
    /// All waiting, with or without a known blocker.
    pub total_seconds: f64,
    /// The part of the total for which no blocker could be named.
    pub unknown_seconds: f64,
    pub by_reason: WaitingByReason,
    /// Every robot, in the order of the scenario.
    pub robots: Vec<WaitingRobot>,
    /// The [`TOP`] pairs with the most waiting of `waiter` for `blocker`, longest
    /// first (waits with an unknown blocker are in no pair).
    pub top_pairs: Vec<WaitingPair>,
    /// The [`TOP`] places with the most waiting, longest first.
    pub top_places: Vec<WaitingPlace>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct WaitingByReason {
    pub release: f64,
    pub zone: f64,
    pub lift: f64,
    pub door: f64,
    pub safety_stop: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct WaitingRobot {
    /// `manufacturer/serialNumber`.
    pub robot: String,
    /// Time other robots waited for this one.
    pub caused_seconds: f64,
    /// Time this robot waited, the unknown part included.
    pub suffered_seconds: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct WaitingPair {
    pub blocker: String,
    pub waiter: String,
    pub seconds: f64,
}

/// A node or an edge of the layout where robots stood while they waited.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct WaitingPlace {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub node_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub edge_id: Option<String>,
    pub seconds: f64,
}

/// The `TOP` entries with the most steps; equal ones in the order of their keys.
fn top<K: Ord + Copy>(totals: &BTreeMap<K, u64>) -> Vec<(K, u64)> {
    let mut list: Vec<(K, u64)> = totals.iter().map(|(&k, &t)| (k, t)).collect();
    list.sort_by_key(|&(k, t)| (Reverse(t), k));
    list.truncate(TOP);
    list
}

/// The figures of the run so far.
pub fn waiting_summary(world: &World) -> WaitingSummary {
    let names: Vec<String> = world
        .robot_ids()
        .map(|id| format!("{}/{}", id.manufacturer, id.serial_number))
        .collect();
    let name = |i: usize| names.get(i).cloned().unwrap_or_default();
    let (mut total, mut unknown) = (0u64, 0u64);
    let mut reasons: BTreeMap<WaitReason, u64> = BTreeMap::new();
    let mut caused = vec![0u64; names.len()];
    let mut suffered = vec![0u64; names.len()];
    let mut pairs: BTreeMap<(usize, usize), u64> = BTreeMap::new();
    let mut places: BTreeMap<Place, u64> = BTreeMap::new();
    for w in world.wait_totals() {
        total += w.ticks;
        *reasons.entry(w.reason).or_insert(0) += w.ticks;
        *places.entry(w.place).or_insert(0) += w.ticks;
        if let Some(s) = suffered.get_mut(w.waiter) {
            *s += w.ticks;
        }
        match w.blocker {
            Some(b) => {
                if let Some(c) = caused.get_mut(b) {
                    *c += w.ticks;
                }
                *pairs.entry((b, w.waiter)).or_insert(0) += w.ticks;
            }
            None => unknown += w.ticks,
        }
    }
    // steps to seconds, rounded to a microsecond like the other times
    let dt = world.dt();
    let seconds = |ticks: u64| (ticks as f64 * dt * 1e6).round() / 1e6;
    let reason = |r: WaitReason| seconds(reasons.get(&r).copied().unwrap_or(0));
    let graph = world.graph();
    WaitingSummary {
        total_seconds: seconds(total),
        unknown_seconds: seconds(unknown),
        by_reason: WaitingByReason {
            release: reason(WaitReason::Release),
            zone: reason(WaitReason::Zone),
            lift: reason(WaitReason::Lift),
            door: reason(WaitReason::Door),
            safety_stop: reason(WaitReason::SafetyStop),
        },
        robots: (0..names.len())
            .map(|i| WaitingRobot {
                robot: name(i),
                caused_seconds: seconds(caused[i]),
                suffered_seconds: seconds(suffered[i]),
            })
            .collect(),
        top_pairs: top(&pairs)
            .into_iter()
            .map(|((blocker, waiter), t)| WaitingPair {
                blocker: name(blocker),
                waiter: name(waiter),
                seconds: seconds(t),
            })
            .collect(),
        top_places: top(&places)
            .into_iter()
            .map(|(place, t)| {
                let (node_id, edge_id) = match place {
                    Place::Node(n) => (graph.nodes.get(n).map(|n| n.id.clone()), None),
                    Place::Edge(e) => (None, graph.edges.get(e).map(|e| e.id.clone())),
                };
                WaitingPlace {
                    node_id,
                    edge_id,
                    seconds: seconds(t),
                }
            })
            .collect(),
    }
}

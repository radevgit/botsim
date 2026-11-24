//! How much each road of the layout was used in a run, from the recording.

use botsim_record::{LayoutLine, Recording, StatusRecord};
use serde::Deserialize;

/// What a recording says about the use of the roads. The numbers are simulated seconds, summed
/// over the snapshots (each counts until the next one).
#[derive(Debug, Clone, PartialEq)]
pub struct Usage {
    /// The time counted (s): the whole recording, or the window.
    pub duration: f64,
    /// For each edge of the layout: how long some robot had it reserved (a robot at a time).
    pub edge_seconds: Vec<f64>,
    /// For each node: how long robots stood there waiting (`WaitingForRelease`, `Obstructed`),
    /// at the node they last passed.
    pub node_wait_seconds: Vec<f64>,
}

/// Count the use of every edge and the waiting at every node over the whole recording.
pub fn usage(recording: &Recording) -> Usage {
    usage_between(recording, f64::NEG_INFINITY, f64::INFINITY)
}

/// The same for the time from `from` to `to` (simulated seconds): a run that freezes after a
/// while says most about the roads in its first part.
pub fn usage_between(recording: &Recording, from: f64, to: f64) -> Usage {
    let layout = &recording.layout;
    let mut edge_seconds = vec![0.0; layout.edges.len()];
    let mut node_wait_seconds = vec![0.0; layout.nodes.len()];
    let snaps = &recording.snapshots;
    let mut previous_dt = 0.0;
    let mut covered = 0.0;
    for (k, snap) in snaps.iter().enumerate() {
        let full = snaps.get(k + 1).map_or(previous_dt, |next| next.time - snap.time);
        previous_dt = full;
        // (a snapshot counts until the next one, cut to the window)
        let dt = (snap.time + full).min(to) - snap.time.max(from);
        if dt <= 0.0 {
            continue;
        }
        covered += dt;
        for reservation in &snap.reservations {
            for &e in &reservation.edges {
                if let Some(slot) = edge_seconds.get_mut(e) {
                    *slot += dt;
                }
            }
        }
        for robot in &snap.robots {
            let waiting = matches!(
                robot.status,
                StatusRecord::WaitingForRelease | StatusRecord::Obstructed
            );
            if let (true, Some(node)) = (waiting, robot.last_node)
                && let Some(slot) = node_wait_seconds.get_mut(node)
            {
                *slot += dt;
            }
        }
    }
    Usage {
        duration: covered,
        edge_seconds,
        node_wait_seconds,
    }
}

impl Usage {
    /// The share of the run (0 to 1) that the edge was reserved.
    pub fn edge_share(&self, edge: usize) -> f64 {
        if self.duration <= 0.0 {
            return 0.0;
        }
        self.edge_seconds.get(edge).copied().unwrap_or(0.0) / self.duration
    }

    /// The share of the run that robots waited at the node, counted for all robots together (it
    /// can be over 1 if several waited at once).
    pub fn node_wait_share(&self, node: usize) -> f64 {
        if self.duration <= 0.0 {
            return 0.0;
        }
        self.node_wait_seconds.get(node).copied().unwrap_or(0.0) / self.duration
    }

    /// The biggest edge share.
    pub fn max_edge_share(&self) -> f64 {
        (0..self.edge_seconds.len())
            .map(|e| self.edge_share(e))
            .fold(0.0, f64::max)
    }

    /// The biggest waiting share of a node.
    pub fn max_node_wait_share(&self) -> f64 {
        (0..self.node_wait_seconds.len())
            .map(|n| self.node_wait_share(n))
            .fold(0.0, f64::max)
    }
}

/// A group of roads for the table, as a list of boxes `[x0, y0, x1, y1]` (m): an edge belongs
/// to the first group, in the order of the file, that has a box with the middle of the edge
/// (the mean of its two nodes) in it. The file is a JSON array of
/// `{"name": "...", "boxes": [[x0, y0, x1, y1], ...]}`; edges that fit no group are "other".
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Group {
    pub name: String,
    pub boxes: Vec<[f64; 4]>,
}

/// Read the groups from the text of a groups file.
pub fn groups_from_json(text: &str) -> Result<Vec<Group>, String> {
    serde_json::from_str(text).map_err(|e| e.to_string())
}

/// One row of the table of the groups.
#[derive(Debug, Clone, PartialEq)]
pub struct GroupRow {
    pub name: String,
    pub edges: usize,
    /// The mean over the edges of the share of the run they were reserved.
    pub mean_share: f64,
    /// Robot-edge-seconds: all the time any robot had an edge of the group reserved.
    pub seconds: f64,
    /// The biggest share of one edge of the group.
    pub max_share: f64,
}

/// The use of each group, the most used (by mean share) first; "other" for the edges that fit
/// no group, if there are any. Edges on other floors than the first are counted by their
/// coordinates too.
pub fn group_table(layout: &LayoutLine, usage: &Usage, groups: &[Group]) -> Vec<GroupRow> {
    let mut rows: Vec<GroupRow> = groups
        .iter()
        .map(|g| GroupRow {
            name: g.name.clone(),
            edges: 0,
            mean_share: 0.0,
            seconds: 0.0,
            max_share: 0.0,
        })
        .collect();
    let mut other = GroupRow {
        name: "other".into(),
        edges: 0,
        mean_share: 0.0,
        seconds: 0.0,
        max_share: 0.0,
    };
    for (e, edge) in layout.edges.iter().enumerate() {
        let (Some(a), Some(b)) = (layout.nodes.get(edge.start), layout.nodes.get(edge.end)) else {
            continue;
        };
        let (mx, my) = ((a.x + b.x) / 2.0, (a.y + b.y) / 2.0);
        let at = groups.iter().position(|g| {
            g.boxes
                .iter()
                .any(|[x0, y0, x1, y1]| (*x0..=*x1).contains(&mx) && (*y0..=*y1).contains(&my))
        });
        let row = match at {
            Some(i) => &mut rows[i],
            None => &mut other,
        };
        let share = usage.edge_share(e);
        row.edges += 1;
        row.mean_share += share;
        row.seconds += usage.edge_seconds.get(e).copied().unwrap_or(0.0);
        row.max_share = row.max_share.max(share);
    }
    rows.push(other);
    rows.retain(|r| r.edges > 0);
    for r in &mut rows {
        r.mean_share /= r.edges as f64;
    }
    rows.sort_by(|a, b| b.mean_share.total_cmp(&a.mean_share));
    rows
}

/// The `count` busiest edges as `(edge id, share of the run reserved)`, the busiest first.
pub fn busiest_edges(layout: &LayoutLine, usage: &Usage, count: usize) -> Vec<(String, f64)> {
    let mut all: Vec<(usize, f64)> = (0..layout.edges.len())
        .map(|e| (e, usage.edge_share(e)))
        .collect();
    all.sort_by(|a, b| b.1.total_cmp(&a.1));
    all.into_iter()
        .take(count)
        .map(|(e, share)| (layout.edges[e].id.clone(), share))
        .collect()
}

/// The `count` nodes where robots waited most as `(node id, waiting share)`, the most first.
pub fn most_waited_nodes(layout: &LayoutLine, usage: &Usage, count: usize) -> Vec<(String, f64)> {
    let mut all: Vec<(usize, f64)> = (0..layout.nodes.len())
        .map(|n| (n, usage.node_wait_share(n)))
        .collect();
    all.sort_by(|a, b| b.1.total_cmp(&a.1));
    all.into_iter()
        .take(count)
        .filter(|(_, share)| *share > 0.0)
        .map(|(n, share)| (layout.nodes[n].id.clone(), share))
        .collect()
}

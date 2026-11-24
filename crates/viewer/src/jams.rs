//! How many robots are in each status at each moment of a run (the stacked strip).

use botsim_record::{Recording, StatusRecord};

/// Every status a robot can have, in the order of the recording's enum.
pub const ALL_STATUSES: [StatusRecord; 10] = [
    StatusRecord::Idle,
    StatusRecord::Driving,
    StatusRecord::Turning,
    StatusRecord::Acting,
    StatusRecord::WaitingForRelease,
    StatusRecord::WaitingForZone,
    StatusRecord::WaitingForLift,
    StatusRecord::RidingLift,
    StatusRecord::Held,
    StatusRecord::Obstructed,
];

/// The colour of a status (red, green, blue), the same in the strip and in the table.
pub fn status_colour(status: StatusRecord) -> [u8; 3] {
    match status {
        StatusRecord::Idle => [170, 170, 170],
        StatusRecord::Driving => [60, 170, 90],
        StatusRecord::Turning => [30, 150, 160],
        StatusRecord::Acting => [60, 110, 210],
        StatusRecord::WaitingForRelease => [240, 150, 30],
        StatusRecord::WaitingForZone => [235, 200, 40],
        StatusRecord::WaitingForLift => [150, 90, 200],
        StatusRecord::RidingLift => [200, 120, 220],
        StatusRecord::Held => [140, 100, 60],
        StatusRecord::Obstructed => [220, 30, 30],
    }
}

/// The number of robots in each status (index as in [`ALL_STATUSES`]) at one snapshot.
#[derive(Debug, Clone, PartialEq)]
pub struct StatusPoint {
    pub time: f64,
    pub counts: [usize; 10],
}

/// The counts at each snapshot of the recording.
pub fn status_series(recording: &Recording) -> Vec<StatusPoint> {
    recording
        .snapshots
        .iter()
        .map(|s| {
            let mut counts = [0usize; 10];
            for r in &s.robots {
                if let Some(k) = ALL_STATUSES.iter().position(|&a| a == r.status) {
                    counts[k] += 1;
                }
            }
            StatusPoint {
                time: s.time,
                counts,
            }
        })
        .collect()
}

/// The point at `time`: the last one at or before it (the first if none is).
pub fn point_at(series: &[StatusPoint], time: f64) -> Option<&StatusPoint> {
    let k = series.partition_point(|p| p.time <= time + 1e-9);
    series.get(k.saturating_sub(1))
}

//! The botsim replay viewer: the parts that need no window.

mod info;
mod jams;
mod playback;
mod select;
mod usage;

pub use info::{RobotInfo, describe};
pub use jams::{ALL_STATUSES, StatusPoint, point_at, status_colour, status_series};
pub use playback::{Playback, SPEEDS};
pub use select::{PICK_TOLERANCE_PX, on_floor, pick_robot};
pub use usage::{
    Group, GroupRow, Usage, busiest_edges, group_table, groups_from_json, most_waited_nodes, usage,
    usage_between,
};

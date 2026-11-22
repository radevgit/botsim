//! The botsim replay viewer: the parts that need no window.

mod info;
mod playback;
mod select;

pub use info::{RobotInfo, describe};
pub use playback::{Playback, SPEEDS};
pub use select::{PICK_TOLERANCE_PX, on_floor, pick_robot};

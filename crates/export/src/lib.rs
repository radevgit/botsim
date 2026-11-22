//! Headless PNG frames from a botsim recording (V4): no window, no GPU. Draws
//! with tiny-skia; the colours are in [`palette`] and are the viewer's too.

mod clock;
mod export;
pub mod palette;
mod render;
mod sample;
mod view;

pub use clock::{Clock, NoFont};
pub use export::{
    ExportError, ExportOptions, ExportSummary, export_frames, frame_count, frame_path,
};
pub use render::Renderer;
pub use sample::sample;
pub use view::{View, ViewError};

/// Default image size of a video frame (YouTube 1080p).
pub const DEFAULT_WIDTH: u32 = 1920;
pub const DEFAULT_HEIGHT: u32 = 1080;
/// Default video frame rate.
pub const DEFAULT_FPS: u32 = 30;

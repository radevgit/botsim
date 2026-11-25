//! Writing the numbered PNG frames of a recording.

use std::fmt;
use std::path::{Path, PathBuf};

use botsim_record::Recording;

use crate::{DEFAULT_FPS, DEFAULT_HEIGHT, DEFAULT_WIDTH, Renderer, ViewError, sample};

/// What to draw and at what rate.
#[derive(Debug, Clone, PartialEq)]
pub struct ExportOptions {
    /// The floor, by its `mapId`; the first map of the layout if `None`.
    pub map: Option<String>,
    pub width: u32,
    pub height: u32,
    /// Video frames per second.
    pub fps: u32,
    /// Simulated seconds per video second (10 = ten times faster than real time).
    pub speed: f64,
    /// Simulated time of the first frame; the first snapshot if `None`.
    pub from: Option<f64>,
    /// Simulated time after which no frame is made; the last snapshot if `None`.
    pub to: Option<f64>,
    /// Draw the simulated time in a corner of every frame.
    pub clock: bool,
    /// Draw the waits (who waits for whom, and why) instead of the blocking arrows.
    pub waits: bool,
}

impl Default for ExportOptions {
    fn default() -> Self {
        ExportOptions {
            map: None,
            width: DEFAULT_WIDTH,
            height: DEFAULT_HEIGHT,
            fps: DEFAULT_FPS,
            speed: 1.0,
            from: None,
            to: None,
            clock: true,
            waits: false,
        }
    }
}

#[derive(Debug)]
pub enum ExportError {
    /// The recording has no snapshots, so there is nothing to draw.
    NoSnapshots,
    UnknownMap {
        map: String,
        available: Vec<String>,
    },
    View(ViewError),
    /// An option has a value that cannot work (message says which).
    BadOption(String),
    /// A frame or the directory could not be written.
    Write {
        path: PathBuf,
        message: String,
    },
}

impl fmt::Display for ExportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ExportError::NoSnapshots => write!(f, "the recording has no snapshots"),
            ExportError::UnknownMap { map, available } => write!(
                f,
                "the recording has no map '{map}' (maps: {})",
                available.join(", ")
            ),
            ExportError::View(e) => write!(f, "{e}"),
            ExportError::BadOption(m) => write!(f, "{m}"),
            ExportError::Write { path, message } => {
                write!(f, "cannot write {}: {message}", path.display())
            }
        }
    }
}

impl std::error::Error for ExportError {}

/// What `export_frames` did.
#[derive(Debug, Clone, PartialEq)]
pub struct ExportSummary {
    pub frames: usize,
    pub width: u32,
    pub height: u32,
    pub map_id: String,
    /// Simulated time of the first frame and the end of the range.
    pub from: f64,
    pub to: f64,
    pub fps: u32,
    pub speed: f64,
}

/// The file of frame number `index` (counted from 0): `frame_000001.png` for 0.
/// Six digits; the ffmpeg pattern is `frame_%06d.png`.
pub fn frame_path(dir: &Path, index: usize) -> PathBuf {
    dir.join(format!("frame_{:06}.png", index + 1))
}

/// How many frames cover `from` … `to` (simulated seconds): one at `from`, then
/// one every `speed / fps` seconds while not past `to`.
pub fn frame_count(from: f64, to: f64, fps: u32, speed: f64) -> usize {
    if to < from {
        return 0;
    }
    // the small allowance keeps a range of exactly n frame steps at n + 1 frames
    ((to - from) * fps as f64 / speed + 1e-9).floor() as usize + 1
}

fn check(o: &ExportOptions) -> Result<(), ExportError> {
    let bad = |m: &str| Err(ExportError::BadOption(m.into()));
    if o.width == 0 || o.height == 0 {
        return bad("the image size must be at least 1×1");
    }
    if !o.width.is_multiple_of(2) || !o.height.is_multiple_of(2) {
        return bad("width and height must be even (ffmpeg yuv420p needs it)");
    }
    if o.fps == 0 {
        return bad("fps must be above 0");
    }
    if !(o.speed.is_finite() && o.speed > 0.0) {
        return bad("speed must be above 0");
    }
    for (name, v) in [("from", o.from), ("to", o.to)] {
        if v.is_some_and(|v| !v.is_finite()) {
            return Err(ExportError::BadOption(format!("{name} must be a number")));
        }
    }
    Ok(())
}

/// Draw the frames of `recording` into `out_dir` (created if missing) as
/// `frame_000001.png`, `frame_000002.png`, ...; existing files of those names are
/// replaced, other files stay. The same recording and options give the same
/// files.
pub fn export_frames(
    recording: &Recording,
    out_dir: &Path,
    options: &ExportOptions,
) -> Result<ExportSummary, ExportError> {
    check(options)?;
    let (Some(first), Some(last)) = (recording.snapshots.first(), recording.snapshots.last())
    else {
        return Err(ExportError::NoSnapshots);
    };
    let layout = &recording.layout;
    let map = match &options.map {
        None => 0,
        Some(id) => {
            layout
                .maps
                .iter()
                .position(|m| m == id)
                .ok_or_else(|| ExportError::UnknownMap {
                    map: id.clone(),
                    available: layout.maps.clone(),
                })?
        }
    };
    let mut renderer = Renderer::new(layout, map, options.width, options.height)
        .map_err(ExportError::View)?
        .with_clock(options.clock)
        // the name `botsim` is in every frame
        .and_then(|r| r.with_brand(true))
        .map_err(|e| ExportError::BadOption(e.to_string()))?;
    renderer.set_waits(options.waits);

    let from = options.from.unwrap_or(first.time);
    let to = options.to.unwrap_or(last.time);
    if to < from {
        return Err(ExportError::BadOption(format!(
            "to ({to}) is before from ({from})"
        )));
    }
    let frames = frame_count(from, to, options.fps, options.speed);

    std::fs::create_dir_all(out_dir).map_err(|e| ExportError::Write {
        path: out_dir.to_path_buf(),
        message: e.to_string(),
    })?;
    for k in 0..frames {
        let t = from + k as f64 * options.speed / options.fps as f64;
        let snapshot = sample(&recording.snapshots, t).expect("there are snapshots");
        let path = frame_path(out_dir, k);
        renderer
            .frame(&snapshot)
            .save_png(&path)
            .map_err(|e| ExportError::Write {
                path,
                message: e.to_string(),
            })?;
    }
    Ok(ExportSummary {
        frames,
        width: options.width,
        height: options.height,
        map_id: layout.maps[map].clone(),
        from,
        to,
        fps: options.fps,
        speed: options.speed,
    })
}

//! `botsim-export <recording.jsonl> <out-dir> [--map <mapId>] [--size 1920x1080]
//! [--fps 30] [--speed 1] [--from <s>] [--to <s>] [--no-clock]`: draw the frames of
//! a recording as numbered PNGs (`frame_000001.png`, ...), for a video:
//!
//! ```text
//! ffmpeg -framerate 30 -i out-dir/frame_%06d.png -c:v libx264 -preset slow -crf 18 \
//!   -vf "scale=out_color_matrix=bt709:out_range=tv,format=yuv420p" -colorspace bt709 \
//!   -movflags +faststart out.mp4
//! ```
//!
//! (the options are explained in the README of the crate)
//!
//! Frame k shows the simulated time `from + k × speed / fps`; `--speed 10` makes
//! the video ten times faster than the run. Errors go to stderr with exit code 1,
//! a wrong command line gives exit code 2.

use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;
use std::process::ExitCode;

use botsim_export::{ExportOptions, export_frames};
use botsim_record::read_recording_partial;

const USAGE: &str = "usage: botsim-export <recording.jsonl> <out-dir> [--map <mapId>] \
[--size WxH] [--fps N] [--speed X] [--from <seconds>] [--to <seconds>] [--no-clock]";

#[derive(Debug)]
struct Args {
    recording: PathBuf,
    out_dir: PathBuf,
    options: ExportOptions,
}

/// The value of an option, parsed.
fn value<T: std::str::FromStr>(
    it: &mut std::slice::Iter<'_, String>,
    option: &str,
    what: &str,
) -> Result<T, String> {
    let text = it.next().ok_or(format!("{option} needs {what}"))?;
    text.parse()
        .map_err(|_| format!("bad {option} '{text}' ({what})"))
}

fn parse_size(text: &str) -> Option<(u32, u32)> {
    let (w, h) = text.split_once('x')?;
    Some((w.parse().ok()?, h.parse().ok()?))
}

fn parse_args(args: &[String]) -> Result<Args, String> {
    let mut options = ExportOptions::default();
    let mut positional = Vec::new();
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--map" => options.map = Some(value::<String>(&mut it, "--map", "a map id")?),
            "--size" => {
                let text: String = value(&mut it, "--size", "WIDTHxHEIGHT, e.g. 1920x1080")?;
                let (w, h) = parse_size(&text).ok_or(format!(
                    "bad --size '{text}' (WIDTHxHEIGHT, e.g. 1920x1080)"
                ))?;
                (options.width, options.height) = (w, h);
            }
            "--fps" => options.fps = value(&mut it, "--fps", "frames per second")?,
            "--speed" => options.speed = value(&mut it, "--speed", "a number above 0")?,
            "--from" => options.from = Some(value(&mut it, "--from", "seconds")?),
            "--to" => options.to = Some(value(&mut it, "--to", "seconds")?),
            "--no-clock" => options.clock = false,
            option if option.starts_with("--") => return Err(format!("unknown option {option}")),
            path => positional.push(PathBuf::from(path)),
        }
    }
    let [recording, out_dir] = <[PathBuf; 2]>::try_from(positional.clone()).map_err(|_| {
        let seen: Vec<String> = positional.iter().map(|p| format!("'{}'", p.display())).collect();
        format!(
            "needs a recording and an output directory, and nothing else without a -- name; got {}: {}",
            positional.len(),
            seen.join(" ")
        )
    })?;
    Ok(Args {
        recording,
        out_dir,
        options,
    })
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let args = match parse_args(&args) {
        Ok(a) => a,
        Err(message) => {
            if !message.is_empty() {
                eprintln!("{message}");
            }
            eprintln!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    match run(&args) {
        Ok(line) => {
            println!("{line}");
            ExitCode::SUCCESS
        }
        Err(message) => {
            eprintln!("error: {message}");
            ExitCode::from(1)
        }
    }
}

fn run(args: &Args) -> Result<String, String> {
    let file = File::open(&args.recording)
        .map_err(|e| format!("cannot read {}: {e}", args.recording.display()))?;
    // (a run that was stopped leaves a recording whose last line is cut off: the viewer shows
    // it, so the export draws what there is)
    let (recording, cut) = read_recording_partial(BufReader::new(file))
        .map_err(|e| format!("{}: {e}", args.recording.display()))?;
    if cut {
        let last = recording.snapshots.last().map_or(0.0, |s| s.time);
        eprintln!(
            "note: {} is cut off (the run was stopped while it was written): drawing what there is, up to {last:.1} s",
            args.recording.display()
        );
    }
    if let (Some(to), Some(last)) = (args.options.to, recording.snapshots.last()) {
        if to > last.time {
            eprintln!(
                "note: --to {to} is after the end of the recording ({:.1} s): the frames after it all show the last snapshot",
                last.time
            );
        }
    }
    let s = export_frames(&recording, &args.out_dir, &args.options).map_err(|e| e.to_string())?;
    Ok(format!(
        "wrote {} frames {}x{} of map {} to {} (simulated {:.1} s to {:.1} s, {} fps, speed {})",
        s.frames,
        s.width,
        s.height,
        s.map_id,
        args.out_dir.display(),
        s.from,
        s.to,
        s.fps,
        s.speed
    ))
}

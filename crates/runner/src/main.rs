//! `botsim [--timing] [--record <file.jsonl> [--record-interval <seconds>]
//! [--geometry <geometry.json>]] <scenario.toml>`: run a scenario headless and print the JSON summary on
//! stdout. Errors go to stderr with exit code 1; a wrong command line gives exit
//! code 2.
//!
//! `--timing` adds the real time the controller took per step (it differs from
//! run to run, so without the flag the output of a scenario is always identical).
//!
//! `--record <file.jsonl>` also writes a recording of the run (JSON Lines, see
//! `botsim-record`) for the viewer and the PNG exporter, one snapshot every
//! `--record-interval` simulated seconds (default 0.1). The summary on stdout is
//! the same with or without it. `--geometry` adds static geometry (walls, racks,
//! ...; a `botsim-record` geometry file) to the recording's layout line; the
//! file is checked against the scenario's maps before the run starts. The layout
//! line also lists the type of every robot (`manufacturer/seriesName`), so that a
//! viewer can pick the curve a robot follows on an edge.

use std::fs::File;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use botsim::scenario::Scenario;
use botsim::summary;
use botsim_record::{GeometryFile, LayoutLine, LayoutRobot, Recorder};

const USAGE: &str = "usage: botsim [--timing] [--record <file.jsonl> \
[--record-interval <seconds>] [--geometry <geometry.json>]] <scenario.toml>";

/// Snapshot interval of a recording, simulated seconds.
const DEFAULT_RECORD_INTERVAL: f64 = 0.1;

#[derive(Debug)]
struct Options {
    scenario: PathBuf,
    timing: bool,
    record: Option<PathBuf>,
    /// Static geometry file (walls, racks, ...) for the recording.
    geometry: Option<PathBuf>,
    record_interval: f64,
}

fn parse_args(args: &[String]) -> Result<Options, String> {
    let mut timing = false;
    let mut record = None;
    let mut record_interval = None;
    let mut geometry = None;
    let mut scenario = Vec::new();
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--timing" => timing = true,
            "--record" => {
                let value = it.next().ok_or("--record needs a file name")?;
                record = Some(PathBuf::from(value));
            }
            "--geometry" => {
                let value = it.next().ok_or("--geometry needs a file name")?;
                geometry = Some(PathBuf::from(value));
            }
            "--record-interval" => {
                let value = it
                    .next()
                    .ok_or("--record-interval needs a number of seconds")?;
                match value.parse::<f64>() {
                    Ok(s) if s.is_finite() && s > 0.0 => record_interval = Some(s),
                    _ => return Err(format!("bad --record-interval '{value}' (seconds above 0)")),
                }
            }
            option if option.starts_with("--") => return Err(format!("unknown option {option}")),
            path => scenario.push(PathBuf::from(path)),
        }
    }
    if record_interval.is_some() && record.is_none() {
        return Err("--record-interval needs --record".into());
    }
    if geometry.is_some() && record.is_none() {
        return Err("--geometry needs --record".into());
    }
    let [scenario] = <[PathBuf; 1]>::try_from(scenario).map_err(|_| String::new())?;
    Ok(Options {
        scenario,
        timing,
        record,
        geometry,
        record_interval: record_interval.unwrap_or(DEFAULT_RECORD_INTERVAL),
    })
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let options = match parse_args(&args) {
        Ok(o) => o,
        Err(message) => {
            if !message.is_empty() {
                eprintln!("{message}");
            }
            eprintln!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    match run(&options) {
        Ok(json) => {
            println!("{json}");
            ExitCode::SUCCESS
        }
        Err(message) => {
            eprintln!("error: {message}");
            ExitCode::from(1)
        }
    }
}

fn run(options: &Options) -> Result<String, String> {
    let path = options.scenario.as_path();
    let scenario = Scenario::load(path).map_err(|e| e.to_string())?;
    // layout and factsheet paths are relative to the scenario file
    let base = path.parent().unwrap_or(Path::new("."));
    let mut world = scenario.build_world(base).map_err(|e| e.to_string())?;
    let recording = match &options.record {
        Some(file) => {
            // check the geometry before anything is created or run
            let geometry = match &options.geometry {
                Some(g) => {
                    let g = GeometryFile::load(g).map_err(|e| e.to_string())?;
                    LayoutLine::from_graph(world.graph())
                        .with_geometry(&g)
                        .map_err(|e| e.to_string())?;
                    Some(g)
                }
                None => None,
            };
            let out =
                File::create(file).map_err(|e| format!("cannot write {}: {e}", file.display()))?;
            let (recorder, handle) = match geometry {
                Some(g) => Recorder::with_geometry(out, g),
                None => Recorder::new(out),
            };
            // the type of each robot, so that a viewer can pick the curve a robot
            // follows on an edge (the ids are as in the snapshots)
            let robots: Vec<LayoutRobot> = world
                .robot_ids()
                .filter_map(|id| {
                    let spec = world.spec(id).ok()?;
                    Some(LayoutRobot {
                        id: format!("{}/{}", id.manufacturer, id.serial_number),
                        robot_type: format!("{}/{}", spec.manufacturer, spec.series_name),
                    })
                })
                .collect();
            let recorder = recorder.with_robots(robots);
            world
                .set_observer_at(Box::new(recorder), options.record_interval)
                .map_err(|e| e.to_string())?;
            Some((file, handle))
        }
        None => None,
    };
    let result = summary::run_with(&scenario, world, options.timing).map_err(|e| e.to_string())?;
    if let Some((file, handle)) = recording {
        handle
            .finish()
            .map_err(|e| format!("cannot write {}: {e}", file.display()))?;
    }
    serde_json::to_string_pretty(&result).map_err(|e| e.to_string())
}

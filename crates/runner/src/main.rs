//! `botsim [--policy baseline|traffic] [--timing] [--record <file.jsonl>
//! [--record-interval <seconds>] [--geometry <geometry.json>]]
//! [--record-inputs <file>] <scenario.toml>`:
//! run a scenario headless and print the JSON summary on stdout. Errors go to
//! stderr with exit code 1; a wrong command line gives exit code 2.
//!
//! `botsim --replay <file>`: replay the fleet server's inputs recorded by an
//! earlier run, and say whether it matches (see below).
//!
//! `--policy` says who decides which robot may go: the bundled baseline rules (the
//! default) or the traffic module (see `botsim::traffic_setup`). `--timing`
//! cannot be used with `traffic` yet.
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
//!
//! `--record-inputs <file>` also writes everything the fleet server decides on
//! (`botsim-replay`): its setup, and the time and the inbox of every cycle with a
//! checksum of what it did. The summary is the same with or without it. It needs a
//! scenario with `[jobs]`, since only then a server runs.
//!
//! `--replay <file>` (alone) makes the server of such a file, with no other file,
//! runs its cycles and prints JSON: `recorded_with` and `replayed_with` (botsim
//! versions), `cycles` compared, `same` (all of them gave what was recorded),
//! `complete` (the file could be read to its end) and, for the first cycle that
//! differs, `first_difference` with its number (`cycle`, from 1), its `time_ms`
//! and whether the `messages` and the `state` differ; the replay stops there. Exit
//! code 0: the same to the end; 3: a cycle differs; 1: the file cannot be read, is
//! broken or cut off (the cycles before are replayed and printed first). The same
//! result is promised only for the same build on the same kind of machine.

use std::fs::File;
use std::io::{BufReader, BufWriter};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use botsim::scenario::Scenario;
use botsim::summary::{self, InputRecording};
use botsim::traffic_setup;
use botsim_record::{GeometryFile, LayoutLine, LayoutRobot, Recorder};
use botsim_replay::{Outcome, Rules, VERSION};
use serde::Serialize;

const USAGE: &str = "usage: botsim [--policy baseline|traffic] [--timing] \
[--record <file.jsonl> [--record-interval <seconds>] [--geometry <geometry.json>]] \
[--record-inputs <file>] <scenario.toml>
       botsim --replay <file>";

/// Snapshot interval of a recording, simulated seconds.
const DEFAULT_RECORD_INTERVAL: f64 = 0.1;

/// The exit code of a replay that differs from what was recorded.
const DIFFERS: u8 = 3;

/// Who decides which robot may go.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Policy {
    Baseline,
    Traffic,
}

/// What the command line asks for.
#[derive(Debug)]
enum Command {
    Run(Options),
    Replay(PathBuf),
}

#[derive(Debug)]
struct Options {
    scenario: PathBuf,
    policy: Policy,
    timing: bool,
    record: Option<PathBuf>,
    /// Static geometry file (walls, racks, ...) for the recording.
    geometry: Option<PathBuf>,
    record_interval: f64,
    /// Where the server's inputs are recorded.
    record_inputs: Option<PathBuf>,
}

fn parse_args(args: &[String]) -> Result<Command, String> {
    if args.iter().any(|a| a == "--replay") {
        return match args {
            [option, file] if option == "--replay" && !file.starts_with("--") => {
                Ok(Command::Replay(PathBuf::from(file)))
            }
            _ => Err("--replay needs a file name and no other argument".into()),
        };
    }
    let mut policy = Policy::Baseline;
    let mut timing = false;
    let mut record = None;
    let mut record_interval = None;
    let mut geometry = None;
    let mut record_inputs = None;
    let mut scenario = Vec::new();
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--policy" => {
                let value = it.next().ok_or("--policy needs baseline or traffic")?;
                policy = match value.as_str() {
                    "baseline" => Policy::Baseline,
                    "traffic" => Policy::Traffic,
                    _ => return Err(format!("bad --policy '{value}' (baseline or traffic)")),
                };
            }
            "--timing" => timing = true,
            "--record" => {
                let value = it.next().ok_or("--record needs a file name")?;
                record = Some(PathBuf::from(value));
            }
            "--record-inputs" => {
                let value = it.next().ok_or("--record-inputs needs a file name")?;
                record_inputs = Some(PathBuf::from(value));
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
    if timing && policy == Policy::Traffic {
        return Err("--timing cannot be used with --policy traffic".into());
    }
    if record.is_some() && record == record_inputs {
        return Err("--record and --record-inputs need two files".into());
    }
    let [scenario] = <[PathBuf; 1]>::try_from(scenario).map_err(|_| String::new())?;
    Ok(Command::Run(Options {
        scenario,
        policy,
        timing,
        record,
        geometry,
        record_interval: record_interval.unwrap_or(DEFAULT_RECORD_INTERVAL),
        record_inputs,
    }))
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let command = match parse_args(&args) {
        Ok(c) => c,
        Err(message) => {
            if !message.is_empty() {
                eprintln!("{message}");
            }
            eprintln!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    let result = match &command {
        Command::Run(options) => run(options).map(|json| {
            println!("{json}");
            ExitCode::SUCCESS
        }),
        Command::Replay(file) => replay(file),
    };
    result.unwrap_or_else(|message| {
        eprintln!("error: {message}");
        ExitCode::from(1)
    })
}

/// What `--replay` prints.
#[derive(Serialize)]
struct ReplayReport<'a> {
    recorded_with: &'a str,
    replayed_with: &'a str,
    cycles: u64,
    same: bool,
    complete: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    first_difference: Option<DifferenceReport>,
}

#[derive(Serialize)]
struct DifferenceReport {
    cycle: u64,
    time_ms: i64,
    messages: bool,
    state: bool,
}

fn replay(file: &Path) -> Result<ExitCode, String> {
    let failed = |e: &dyn std::fmt::Display| format!("{}: {e}", file.display());
    let input = File::open(file).map_err(|e| failed(&e))?;
    let Outcome {
        recorded_with,
        cycles,
        difference,
        broken,
    } = botsim_replay::replay(BufReader::new(input)).map_err(|e| failed(&e))?;
    if recorded_with != VERSION {
        eprintln!("note: recorded with botsim {recorded_with}, replayed with {VERSION}");
    }
    let report = ReplayReport {
        recorded_with: &recorded_with,
        replayed_with: VERSION,
        cycles,
        same: difference.is_none(),
        complete: broken.is_none() && difference.is_none(),
        first_difference: difference.map(|d| DifferenceReport {
            cycle: d.cycle,
            time_ms: d.time.0,
            messages: d.messages,
            state: d.state,
        }),
    };
    let json = serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?;
    println!("{json}");
    match (difference, broken) {
        (Some(_), _) => Ok(ExitCode::from(DIFFERS)),
        (None, Some(error)) => Err(failed(&error)),
        (None, None) => Ok(ExitCode::SUCCESS),
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
    let result = match &options.record_inputs {
        Some(file) => run_recorded(options, &scenario, world, file)?,
        None => match options.policy {
            Policy::Baseline => summary::run_with(&scenario, world, options.timing),
            Policy::Traffic => {
                let adapter = traffic_setup::adapter(&scenario, path)?;
                summary::run_with_policy(&scenario, world, Box::new(adapter))
            }
        }
        .map_err(|e| e.to_string())?,
    };
    if let Some((file, handle)) = recording {
        handle
            .finish()
            .map_err(|e| format!("cannot write {}: {e}", file.display()))?;
    }
    serde_json::to_string_pretty(&result).map_err(|e| e.to_string())
}

/// The run with the server's inputs recorded into `file`.
fn run_recorded(
    options: &Options,
    scenario: &Scenario,
    world: botsim_sim::world::World,
    file: &Path,
) -> Result<summary::Summary, String> {
    // before the file is created
    if scenario.jobs.is_none() {
        return Err("--record-inputs needs a scenario with [jobs]".into());
    }
    let path = options.scenario.as_path();
    let lif = traffic_setup::layout_text(scenario, path)?;
    let rules = match options.policy {
        Policy::Baseline => Rules::Baseline,
        Policy::Traffic => traffic_setup::rules(scenario, path, &lif)?,
    };
    let cannot = |e: &dyn std::fmt::Display| format!("cannot write {}: {e}", file.display());
    let output = File::create(file).map_err(|e| cannot(&e))?;
    let recording = InputRecording {
        lif,
        rules,
        output: Box::new(BufWriter::new(output)),
    };
    let (result, written) = summary::run_recorded(scenario, world, options.timing, recording)?;
    written.map_err(|e| cannot(&e))?;
    Ok(result)
}

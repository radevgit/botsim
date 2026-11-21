//! `botsim-runner [--timing] <scenario.toml>`: run a scenario headless and print
//! the JSON summary on stdout. Errors go to stderr with exit code 1; a wrong
//! command line gives exit code 2.
//!
//! `--timing` adds the real time the controller took per step (it differs from
//! run to run, so without the flag the output of a scenario is always identical).

use std::path::Path;
use std::process::ExitCode;

use botsim_runner::scenario::Scenario;
use botsim_runner::summary;

const USAGE: &str = "usage: botsim-runner [--timing] <scenario.toml>";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let timing = args.iter().any(|a| a == "--timing");
    let rest: Vec<&String> = args.iter().filter(|a| *a != "--timing").collect();
    if let Some(option) = rest.iter().find(|a| a.starts_with("--")) {
        eprintln!("unknown option {option}\n{USAGE}");
        return ExitCode::from(2);
    }
    let [path] = rest.as_slice() else {
        eprintln!("{USAGE}");
        return ExitCode::from(2);
    };
    match run(Path::new(path.as_str()), timing) {
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

fn run(path: &Path, timing: bool) -> Result<String, String> {
    let scenario = Scenario::load(path).map_err(|e| e.to_string())?;
    // layout and factsheet paths are relative to the scenario file
    let base = path.parent().unwrap_or(Path::new("."));
    let world = scenario.build_world(base).map_err(|e| e.to_string())?;
    let result = summary::run_with(&scenario, world, timing).map_err(|e| e.to_string())?;
    serde_json::to_string_pretty(&result).map_err(|e| e.to_string())
}

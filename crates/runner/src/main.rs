//! `botsim-runner <scenario.toml>`: run a scenario headless and print the JSON
//! summary on stdout. Errors go to stderr with exit code 1; a wrong command
//! line gives exit code 2.

use std::path::Path;
use std::process::ExitCode;

use botsim_runner::scenario::Scenario;
use botsim_runner::summary;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [path] = args.as_slice() else {
        eprintln!("usage: botsim-runner <scenario.toml>");
        return ExitCode::from(2);
    };
    match run(Path::new(path)) {
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

fn run(path: &Path) -> Result<String, String> {
    let scenario = Scenario::load(path).map_err(|e| e.to_string())?;
    // layout and factsheet paths are relative to the scenario file
    let base = path.parent().unwrap_or(Path::new("."));
    let world = scenario.build_world(base).map_err(|e| e.to_string())?;
    let result = summary::run(&scenario, world);
    serde_json::to_string_pretty(&result).map_err(|e| e.to_string())
}

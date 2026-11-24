//! `botsim-check [--geometry <file>] [--clearance <m>] [--conflicts <out.json>] [--json] <scenario.toml>`
//!
//! Exit code: 0 no errors found, 1 at least one error finding, 2 usage or load error.

use std::path::PathBuf;
use std::process::ExitCode;

use botsim_check::{Level, Options, conflict, input, run, to_json, to_text};

fn main() -> ExitCode {
    let mut scenario: Option<PathBuf> = None;
    let mut geometry: Option<PathBuf> = None;
    let mut conflicts: Option<PathBuf> = None;
    let mut options = Options::default();
    let mut json = false;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--json" => json = true,
            "--conflicts" => match args.next() {
                Some(p) => conflicts = Some(PathBuf::from(p)),
                None => return usage("--conflicts needs a file"),
            },
            "--geometry" => match args.next() {
                Some(p) => geometry = Some(PathBuf::from(p)),
                None => return usage("--geometry needs a file"),
            },
            "--clearance" => match args.next().and_then(|v| v.parse::<f64>().ok()) {
                Some(m) if m.is_finite() && m >= 0.0 => options.clearance = m,
                _ => return usage("--clearance needs a distance in metres (0 or more)"),
            },
            s if s.starts_with("--") => return usage(&format!("unknown option {s}")),
            _ => {
                if scenario.is_some() {
                    return usage("only one scenario file");
                }
                scenario = Some(PathBuf::from(a));
            }
        }
    }
    let Some(scenario) = scenario else {
        return usage("no scenario file");
    };
    let inputs = match input::load_with_geometry(&scenario, geometry.as_deref()) {
        Ok(i) => i,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::from(2);
        }
    };

    let report = run(&inputs, &options);
    let findings = &report.findings;
    if let Some(path) = &conflicts {
        let json = report
            .conflict_map
            .as_ref()
            .map_or("null".to_string(), conflict::to_json);
        if let Err(e) = std::fs::write(path, json) {
            eprintln!("error: cannot write {}: {e}", path.display());
            return ExitCode::from(2);
        }
    }

    if json {
        println!("{}", to_json(findings));
    } else {
        print!("{}", to_text(findings));
        eprintln!(
            "{} finding(s), {} error(s)",
            findings.len(),
            findings.iter().filter(|f| f.level == Level::Error).count()
        );
    }
    if findings.iter().any(|f| f.level == Level::Error) {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    }
}

fn usage(why: &str) -> ExitCode {
    eprintln!("error: {why}");
    eprintln!(
        "usage: botsim-check [--geometry <file>] [--clearance <m>] [--conflicts <out.json>] [--json] <scenario.toml>"
    );
    ExitCode::from(2)
}

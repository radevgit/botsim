//! `botsim-check [--geometry <file>] [--json] <scenario.toml>`
//!
//! Exit code: 0 no errors found, 1 at least one error finding, 2 usage or load error.

use std::path::PathBuf;
use std::process::ExitCode;

use botsim_check::{Level, check_inputs, input, to_json, to_text};

fn main() -> ExitCode {
    let mut scenario: Option<PathBuf> = None;
    let mut geometry: Option<PathBuf> = None;
    let mut json = false;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--json" => json = true,
            "--geometry" => match args.next() {
                Some(p) => geometry = Some(PathBuf::from(p)),
                None => return usage("--geometry needs a file"),
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
    if geometry.is_some() {
        eprintln!("note: --geometry is accepted but the clearance check does not exist yet");
    }

    let inputs = match input::load(&scenario) {
        Ok(i) => i,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::from(2);
        }
    };

    let findings = check_inputs(&inputs);

    if json {
        println!("{}", to_json(&findings));
    } else {
        print!("{}", to_text(&findings));
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
    eprintln!("usage: botsim-check [--geometry <file>] [--json] <scenario.toml>");
    ExitCode::from(2)
}

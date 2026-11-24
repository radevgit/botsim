//! `usage_report <recording.jsonl> [--from S] [--to S] [--count N] [--groups FILE]`: the use by
//! group of roads (with a groups file: `--groups`, else `scenarios/<name>.groups.json` if it
//! exists), the busiest edges and the nodes where robots waited most, from a recording, over
//! the whole run or the window from S to S simulated seconds (the same numbers as the viewer's
//! usage heat map).

use std::fs::File;
use std::io::BufReader;
use std::process::ExitCode;

use botsim_record::read_recording_partial;
use botsim_viewer::{busiest_edges, group_table, groups_from_json, most_waited_nodes, usage_between};

const USAGE: &str =
    "usage: usage_report <recording.jsonl> [--from S] [--to S] [--count N] [--groups FILE]";

fn main() -> ExitCode {
    let mut path: Option<String> = None;
    let (mut from, mut to) = (f64::NEG_INFINITY, f64::INFINITY);
    let mut count = 20usize;
    let mut groups_path: Option<String> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut value = |what: &str| args.next().ok_or(format!("{what} needs a value"));
        let parsed: Result<(), String> = match a.as_str() {
            "--from" => value("--from").and_then(|v| {
                from = v.parse().map_err(|_| format!("--from: not a number: {v}"))?;
                Ok(())
            }),
            "--to" => value("--to").and_then(|v| {
                to = v.parse().map_err(|_| format!("--to: not a number: {v}"))?;
                Ok(())
            }),
            "--count" => value("--count").and_then(|v| {
                count = v.parse().map_err(|_| format!("--count: not a number: {v}"))?;
                Ok(())
            }),
            "--groups" => value("--groups").map(|v| groups_path = Some(v)),
            s if s.starts_with("--") => Err(format!("unknown option {s}")),
            _ if path.is_none() => {
                path = Some(a.clone());
                Ok(())
            }
            _ => Err("only one recording".to_string()),
        };
        if let Err(e) = parsed {
            eprintln!("error: {e}\n{USAGE}");
            return ExitCode::from(2);
        }
    }
    let Some(path) = path else {
        eprintln!("{USAGE}");
        return ExitCode::from(2);
    };
    let recording = match File::open(&path)
        .map_err(|e| e.to_string())
        .and_then(|f| read_recording_partial(BufReader::new(f)).map_err(|e| e.to_string()))
    {
        Ok((r, _)) => r,
        Err(e) => {
            eprintln!("error: {path}: {e}");
            return ExitCode::from(1);
        }
    };
    let u = usage_between(&recording, from, to);
    let groups_path = groups_path.or_else(|| {
        let stem = std::path::Path::new(&path).file_stem()?.to_str()?;
        let p = format!("scenarios/{stem}.groups.json");
        std::path::Path::new(&p).exists().then_some(p)
    });
    println!(
        "{} snapshots, {:.0} s counted, {} edges, {} nodes",
        recording.snapshots.len(),
        u.duration,
        recording.layout.edges.len(),
        recording.layout.nodes.len()
    );
    if let Some(p) = &groups_path {
        match std::fs::read_to_string(p)
            .map_err(|e| e.to_string())
            .and_then(|t| groups_from_json(&t))
        {
            Ok(groups) => {
                println!("groups of roads ({p}); mean share of the time an edge was reserved:");
                for r in group_table(&recording.layout, &u, &groups) {
                    println!(
                        "  {:<24} {:4} edges  mean {:5.1} %  busiest {:5.1} %  {:7.0} s",
                        r.name,
                        r.edges,
                        100.0 * r.mean_share,
                        100.0 * r.max_share,
                        r.seconds
                    );
                }
            }
            Err(e) => eprintln!("note: {p}: {e}"),
        }
    }
    println!("busiest edges (share of the time reserved):");
    for (id, share) in busiest_edges(&recording.layout, &u, count) {
        println!("  {id:<14} {:5.1} %", 100.0 * share);
    }
    println!("nodes where robots waited most (robot-seconds per second):");
    for (id, share) in most_waited_nodes(&recording.layout, &u, count) {
        println!("  {id:<14} {share:6.2}");
    }
    ExitCode::SUCCESS
}

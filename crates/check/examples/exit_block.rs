//! Counts of the exit-block check (`exit_block::cases`) for scenario files, with
//! and without the condition that the robot on the blocking node is waiting for the
//! place.
//!
//! ```sh
//! cargo run --release -p botsim-check --example exit_block -- [--place <node>] <scenario.toml>...
//! ```
//!
//! `--place <node>` also lists the cases of that place.

use std::collections::BTreeSet;
use std::path::Path;
use std::time::Instant;

use botsim_check::conflict::{MapOptions, compute};
use botsim_check::exit_block::{Case, cases};
use botsim_check::input;
use botsim_check::reach::TypeRef;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let mut show = None;
    if let Some(i) = args.iter().position(|a| a == "--place")
        && i + 1 < args.len()
    {
        show = args.drain(i..i + 2).nth(1);
    }
    for scenario in &args {
        let inputs = input::load(Path::new(scenario))?;
        let mut types: Vec<TypeRef> = Vec::new();
        for f in &inputs.fleet {
            let t = TypeRef::from_factsheet(&f.factsheet)?;
            if !types.iter().any(|x| x.name == t.name) {
                types.push(t);
            }
        }
        let map = compute(&inputs.lif, &types, &MapOptions::default())
            .ok_or("the layout cannot be turned into a graph")?;
        let start = Instant::now();
        let all = cases(&inputs.lif, &types, &map);
        let time = start.elapsed().as_secs_f64();
        println!(
            "{scenario}: without the condition {}; with it {}; {time:.2} s",
            count(&all, false),
            count(&all, true)
        );
        for c in all.iter().filter(|c| Some(&c.place) == show.as_ref()) {
            let nodes = |waiting: bool| {
                c.blockers
                    .iter()
                    .filter(|b| b.waiting == waiting)
                    .map(|b| b.node.as_str())
                    .collect::<Vec<_>>()
                    .join(" ")
            };
            println!(
                "  {} on {} ({}), {} waiting\n    way: {}\n    waiting: {}\n    removed by the condition: {}",
                c.type_place,
                c.place,
                c.station.as_deref().unwrap_or("no station"),
                c.type_wait,
                c.way.join(" "),
                nodes(true),
                nodes(false)
            );
        }
    }
    Ok(())
}

/// "places P, (type pair, place, node) cases N".
fn count(all: &[Case], only_waiting: bool) -> String {
    let mut places = BTreeSet::new();
    let mut pairs = 0;
    for c in all {
        let n = c
            .blockers
            .iter()
            .filter(|b| b.waiting || !only_waiting)
            .count();
        if n > 0 {
            places.insert(c.place.as_str());
            pairs += n;
        }
    }
    format!("{} place(s), {pairs} case(s)", places.len())
}

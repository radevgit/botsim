//! Size and computing time of the conflict map of a layout, without and with the
//! protective field (`MapOptions::protective_field`), default options otherwise.
//!
//! ```sh
//! cargo run --release -p botsim-space --example map_stats -- <layout.lif.json> <factsheet.json>...
//! ```
//!
//! Prints one line per setting: conflicts (edge-edge, node-edge), the ones of a
//! controller (no common node and near a junction), and the best of three times.

use std::time::{Duration, Instant};

use botsim_space::{ConflictMap, MapOptions, TypeRef, compute};
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

const RUNS: usize = 3;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    // `--pair <node> <edge>`: also say whether this node-edge conflict is listed
    let mut pair = None;
    if let Some(i) = args.iter().position(|a| a == "--pair")
        && i + 2 < args.len()
    {
        let p: Vec<String> = args.drain(i..i + 3).collect();
        pair = Some((p[1].clone(), p[2].clone()));
    }
    // `--<name> <value>` options
    let mut option = |name: &str| {
        let i = args.iter().position(|a| a == name)?;
        (i + 1 < args.len()).then(|| args.drain(i..i + 2).nth(1))?
    };
    // `--dump <file>`: write the map without the field as JSON
    let dump = option("--dump");
    // `--step <m>`, `--clearance <m>`: other than the default
    let defaults = MapOptions::default();
    let step: f64 = option("--step").map_or(Ok(defaults.sample_step), |v| v.parse())?;
    let clearance: f64 = option("--clearance").map_or(Ok(defaults.clearance), |v| v.parse())?;
    // `--runs <n>`: how many times each map is computed (the best time counts)
    let runs: usize = option("--runs").map_or(Ok(RUNS), |v| v.parse())?;
    let [lif, factsheets @ ..] = args.as_slice() else {
        return Err("usage: map_stats [--pair <node> <edge>] [--dump <file>] [--step <m>] [--clearance <m>] <layout.lif.json> <factsheet.json>...".into());
    };
    let lif: Lif = serde_json::from_str(&std::fs::read_to_string(lif)?)?;
    let mut types: Vec<TypeRef> = Vec::new();
    for path in factsheets {
        let f: Factsheet = serde_json::from_str(&std::fs::read_to_string(path)?)?;
        let t = TypeRef::from_factsheet(&f)?;
        if !types.iter().any(|x| x.name == t.name) {
            types.push(t);
        }
    }
    for protective_field in [false, true] {
        let options = MapOptions {
            protective_field,
            sample_step: step,
            clearance,
            ..MapOptions::default()
        };
        let mut best = Duration::MAX;
        let mut map = None;
        for _ in 0..runs.max(1) {
            let start = Instant::now();
            map = compute(&lif, &types, &options);
            best = best.min(start.elapsed());
        }
        let map = map.ok_or("the layout cannot be turned into a graph")?;
        report(&map, protective_field, best);
        if let (false, Some(file)) = (protective_field, &dump) {
            std::fs::write(file, botsim_space::to_json(&map))?;
        }
        if let Some((node, edge)) = &pair {
            let found = map
                .node_pairs
                .iter()
                .flat_map(|p| &p.conflicts)
                .find(|c| &c.node == node && &c.edge == edge);
            match found {
                Some(c) => println!(
                    "  node {node} / edge {edge}: listed (depth {:.3}, distance {:.3}, by field {})",
                    c.depth,
                    c.distance,
                    map.by_field(c.depth, c.distance)
                ),
                None => println!("  node {node} / edge {edge}: not listed"),
            }
        }
    }
    Ok(())
}

fn report(map: &ConflictMap, protective_field: bool, time: Duration) {
    // (all, those a controller waits for, those added by the field)
    let mut edges = (0, 0, 0);
    for c in map.pairs.iter().flat_map(|p| &p.conflicts) {
        edges.0 += 1;
        edges.1 += usize::from(!c.shares_node && c.junction_nodes.is_some());
        edges.2 += usize::from(map.by_field(c.depth, c.distance));
    }
    let mut nodes = (0, 0, 0);
    for c in map.node_pairs.iter().flat_map(|p| &p.conflicts) {
        nodes.0 += 1;
        nodes.1 += usize::from(c.junction_nodes.is_some());
        nodes.2 += usize::from(map.by_field(c.depth, c.distance));
    }
    let node_nodes: usize = map.node_node_pairs.iter().map(|p| p.conflicts.len()).sum();
    println!(
        "field {}: edge pairs {} (controller {}, by field {}), node pairs {} (controller {}, by field {}), node-node pairs {node_nodes}, {:.3} s",
        if protective_field { "on " } else { "off" },
        edges.0,
        edges.1,
        edges.2,
        nodes.0,
        nodes.1,
        nodes.2,
        time.as_secs_f64()
    );
}

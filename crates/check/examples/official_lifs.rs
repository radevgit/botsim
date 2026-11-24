//! Run the checks on LIF files that come with no scenario (e.g. the official examples of
//! `vdma-lif`): every robot type named in the LIF gets a copy of one template factsheet.
//!
//! `cargo run -p botsim-check --example official_lifs -- <template.factsheet.json> <lif.json>...`

use std::collections::BTreeMap;
use std::path::PathBuf;

use botsim_check::{FleetType, Options, run_layout};
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

fn main() {
    let mut args = std::env::args().skip(1);
    let Some(template_path) = args.next() else {
        eprintln!("usage: official_lifs <template.factsheet.json> <lif.json>...");
        return;
    };
    let template: Factsheet = match std::fs::read_to_string(&template_path)
        .map_err(|e| e.to_string())
        .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
    {
        Ok(f) => f,
        Err(e) => {
            eprintln!("{template_path}: {e}");
            return;
        }
    };

    for path in args {
        let name = PathBuf::from(&path)
            .file_name()
            .map_or(path.clone(), |n| n.to_string_lossy().into_owned());
        let lif: Lif = match std::fs::read_to_string(&path)
            .map_err(|e| e.to_string())
            .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
        {
            Ok(l) => l,
            Err(e) => {
                println!("{name}: NOT PARSED: {e}");
                continue;
            }
        };
        // every robot type named in edge or node properties
        let mut types: BTreeMap<(String, String), ()> = BTreeMap::new();
        let layouts = lif.origins.iter().flat_map(|o| &o.layouts);
        let (mut nodes, mut edges) = (0, 0);
        for l in layouts {
            nodes += l.nodes.len();
            edges += l.edges.len();
            for e in &l.edges {
                for p in &e.mobile_robot_type_edge_properties {
                    for t in &p.mobile_robot_types {
                        types.insert((t.manufacturer.clone(), t.series_name.clone()), ());
                    }
                }
            }
            for n in &l.nodes {
                for p in &n.mobile_robot_type_node_properties {
                    for t in &p.mobile_robot_types {
                        types.insert((t.manufacturer.clone(), t.series_name.clone()), ());
                    }
                }
            }
        }
        let fleet: Vec<FleetType> = types
            .keys()
            .enumerate()
            .map(|(i, (m, s))| {
                let mut f = template.clone();
                f.manufacturer = m.clone();
                f.type_specification.series_name = s.clone();
                FleetType {
                    group: i,
                    factsheet_path: PathBuf::from(&template_path),
                    factsheet: f,
                }
            })
            .collect();
        let report = run_layout(&lif, &fleet, None, &Options::default());
        let mut count: BTreeMap<String, usize> = BTreeMap::new();
        for f in &report.findings {
            *count.entry(format!("{} {}", f.level, f.check)).or_default() += 1;
        }
        println!(
            "{name}: {nodes} nodes, {edges} edges, {} robot type(s), {} finding(s): {count:?}",
            fleet.len(),
            report.findings.len()
        );
        for f in report
            .findings
            .iter()
            .filter(|f| f.level != botsim_check::Level::Info)
        {
            println!("    {f}");
        }
    }
}

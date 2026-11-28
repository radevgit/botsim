//! `--policy traffic`: the traffic rules of the fleet server
//! ([`botsim_server::traffic_policy`]) for a scenario. The server knows no scenario
//! file: the layout, the factsheets and what the conflict map counts are read here
//! and given to it, or written into a recording of its inputs
//! ([`botsim_replay`]).

use std::path::Path;

use botsim_replay::Rules;
use botsim_server::traffic_policy::TrafficAdapter;
use botsim_space::{MapOptions, TypeRef};

use crate::scenario::Scenario;

/// What the module's conflict map counts for a scenario.
fn map_options(scenario: &Scenario) -> MapOptions {
    MapOptions {
        clearance: scenario.conflict_clearance,
        protective_field: scenario.safety_stop,
        ..MapOptions::default()
    }
}

/// A file of the scenario read from `scenario_file`, as text.
fn read(scenario_file: &Path, path: &Path) -> Result<String, String> {
    let base = scenario_file.parent().unwrap_or(Path::new("."));
    let path = base.join(path);
    std::fs::read_to_string(&path).map_err(|e| format!("cannot read {}: {e}", path.display()))
}

/// The text of the layout file of a scenario read from `scenario_file`.
pub fn layout_text(scenario: &Scenario, scenario_file: &Path) -> Result<String, String> {
    read(scenario_file, &scenario.lif)
}

/// What the traffic adapter of a scenario is made from.
type Inputs = (vdma_lif::Lif, Vec<TypeRef>, MapOptions);

/// Reads the layout (`text`) and the factsheets of the scenario.
fn inputs(scenario: &Scenario, scenario_file: &Path, text: &str) -> Result<Inputs, String> {
    let lif: vdma_lif::Lif =
        serde_json::from_str(text).map_err(|e| format!("{}: {e}", scenario.lif.display()))?;
    let mut types: Vec<TypeRef> = Vec::new();
    for group in &scenario.robots {
        let file = group.factsheet.display();
        let factsheet: vda5050::factsheet::Factsheet =
            serde_json::from_str(&read(scenario_file, &group.factsheet)?)
                .map_err(|e| format!("{file}: {e}"))?;
        types.push(TypeRef::from_factsheet(&factsheet).map_err(|e| format!("{file}: {e}"))?);
    }
    Ok((lif, types, map_options(scenario)))
}

/// The adapter for a scenario read from `scenario_file`: reads its layout and
/// factsheets again.
pub fn adapter(scenario: &Scenario, scenario_file: &Path) -> Result<TrafficAdapter, String> {
    let text = layout_text(scenario, scenario_file)?;
    let (lif, types, options) = inputs(scenario, scenario_file, &text)?;
    TrafficAdapter::new(&lif, &types, &options)
}

/// The same as data, for a recording: the traffic rules the adapter is made from,
/// with the layout as `text` ([`layout_text`]).
pub fn rules(scenario: &Scenario, scenario_file: &Path, text: &str) -> Result<Rules, String> {
    let (_, types, options) = inputs(scenario, scenario_file, text)?;
    Ok(Rules::Traffic { types, options })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_map_counts_the_protective_field_only_with_the_safety_stop() {
        let text = include_str!("../../../scenarios/m1_short.toml");
        let mut scenario = Scenario::from_toml(text).expect("scenario");
        assert!(scenario.safety_stop, "on unless a scenario says otherwise");
        for safety_stop in [true, false] {
            scenario.safety_stop = safety_stop;
            let options = map_options(&scenario);
            assert_eq!(options.protective_field, safety_stop);
            assert_eq!(options.clearance, scenario.conflict_clearance);
        }
    }
}

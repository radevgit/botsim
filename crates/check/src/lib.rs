//! Static checks of a LIF layout for a given mixed robot fleet.
//!
//! To use it from another program (e.g. the viewer), call [`check_scenario_json`]
//! with the path of a scenario file: it returns the findings as a JSON string.
//! [`check_scenario`] returns the findings as values, and [`check_inputs`]
//! checks inputs that are already loaded.

use std::path::Path;

use botsim_core::spec::RobotSpec;

pub mod finding;
pub mod input;
pub mod places;
pub mod reach;
pub mod structure;

pub use finding::{Finding, FindingPlace, Level, PlaceKind, to_json, to_text};
pub use input::{FleetType, Inputs, LoadError, load};

/// Run all checks on loaded inputs.
pub fn check_inputs(inputs: &Inputs) -> Vec<Finding> {
    let fleet_types: Vec<String> = inputs.fleet.iter().map(|f| f.type_name()).collect();
    let mut out = structure::check(&inputs.lif, &fleet_types);
    let mut seen: Vec<String> = Vec::new();
    for f in &inputs.fleet {
        let name = f.type_name();
        if seen.contains(&name) {
            continue;
        }
        seen.push(name.clone());
        let kinematics = match RobotSpec::from_factsheet(&f.factsheet) {
            Ok(s) => s.kinematics,
            Err(e) => {
                out.push(
                    Finding::new(
                        Level::Error,
                        "factsheet",
                        format!(
                            "{}: robot type cannot be used by the checks: {e}",
                            f.factsheet_path.display()
                        ),
                    )
                    .robot(&name),
                );
                continue;
            }
        };
        let t = reach::TypeRef {
            name,
            manufacturer: f.factsheet.manufacturer.clone(),
            series: f.factsheet.type_specification.series_name.clone(),
            kinematics,
        };
        out.extend(reach::check_type(&inputs.lif, &t));
    }
    out
}

/// Load a scenario (and the LIF and factsheets it names) and run all checks.
pub fn check_scenario(scenario_path: &Path) -> Result<Vec<Finding>, LoadError> {
    Ok(check_inputs(&load(scenario_path)?))
}

/// Same as [`check_scenario`], with the findings as a JSON array.
pub fn check_scenario_json(scenario_path: &Path) -> Result<String, LoadError> {
    Ok(to_json(&check_scenario(scenario_path)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_for_a_scenario() {
        let p = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/mixed_fleet.toml");
        let j = check_scenario_json(&p).expect("loads");
        let v: serde_json::Value = serde_json::from_str(&j).expect("valid JSON");
        assert!(v.is_array());
    }

    #[test]
    fn load_error_is_returned_not_panicked() {
        assert!(check_scenario_json(Path::new("/nonexistent.toml")).is_err());
    }
}

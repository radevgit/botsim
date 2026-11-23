//! Loading the inputs of a check: the scenario, its LIF file and its factsheets.

use std::fmt;
use std::path::{Path, PathBuf};

use botsim_scenario::Scenario;
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

/// Everything the checks need, read from the files a scenario names.
#[derive(Debug, Clone)]
pub struct Inputs {
    pub scenario: Scenario,
    pub lif: Lif,
    /// One entry per `[[robot]]` group of the scenario, in the same order.
    pub fleet: Vec<FleetType>,
}

/// A robot type of the fleet: one `[[robot]]` group and its factsheet.
#[derive(Debug, Clone)]
pub struct FleetType {
    /// Index of the `[[robot]]` group in the scenario.
    pub group: usize,
    pub factsheet_path: PathBuf,
    pub factsheet: Factsheet,
}

impl FleetType {
    /// `manufacturer/seriesName`, the key that LIF edge properties use.
    pub fn type_name(&self) -> String {
        format!(
            "{}/{}",
            self.factsheet.manufacturer, self.factsheet.type_specification.series_name
        )
    }
}

/// A file could not be loaded; nothing can be checked.
#[derive(Debug, Clone, PartialEq)]
pub struct LoadError {
    pub path: PathBuf,
    pub message: String,
}

impl fmt::Display for LoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.path.display(), self.message)
    }
}

impl std::error::Error for LoadError {}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T, LoadError> {
    let err = |message: String| LoadError {
        path: path.to_path_buf(),
        message,
    };
    let text = std::fs::read_to_string(path).map_err(|e| err(e.to_string()))?;
    serde_json::from_str(&text).map_err(|e| err(e.to_string()))
}

/// Load a scenario file and the LIF and factsheet files it names (paths are
/// relative to the scenario file).
pub fn load(scenario_path: &Path) -> Result<Inputs, LoadError> {
    let scenario = Scenario::load(scenario_path).map_err(|e| LoadError {
        path: scenario_path.to_path_buf(),
        message: e.to_string(),
    })?;
    let base = scenario_path.parent().unwrap_or(Path::new("."));
    let lif: Lif = read_json(&base.join(&scenario.lif))?;
    let mut fleet = Vec::new();
    for (group, g) in scenario.robots.iter().enumerate() {
        let factsheet_path = base.join(&g.factsheet);
        let factsheet: Factsheet = read_json(&factsheet_path)?;
        fleet.push(FleetType {
            group,
            factsheet_path,
            factsheet,
        });
    }
    Ok(Inputs {
        scenario,
        lif,
        fleet,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenarios() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
    }

    #[test]
    fn loads_mixed_fleet() {
        let i = load(&scenarios().join("mixed_fleet.toml")).expect("loads");
        assert_eq!(i.fleet.len(), i.scenario.robots.len());
        assert!(!i.lif.origins.is_empty());
        assert!(i.fleet.iter().all(|f| f.type_name().contains('/')));
    }

    #[test]
    fn missing_scenario_is_an_error_not_a_panic() {
        let e = load(Path::new("/nonexistent/x.toml")).unwrap_err();
        assert!(e.path.ends_with("x.toml"));
    }
}

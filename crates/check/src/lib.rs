//! Static checks of a LIF layout for a given mixed robot fleet.
//!
//! This crate is meant to be used as a library (the viewer calls it); the
//! `botsim-check` binary is a thin wrapper.
//!
//! ```no_run
//! use botsim_check::{Options, run_scenario};
//! let report = run_scenario(std::path::Path::new("scenarios/warehouse4.toml"), &Options::default())?;
//! for f in &report.findings {
//!     // f.level, f.check, f.text, f.ids, f.places (kind, id, map, points)
//! }
//! let json = report.to_json(); // {"findings": [...], "conflict_map": {...} | null}
//! # Ok::<(), botsim_check::LoadError>(())
//! ```
//!
//! - [`run_scenario`]: scenario file in (the LIF and factsheets it names are loaded),
//!   [`Report`] out.
//! - [`run`]: the same on [`Inputs`] that the caller already holds (all fields are
//!   public, so a program with the LIF and factsheets in memory can build them).
//! - [`Options`]: the sampling step and whether to compute the conflict map (the
//!   slow part: about 5 s for the 1300-edge warehouse4 layout in a release build).
//! - Never panics on input data: unusable files are a [`LoadError`], layout problems
//!   are findings.

use std::path::Path;

use botsim_core::spec::{RobotSpec, Shape};
use serde::Serialize;

pub mod clearance;
pub mod conflict;
pub mod corners;
pub mod doors;
pub mod exits;
pub mod finding;
pub mod input;
pub mod long_edges;
pub mod loops;
pub mod places;
pub mod reach;
pub mod shelving;
pub mod spacing;
pub mod sweep;
pub mod start;
pub mod structure;

pub use finding::{Finding, FindingPlace, Level, PlaceKind, to_json, to_text};
pub use input::{FleetType, Inputs, LoadError, load, load_with_geometry};

/// What to compute.
#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    /// Geometry file (walls, racks, shelves) for the clearance check; used by
    /// [`run_scenario`]. [`run`] uses `Inputs::geometry` instead.
    pub geometry: Option<std::path::PathBuf>,
    /// Distance between the samples of a swept outline (m).
    pub sample_step: f64,
    /// Conflicts with an overlap less deep than this (m) are left out of the map
    /// and the findings. 0 keeps all, touching too.
    pub min_overlap: f64,
    /// Compute the conflict map and its findings (the slow part).
    pub conflict_map: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            geometry: None,
            sample_step: sweep::SAMPLE_STEP,
            min_overlap: conflict::MIN_OVERLAP,
            conflict_map: true,
        }
    }
}

/// The result of a run.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Report {
    pub findings: Vec<Finding>,
    /// `None` if switched off in [`Options`], or if the layout cannot be turned
    /// into a graph (the findings say why).
    pub conflict_map: Option<conflict::ConflictMap>,
}

impl Report {
    /// Findings are ordered by level, errors first; within a level in the order
    /// the checks ran (stable).
    fn new(mut findings: Vec<Finding>, conflict_map: Option<conflict::ConflictMap>) -> Report {
        findings.sort_by_key(|f| std::cmp::Reverse(f.level));
        Report {
            findings,
            conflict_map,
        }
    }

    /// `{"findings": [...], "conflict_map": {...} | null}`.
    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).unwrap_or_else(|_| "{}".to_string())
    }

    /// The worst level among the findings, if there are any.
    pub fn worst(&self) -> Option<Level> {
        self.findings.iter().map(|f| f.level).max()
    }
}

/// Load a scenario (and the LIF and factsheets it names, and the geometry file if
/// `options.geometry` is set) and run the checks.
pub fn run_scenario(scenario_path: &Path, options: &Options) -> Result<Report, LoadError> {
    Ok(run(
        &load_with_geometry(scenario_path, options.geometry.as_deref())?,
        options,
    ))
}

/// Run all checks on loaded inputs.
pub fn run(inputs: &Inputs, options: &Options) -> Report {
    let (findings, conflict_map) = run_inner(inputs, options);
    Report::new(findings, conflict_map)
}

/// Run all checks on loaded inputs (findings only; with the default options).
pub fn check_inputs(inputs: &Inputs) -> Vec<Finding> {
    run(inputs, &Options::default()).findings
}

/// Run all checks and also return the conflict map (default options).
pub fn check_all(inputs: &Inputs) -> (Vec<Finding>, Option<conflict::ConflictMap>) {
    let r = run(inputs, &Options::default());
    (r.findings, r.conflict_map)
}

/// Check a layout for a fleet **without a scenario** (no start nodes to check):
/// the LIF, one entry per robot type with its factsheet, and optionally the geometry.
pub fn run_layout(
    lif: &vdma_lif::Lif,
    fleet: &[FleetType],
    geometry: Option<&botsim_record::GeometryFile>,
    options: &Options,
) -> Report {
    // no scenario: no robot counts, so no loop capacity check
    let none = std::collections::HashMap::new();
    let (findings, conflict_map) = run_checks(lif, fleet, geometry, &none, options);
    Report::new(findings, conflict_map)
}

fn run_inner(
    inputs: &Inputs,
    options: &Options,
) -> (Vec<Finding>, Option<conflict::ConflictMap>) {
    // how many robots of each type the scenario has (group i uses fleet entry i)
    let mut robots: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
    for (g, f) in inputs.scenario.robots.iter().zip(&inputs.fleet) {
        *robots.entry(f.type_name()).or_default() += g.start_nodes.len();
    }
    let (mut out, map) = run_checks(
        &inputs.lif,
        &inputs.fleet,
        inputs.geometry.as_ref(),
        &robots,
        options,
    );
    out.extend(start::check(inputs));
    // factsheets that cannot be used are reported by run_checks
    let mut types: Vec<reach::TypeRef> = Vec::new();
    for t in inputs.fleet.iter().filter_map(|f| type_ref(f).ok()) {
        if !types.iter().any(|x| x.name == t.name) {
            types.push(t);
        }
    }
    out.extend(doors::check(inputs, &types));
    (out, map)
}

fn run_checks(
    lif: &vdma_lif::Lif,
    fleet: &[FleetType],
    geometry: Option<&botsim_record::GeometryFile>,
    robots: &std::collections::HashMap<String, usize>,
    options: &Options,
) -> (Vec<Finding>, Option<conflict::ConflictMap>) {
    let fleet_types: Vec<String> = fleet.iter().map(|f| f.type_name()).collect();
    let mut out = structure::check(lif, &fleet_types);
    let mut seen: Vec<String> = Vec::new();
    let mut types: Vec<reach::TypeRef> = Vec::new();
    for f in fleet {
        let name = f.type_name();
        if seen.contains(&name) {
            continue;
        }
        seen.push(name.clone());
        match type_ref(f) {
            Ok(t) => {
                out.extend(reach::check_type(lif, &t));
                if let Ok(a) = reach::analyze(lif, &t) {
                    out.extend(exits::check(lif, &a, &t));
                    let n = robots.get(&t.name).copied().unwrap_or(0);
                    out.extend(loops::check(lif, &a, &t, n));
                }
                types.push(t);
            }
            Err(finding) => out.push(finding),
        }
    }
    out.extend(spacing::check(lif, &types));
    out.extend(long_edges::check(lif, &types));
    let map = if options.conflict_map {
        conflict::compute(lif, &types, options.sample_step, options.min_overlap)
    } else {
        None
    };
    if let Some(m) = &map {
        out.extend(conflict::findings(lif, m));
    }
    out.extend(corners::check(lif, map.as_ref()));
    if let Some(g) = geometry {
        out.extend(shelving::check(lif, g));
        out.extend(clearance::check(
            lif,
            &types,
            g,
            options.sample_step,
            options.min_overlap,
        ));
    }
    (out, map)
}

/// Load a scenario and return the conflict map as JSON (`"null"` if the layout
/// cannot be turned into a graph).
pub fn conflict_map_json(scenario_path: &Path) -> Result<String, LoadError> {
    let (_, map) = check_all(&load(scenario_path)?);
    Ok(map.as_ref().map_or("null".to_string(), conflict::to_json))
}

/// How far the protective field reaches in front of the body (m): from the
/// factsheet's `envelopes2d` whose id says "safety" or "field" (the rule
/// `botsim-core` uses to tell them from the body); if the factsheet has none,
/// the fixed margin of the simulator's own protective field.
/// The second value says whether the number is from the factsheet.
fn field_margin(f: &vda5050::factsheet::Factsheet, body: &Shape) -> (f64, bool) {
    let body_front = match body {
        Shape::Rect {
            cx, half_length, ..
        } => cx + half_length,
        Shape::Circle { cx, radius, .. } => cx + radius,
    };
    let field_front = f
        .mobile_robot_geometry
        .envelopes2d
        .iter()
        .flatten()
        .filter(|e| {
            let id = e.envelope_2d_id.to_ascii_lowercase();
            id.contains("safety") || id.contains("field")
        })
        .flat_map(|e| e.vertices.iter())
        .map(|v| v.x)
        .filter(|x| x.is_finite())
        .fold(f64::NEG_INFINITY, f64::max);
    if field_front.is_finite() {
        ((field_front - body_front).max(0.0), true)
    } else {
        (botsim_core::safety::FIELD_MARGIN, false)
    }
}

/// The robot type of a fleet entry as the checks need it; the error is a
/// finding (the factsheet cannot be used).
pub(crate) fn type_ref(f: &FleetType) -> Result<reach::TypeRef, Finding> {
    let name = f.type_name();
    match RobotSpec::from_factsheet(&f.factsheet) {
        Ok(s) => {
            let (field_margin, field_from_factsheet) = field_margin(&f.factsheet, &s.shape);
            Ok(reach::TypeRef {
            name,
            manufacturer: f.factsheet.manufacturer.clone(),
            series: f.factsheet.type_specification.series_name.clone(),
            kinematics: s.kinematics,
            length: match s.shape {
                Shape::Rect { half_length, .. } => 2.0 * half_length,
                Shape::Circle { radius, .. } => 2.0 * radius,
            },
            width: match s.shape {
                Shape::Rect { half_width, .. } => 2.0 * half_width,
                Shape::Circle { radius, .. } => 2.0 * radius,
            },
            field_margin,
            field_from_factsheet,
            shape: s.shape,
            })
        }
        Err(e) => Err(Finding::new(
            Level::Error,
            "factsheet",
            format!(
                "{}: robot type cannot be used by the checks: {e}",
                f.factsheet_path.display()
            ),
        )
        .robot(&name)),
    }
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
    fn report_with_and_without_the_conflict_map() {
        let p = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/mixed_fleet.toml");
        let full = run_scenario(&p, &Options::default()).expect("loads");
        assert!(full.conflict_map.is_some());
        let v: serde_json::Value = serde_json::from_str(&full.to_json()).expect("valid JSON");
        assert!(v["findings"].is_array());
        assert!(v["conflict_map"]["pairs"].is_array());

        let fast = run_scenario(
            &p,
            &Options {
                conflict_map: false,
                ..Options::default()
            },
        )
        .expect("loads");
        assert!(fast.conflict_map.is_none());
        let v: serde_json::Value = serde_json::from_str(&fast.to_json()).expect("valid JSON");
        assert!(v["conflict_map"].is_null());
        // the conflict findings are the only difference
        assert!(fast.findings.iter().all(|f| f.check != "conflict"));
        assert!(fast.worst().is_some());
        // errors first, then warnings, then infos
        let levels: Vec<Level> = fast.findings.iter().map(|f| f.level).collect();
        assert!(levels.windows(2).all(|w| w[0] >= w[1]), "{levels:?}");
    }

    #[test]
    fn load_error_is_returned_not_panicked() {
        assert!(check_scenario_json(Path::new("/nonexistent.toml")).is_err());
    }
}

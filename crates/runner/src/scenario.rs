//! A run is described by one TOML scenario file. Unknown keys are errors, so a
//! misspelled key does not silently fall back to a default.
//!
//! ```toml
//! name = "two-robots"
//! seed = 1                       # not used yet; goes into the summary later
//! duration = 600.0               # seconds of simulated time
//! time_step = 0.05               # seconds; there is deliberately no default
//! start_time = 2026-11-20T10:00:00Z   # tick 0 in message timestamps (UTC)
//! lif = "layout.lif.json"        # paths are relative to this file
//! deadlock_timeout = 30.0        # optional; seconds, default 30
//!
//! [[robot]]                      # a group of robots of one type
//! manufacturer = "ACME"
//! serial_prefix = "R"            # serials R-001, R-002, ...
//! factsheet = "diff_robot.factsheet.json"
//! start_nodes = ["A", "B"]       # one per robot, so this also sets the count
//! start_heading = 0.0            # optional, radians, default 0
//! action_time = 2.0              # seconds an action takes, unless listed below
//! [robot.action_times]           # optional, per action type
//! pick = 3.0
//! ```
//!
//! ```toml
//! [jobs]                         # optional: without it no jobs and no controller
//! rate_per_hour = 60.0           # average new jobs per simulated hour
//! nodes = ["A", "C"]             # where jobs start and end (>= 2 different)
//! ```
//!
//! With `[jobs]` the bundled baseline controller runs: the seed (above) fixes
//! the random jobs, so a run is repeatable.
//!
//! Not yet: zone sets, disturbances, a choice of controller.

use std::collections::BTreeMap;
use std::fmt;
use std::path::{Path, PathBuf};

use botsim_core::actions::ActionTimes;
use botsim_core::graph::Graph;
use botsim_core::jobs::JobGenerator;
use botsim_core::spec::RobotSpec;
use botsim_core::world::{RobotId, World, unix_ms_from_utc};
use serde::Deserialize;
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

/// How long `pick` and `drop` take if `[robot.action_times]` does not list them
/// (user decision S30).
pub const DEFAULT_PICK_DROP_SECONDS: f64 = 5.0;

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scenario {
    pub name: String,
    pub seed: u64,
    /// Simulated seconds.
    pub duration: f64,
    pub time_step: f64,
    pub start_time: toml::value::Datetime,
    pub lif: PathBuf,
    #[serde(rename = "robot")]
    pub robots: Vec<RobotGroup>,
    /// Without this section no jobs are created and no controller runs.
    #[serde(default)]
    pub jobs: Option<JobsConfig>,
    /// Seconds a cycle of robots waiting for each other must last to count as a
    /// deadlock; default 30 (S35).
    #[serde(default)]
    pub deadlock_timeout: Option<f64>,
}

/// The `[jobs]` section: random transport jobs between the listed nodes,
/// handled by the baseline controller.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JobsConfig {
    /// Average number of new jobs per simulated hour.
    pub rate_per_hour: f64,
    /// Node ids where jobs start and end (at least two different ones).
    pub nodes: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotGroup {
    pub manufacturer: String,
    pub serial_prefix: String,
    pub factsheet: PathBuf,
    pub start_nodes: Vec<String>,
    #[serde(default)]
    pub start_heading: f64,
    pub action_time: f64,
    #[serde(default)]
    pub action_times: BTreeMap<String, f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ScenarioError {
    /// A file could not be read.
    Read { path: PathBuf, message: String },
    /// The file is not valid TOML / has wrong or unknown keys.
    Parse(String),
    /// A value is not allowed (the text says which and why).
    Invalid(String),
    /// A referenced file (layout or factsheet) is not usable.
    Input { path: PathBuf, message: String },
}

impl fmt::Display for ScenarioError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ScenarioError::Read { path, message } => {
                write!(f, "cannot read {}: {message}", path.display())
            }
            ScenarioError::Parse(m) => write!(f, "scenario: {m}"),
            ScenarioError::Invalid(m) => write!(f, "scenario: {m}"),
            ScenarioError::Input { path, message } => write!(f, "{}: {message}", path.display()),
        }
    }
}

impl std::error::Error for ScenarioError {}

fn invalid<T>(text: String) -> Result<T, ScenarioError> {
    Err(ScenarioError::Invalid(text))
}

fn positive(name: &str, v: f64) -> Result<(), ScenarioError> {
    if v.is_finite() && v > 0.0 {
        Ok(())
    } else {
        invalid(format!("{name} must be a finite number > 0, got {v}"))
    }
}

fn non_negative(name: &str, v: f64) -> Result<(), ScenarioError> {
    if v.is_finite() && v >= 0.0 {
        Ok(())
    } else {
        invalid(format!("{name} must be a finite number >= 0, got {v}"))
    }
}

impl Scenario {
    /// Parse and check the scenario text (no other file is read).
    pub fn from_toml(text: &str) -> Result<Scenario, ScenarioError> {
        let s: Scenario = toml::from_str(text).map_err(|e| ScenarioError::Parse(e.to_string()))?;
        s.check()?;
        Ok(s)
    }

    /// Read and parse a scenario file.
    pub fn load(path: &Path) -> Result<Scenario, ScenarioError> {
        let text = read(path)?;
        Scenario::from_toml(&text)
    }

    fn check(&self) -> Result<(), ScenarioError> {
        positive("duration", self.duration)?;
        positive("time_step", self.time_step)?;
        if let Some(t) = self.deadlock_timeout {
            positive("deadlock_timeout", t)?;
        }
        self.start_unix_ms()?;
        if let Some(j) = &self.jobs {
            non_negative("[jobs] rate_per_hour", j.rate_per_hour)?;
            if j.nodes.len() < 2 {
                return invalid("[jobs] nodes needs at least two nodes".into());
            }
        }
        if self.robots.is_empty() {
            return invalid("at least one [[robot]] group is needed".into());
        }
        for (i, g) in self.robots.iter().enumerate() {
            let at = |what: &str| format!("robot group {}: {what}", i + 1);
            if g.start_nodes.is_empty() {
                return invalid(at("start_nodes is empty"));
            }
            if !g.start_heading.is_finite() {
                return invalid(at("start_heading must be a finite number"));
            }
            non_negative(&at("action_time"), g.action_time)?;
            for (t, v) in &g.action_times {
                non_negative(&at(&format!("action_times.{t}")), *v)?;
            }
        }
        Ok(())
    }

    /// The job generator for this scenario's `[jobs]` section (seeded with the
    /// scenario seed), or `None` without that section.
    pub fn job_generator(&self, graph: &Graph) -> Result<Option<JobGenerator>, ScenarioError> {
        let Some(j) = &self.jobs else {
            return Ok(None);
        };
        let mut nodes = Vec::new();
        for id in &j.nodes {
            match graph.node_idx(id) {
                Some(n) => nodes.push(n),
                None => return invalid(format!("[jobs] node \"{id}\" is not in the layout")),
            }
        }
        JobGenerator::new(self.seed, &nodes, j.rate_per_hour, self.time_step)
            .map(Some)
            .map_err(|e| ScenarioError::Invalid(format!("[jobs] {e}")))
    }

    /// `start_time` as milliseconds since the Unix epoch. It must be a full UTC
    /// date and time (`...Z`): an offset or a missing part would be a guess.
    pub fn start_unix_ms(&self) -> Result<i64, ScenarioError> {
        use toml::value::Offset;
        let t = &self.start_time;
        match (t.date, t.time, t.offset) {
            (Some(d), Some(tm), Some(Offset::Z)) => Ok(unix_ms_from_utc(
                d.year.into(),
                d.month.into(),
                d.day.into(),
                tm.hour.into(),
                tm.minute.into(),
                tm.second.unwrap_or(0).into(),
            )),
            _ => invalid(format!(
                "start_time must be a full UTC date and time like 2026-11-20T10:00:00Z, got {t}"
            )),
        }
    }

    /// Build the world: read the layout and the factsheets (paths relative to
    /// `base_dir`, normally the scenario file's folder) and place all robots.
    pub fn build_world(&self, base_dir: &Path) -> Result<World, ScenarioError> {
        let lif_path = base_dir.join(&self.lif);
        let lif: Lif = parse_json(&lif_path)?;
        let graph = Graph::from_lif(&lif).map_err(|e| ScenarioError::Input {
            path: lif_path.clone(),
            message: e.to_string(),
        })?;

        let mut world = World::new(graph, self.time_step, self.start_unix_ms()?)
            .map_err(|e| ScenarioError::Invalid(e.to_string()))?;
        if let Some(t) = self.deadlock_timeout {
            world.set_deadlock_timeout(t);
        }

        for (gi, g) in self.robots.iter().enumerate() {
            let path = base_dir.join(&g.factsheet);
            let factsheet: Factsheet = parse_json(&path)?;
            let spec = RobotSpec::from_factsheet(&factsheet).map_err(|e| ScenarioError::Input {
                path: path.clone(),
                message: e.to_string(),
            })?;
            // transport jobs use `pick` and `drop`: 5 s unless the scenario says otherwise
            let mut times = ActionTimes::new(g.action_time)
                .with("pick", DEFAULT_PICK_DROP_SECONDS)
                .with("drop", DEFAULT_PICK_DROP_SECONDS);
            for (t, v) in &g.action_times {
                times = times.with(t, *v);
            }
            for (i, node) in g.start_nodes.iter().enumerate() {
                let Some(start) = world.graph().node_idx(node) else {
                    return invalid(format!(
                        "robot group {}: start node \"{node}\" is not in the layout",
                        gi + 1
                    ));
                };
                let id = RobotId::new(
                    &g.manufacturer,
                    &format!("{}-{:03}", g.serial_prefix, i + 1),
                );
                world
                    .add_robot(id, spec.clone(), times.clone(), start, g.start_heading)
                    .map_err(|e| ScenarioError::Invalid(e.to_string()))?;
            }
        }
        Ok(world)
    }
}

fn read(path: &Path) -> Result<String, ScenarioError> {
    std::fs::read_to_string(path).map_err(|e| ScenarioError::Read {
        path: path.to_path_buf(),
        message: e.to_string(),
    })
}

fn parse_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T, ScenarioError> {
    serde_json::from_str(&read(path)?).map_err(|e| ScenarioError::Input {
        path: path.to_path_buf(),
        message: e.to_string(),
    })
}

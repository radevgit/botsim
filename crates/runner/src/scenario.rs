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
//! zone_set = "zones.json"        # optional VDA 5050 zone set; loaded and checked only
//!
//! [[lift]]                       # optional, any number: a lift between maps
//! id = "L1"
//! floors = ["Map_Z-Level_1", "Map_Z-Level_2"]   # mapIds, lowest first, >= 2
//! doors = ["front", "rear"]      # optional, default: ["front"]; one open at a time
//! start_floor = "Map_Z-Level_1"  # optional, default: the first floor
//! travel_seconds_per_floor = 5.0
//! door_open_seconds = 3.0
//! door_close_seconds = 3.0
//! door_hold_seconds = 20.0       # optional: the door closes by itself after this
//! ```
//!
//! ```toml
//!
//! [[robot]]                      # a group of robots of one type
//! manufacturer = "ACME"
//! serial_prefix = "R"            # serials R-001, R-002, ...
//! factsheet = "diff_robot.factsheet.json"
//! start_nodes = ["A", "B"]       # one per robot, so this also sets the count
//! start_heading = 0.0            # optional, radians, default 0
//! action_time = 2.0              # seconds an action takes, unless listed below
//! zone_requests = "on_release"   # optional: when to request a RELEASE zone:
//!                                # "on_release" (default) or "on_horizon"
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
//! ```toml
//! [disturbances]                 # optional: without it nothing is disturbed
//! enabled = true                 # switch; false turns the values off
//! speed_variation = 0.05         # each robot's top speed is lowered by 0 to 5 %
//! action_variation = 0.2         # every action takes up to 20 % longer (0 = none)
//! stops_per_hour = 2.0           # random short stops per hour of driving (0 = none)
//! stop_seconds_min = 5.0         # a stop lasts between these (defaults 5 and 15)
//! stop_seconds_max = 15.0
//! ```
//!
//! With `[jobs]` the bundled baseline controller runs: the seed (above) fixes
//! the random jobs, so a run is repeatable.
//!
//! Not yet: a choice of controller; robots riding a lift (the lift only exists
//! as a peripheral that the controller can call).

use std::collections::BTreeMap;
use std::fmt;
use std::path::{Path, PathBuf};

use botsim_core::actions::ActionTimes;
use botsim_core::disturbances::{Disturbances, StopSpec, check_duration_variation};
use botsim_core::graph::Graph;
use botsim_core::jobs::JobGenerator;
use botsim_core::lift::LiftSpec;
use botsim_core::spec::RobotSpec;
use botsim_core::world::{RobotId, World, unix_ms_from_utc};
use botsim_core::zone_requests::ZoneRequestTiming;
use botsim_core::zones::Zones;
use serde::Deserialize;
use vda5050::factsheet::Factsheet;
use vda5050::zone_set::ZoneSet;
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
    /// Optional VDA 5050 zone set (a JSON file with the `zoneSet` object, or a
    /// whole `zoneSet` message). It is loaded and checked; no behaviour uses the
    /// zones yet.
    #[serde(default)]
    pub zone_set: Option<PathBuf>,
    #[serde(rename = "robot")]
    pub robots: Vec<RobotGroup>,
    /// Without this section no jobs are created and no controller runs.
    #[serde(default)]
    pub jobs: Option<JobsConfig>,
    /// Seconds a cycle of robots waiting for each other must last to count as a
    /// deadlock; default 30 (S35).
    #[serde(default)]
    pub deadlock_timeout: Option<f64>,
    /// Without this section nothing is disturbed.
    #[serde(default)]
    pub disturbances: Option<DisturbancesConfig>,
    /// Lifts (peripherals with their own command port); see [`LiftConfig`].
    #[serde(default, rename = "lift")]
    pub lifts: Vec<LiftConfig>,
}

/// A `[[lift]]` section: a lift between maps (floors). The lift model is in
/// `botsim_core::lift`; no robot rides it yet.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LiftConfig {
    pub id: String,
    /// The maps (`mapId`) the lift stops at, lowest floor first; at least two.
    pub floors: Vec<String>,
    /// The names of the cabin's doors (S45), e.g. `["front", "rear"]` for a
    /// through-cabin. Default: one door, `front`. Only one door is open at a time.
    #[serde(default = "one_door")]
    pub doors: Vec<String>,
    /// The map the cabin starts at (default: the first floor).
    #[serde(default)]
    pub start_floor: Option<String>,
    /// Seconds for the cabin to pass one floor.
    pub travel_seconds_per_floor: f64,
    pub door_open_seconds: f64,
    pub door_close_seconds: f64,
    /// The door closes by itself after being open this long; without it the
    /// door stays open until the controller releases it.
    #[serde(default)]
    pub door_hold_seconds: Option<f64>,
}

/// The `[disturbances]` section (S36). Seeded by the scenario seed.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DisturbancesConfig {
    /// Switch: `false` keeps the values below in the file but turns them off.
    #[serde(default = "yes")]
    pub enabled: bool,
    /// Each robot's top speed is lowered by a random share of at most this
    /// (0.05 = up to 5 %); from 0 up to, not including, 1.
    #[serde(default)]
    pub speed_variation: f64,
    /// Every action (`pick`, `drop`, …) takes up to this much longer, drawn at
    /// random per action (S38): 0.2 = up to 20 %; 0 = none.
    #[serde(default)]
    pub action_variation: f64,
    /// Random short stops (S37): average number per hour of driving; 0 = none.
    #[serde(default)]
    pub stops_per_hour: f64,
    /// Length of a stop, drawn between these (seconds).
    #[serde(default = "default_stop_min")]
    pub stop_seconds_min: f64,
    #[serde(default = "default_stop_max")]
    pub stop_seconds_max: f64,
}

fn default_stop_min() -> f64 {
    5.0
}

fn default_stop_max() -> f64 {
    15.0
}

fn yes() -> bool {
    true
}

fn one_door() -> Vec<String> {
    vec!["front".to_string()]
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
    /// When these robots request a RELEASE zone (S41): `"on_release"` (default,
    /// when the edge that enters the zone is released) or `"on_horizon"` (as soon
    /// as it is in the order). A real fleet may contain both kinds.
    #[serde(default)]
    pub zone_requests: ZoneRequestsWhen,
}

/// The two values of `zone_requests` in a `[[robot]]` group.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ZoneRequestsWhen {
    #[default]
    OnRelease,
    OnHorizon,
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
        self.disturbances()?; // checks the values
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

        let zones = match &self.zone_set {
            Some(file) => {
                let path = base_dir.join(file);
                let set = load_zone_set(&path)?;
                Some(Zones::new(&graph, set).map_err(|e| ScenarioError::Input {
                    path: path.clone(),
                    message: e.to_string(),
                })?)
            }
            None => None,
        };

        let mut world = World::new(graph, self.time_step, self.start_unix_ms()?)
            .map_err(|e| ScenarioError::Invalid(e.to_string()))?;
        if let Some(z) = zones {
            world.set_zones(z);
        }
        if let Some(t) = self.deadlock_timeout {
            world.set_deadlock_timeout(t);
        }
        for l in &self.lifts {
            let map = |name: &str| {
                world.graph().map_idx(name).ok_or_else(|| {
                    ScenarioError::Invalid(format!(
                        "lift \"{}\": the layout has no map \"{name}\"",
                        l.id
                    ))
                })
            };
            let floors = l
                .floors
                .iter()
                .map(|f| map(f))
                .collect::<Result<Vec<_>, _>>()?;
            let start_floor = match &l.start_floor {
                None => 0,
                Some(s) => {
                    let m = map(s)?;
                    floors.iter().position(|f| *f == m).ok_or_else(|| {
                        ScenarioError::Invalid(format!(
                            "lift \"{}\": start_floor \"{s}\" is not one of its floors",
                            l.id
                        ))
                    })?
                }
            };
            let spec = LiftSpec {
                id: l.id.clone(),
                floors,
                doors: l.doors.clone(),
                start_floor,
                travel_seconds_per_floor: l.travel_seconds_per_floor,
                door_open_seconds: l.door_open_seconds,
                door_close_seconds: l.door_close_seconds,
                door_hold_seconds: l.door_hold_seconds,
            };
            world
                .add_lift(spec)
                .map_err(|e| ScenarioError::Invalid(format!("lift \"{}\": {e}", l.id)))?;
        }
        let mut disturbances = self.disturbances()?;
        if let Some(d) = &self.disturbances
            && d.enabled
            && d.action_variation > 0.0
        {
            world
                .enable_duration_variation(self.seed, d.action_variation)
                .map_err(|e| ScenarioError::Invalid(e.to_string()))?;
        }
        if let Some(spec) = self.stop_spec() {
            // robots added below get their stops too
            world
                .enable_random_stops(self.seed, spec)
                .map_err(|e| ScenarioError::Invalid(e.to_string()))?;
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
                // S36: this robot's top speed is lowered by a random share
                let spec = match disturbances.as_mut() {
                    Some(d) => spec.with_speed_factor(d.next_speed_factor()),
                    None => spec.clone(),
                };
                world
                    .add_robot(id.clone(), spec, times.clone(), start, g.start_heading)
                    .map_err(|e| ScenarioError::Invalid(e.to_string()))?;
                // S41: when this robot type requests RELEASE zones
                let timing = match g.zone_requests {
                    ZoneRequestsWhen::OnRelease => ZoneRequestTiming::OnRelease,
                    ZoneRequestsWhen::OnHorizon => ZoneRequestTiming::OnHorizon,
                };
                world
                    .set_zone_request_timing(&id, timing)
                    .map_err(|e| ScenarioError::Invalid(e.to_string()))?;
            }
        }
        Ok(world)
    }

    /// The disturbances to apply: `None` without a `[disturbances]` section or
    /// with `enabled = false`.
    fn disturbances(&self) -> Result<Option<Disturbances>, ScenarioError> {
        let Some(d) = &self.disturbances else {
            return Ok(None);
        };
        let made = Disturbances::new(self.seed, d.speed_variation)
            .map_err(|e| ScenarioError::Invalid(format!("[disturbances] {e}")))?;
        self.stop_spec_checked()?;
        check_duration_variation(d.action_variation)
            .map_err(|e| ScenarioError::Invalid(format!("[disturbances] {e}")))?;
        Ok(d.enabled.then_some(made))
    }

    /// The random stops to apply: `None` without a `[disturbances]` section,
    /// with `enabled = false` or with `stops_per_hour = 0`.
    pub fn stop_spec(&self) -> Option<StopSpec> {
        let d = self.disturbances.as_ref()?;
        (d.enabled && d.stops_per_hour > 0.0).then_some(StopSpec {
            per_hour: d.stops_per_hour,
            min_seconds: d.stop_seconds_min,
            max_seconds: d.stop_seconds_max,
        })
    }

    /// Check the stop settings (also when switched off or 0).
    fn stop_spec_checked(&self) -> Result<(), ScenarioError> {
        let Some(d) = &self.disturbances else {
            return Ok(());
        };
        StopSpec {
            per_hour: d.stops_per_hour,
            min_seconds: d.stop_seconds_min,
            max_seconds: d.stop_seconds_max,
        }
        .check(self.time_step)
        .map_err(|e| ScenarioError::Invalid(format!("[disturbances] {e}")))
    }
}

/// A zone set file: the `zoneSet` object, or a whole `zoneSet` message (the
/// header is ignored; it names one robot, the zones are for the whole run).
fn load_zone_set(path: &Path) -> Result<ZoneSet, ScenarioError> {
    let value: serde_json::Value = parse_json(path)?;
    let object = match value.get("zoneSet") {
        Some(inner) => inner.clone(),
        None => value,
    };
    serde_json::from_value(object).map_err(|e| ScenarioError::Input {
        path: path.to_path_buf(),
        message: e.to_string(),
    })
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

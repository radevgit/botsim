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
//! safety_stop = true             # optional, default true: robots brake for robots ahead
//! drive_through_nodes = true     # optional, default true: no stop at nodes straight on
//! message_delay = 0.0            # optional: seconds a message takes, each way (default 0)
//! stop_at_first_deadlock = false # optional debug: end the run when a deadlock is reported
//! zone_set = "zones.json"        # optional VDA 5050 zone set; loaded and checked only
//!
//! [[door]]                       # optional, any number: a door on layout edges
//! id = "D1"
//! edges = ["A-B", "B-A"]         # edgeIds through the door
//! passage_width = 1.0            # optional (m): wider robots cannot pass
//! open_seconds = 3.0
//! close_seconds = 3.0
//! hold_seconds = 20.0            # optional: releases itself after this long
//!
//! [[lift]]                       # optional, any number: a lift between maps
//! id = "L1"
//! floors = ["Map_Z-Level_1", "Map_Z-Level_2"]   # mapIds, lowest first, >= 2
//! doors = ["front", "rear"]      # optional, default: ["front"]; one open at a time
//! stops = [{ node = "N2", waiting = "W1" }, { node = "N102", door = "rear" }]   # optional: where robots ride
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
//! start_heading = 0.0            # optional, radians, default 0; or "road": face along
//!                                # the first road that leaves the start node
//! # start_headings = [0.0, 1.57] # optional: one heading per start node instead
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
//! nodes = ["A", "C"]             # where jobs start and end (>= 2 different), or
//! # stations = ["S1", "S2"]       # LIF stations instead (not both); with neither:
//! #                               # between all the stations of the layout
//! # pickup_stations = ["P1"]      # or: jobs from these stations to the drop
//! # drop_stations = ["D1", "D2"]  # stations (both lists, no nodes / stations)
//! # both_directions = true        # and also back, with equal chance
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
use botsim_core::battery::{Battery, BatterySpec};
use botsim_core::disturbances::{Disturbances, StopSpec, check_duration_variation};
use botsim_core::door::DoorSpec;
use botsim_core::jobs::{JobGenerator, Rng};
use botsim_core::lift::{LiftSpec, LiftStop};
use botsim_core::world::World;
use botsim_core::zone_requests::ZoneRequestTiming;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_model::time::unix_ms_from_utc;
use botsim_model::zones::Zones;
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
    /// How long a message takes between the controller and a robot, in each
    /// direction, in seconds (S58): orders and answers arrive at the robot that
    /// long after they were sent, and the controller reads states that are that
    /// old. Rounded to whole time steps. Default 0 (instant).
    #[serde(default)]
    pub message_delay: Option<f64>,
    /// Debug option (S59): end the run at the step when the first deadlock is
    /// reported (after `deadlock_timeout`), so that the situation can be looked
    /// at. The result says so (`stopped_at_first_deadlock`) and `ticks` is the
    /// number of steps actually run. Default `false`.
    #[serde(default)]
    pub stop_at_first_deadlock: bool,
    /// The robots' onboard safety stop (S54): a robot brakes when another robot
    /// is in its protective field. Default `true`; `false` lets robots drive
    /// into each other (to test collision detection).
    #[serde(default = "yes")]
    pub safety_stop: bool,
    /// Robots drive through nodes without stopping where they can (S66): straight
    /// on, nothing to do at the node, the next edge released and free, not in
    /// front of a door or a lift. Default `true`; `false` makes every robot stop at
    /// every node (slower, as before S66).
    #[serde(default = "yes")]
    pub drive_through_nodes: bool,
    /// Route with load (baseline): metres added to the cost of an edge for each
    /// other robot that holds it or the node it ends at, when a route is planned,
    /// so that an empty parallel lane is preferred to a busy one. Default 0 (the
    /// plain shortest route).
    #[serde(default)]
    pub route_load_weight: f64,
    /// Route with load, second part (baseline): metres added to the cost of an edge
    /// for each robot that has recently waited on the node it ends at (a moving
    /// average over 60 s). Default 0.
    #[serde(default)]
    pub route_wait_weight: f64,
    /// Swept-area reservation (baseline): a robot is released onto an edge only if
    /// no other robot holds an edge whose swept outline overlaps it or comes nearer
    /// than `conflict_clearance` (the conflict map of `botsim-space`, per pair of
    /// robot types). Default `true`; `false` reserves nodes and edges only. It needs
    /// the scenario file (the layout and factsheets are read again), so a scenario
    /// from text has none.
    #[serde(default = "yes")]
    pub conflict_reservation: bool,
    /// Charging power (W) of the stations of the layout whose `stationType` is
    /// `charger`; without it there are no chargers. A robot with a battery standing on
    /// the interaction node of such a station charges after `startCharging`.
    #[serde(default)]
    pub charger_power_w: Option<f64>,
    /// Baseline: a free robot whose charge is below this (percent) drives to a free
    /// charger and charges there. Default 25. Used only with `charger_power_w`.
    #[serde(default = "default_charge_low")]
    pub charge_low: f64,
    /// Swept-area reservation: two bodies nearer than this (m), overlap included,
    /// conflict. Default 0.10; 0 keeps only overlaps of 5 cm or more.
    #[serde(default = "default_conflict_clearance")]
    pub conflict_clearance: f64,
    /// The file the scenario was read from (set by [`Scenario::load`]).
    #[serde(skip)]
    pub source: Option<PathBuf>,
    /// Without this section nothing is disturbed.
    #[serde(default)]
    pub disturbances: Option<DisturbancesConfig>,
    /// Lifts (peripherals with their own command port); see [`LiftConfig`].
    #[serde(default, rename = "lift")]
    pub lifts: Vec<LiftConfig>,
    /// Doors (peripherals with their own command port); see [`DoorConfig`].
    #[serde(default, rename = "door")]
    pub doors: Vec<DoorConfig>,
}

/// A `[[door]]` section (S50): a door that guards layout edges. The door model
/// is in `botsim_core::door`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DoorConfig {
    pub id: String,
    /// The `edgeId`s that lead through the door (at least one, e.g. both
    /// directions of the passage).
    pub edges: Vec<String>,
    /// How wide the passage is (metres); a robot wider than this cannot pass
    /// (its order with such an edge is refused). Default: any robot.
    #[serde(default)]
    pub passage_width: Option<f64>,
    pub open_seconds: f64,
    pub close_seconds: f64,
    /// The door releases itself after being open this long; without it the door
    /// stays open until the controller releases it.
    #[serde(default)]
    pub hold_seconds: Option<f64>,
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
    /// Where robots get in and out (S46), e.g.
    /// `stops = [{ node = "N2" }, { node = "N102", door = "rear" }]`. The floor of
    /// a stop is the map of its node. A layout edge between two stops of the lift
    /// is a ride. Without stops no robot can ride the lift.
    #[serde(default)]
    pub stops: Vec<LiftStopConfig>,
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

/// A stop of a lift: the node where robots get in and out, and the door that
/// opens there.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LiftStopConfig {
    /// A `nodeId` of the layout, on one of the lift's floors.
    pub node: String,
    /// The door that opens at this stop (default: the lift's first door).
    #[serde(default)]
    pub door: Option<String>,
    /// A waiting node (lay-by) on the same map, off the lane to the stop (S49):
    /// the baseline controller routes robots through it and parks them there
    /// while the lift is busy.
    #[serde(default)]
    pub waiting: Option<String>,
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

fn default_conflict_clearance() -> f64 {
    0.10
}

fn default_charge_low() -> f64 {
    25.0
}

fn one_door() -> Vec<String> {
    vec!["front".to_string()]
}

/// The `[jobs]` section: random transport jobs between the listed nodes or
/// stations, handled by the baseline controller.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JobsConfig {
    /// Average number of new jobs per simulated hour.
    pub rate_per_hour: f64,
    /// Node ids where jobs start and end (at least two different ones).
    #[serde(default)]
    pub nodes: Vec<String>,
    /// Station ids (LIF `stationId`) where jobs start and end (S60): a job goes
    /// from one station to another, to one of their interaction nodes. Use
    /// `nodes` **or** `stations`. With neither, jobs go between all the stations
    /// of the layout.
    #[serde(default)]
    pub stations: Vec<String>,
    /// Station ids where jobs are picked up; with `drop_stations`, instead of
    /// `nodes` and `stations`: a job goes from one of these stations to one of the
    /// drop stations.
    #[serde(default)]
    pub pickup_stations: Vec<String>,
    /// Station ids where those jobs are dropped (none of them a pickup station).
    #[serde(default)]
    pub drop_stations: Vec<String>,
    /// With `pickup_stations` and `drop_stations`: also make jobs the other way
    /// round (picked up at a drop station, dropped at a pickup station), with equal
    /// chance. Default `false`.
    #[serde(default)]
    pub both_directions: bool,
}

/// The `start_heading` of a robot group: an angle in radians for all robots, or
/// `"road"` to face along the road that leaves the start node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StartHeading {
    Angle(f64),
    Road,
}

impl Default for StartHeading {
    fn default() -> Self {
        StartHeading::Angle(0.0)
    }
}

impl<'de> Deserialize<'de> for StartHeading {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        #[serde(untagged)]
        enum Raw {
            Angle(f64),
            Text(String),
        }
        match Raw::deserialize(d)? {
            Raw::Angle(a) => Ok(StartHeading::Angle(a)),
            Raw::Text(t) if t == "road" => Ok(StartHeading::Road),
            Raw::Text(t) => Err(serde::de::Error::custom(format!(
                "start_heading must be a number or \"road\", not \"{t}\""
            ))),
        }
    }
}

/// The heading (radians) a robot of type (`manufacturer`, `series`) starts with on
/// `node` so that it faces along the road: the direction at the start of the first
/// edge in the layout that leaves the node and that the type may drive (an edge
/// between maps comes last). `None` if no such edge leaves the node.
pub fn road_heading(
    graph: &botsim_model::graph::Graph,
    node: botsim_model::graph::NodeIdx,
    manufacturer: &str,
    series: &str,
) -> Option<f64> {
    let usable = |e: &&botsim_model::graph::EdgeIdx| graph.edge_allows(**e, manufacturer, series);
    let edges = graph.out_edges(node);
    let e = edges
        .iter()
        .filter(usable)
        .find(|&&e| !graph.edges[e].map_transition)
        .or_else(|| edges.iter().find(usable))?;
    Some(graph.edge_directions(*e, manufacturer, series).0)
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotGroup {
    pub manufacturer: String,
    pub serial_prefix: String,
    pub factsheet: PathBuf,
    pub start_nodes: Vec<String>,
    /// How the robots of the group face at the start: a number (radians, 0 = east,
    /// the same for all) or `"road"` (each faces along the first road that leaves
    /// its start node and that its type may drive; see [`road_heading`]). Default 0.
    #[serde(default)]
    pub start_heading: StartHeading,
    /// One heading (radians) for each start node, in the same order; instead of
    /// `start_heading` when it is not empty. Must have as many entries as
    /// `start_nodes`.
    #[serde(default)]
    pub start_headings: Vec<f64>,
    pub action_time: f64,
    #[serde(default)]
    pub action_times: BTreeMap<String, f64>,
    /// When these robots request a RELEASE zone (S41): `"on_release"` (default,
    /// when the edge that enters the zone is released) or `"on_horizon"` (as soon
    /// as it is in the order). A real fleet may contain both kinds.
    #[serde(default)]
    pub zone_requests: ZoneRequestsWhen,
    /// Battery capacity (Wh). Without it the robots of the group have no battery
    /// model (they report 100 % for ever); the other battery settings need it.
    #[serde(default)]
    pub battery_wh: Option<f64>,
    /// Energy used per metre driven (Wh/m); default 0.
    #[serde(default)]
    pub drive_wh_per_m: Option<f64>,
    /// Power while standing, rotating or waiting (W); default 0.
    #[serde(default)]
    pub idle_w: Option<f64>,
    /// Power while an action runs at a node (W); default 0.
    #[serde(default)]
    pub work_w: Option<f64>,
    /// The charge at the start, percent: a number (the same for all robots of the
    /// group) or a range `[low, high]` from which each robot gets a random value
    /// (drawn from the scenario seed: the same seed gives the same charges). Default
    /// 100. A random start keeps the robots from reaching a charge limit together.
    #[serde(default)]
    pub start_charge: Option<StartCharge>,
}

impl RobotGroup {
    /// The battery of this group: `None` without `battery_wh`; the text says why
    /// the settings cannot be used.
    pub fn battery_spec(&self) -> Result<Option<BatterySpec>, String> {
        let Some(capacity_wh) = self.battery_wh else {
            if self.drive_wh_per_m.is_some()
                || self.idle_w.is_some()
                || self.work_w.is_some()
                || self.start_charge.is_some()
            {
                return Err("battery settings need battery_wh".to_string());
            }
            return Ok(None);
        };
        let spec = BatterySpec {
            capacity_wh,
            drive_wh_per_m: self.drive_wh_per_m.unwrap_or(0.0),
            idle_w: self.idle_w.unwrap_or(0.0),
            work_w: self.work_w.unwrap_or(0.0),
        };
        spec.check().map_err(|e| e.to_string())?;
        if let Some(StartCharge(lo, hi)) = self.start_charge
            && !(lo.is_finite() && hi.is_finite() && 0.0 <= lo && lo <= hi && hi <= 100.0)
        {
            return Err(format!(
                "start_charge must be 0 to 100 percent, low not above high, got [{lo}, {hi}]"
            ));
        }
        Ok(Some(spec))
    }
}

/// `start_charge`: a percent range; a single number is the range with both ends equal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StartCharge(pub f64, pub f64);

impl<'de> Deserialize<'de> for StartCharge {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        #[serde(untagged)]
        enum Raw {
            One(f64),
            Range([f64; 2]),
        }
        Ok(match Raw::deserialize(d)? {
            Raw::One(v) => StartCharge(v, v),
            Raw::Range([lo, hi]) => StartCharge(lo, hi),
        })
    }
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
        let mut s = Scenario::from_toml(&text)?;
        s.source = Some(path.to_path_buf());
        Ok(s)
    }

    fn check(&self) -> Result<(), ScenarioError> {
        positive("duration", self.duration)?;
        positive("time_step", self.time_step)?;
        if !(self.charge_low.is_finite() && (0.0..=100.0).contains(&self.charge_low)) {
            return invalid(format!(
                "charge_low must be 0 to 100 percent, got {}",
                self.charge_low
            ));
        }
        if let Some(t) = self.deadlock_timeout {
            positive("deadlock_timeout", t)?;
        }
        if let Some(d) = self.message_delay
            && !(d.is_finite() && d >= 0.0)
        {
            return invalid(format!(
                "message_delay must be a finite number >= 0, got {d}"
            ));
        }
        self.disturbances()?; // checks the values
        self.start_unix_ms()?;
        if let Some(j) = &self.jobs {
            non_negative("[jobs] rate_per_hour", j.rate_per_hour)?;
            if !j.nodes.is_empty() && !j.stations.is_empty() {
                return invalid("[jobs] use nodes or stations, not both".into());
            }
            if !j.nodes.is_empty() && j.nodes.len() < 2 {
                return invalid("[jobs] nodes needs at least two nodes".into());
            }
            if !j.pickup_stations.is_empty() || !j.drop_stations.is_empty() {
                if j.pickup_stations.is_empty() || j.drop_stations.is_empty() {
                    return invalid("[jobs] pickup_stations and drop_stations go together".into());
                }
                if !j.nodes.is_empty() || !j.stations.is_empty() {
                    return invalid(
                        "[jobs] use pickup_stations and drop_stations without nodes or stations"
                            .into(),
                    );
                }
                if let Some(s) = j
                    .pickup_stations
                    .iter()
                    .find(|s| j.drop_stations.contains(s))
                {
                    return invalid(format!(
                        "[jobs] station \"{s}\" is both a pickup and a drop station"
                    ));
                }
            } else if j.both_directions {
                return invalid(
                    "[jobs] both_directions needs pickup_stations and drop_stations".into(),
                );
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
            if let StartHeading::Angle(a) = g.start_heading
                && !a.is_finite()
            {
                return invalid(at("start_heading must be a finite number"));
            }
            if !g.start_headings.is_empty() && g.start_headings.len() != g.start_nodes.len() {
                return invalid(at("start_headings needs one entry for each start node"));
            }
            if g.start_headings.iter().any(|a| !a.is_finite()) {
                return invalid(at("start_headings must be finite numbers"));
            }
            g.battery_spec()
                .map_err(|m| ScenarioError::Invalid(at(&m)))?;
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
        let too_few =
            |e: botsim_core::jobs::JobError| ScenarioError::Invalid(format!("[jobs] {e}"));
        if !j.pickup_stations.is_empty() {
            let places = |ids: &[String]| -> Result<Vec<Vec<_>>, ScenarioError> {
                ids.iter()
                    .map(|id| {
                        graph
                            .station_idx(id)
                            .map(|s| graph.stations[s].interaction_nodes.clone())
                            .ok_or_else(|| {
                                ScenarioError::Invalid(format!(
                                    "[jobs] station \"{id}\" is not in the layout"
                                ))
                            })
                    })
                    .collect()
            };
            return JobGenerator::between_groups(
                self.seed,
                &places(&j.pickup_stations)?,
                &places(&j.drop_stations)?,
                j.both_directions,
                j.rate_per_hour,
                self.time_step,
            )
            .map(Some)
            .map_err(too_few);
        }
        if j.nodes.is_empty() {
            // between stations: the listed ones, or all of the layout
            let stations: Vec<usize> = if j.stations.is_empty() {
                (0..graph.stations.len()).collect()
            } else {
                j.stations
                    .iter()
                    .map(|id| {
                        graph.station_idx(id).ok_or_else(|| {
                            ScenarioError::Invalid(format!(
                                "[jobs] station \"{id}\" is not in the layout"
                            ))
                        })
                    })
                    .collect::<Result<_, _>>()?
            };
            if stations.len() < 2 {
                return invalid(
                    "[jobs] needs `nodes`, or at least two stations (listed or in the layout)"
                        .into(),
                );
            }
            let places: Vec<Vec<_>> = stations
                .iter()
                .map(|&s| graph.stations[s].interaction_nodes.clone())
                .collect();
            return JobGenerator::between_places(
                self.seed,
                &places,
                j.rate_per_hour,
                self.time_step,
            )
            .map(Some)
            .map_err(too_few);
        }
        let mut nodes = Vec::new();
        for id in &j.nodes {
            match graph.node_idx(id) {
                Some(n) => nodes.push(n),
                None => return invalid(format!("[jobs] node \"{id}\" is not in the layout")),
            }
        }
        JobGenerator::new(self.seed, &nodes, j.rate_per_hour, self.time_step)
            .map(Some)
            .map_err(too_few)
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
            // the controller sends the zone set to the robots and enables it (S62)
            world.set_site_zones(z);
        }
        world.set_safety_stop(self.safety_stop);
        world.set_pass_through(self.drive_through_nodes);
        if let Some(d) = self.message_delay {
            world.set_message_delay((d / self.time_step).round() as u64);
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
            let bad = |what: String| ScenarioError::Invalid(format!("lift \"{}\": {what}", l.id));
            let mut stops = Vec::new();
            for s in &l.stops {
                let node = world
                    .graph()
                    .node_idx(&s.node)
                    .ok_or_else(|| bad(format!("the layout has no node \"{}\"", s.node)))?;
                let map = world.graph().nodes[node].map;
                let floor = floors.iter().position(|f| *f == map).ok_or_else(|| {
                    bad(format!(
                        "stop node \"{}\" is on a map that is not one of the lift's floors",
                        s.node
                    ))
                })?;
                let door = match &s.door {
                    None => 0,
                    Some(d) => l.doors.iter().position(|x| x == d).ok_or_else(|| {
                        bad(format!("stop \"{}\": the lift has no door \"{d}\"", s.node))
                    })?,
                };
                let waiting = match &s.waiting {
                    None => None,
                    Some(w) => Some(world.graph().node_idx(w).ok_or_else(|| {
                        bad(format!(
                            "stop \"{}\": the layout has no waiting node \"{w}\"",
                            s.node
                        ))
                    })?),
                };
                stops.push(LiftStop {
                    node,
                    floor,
                    door,
                    waiting,
                });
            }
            let spec = LiftSpec {
                id: l.id.clone(),
                floors,
                doors: l.doors.clone(),
                stops,
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
        for d in &self.doors {
            let edges = d
                .edges
                .iter()
                .map(|e| {
                    world.graph().edge_idx(e).ok_or_else(|| {
                        ScenarioError::Invalid(format!(
                            "door \"{}\": the layout has no edge \"{e}\"",
                            d.id
                        ))
                    })
                })
                .collect::<Result<Vec<_>, _>>()?;
            world
                .add_door(DoorSpec {
                    id: d.id.clone(),
                    edges,
                    passage_width: d.passage_width,
                    open_seconds: d.open_seconds,
                    close_seconds: d.close_seconds,
                    hold_seconds: d.hold_seconds,
                })
                .map_err(|e| ScenarioError::Invalid(format!("door \"{}\": {e}", d.id)))?;
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

        // an own random stream: runs without batteries are not changed by it
        let mut battery_rng = Rng::stream(self.seed, "battery");
        for (gi, g) in self.robots.iter().enumerate() {
            let battery = g
                .battery_spec()
                .map_err(|m| ScenarioError::Invalid(format!("robot group {}: {m}", gi + 1)))?;
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
                let heading = match (g.start_headings.get(i), g.start_heading) {
                    (Some(&a), _) | (None, StartHeading::Angle(a)) => a,
                    // a node without a road: east, as the default
                    (None, StartHeading::Road) => road_heading(
                        world.graph(),
                        start,
                        &factsheet.manufacturer,
                        &factsheet.type_specification.series_name,
                    )
                    .unwrap_or(0.0),
                };
                world
                    .add_robot(id.clone(), spec, times.clone(), start, heading)
                    .map_err(|e| ScenarioError::Invalid(e.to_string()))?;
                // S41: when this robot type requests RELEASE zones
                let timing = match g.zone_requests {
                    ZoneRequestsWhen::OnRelease => ZoneRequestTiming::OnRelease,
                    ZoneRequestsWhen::OnHorizon => ZoneRequestTiming::OnHorizon,
                };
                world
                    .set_zone_request_timing(&id, timing)
                    .map_err(|e| ScenarioError::Invalid(e.to_string()))?;
                // the battery, with a start charge drawn from the seed
                if let Some(bspec) = battery {
                    let StartCharge(lo, hi) = g.start_charge.unwrap_or(StartCharge(100.0, 100.0));
                    let percent = lo + (hi - lo) * battery_rng.next_f64();
                    let b = Battery::new(bspec, percent)
                        .map_err(|e| ScenarioError::Invalid(e.to_string()))?;
                    world
                        .set_battery(&id, b)
                        .map_err(|e| ScenarioError::Invalid(e.to_string()))?;
                }
            }
        }
        if let Some(p) = self.charger_power_w {
            world
                .set_chargers(p)
                .map_err(|e| ScenarioError::Invalid(e.to_string()))?;
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

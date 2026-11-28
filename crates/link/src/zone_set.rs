//! A zone set: the areas of one map that have rules of their own (VDA 5050
//! `zoneSet`).
//!
//! Nothing is checked here: whether the polygons and numbers make sense is for the
//! reader to check.

use std::sync::Arc;

use botsim_model::graph::MapIdx;
use serde_json::Value;

/// VDA 5050 `releaseLossBehavior` of a release zone.
pub use botsim_model::zones::LossBehavior;

use crate::ids::{ActionType, ZoneId, ZoneSetId};
use crate::order::Blocking;

#[derive(Debug, Clone, PartialEq)]
pub struct ZoneSet {
    pub id: ZoneSetId,
    pub map: MapIdx,
    pub descriptor: Option<Arc<str>>,
    pub zones: Vec<Zone>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Zone {
    pub id: ZoneId,
    pub descriptor: Option<Arc<str>>,
    /// The corners of the polygon, `(x, y)` in metres on the map.
    pub vertices: Vec<(f64, f64)>,
    pub kind: ZoneKind,
}

/// VDA 5050 `zoneType` with the parameters of that type.
#[derive(Debug, Clone, PartialEq)]
pub enum ZoneKind {
    Blocked,
    LineGuided,
    /// A robot needs a release to enter.
    Release {
        /// What the robot does when it loses the release, if the zone says it.
        loss: Option<LossBehavior>,
    },
    CoordinatedReplanning,
    SpeedLimit {
        /// m/s.
        maximum_speed: f64,
    },
    Action {
        entry: Vec<ZoneAction>,
        during: Vec<ZoneAction>,
        exit: Vec<ZoneAction>,
    },
    Priority {
        factor: f64,
    },
    Penalty {
        factor: f64,
    },
    Directed {
        /// rad.
        direction: f64,
        limitation: Directed,
    },
    Bidirected {
        /// rad.
        direction: f64,
        limitation: Bidirected,
    },
}

/// VDA 5050 `directedLimitation`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Directed {
    Soft,
    Restricted,
    Strict,
}

/// VDA 5050 `bidirectedLimitation`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bidirected {
    Soft,
    Restricted,
}

/// An action of an action zone. It has no id.
#[derive(Debug, Clone, PartialEq)]
pub struct ZoneAction {
    pub kind: ActionType,
    pub descriptor: Option<Arc<str>>,
    pub blocking: Blocking,
    /// `actionParameters` as key and value; a value can be any JSON value.
    pub parameters: Option<Vec<(Arc<str>, Value)>>,
    pub retriable: Option<bool>,
}

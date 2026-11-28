//! A zone set: the areas of one map that have rules of their own (VDA 5050
//! `zoneSet`).
//!
//! Nothing is checked here: whether the polygons and numbers make sense is for the
//! reader to check.

use std::hash::{Hash, Hasher};
use std::sync::Arc;

use botsim_model::graph::MapIdx;
use serde_json::Value;

/// VDA 5050 `releaseLossBehavior` of a release zone.
pub use botsim_model::zones::LossBehavior;

use crate::ids::{ActionType, ZoneId, ZoneSetId};
use crate::order::Blocking;

#[derive(Debug, Clone, PartialEq, Hash)]
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

impl Hash for Zone {
    fn hash<H: Hasher>(&self, h: &mut H) {
        let Zone {
            id,
            descriptor,
            vertices,
            kind,
        } = self;
        id.hash(h);
        descriptor.hash(h);
        vertices.len().hash(h);
        for (x, y) in vertices {
            x.to_bits().hash(h);
            y.to_bits().hash(h);
        }
        kind.hash(h);
    }
}

impl Hash for ZoneKind {
    fn hash<H: Hasher>(&self, h: &mut H) {
        std::mem::discriminant(self).hash(h);
        match self {
            ZoneKind::Blocked | ZoneKind::LineGuided | ZoneKind::CoordinatedReplanning => {}
            ZoneKind::Release { loss } => loss.hash(h),
            ZoneKind::SpeedLimit { maximum_speed } => maximum_speed.to_bits().hash(h),
            ZoneKind::Action {
                entry,
                during,
                exit,
            } => {
                entry.hash(h);
                during.hash(h);
                exit.hash(h);
            }
            ZoneKind::Priority { factor } | ZoneKind::Penalty { factor } => {
                factor.to_bits().hash(h);
            }
            ZoneKind::Directed {
                direction,
                limitation,
            } => {
                direction.to_bits().hash(h);
                limitation.hash(h);
            }
            ZoneKind::Bidirected {
                direction,
                limitation,
            } => {
                direction.to_bits().hash(h);
                limitation.hash(h);
            }
        }
    }
}

/// VDA 5050 `directedLimitation`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Directed {
    Soft,
    Restricted,
    Strict,
}

/// VDA 5050 `bidirectedLimitation`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Bidirected {
    Soft,
    Restricted,
}

/// An action of an action zone. It has no id.
#[derive(Debug, Clone, PartialEq, Hash)]
pub struct ZoneAction {
    pub kind: ActionType,
    pub descriptor: Option<Arc<str>>,
    pub blocking: Blocking,
    /// `actionParameters` as key and value; a value can be any JSON value.
    pub parameters: Option<Vec<(Arc<str>, Value)>>,
    pub retriable: Option<bool>,
}

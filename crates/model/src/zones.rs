//! Zones of a map (VDA 5050 `zoneSet`): loaded and checked.
//!
//! [`Zones`] is the zone set of a site's configuration, in the data types of the
//! `vda5050` crate ([`ZoneSet`], [`Zone`], [`ZoneKind`]). The checks and the
//! geometry also work on plain corner points ([`check_zone`], [`contains_point`],
//! [`enters`]), and [`Area`] is a zone in plain data. Checks made when a zone set
//! is loaded:
//! - its `mapId` is a map of the layout;
//! - zone ids are unique;
//! - every polygon has at least 3 vertices, all finite, no zero-length side,
//!   a non-zero area, and does not cross itself;
//! - type parameters make sense: `SPEED_LIMIT` speed finite and > 0, `PRIORITY`
//!   and `PENALTY` factors from 0 to 1, directions finite.
//!
//! A zone is a filled shape; **a point on its border is inside**.
//!
//! One zone set is for one map. Several maps would need one zone set each (not
//! supported yet).

use std::fmt;

use vda5050::zone_set::{Vertex, Zone, ZoneKind, ZoneReleaseLossBehavior, ZoneSet};

use crate::graph::{Graph, MapIdx};

/// VDA 5050 `releaseLossBehavior` of a release zone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LossBehavior {
    Stop,
    Continue,
    Evacuate,
}

/// A parameter of a zone type that is checked.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Parameter {
    /// `maximumSpeed` of a `SPEED_LIMIT` zone, m/s.
    MaximumSpeed(f64),
    /// `priorityFactor` of a `PRIORITY` zone.
    PriorityFactor(f64),
    /// `penaltyFactor` of a `PENALTY` zone.
    PenaltyFactor(f64),
    /// `direction` of a `DIRECTED` or `BIDIRECTED` zone, rad.
    Direction(f64),
}

/// A zone in plain data: what a robot uses of one.
#[derive(Debug, Clone, PartialEq)]
pub struct Area {
    /// `zoneId`.
    pub id: String,
    /// The corners of the polygon, `(x, y)` in metres on the map.
    pub corners: Vec<(f64, f64)>,
    pub kind: AreaKind,
}

/// What a robot makes of a zone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AreaKind {
    /// A `RELEASE` zone, with what a robot does when it loses the release, if the
    /// zone says it.
    Release { loss: Option<LossBehavior> },
    /// Any other zone type.
    Other,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ZoneError {
    /// The zone set is for a map the layout does not have.
    UnknownMap(String),
    DuplicateZone(String),
    /// Fewer than 3 vertices.
    TooFewVertices(String),
    /// A coordinate is not a finite number.
    BadVertex(String),
    /// Two vertices in a row are the same point.
    RepeatedVertex(String),
    /// The polygon has no area (all vertices on a line).
    NoArea(String),
    /// Two sides of the polygon cross or touch.
    SelfIntersecting(String),
    /// A type parameter is out of range; the text says which.
    BadParameter {
        zone: String,
        what: &'static str,
    },
}

impl fmt::Display for ZoneError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ZoneError::UnknownMap(m) => write!(
                f,
                "the zone set is for map \"{m}\", which the layout does not have"
            ),
            ZoneError::DuplicateZone(z) => write!(f, "zone \"{z}\" appears twice"),
            ZoneError::TooFewVertices(z) => write!(f, "zone \"{z}\" needs at least 3 vertices"),
            ZoneError::BadVertex(z) => {
                write!(f, "zone \"{z}\" has a vertex that is not a finite number")
            }
            ZoneError::RepeatedVertex(z) => {
                write!(f, "zone \"{z}\" has the same vertex twice in a row")
            }
            ZoneError::NoArea(z) => {
                write!(f, "zone \"{z}\" has no area (its vertices lie on a line)")
            }
            ZoneError::SelfIntersecting(z) => write!(f, "zone \"{z}\" crosses itself"),
            ZoneError::BadParameter { zone, what } => write!(f, "zone \"{zone}\": {what}"),
        }
    }
}

impl std::error::Error for ZoneError {}

/// The checked zones of one map.
#[derive(Debug, Clone, PartialEq)]
pub struct Zones {
    pub map: MapIdx,
    pub zone_set_id: String,
    pub zones: Vec<Zone>,
}

impl Zones {
    /// Check `set` against the layout.
    pub fn new(graph: &Graph, set: ZoneSet) -> Result<Zones, ZoneError> {
        let map = graph
            .map_idx(&set.map_id)
            .ok_or_else(|| ZoneError::UnknownMap(set.map_id.clone()))?;
        for (i, z) in set.zones.iter().enumerate() {
            if set.zones[..i].iter().any(|o| o.zone_id == z.zone_id) {
                return Err(ZoneError::DuplicateZone(z.zone_id.clone()));
            }
            check(&z.zone_id, &z.vertices, parameter(&z.kind))?;
        }
        Ok(Zones {
            map,
            zone_set_id: set.zone_set_id,
            zones: set.zones,
        })
    }

    pub fn zone(&self, zone_id: &str) -> Option<&Zone> {
        self.zones.iter().find(|z| z.zone_id == zone_id)
    }

    /// Is the point inside the zone (or on its border)?
    pub fn contains(zone: &Zone, x: f64, y: f64) -> bool {
        inside(&zone.vertices, x, y)
    }

    /// The `RELEASE` zones that a straight move from `a` to `b` **enters**: it
    /// starts outside the zone and reaches its inside or border.
    pub fn release_zones_entered(&self, a: (f64, f64), b: (f64, f64)) -> Vec<usize> {
        (0..self.zones.len())
            .filter(|&i| matches!(self.zones[i].kind, ZoneKind::Release { .. }))
            .filter(|&i| entered(&self.zones[i].vertices, a, b))
            .collect()
    }

    /// Indices of the zones that contain the point (in this zone set's map).
    pub fn at(&self, x: f64, y: f64) -> Vec<usize> {
        (0..self.zones.len())
            .filter(|&i| inside(&self.zones[i].vertices, x, y))
            .collect()
    }

    /// The zones in plain data, in their order.
    pub fn areas(&self) -> Vec<Area> {
        self.zones
            .iter()
            .map(|z| Area {
                id: z.zone_id.clone(),
                corners: z.vertices.iter().map(|v| (v.x, v.y)).collect(),
                kind: match &z.kind {
                    ZoneKind::Release {
                        release_loss_behavior,
                    } => AreaKind::Release {
                        loss: release_loss_behavior.map(loss),
                    },
                    _ => AreaKind::Other,
                },
            })
            .collect()
    }
}

fn loss(l: ZoneReleaseLossBehavior) -> LossBehavior {
    match l {
        ZoneReleaseLossBehavior::Stop => LossBehavior::Stop,
        ZoneReleaseLossBehavior::Continue => LossBehavior::Continue,
        ZoneReleaseLossBehavior::Evacuate => LossBehavior::Evacuate,
    }
}

/// The parameter of a zone type that is checked.
fn parameter(kind: &ZoneKind) -> Option<Parameter> {
    match *kind {
        ZoneKind::SpeedLimit { maximum_speed } => Some(Parameter::MaximumSpeed(maximum_speed)),
        ZoneKind::Priority { priority_factor } => Some(Parameter::PriorityFactor(priority_factor)),
        ZoneKind::Penalty { penalty_factor } => Some(Parameter::PenaltyFactor(penalty_factor)),
        ZoneKind::Directed { direction, .. } | ZoneKind::Bidirected { direction, .. } => {
            Some(Parameter::Direction(direction))
        }
        _ => None,
    }
}

/// The checks of one zone: its polygon, then the parameter of its type. That the
/// ids of a zone set differ is for the caller.
pub fn check_zone(
    id: &str,
    corners: &[(f64, f64)],
    parameter: Option<Parameter>,
) -> Result<(), ZoneError> {
    check(id, corners, parameter)
}

/// Is the point inside the polygon (or on its border)?
pub fn contains_point(corners: &[(f64, f64)], x: f64, y: f64) -> bool {
    inside(corners, x, y)
}

/// Does a straight move from `a` to `b` enter the polygon: it starts outside and
/// reaches its inside or border?
pub fn enters(corners: &[(f64, f64)], a: (f64, f64), b: (f64, f64)) -> bool {
    entered(corners, a, b)
}

/// A corner of a polygon, in either form.
trait Corner {
    fn point(&self) -> (f64, f64);
}

impl Corner for (f64, f64) {
    fn point(&self) -> (f64, f64) {
        *self
    }
}

impl Corner for Vertex {
    fn point(&self) -> (f64, f64) {
        (self.x, self.y)
    }
}

fn check<C: Corner>(id: &str, v: &[C], parameter: Option<Parameter>) -> Result<(), ZoneError> {
    check_polygon(id, v)?;
    match parameter {
        Some(p) => check_parameter(id, p),
        None => Ok(()),
    }
}

fn check_polygon<C: Corner>(id: &str, v: &[C]) -> Result<(), ZoneError> {
    let id = || id.to_string();
    if v.len() < 3 {
        return Err(ZoneError::TooFewVertices(id()));
    }
    let v: Vec<(f64, f64)> = v.iter().map(Corner::point).collect();
    if v.iter().any(|p| !(p.0.is_finite() && p.1.is_finite())) {
        return Err(ZoneError::BadVertex(id()));
    }
    let n = v.len();
    // a closing repeat (last = first) is a repeat too
    if (0..n).any(|i| same(v[i], v[(i + 1) % n])) {
        return Err(ZoneError::RepeatedVertex(id()));
    }
    for i in 0..n {
        for j in i + 1..n {
            let adjacent = j == i + 1 || (i == 0 && j == n - 1);
            if adjacent {
                continue;
            }
            let (a, b) = (v[i], v[(i + 1) % n]);
            let (c, d) = (v[j], v[(j + 1) % n]);
            if segments_meet(a, b, c, d) {
                return Err(ZoneError::SelfIntersecting(id()));
            }
        }
    }
    // after the crossing test: a bow tie has a signed area of 0 as well
    if area2(&v) == 0.0 {
        return Err(ZoneError::NoArea(id()));
    }
    Ok(())
}

fn check_parameter(id: &str, parameter: Parameter) -> Result<(), ZoneError> {
    let bad = |what| {
        Err(ZoneError::BadParameter {
            zone: id.to_string(),
            what,
        })
    };
    match parameter {
        Parameter::MaximumSpeed(v) if !(v.is_finite() && v > 0.0) => {
            bad("maximumSpeed must be a number > 0")
        }
        Parameter::PriorityFactor(f) if !(0.0..=1.0).contains(&f) => {
            bad("priorityFactor must be from 0 to 1")
        }
        Parameter::PenaltyFactor(f) if !(0.0..=1.0).contains(&f) => {
            bad("penaltyFactor must be from 0 to 1")
        }
        Parameter::Direction(d) if !d.is_finite() => bad("direction must be a finite number"),
        _ => Ok(()),
    }
}

fn same(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9
}

/// Twice the signed area (shoelace); 0 for a polygon on a line.
fn area2(v: &[(f64, f64)]) -> f64 {
    let n = v.len();
    (0..n)
        .map(|i| v[i].0 * v[(i + 1) % n].1 - v[(i + 1) % n].0 * v[i].1)
        .sum()
}

/// Cross product of (b - a) and (c - a): > 0 if c is left of a→b.
fn cross(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

/// Is `p` on the segment a–b (it is known to be on the line)?
fn within(a: (f64, f64), b: (f64, f64), p: (f64, f64)) -> bool {
    p.0 >= a.0.min(b.0) && p.0 <= a.0.max(b.0) && p.1 >= a.1.min(b.1) && p.1 <= a.1.max(b.1)
}

/// Do the segments a–b and c–d share a point (crossing, touching or overlapping)?
fn segments_meet(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> bool {
    let (d1, d2) = (cross(a, b, c), cross(a, b, d));
    let (d3, d4) = (cross(c, d, a), cross(c, d, b));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    (d1 == 0.0 && within(a, b, c))
        || (d2 == 0.0 && within(a, b, d))
        || (d3 == 0.0 && within(c, d, a))
        || (d4 == 0.0 && within(c, d, b))
}

/// See [`enters`].
fn entered<C: Corner>(v: &[C], a: (f64, f64), b: (f64, f64)) -> bool {
    if inside(v, a.0, a.1) {
        return false; // already inside: not an entry
    }
    let n = v.len();
    inside(v, b.0, b.1) || (0..n).any(|k| segments_meet(a, b, v[k].point(), v[(k + 1) % n].point()))
}

/// Point in polygon by ray casting; a point on the border counts as inside.
fn inside<C: Corner>(v: &[C], x: f64, y: f64) -> bool {
    let p = (x, y);
    let n = v.len();
    let mut inside = false;
    for i in 0..n {
        let (a, b) = (v[i].point(), v[(i + 1) % n].point());
        if cross(a, b, p) == 0.0 && within(a, b, p) {
            return true; // on the border
        }
        if (a.1 > y) != (b.1 > y) && x < (b.0 - a.0) * (y - a.1) / (b.1 - a.1) + a.0 {
            inside = !inside;
        }
    }
    inside
}

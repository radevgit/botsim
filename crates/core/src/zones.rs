//! Zones of a map (VDA 5050 `zoneSet`): loaded and checked. No robot behaviour
//! uses them yet (RELEASE zones come next).
//!
//! The data types are those of the `vda5050` crate ([`ZoneSet`], [`Zone`],
//! [`ZoneKind`]). Checks made when a zone set is loaded:
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

use vda5050::zone_set::{Vertex, Zone, ZoneKind, ZoneSet};

use botsim_model::graph::{Graph, MapIdx};

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
            check_polygon(z)?;
            check_parameters(z)?;
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
        let (pa, pb) = (Vertex { x: a.0, y: a.1 }, Vertex { x: b.0, y: b.1 });
        (0..self.zones.len())
            .filter(|&i| matches!(self.zones[i].kind, ZoneKind::Release { .. }))
            .filter(|&i| {
                let v = &self.zones[i].vertices;
                if inside(v, a.0, a.1) {
                    return false; // already inside: not an entry
                }
                let n = v.len();
                inside(v, b.0, b.1)
                    || (0..n).any(|k| segments_meet(&pa, &pb, &v[k], &v[(k + 1) % n]))
            })
            .collect()
    }

    /// Indices of the zones that contain the point (in this zone set's map).
    pub fn at(&self, x: f64, y: f64) -> Vec<usize> {
        (0..self.zones.len())
            .filter(|&i| inside(&self.zones[i].vertices, x, y))
            .collect()
    }
}

fn check_polygon(z: &Zone) -> Result<(), ZoneError> {
    let id = || z.zone_id.clone();
    let v = &z.vertices;
    if v.len() < 3 {
        return Err(ZoneError::TooFewVertices(id()));
    }
    if v.iter().any(|p| !(p.x.is_finite() && p.y.is_finite())) {
        return Err(ZoneError::BadVertex(id()));
    }
    let n = v.len();
    // a closing repeat (last = first) is a repeat too
    if (0..n).any(|i| same(&v[i], &v[(i + 1) % n])) {
        return Err(ZoneError::RepeatedVertex(id()));
    }
    for i in 0..n {
        for j in i + 1..n {
            let adjacent = j == i + 1 || (i == 0 && j == n - 1);
            if adjacent {
                continue;
            }
            let (a, b) = (&v[i], &v[(i + 1) % n]);
            let (c, d) = (&v[j], &v[(j + 1) % n]);
            if segments_meet(a, b, c, d) {
                return Err(ZoneError::SelfIntersecting(id()));
            }
        }
    }
    // after the crossing test: a bow tie has a signed area of 0 as well
    if area2(v) == 0.0 {
        return Err(ZoneError::NoArea(id()));
    }
    Ok(())
}

fn check_parameters(z: &Zone) -> Result<(), ZoneError> {
    let bad = |what| {
        Err(ZoneError::BadParameter {
            zone: z.zone_id.clone(),
            what,
        })
    };
    match &z.kind {
        ZoneKind::SpeedLimit { maximum_speed }
            if !(maximum_speed.is_finite() && *maximum_speed > 0.0) =>
        {
            bad("maximumSpeed must be a number > 0")
        }
        ZoneKind::Priority { priority_factor } if !(0.0..=1.0).contains(priority_factor) => {
            bad("priorityFactor must be from 0 to 1")
        }
        ZoneKind::Penalty { penalty_factor } if !(0.0..=1.0).contains(penalty_factor) => {
            bad("penaltyFactor must be from 0 to 1")
        }
        ZoneKind::Directed { direction, .. } | ZoneKind::Bidirected { direction, .. }
            if !direction.is_finite() =>
        {
            bad("direction must be a finite number")
        }
        _ => Ok(()),
    }
}

fn same(a: &Vertex, b: &Vertex) -> bool {
    (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9
}

/// Twice the signed area (shoelace); 0 for a polygon on a line.
fn area2(v: &[Vertex]) -> f64 {
    let n = v.len();
    (0..n)
        .map(|i| v[i].x * v[(i + 1) % n].y - v[(i + 1) % n].x * v[i].y)
        .sum()
}

/// Cross product of (b - a) and (c - a): > 0 if c is left of a→b.
fn cross(a: &Vertex, b: &Vertex, c: &Vertex) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

/// Is `p` on the segment a–b (it is known to be on the line)?
fn within(a: &Vertex, b: &Vertex, p: &Vertex) -> bool {
    p.x >= a.x.min(b.x) && p.x <= a.x.max(b.x) && p.y >= a.y.min(b.y) && p.y <= a.y.max(b.y)
}

/// Do the segments a–b and c–d share a point (crossing, touching or overlapping)?
fn segments_meet(a: &Vertex, b: &Vertex, c: &Vertex, d: &Vertex) -> bool {
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

/// Point in polygon by ray casting; a point on the border counts as inside.
fn inside(v: &[Vertex], x: f64, y: f64) -> bool {
    let p = Vertex { x, y };
    let n = v.len();
    let mut inside = false;
    for i in 0..n {
        let (a, b) = (&v[i], &v[(i + 1) % n]);
        if cross(a, b, &p) == 0.0 && within(a, b, &p) {
            return true; // on the border
        }
        if (a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
    }
    inside
}

//! A zone set as a simulated robot keeps it: of each zone what the robot uses,
//! its id, its polygon and whether it must be released to enter.
//!
//! It is made from the zone set of a `zoneSet` message
//! ([`RobotZones::from_message`]), checked there as the zone set of a site's
//! configuration is ([`botsim_model::zones`]), or from that checked configuration
//! itself ([`RobotZones::from_setup`]).

use std::collections::HashSet;

use botsim_link::ids::{ZoneId, ZoneSetId};
use botsim_link::zone_set::{ZoneKind, ZoneSet};
use botsim_model::graph::{Graph, MapIdx};
use botsim_model::zones::{
    AreaKind, Parameter, ZoneError, Zones, check_zone, contains_point, enters,
};

#[derive(Debug, Clone, PartialEq)]
pub struct RobotZones {
    pub id: ZoneSetId,
    pub map: MapIdx,
    pub zones: Vec<RobotZone>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RobotZone {
    pub id: ZoneId,
    /// The corners of the polygon, `(x, y)` in metres on the map.
    pub corners: Vec<(f64, f64)>,
    pub kind: AreaKind,
}

impl RobotZones {
    /// The zone set of a site's configuration.
    pub fn from_setup(zones: &Zones) -> RobotZones {
        RobotZones {
            id: ZoneSetId::new(zones.zone_set_id.as_str()),
            map: zones.map,
            zones: zones
                .areas()
                .into_iter()
                .map(|a| RobotZone {
                    id: ZoneId::new(a.id),
                    corners: a.corners,
                    kind: a.kind,
                })
                .collect(),
        }
    }

    /// The zone set `set` of a message, checked against the layout.
    pub fn from_message(graph: &Graph, set: &ZoneSet) -> Result<RobotZones, ZoneError> {
        if set.map >= graph.maps.len() {
            return Err(ZoneError::UnknownMap(format!("number {}", set.map)));
        }
        let mut seen = HashSet::new();
        let mut zones = Vec::with_capacity(set.zones.len());
        for z in &set.zones {
            if !seen.insert(z.id.as_str()) {
                return Err(ZoneError::DuplicateZone(z.id.as_str().to_string()));
            }
            let parameter = match z.kind {
                ZoneKind::SpeedLimit { maximum_speed } => {
                    Some(Parameter::MaximumSpeed(maximum_speed))
                }
                ZoneKind::Priority { factor } => Some(Parameter::PriorityFactor(factor)),
                ZoneKind::Penalty { factor } => Some(Parameter::PenaltyFactor(factor)),
                ZoneKind::Directed { direction, .. } | ZoneKind::Bidirected { direction, .. } => {
                    Some(Parameter::Direction(direction))
                }
                _ => None,
            };
            check_zone(z.id.as_str(), &z.vertices, parameter)?;
            zones.push(RobotZone {
                id: z.id.clone(),
                corners: z.vertices.clone(),
                kind: match z.kind {
                    ZoneKind::Release { loss } => AreaKind::Release { loss },
                    _ => AreaKind::Other,
                },
            });
        }
        Ok(RobotZones {
            id: set.id.clone(),
            map: set.map,
            zones,
        })
    }

    /// Indices of the zones that contain the point (in this zone set's map).
    pub fn at(&self, x: f64, y: f64) -> Vec<usize> {
        (0..self.zones.len())
            .filter(|&i| contains_point(&self.zones[i].corners, x, y))
            .collect()
    }

    /// The `RELEASE` zones that a straight move from `a` to `b` **enters**: it
    /// starts outside the zone and reaches its inside or border.
    pub fn release_zones_entered(&self, a: (f64, f64), b: (f64, f64)) -> Vec<usize> {
        (0..self.zones.len())
            .filter(|&i| matches!(self.zones[i].kind, AreaKind::Release { .. }))
            .filter(|&i| enters(&self.zones[i].corners, a, b))
            .collect()
    }
}

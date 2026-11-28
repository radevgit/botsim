//! `zoneSet`. It is taken whole or refused.
//!
//! Outgoing, the message must be one the schema allows. Incoming, only what `link`
//! needs is checked (ids, the map's name, finite numbers): whether the polygons and
//! parameters make sense is for the robot.

use std::sync::Arc;

use botsim_link::ids::{ActionType, ZoneId, ZoneSetId};
use botsim_link::message::{Header, ToRobot, ToRobotBody};
use botsim_link::zone_set::{
    Bidirected, Directed, LossBehavior, Zone, ZoneAction, ZoneKind, ZoneSet,
};
use botsim_model::errors::ErrorType;
use vda5050::zone_set as vda;

use crate::header::{Incoming, finite, usable};
use crate::order::{blocking_from_vda, blocking_to_vda};
use crate::report::{Issue, Problem, Translated};
use crate::translator::Translator;

const ZONE_SET_ID: &str = "zoneSet.zoneSetId";
const ZONE_ID: &str = "zones[].zoneId";
const VERTEX: &str = "zones[].vertices[]";
const MAXIMUM_SPEED: &str = "zones[].maximumSpeed";
const PRIORITY_FACTOR: &str = "zones[].priorityFactor";
const PENALTY_FACTOR: &str = "zones[].penaltyFactor";
const DIRECTION: &str = "zones[].direction";
const ACTION_TYPE: &str = "zones[] actions' actionType";

/// Why a zone set is refused: the problem and the error type that goes with it.
type Refusal = (Problem, ErrorType);

fn invalid(problem: Problem) -> Refusal {
    (problem, ErrorType::ValidationFailure)
}

fn unknown_map(problem: Problem) -> Refusal {
    (problem, ErrorType::UnknownMapId)
}

fn refuse((problem, error): Refusal, zone_set_id: &str) -> Issue {
    Issue::new(problem).with_error(error, "zoneSetId", zone_set_id)
}

impl Translator {
    pub(crate) fn zone_set_to_vda(
        &self,
        header: &Header,
        set: &ZoneSet,
    ) -> Result<Translated<vda::ZoneSetMessage>, Issue> {
        let id = set.id.as_str();
        let head = self.outgoing(header).map_err(|problem| match problem {
            Problem::BadTime { .. } => refuse(invalid(problem), id),
            _ => Issue::new(problem),
        })?;
        let zone_set = self.zones_to_vda(set).map_err(|r| refuse(r, id))?;
        Ok(Translated {
            message: vda::ZoneSetMessage {
                header_id: head.header_id,
                timestamp: head.timestamp,
                version: head.version,
                manufacturer: head.manufacturer,
                serial_number: head.serial_number,
                zone_set,
            },
            issues: Vec::new(),
        })
    }

    fn zones_to_vda(&self, set: &ZoneSet) -> Result<vda::ZoneSet, Refusal> {
        let id = usable(set.id.as_str(), ZONE_SET_ID).map_err(invalid)?;
        let map = self.map_name(set.map).map_err(unknown_map)?;
        let zones = set
            .zones
            .iter()
            .map(zone_to_vda)
            .collect::<Result<_, _>>()
            .map_err(invalid)?;
        Ok(vda::ZoneSet {
            map_id: map.to_string(),
            zone_set_id: id.to_string(),
            zone_set_descriptor: set.descriptor.as_deref().map(str::to_string),
            zones,
        })
    }

    /// What the `zoneSet` message says, as a robot receives it.
    pub fn zone_set_from_vda(
        &self,
        message: &vda::ZoneSetMessage,
    ) -> Result<Translated<ToRobot>, Issue> {
        let set = &message.zone_set;
        let id = set.zone_set_id.as_str();
        let header = self
            .incoming(Incoming {
                header_id: message.header_id,
                timestamp: &message.timestamp,
                version: &message.version,
                manufacturer: &message.manufacturer,
                serial_number: &message.serial_number,
            })
            .map_err(|problem| match problem {
                Problem::UnknownRobot(_) => Issue::new(problem),
                _ => refuse(invalid(problem), id),
            })?;
        let zone_set = self.zone_set_content_from_vda(set)?;
        Ok(Translated {
            message: ToRobot {
                header,
                body: ToRobotBody::ZoneSet(Arc::new(zone_set)),
            },
            issues: Vec::new(),
        })
    }

    /// A zone set on its own, without a message around it (the fleet server reads
    /// the site's zone set when it starts). It is taken whole or refused, as in a
    /// message.
    pub fn zone_set_content_from_vda(&self, set: &vda::ZoneSet) -> Result<ZoneSet, Issue> {
        self.zones_from_vda(set)
            .map_err(|r| refuse(r, &set.zone_set_id))
    }

    fn zones_from_vda(&self, set: &vda::ZoneSet) -> Result<ZoneSet, Refusal> {
        let id = usable(&set.zone_set_id, ZONE_SET_ID).map_err(invalid)?;
        let map = self.map_number(&set.map_id).map_err(unknown_map)?;
        let zones = set
            .zones
            .iter()
            .map(zone_from_vda)
            .collect::<Result<_, _>>()
            .map_err(invalid)?;
        Ok(ZoneSet {
            id: ZoneSetId::new(id),
            map,
            descriptor: set.zone_set_descriptor.as_deref().map(Arc::from),
            zones,
        })
    }
}

/// A factor of a priority or penalty zone, 0 to 1.
fn factor(value: f64, field: &'static str) -> Result<f64, Problem> {
    let value = finite(value, field)?;
    if (0.0..=1.0).contains(&value) {
        Ok(value)
    } else {
        Err(Problem::OutOfRange { field })
    }
}

fn zone_to_vda(z: &Zone) -> Result<vda::Zone, Problem> {
    let id = usable(z.id.as_str(), ZONE_ID)?;
    if z.vertices.len() < 3 {
        return Err(Problem::TooFewVertices {
            zone: id.to_string(),
            there: z.vertices.len(),
        });
    }
    let vertices = z
        .vertices
        .iter()
        .map(|&(x, y)| {
            Ok(vda::Vertex {
                x: finite(x, VERTEX)?,
                y: finite(y, VERTEX)?,
            })
        })
        .collect::<Result<_, Problem>>()?;
    let kind = match &z.kind {
        ZoneKind::Blocked => vda::ZoneKind::Blocked,
        ZoneKind::LineGuided => vda::ZoneKind::LineGuided,
        ZoneKind::Release { loss } => vda::ZoneKind::Release {
            release_loss_behavior: loss.map(loss_to_vda),
        },
        ZoneKind::CoordinatedReplanning => vda::ZoneKind::CoordinatedReplanning,
        ZoneKind::SpeedLimit { maximum_speed } => vda::ZoneKind::SpeedLimit {
            maximum_speed: finite(*maximum_speed, MAXIMUM_SPEED)?,
        },
        ZoneKind::Action {
            entry,
            during,
            exit,
        } => vda::ZoneKind::Action {
            entry_actions: actions_to_vda(entry)?,
            during_actions: actions_to_vda(during)?,
            exit_actions: actions_to_vda(exit)?,
        },
        ZoneKind::Priority { factor: f } => vda::ZoneKind::Priority {
            priority_factor: factor(*f, PRIORITY_FACTOR)?,
        },
        ZoneKind::Penalty { factor: f } => vda::ZoneKind::Penalty {
            penalty_factor: factor(*f, PENALTY_FACTOR)?,
        },
        ZoneKind::Directed {
            direction,
            limitation,
        } => vda::ZoneKind::Directed {
            direction: finite(*direction, DIRECTION)?,
            directed_limitation: directed_to_vda(*limitation),
        },
        ZoneKind::Bidirected {
            direction,
            limitation,
        } => vda::ZoneKind::Bidirected {
            direction: finite(*direction, DIRECTION)?,
            bidirected_limitation: bidirected_to_vda(*limitation),
        },
    };
    Ok(vda::Zone {
        zone_id: id.to_string(),
        zone_descriptor: z.descriptor.as_deref().map(str::to_string),
        vertices,
        kind,
    })
}

fn zone_from_vda(z: &vda::Zone) -> Result<Zone, Problem> {
    let id = usable(&z.zone_id, ZONE_ID)?;
    let vertices = z
        .vertices
        .iter()
        .map(|v| Ok((finite(v.x, VERTEX)?, finite(v.y, VERTEX)?)))
        .collect::<Result<_, Problem>>()?;
    let kind = match &z.kind {
        vda::ZoneKind::Blocked => ZoneKind::Blocked,
        vda::ZoneKind::LineGuided => ZoneKind::LineGuided,
        vda::ZoneKind::Release {
            release_loss_behavior,
        } => ZoneKind::Release {
            loss: release_loss_behavior.map(loss_from_vda),
        },
        vda::ZoneKind::CoordinatedReplanning => ZoneKind::CoordinatedReplanning,
        vda::ZoneKind::SpeedLimit { maximum_speed } => ZoneKind::SpeedLimit {
            maximum_speed: finite(*maximum_speed, MAXIMUM_SPEED)?,
        },
        vda::ZoneKind::Action {
            entry_actions,
            during_actions,
            exit_actions,
        } => ZoneKind::Action {
            entry: actions_from_vda(entry_actions)?,
            during: actions_from_vda(during_actions)?,
            exit: actions_from_vda(exit_actions)?,
        },
        vda::ZoneKind::Priority { priority_factor } => ZoneKind::Priority {
            factor: finite(*priority_factor, PRIORITY_FACTOR)?,
        },
        vda::ZoneKind::Penalty { penalty_factor } => ZoneKind::Penalty {
            factor: finite(*penalty_factor, PENALTY_FACTOR)?,
        },
        vda::ZoneKind::Directed {
            direction,
            directed_limitation,
        } => ZoneKind::Directed {
            direction: finite(*direction, DIRECTION)?,
            limitation: directed_from_vda(*directed_limitation),
        },
        vda::ZoneKind::Bidirected {
            direction,
            bidirected_limitation,
        } => ZoneKind::Bidirected {
            direction: finite(*direction, DIRECTION)?,
            limitation: bidirected_from_vda(*bidirected_limitation),
        },
    };
    Ok(Zone {
        id: ZoneId::new(id),
        descriptor: z.zone_descriptor.as_deref().map(Arc::from),
        vertices,
        kind,
    })
}

fn actions_to_vda(actions: &[ZoneAction]) -> Result<Vec<vda::ZoneAction>, Problem> {
    actions
        .iter()
        .map(|a| {
            Ok(vda::ZoneAction {
                action_type: usable(a.kind.as_str(), ACTION_TYPE)?.to_string(),
                action_descriptor: a.descriptor.as_deref().map(str::to_string),
                blocking_type: blocking_to_vda(a.blocking),
                action_parameters: a.parameters.as_ref().map(|parameters| {
                    parameters
                        .iter()
                        .map(|(key, value)| vda::ActionParameter {
                            key: key.to_string(),
                            value: value.clone(),
                        })
                        .collect()
                }),
                retriable: a.retriable,
            })
        })
        .collect()
}

fn actions_from_vda(actions: &[vda::ZoneAction]) -> Result<Vec<ZoneAction>, Problem> {
    actions
        .iter()
        .map(|a| {
            Ok(ZoneAction {
                kind: ActionType::new(usable(&a.action_type, ACTION_TYPE)?),
                descriptor: a.action_descriptor.as_deref().map(Arc::from),
                blocking: blocking_from_vda(a.blocking_type),
                parameters: a.action_parameters.as_ref().map(|parameters| {
                    parameters
                        .iter()
                        .map(|p| (Arc::from(p.key.as_str()), p.value.clone()))
                        .collect()
                }),
                retriable: a.retriable,
            })
        })
        .collect()
}

fn loss_to_vda(l: LossBehavior) -> vda::ZoneReleaseLossBehavior {
    match l {
        LossBehavior::Stop => vda::ZoneReleaseLossBehavior::Stop,
        LossBehavior::Continue => vda::ZoneReleaseLossBehavior::Continue,
        LossBehavior::Evacuate => vda::ZoneReleaseLossBehavior::Evacuate,
    }
}

fn loss_from_vda(l: vda::ZoneReleaseLossBehavior) -> LossBehavior {
    match l {
        vda::ZoneReleaseLossBehavior::Stop => LossBehavior::Stop,
        vda::ZoneReleaseLossBehavior::Continue => LossBehavior::Continue,
        vda::ZoneReleaseLossBehavior::Evacuate => LossBehavior::Evacuate,
    }
}

fn directed_to_vda(d: Directed) -> vda::DirectedLimitation {
    match d {
        Directed::Soft => vda::DirectedLimitation::Soft,
        Directed::Restricted => vda::DirectedLimitation::Restricted,
        Directed::Strict => vda::DirectedLimitation::Strict,
    }
}

fn directed_from_vda(d: vda::DirectedLimitation) -> Directed {
    match d {
        vda::DirectedLimitation::Soft => Directed::Soft,
        vda::DirectedLimitation::Restricted => Directed::Restricted,
        vda::DirectedLimitation::Strict => Directed::Strict,
    }
}

fn bidirected_to_vda(b: Bidirected) -> vda::BidirectedLimitation {
    match b {
        Bidirected::Soft => vda::BidirectedLimitation::Soft,
        Bidirected::Restricted => vda::BidirectedLimitation::Restricted,
    }
}

fn bidirected_from_vda(b: vda::BidirectedLimitation) -> Bidirected {
    match b {
        vda::BidirectedLimitation::Soft => Bidirected::Soft,
        vda::BidirectedLimitation::Restricted => Bidirected::Restricted,
    }
}

//! The translator: the layout, the table of robots, and the way to a robot.

use std::collections::HashMap;
use std::fmt;
use std::sync::Arc;

use botsim_link::message::{ToRobot, ToRobotBody};
use botsim_model::graph::{EdgeIdx, Graph, MapIdx, NodeIdx};
use botsim_model::robot::{RobotId, RobotNo};

use crate::report::{Issue, Item, Problem, Translated};

/// A VDA 5050 message to a robot.
#[derive(Debug, Clone, PartialEq)]
pub enum RobotMessage {
    Order(vda5050::order::Order),
    InstantActions(vda5050::instant_actions::InstantActions),
    Responses(vda5050::responses::Responses),
    ZoneSet(vda5050::zone_set::ZoneSetMessage),
}

/// Translates between `link` and VDA 5050 for the robots of one layout.
#[derive(Debug, Clone)]
pub struct Translator {
    graph: Arc<Graph>,
    /// By slot of the robot number; a slot that was used stays.
    slots: HashMap<u32, Slot>,
    numbers: HashMap<RobotId, RobotNo>,
}

#[derive(Debug, Clone)]
struct Slot {
    /// Of the robot in the slot, or of the last one that was in it.
    generation: u32,
    /// `None`: the robot is gone.
    id: Option<RobotId>,
}

/// Why the table of robots does not take a change.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RobotTableError {
    /// A `manufacturer` or `serialNumber` that is empty or has one of `/ + # $`.
    UnusableIdentity(RobotId),
    /// The slot of the number holds the robot with this number.
    SlotTaken(RobotNo),
    /// The identity is in the table already, with this number.
    IdentityTaken(RobotNo),
    /// The number is of a robot that is gone, or older than the slot's.
    OldNumber(RobotNo),
    /// The number is not in the table and never was.
    UnknownNumber(RobotNo),
}

impl fmt::Display for RobotTableError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let number = |no: &RobotNo| format!("{} (generation {})", no.slot(), no.generation());
        match self {
            RobotTableError::UnusableIdentity(id) => write!(
                f,
                "'{}' / '{}' cannot be a robot's identity",
                id.manufacturer, id.serial_number
            ),
            RobotTableError::SlotTaken(no) => {
                write!(f, "the slot is taken by robot number {}", number(no))
            }
            RobotTableError::IdentityTaken(no) => {
                write!(f, "the robot has the number {} already", number(no))
            }
            RobotTableError::OldNumber(no) => write!(f, "robot number {} is old", number(no)),
            RobotTableError::UnknownNumber(no) => {
                write!(f, "robot number {} is unknown", number(no))
            }
        }
    }
}

impl std::error::Error for RobotTableError {}

/// Can the text be a level of an MQTT topic (VDA 5050 §4.2)?
fn usable_level(text: &str) -> bool {
    !text.is_empty() && !text.contains(['/', '+', '#', '$'])
}

impl Translator {
    /// A translator for the layout `graph`, with no robots yet.
    pub fn new(graph: Arc<Graph>) -> Translator {
        Translator {
            graph,
            slots: HashMap::new(),
            numbers: HashMap::new(),
        }
    }

    pub fn graph(&self) -> &Graph {
        &self.graph
    }

    /// From now on `no` stands for the robot `id`. A slot that a robot has left
    /// takes only a later generation.
    pub fn add_robot(&mut self, no: RobotNo, id: RobotId) -> Result<(), RobotTableError> {
        if !(usable_level(&id.manufacturer) && usable_level(&id.serial_number)) {
            return Err(RobotTableError::UnusableIdentity(id));
        }
        if let Some(slot) = self.slots.get(&no.slot()) {
            if slot.id.is_some() {
                let holder = RobotNo::new(no.slot(), slot.generation);
                return Err(RobotTableError::SlotTaken(holder));
            }
            if no.generation() <= slot.generation {
                return Err(RobotTableError::OldNumber(no));
            }
        }
        if let Some(&taken) = self.numbers.get(&id) {
            return Err(RobotTableError::IdentityTaken(taken));
        }
        self.numbers.insert(id.clone(), no);
        self.slots.insert(
            no.slot(),
            Slot {
                generation: no.generation(),
                id: Some(id),
            },
        );
        Ok(())
    }

    /// The robot `no` is gone; its identity is returned.
    pub fn remove_robot(&mut self, no: RobotNo) -> Result<RobotId, RobotTableError> {
        match self.slots.get_mut(&no.slot()) {
            Some(slot) if no.generation() <= slot.generation => {
                let current = no.generation() == slot.generation;
                match slot.id.take_if(|_| current) {
                    Some(id) => {
                        self.numbers.remove(&id);
                        Ok(id)
                    }
                    None => Err(RobotTableError::OldNumber(no)),
                }
            }
            _ => Err(RobotTableError::UnknownNumber(no)),
        }
    }

    /// Who the number stands for.
    pub fn robot_id(&self, no: RobotNo) -> Option<&RobotId> {
        self.identity(no).ok()
    }

    /// The number of the robot.
    pub fn robot_no(&self, id: &RobotId) -> Option<RobotNo> {
        self.numbers.get(id).copied()
    }

    pub(crate) fn identity(&self, no: RobotNo) -> Result<&RobotId, Problem> {
        match self.slots.get(&no.slot()) {
            Some(slot) if no.generation() <= slot.generation => slot
                .id
                .as_ref()
                .filter(|_| no.generation() == slot.generation)
                .ok_or(Problem::OldRobotNo(no)),
            _ => Err(Problem::UnknownRobotNo(no)),
        }
    }

    pub(crate) fn node_name(&self, node: NodeIdx) -> Result<&str, Problem> {
        self.graph
            .nodes
            .get(node)
            .map(|n| n.id.as_str())
            .ok_or(Problem::OutsideLayout {
                item: Item::Node,
                number: node,
            })
    }

    pub(crate) fn edge_name(&self, edge: EdgeIdx) -> Result<&str, Problem> {
        self.graph
            .edges
            .get(edge)
            .map(|e| e.id.as_str())
            .ok_or(Problem::OutsideLayout {
                item: Item::Edge,
                number: edge,
            })
    }

    pub(crate) fn map_name(&self, map: MapIdx) -> Result<&str, Problem> {
        self.graph
            .maps
            .get(map)
            .map(String::as_str)
            .ok_or(Problem::OutsideLayout {
                item: Item::Map,
                number: map,
            })
    }

    pub(crate) fn node_number(&self, name: &str) -> Result<NodeIdx, Problem> {
        self.graph
            .node_idx(name)
            .ok_or_else(|| unknown(Item::Node, name))
    }

    pub(crate) fn edge_number(&self, name: &str) -> Result<EdgeIdx, Problem> {
        self.graph
            .edge_idx(name)
            .ok_or_else(|| unknown(Item::Edge, name))
    }

    pub(crate) fn map_number(&self, name: &str) -> Result<MapIdx, Problem> {
        self.graph
            .map_idx(name)
            .ok_or_else(|| unknown(Item::Map, name))
    }

    /// The VDA 5050 message that says what `message` says.
    pub fn to_robot_to_vda(&self, message: &ToRobot) -> Result<Translated<RobotMessage>, Issue> {
        let header = &message.header;
        match &message.body {
            ToRobotBody::Order(order) => self
                .order_to_vda(header, order)
                .map(|t| t.map(RobotMessage::Order)),
            ToRobotBody::InstantActions(actions) => self
                .instant_actions_to_vda(header, actions)
                .map(|t| t.map(RobotMessage::InstantActions)),
            ToRobotBody::Responses(responses) => self
                .responses_to_vda(header, responses)
                .map(|t| t.map(RobotMessage::Responses)),
            ToRobotBody::ZoneSet(zone_set) => self
                .zone_set_to_vda(header, zone_set)
                .map(|t| t.map(RobotMessage::ZoneSet)),
        }
    }
}

fn unknown(item: Item, name: &str) -> Problem {
    Problem::UnknownName {
        item,
        name: name.to_string(),
    }
}

impl<T> Translated<T> {
    pub(crate) fn map<U>(self, f: impl FnOnce(T) -> U) -> Translated<U> {
        Translated {
            message: f(self.message),
            issues: self.issues,
        }
    }
}

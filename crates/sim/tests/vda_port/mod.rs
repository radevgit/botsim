//! The simulated robots in VDA 5050, for tests. The simulator speaks `link`; a
//! test that is written with VDA 5050 messages translates them here, with
//! `botsim-vda`: [`VdaWorld`] and [`VdaRobot`] give the world and a robot the
//! methods that take and give those messages. What the translator refuses is
//! handed to the robot as a message that could not be read.
//!
//! The tests of other crates include this file too.

// (each test uses a part)
#![allow(dead_code)]

use std::sync::Arc;

use botsim_link::message::{FromRobot, Header as LinkHeader, Millis, ToRobotBody};
use botsim_link::state::{ActionState, State as LinkState};
use botsim_model::graph::Graph;
use botsim_model::robot::{RobotId, RobotNo};
use botsim_model::time::unix_ms_from_iso8601;
use botsim_sim::drive::Progress;
use botsim_sim::errors::RobotError;
use botsim_sim::order_route::{self, OrderError, OrderRoute};
use botsim_sim::robot::{InstantEntry, Reception, Robot, Unread};
use botsim_sim::state_report;
use botsim_sim::world::{World, WorldError};
use botsim_vda::report::Issue;
use botsim_vda::translator::Translator;
use vda5050::instant_actions::InstantActions;
use vda5050::order::Order;
use vda5050::responses::Responses;
use vda5050::state::{State, ZoneRequest};
use vda5050::zone_set::ZoneSetMessage;

/// The header fields of a state message.
#[derive(Debug, Clone, PartialEq)]
pub struct Header {
    pub header_id: u32,
    /// ISO 8601 UTC, e.g. `2026-11-20T10:00:00.000Z`.
    pub timestamp: String,
    pub version: String,
    pub manufacturer: String,
    pub serial_number: String,
}

/// Why an order does not become a route: the translator refuses it, or the
/// simulator does.
#[derive(Debug, Clone, PartialEq)]
pub enum Refused {
    Translator(Issue),
    Simulator(OrderError),
}

/// A translator for `graph` in which the robot `id` has the number `no`.
pub fn translator(graph: &Graph, id: RobotId, no: RobotNo) -> Translator {
    let mut translator = Translator::new(Arc::new(graph.clone()));
    translator.add_robot(no, id).expect("the only robot");
    translator
}

/// The error a robot refuses with what the translator refused; `None` where VDA
/// 5050 has no error type for it.
pub fn robot_error(issue: &Issue) -> Option<RobotError> {
    Some(RobotError {
        error_type: issue.error?,
        references: issue
            .references
            .iter()
            .map(|(key, value)| (key.to_string(), value.to_string()))
            .collect(),
        description: Some(issue.to_string()),
    })
}

/// The released base of the order `message` as a route on `graph`.
pub fn route_from_order(graph: &Graph, message: &Order) -> Result<OrderRoute, Refused> {
    let id = RobotId::new(&message.manufacturer, &message.serial_number);
    let translated = translator(graph, id, RobotNo::new(0, 0))
        .order_from_vda(message)
        .map_err(Refused::Translator)?;
    let ToRobotBody::Order(order) = translated.message.body else {
        panic!("an order was translated into something else");
    };
    order_route::route_from_order(graph, &order).map_err(Refused::Simulator)
}

/// The state message that says what `state` says, with the header `header`.
pub fn state_message(graph: &Graph, header: &Header, state: LinkState) -> State {
    let id = RobotId::new(&header.manufacturer, &header.serial_number);
    let no = RobotNo::new(0, 0);
    let time = unix_ms_from_iso8601(&header.timestamp).expect("a timestamp");
    let report = FromRobot {
        header: LinkHeader {
            robot: no,
            seq: header.header_id,
            time: Millis(time),
        },
        state,
    };
    write(&translator(graph, id, no), &report)
}

/// The state message of a report. (An entry that VDA 5050 cannot say, such as an
/// action without an id, is not in it.)
fn write(translator: &Translator, report: &FromRobot) -> State {
    translator
        .state_to_vda(report)
        .expect("a state message")
        .message
}

/// The state message of a robot with the given progress
/// ([`state_report::build_state`]).
pub fn build_state(
    graph: &Graph,
    header: &Header,
    progress: &Progress,
    errors: &[RobotError],
    instant_actions: &[ActionState],
) -> State {
    let state = state_report::build_state(progress, errors, instant_actions);
    state_message(graph, header, state)
}

/// The entries of a message of instant actions of which the translator left some
/// out, in their order.
fn entries(
    message: &InstantActions,
    read: &[botsim_link::instant::InstantAction],
    issues: &[Issue],
) -> Vec<InstantEntry> {
    let mut read = read.iter();
    message
        .actions
        .iter()
        .enumerate()
        .filter_map(
            |(i, a)| match issues.iter().find(|issue| issue.entry == Some(i)) {
                Some(issue) => Some(InstantEntry::Unread {
                    id: a.action_id.as_str().into(),
                    kind: Some(a.action_type.as_str().into()),
                    error: robot_error(issue)?,
                }),
                None => read.next().cloned().map(InstantEntry::Read),
            },
        )
        .collect()
}

pub trait VdaWorld {
    /// Deliver an `order`; the robot is the one named in the message.
    fn send_order(&mut self, message: &Order) -> Result<Reception, WorldError>;
    /// Deliver an `instantActions` message.
    fn send_instant_actions(&mut self, message: &InstantActions) -> Result<(), WorldError>;
    /// Deliver a `responses` message.
    fn send_responses(&mut self, message: &Responses) -> Result<(), WorldError>;
    /// Deliver a `zoneSet` message.
    fn send_zone_set(&mut self, message: &ZoneSetMessage) -> Result<(), WorldError>;
    /// The state message of the robot `id`, as [`World::report`] gives it.
    fn state(&mut self, id: &RobotId) -> Result<State, WorldError>;
    /// The robot's zone requests as it states them now.
    fn zone_requests(&self, id: &RobotId) -> Result<Vec<ZoneRequest>, WorldError>;
}

/// The number of the robot `id` and a translator that knows it.
fn addressed(world: &World, id: RobotId) -> Result<(RobotNo, Translator), WorldError> {
    let no = world
        .robot_no(&id)
        .ok_or_else(|| WorldError::UnknownRobot(id.clone()))?;
    Ok((no, translator(world.graph(), id, no)))
}

impl VdaWorld for World {
    fn send_order(&mut self, message: &Order) -> Result<Reception, WorldError> {
        let id = RobotId::new(&message.manufacturer, &message.serial_number);
        let (no, translator) = addressed(self, id)?;
        match translator.order_from_vda(message) {
            Ok(t) => Ok(self.send(&t.message)?.expect("an answer to an order")),
            Err(issue) => {
                let error = robot_error(&issue).expect("an order is refused with an error");
                let answer = if self.message_delay() > 0 {
                    Reception::InTransit
                } else {
                    Reception::Rejected(error.error_type)
                };
                self.send_unread(no, Unread::Order(error))?;
                Ok(answer)
            }
        }
    }

    fn send_instant_actions(&mut self, message: &InstantActions) -> Result<(), WorldError> {
        let id = RobotId::new(&message.manufacturer, &message.serial_number);
        let (no, translator) = addressed(self, id)?;
        let t = translator
            .instant_actions_from_vda(message)
            .expect("a good header");
        if t.issues.is_empty() {
            self.send(&t.message)?;
            return Ok(());
        }
        let ToRobotBody::InstantActions(read) = &t.message.body else {
            panic!("instant actions were translated into something else");
        };
        let entries = entries(message, read, &t.issues);
        self.send_unread(no, Unread::InstantActions(entries))
    }

    fn send_responses(&mut self, message: &Responses) -> Result<(), WorldError> {
        let id = RobotId::new(&message.manufacturer, &message.serial_number);
        let (_, translator) = addressed(self, id)?;
        let t = translator
            .responses_from_vda(message)
            .expect("a good header");
        self.send(&t.message)?;
        Ok(())
    }

    fn send_zone_set(&mut self, message: &ZoneSetMessage) -> Result<(), WorldError> {
        let id = RobotId::new(&message.manufacturer, &message.serial_number);
        let (no, translator) = addressed(self, id)?;
        match translator.zone_set_from_vda(message) {
            Ok(t) => {
                self.send(&t.message)?;
                Ok(())
            }
            Err(issue) => {
                let error = robot_error(&issue).expect("a zone set is refused with an error");
                self.send_unread(no, Unread::ZoneSet(error))
            }
        }
    }

    fn state(&mut self, id: &RobotId) -> Result<State, WorldError> {
        let (no, translator) = addressed(self, id.clone())?;
        let report = self.report(no)?;
        Ok(write(&translator, &report))
    }

    fn zone_requests(&self, id: &RobotId) -> Result<Vec<ZoneRequest>, WorldError> {
        let requests = self.zone_requests_of(id)?;
        // (the translator reads whole state messages: an empty one with these)
        let mut state = empty_state();
        state.zone_requests = Some(requests);
        let header = Header {
            header_id: 0,
            timestamp: self.timestamp(),
            version: "3.0.1".into(),
            manufacturer: id.manufacturer.clone(),
            serial_number: id.serial_number.clone(),
        };
        let message = state_message(self.graph(), &header, state);
        Ok(message.zone_requests.unwrap_or_default())
    }
}

/// A state report that says nothing.
fn empty_state() -> LinkState {
    use botsim_link::state::{EmergencyStop, OperatingMode, Route};
    LinkState {
        order_id: None,
        update_id: 0,
        route: Route::default(),
        position: None,
        velocity: None,
        driving: false,
        field_violation: false,
        emergency_stop: EmergencyStop::None,
        operating_mode: OperatingMode::Automatic,
        charge: 100.0,
        charging: false,
        actions: Vec::new(),
        instant_actions: Vec::new(),
        zone_requests: None,
        zone_sets: Vec::new(),
        errors: Vec::new(),
    }
}

pub trait VdaRobot {
    /// Receive an `order` message.
    fn receive_order(&mut self, graph: &Graph, message: &Order) -> Reception;
    /// Receive an `instantActions` message.
    fn receive_instant_actions(&mut self, graph: &Graph, message: &InstantActions);
    /// The state message.
    fn state(&self, graph: &Graph, header: &Header) -> State;
}

impl VdaRobot for Robot {
    fn receive_order(&mut self, graph: &Graph, message: &Order) -> Reception {
        let id = RobotId::new(&message.manufacturer, &message.serial_number);
        match translator(graph, id, RobotNo::new(0, 0)).order_from_vda(message) {
            Ok(t) => {
                let ToRobotBody::Order(order) = t.message.body else {
                    panic!("an order was translated into something else");
                };
                self.take_order(graph, &order)
            }
            Err(issue) => {
                let error = robot_error(&issue).expect("an order is refused with an error");
                let answer = Reception::Rejected(error.error_type);
                self.take_unread(graph, &Unread::Order(error));
                answer
            }
        }
    }

    fn receive_instant_actions(&mut self, graph: &Graph, message: &InstantActions) {
        let id = RobotId::new(&message.manufacturer, &message.serial_number);
        let t = translator(graph, id, RobotNo::new(0, 0))
            .instant_actions_from_vda(message)
            .expect("a good header");
        let ToRobotBody::InstantActions(read) = &t.message.body else {
            panic!("instant actions were translated into something else");
        };
        let entries = entries(message, read, &t.issues);
        self.take_unread(graph, &Unread::InstantActions(entries));
    }

    fn state(&self, graph: &Graph, header: &Header) -> State {
        state_message(graph, header, self.report())
    }
}

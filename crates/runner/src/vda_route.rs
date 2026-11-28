//! **The route through `vda`**: the controller connected to a world as if the
//! robots spoke VDA 5050, so that the translation is tested without real robots.
//! Only tests take it; the `botsim` command does not.
//!
//! - Every message of the controller to a robot is translated to VDA 5050 and
//!   back before the robot gets it, and every report of a robot before the
//!   controller gets it (`botsim-vda`; message structures, no JSON text).
//! - What the translator refuses of a message to a robot never reaches the robot
//!   as sent: its error type and references are reported by that robot as for
//!   its own refusals (a zone set's for 10 s), and an instant action left out is
//!   listed `FAILED` in its place. With a message delay this arrives when the
//!   message would have. A refusal with no VDA 5050 error type (responses, the
//!   header of instant actions) is only counted. A VDA 5050 state cannot carry
//!   an action without an id: of an instant action refused for an empty
//!   `actionId`, the controller sees the error but not the `FAILED` entry.
//! - Of a report, a state refused whole is as one that cannot be had: that cycle
//!   the controller only takes the new jobs, so nothing new is released while it
//!   lasts. Of a state translated in part the controller sees the rest.
//! - The site's zone set is read as the content of a VDA 5050 message, with its
//!   checks; without it a site with zones has no robot ready for tasks.
//! - Doors and lifts are outside VDA 5050 and go as on the direct route.
//!
//! Nothing is dropped silently: every case is counted ([`report`],
//! [`Connected::untranslated`]).

pub mod report;

use std::sync::Arc;

use botsim_link::instant::InstantAction;
use botsim_link::message::{FromRobot, ToRobot, ToRobotBody};
use botsim_link::zone_set::ZoneSet;
use botsim_model::robot::{RobotId, RobotNo};
use botsim_model::zones::Zones;
use botsim_server::controller::Controller;
use botsim_server::cycle::{Config, robot_no};
use botsim_server::policy::TrafficPolicy;
use botsim_sim::errors::RobotError;
use botsim_sim::robot::{InstantEntry, Unread};
use botsim_sim::world::World;
use botsim_vda::report::{Issue, Translated};
use botsim_vda::translator::{RobotMessage, Translator};

use crate::connect::{Connected, config_with};
use crate::vda_route::report::{Reason, Subject, Untranslated, What};

/// [`crate::connect::config`] on the route: the site's zone set is read with the
/// checks of a message, and left out of [`Config::zone_set`] if it is refused.
pub fn config(world: &World) -> Config {
    VdaRoute::new(world).0
}

/// [`crate::connect::with_controller`] on the route.
pub fn with_controller<P: TrafficPolicy>(
    world: &World,
    make: impl FnOnce(Config) -> Controller<P>,
) -> Connected<P> {
    let (config, route) = VdaRoute::new(world);
    let lifts = config.lifts.len();
    Connected {
        controller: make(config),
        lifts,
        route: Some(Box::new(route)),
    }
}

/// [`crate::connect::controller`] on the route.
pub fn controller(world: &World) -> Connected {
    with_controller(world, |config| botsim_baseline::controller(&config))
}

/// What translates on the route.
#[derive(Debug)]
pub(crate) struct VdaRoute {
    /// It knows the layout and the robot numbers.
    translator: Translator,
    report: Untranslated,
}

impl VdaRoute {
    /// The configuration of a controller for `world`, and the route that goes
    /// with it.
    fn new(world: &World) -> (Config, VdaRoute) {
        let tick = world.tick();
        let mut route = VdaRoute {
            translator: Translator::new(Arc::new(world.graph().clone())),
            report: Untranslated::default(),
        };
        let config = config_with(world, |zones| route.site_zone_set(zones, tick));
        for (i, robot) in config.robots.iter().enumerate() {
            let no = robot_no(i);
            if let Err(error) = route.translator.add_robot(no, robot.id.clone()) {
                let case = (What::Robot, Subject::Robot(no));
                let name = || robot_name(&robot.id);
                let reason = Reason::Table(error);
                route.report.note(case, true, reason, tick, name);
            }
        }
        (config, route)
    }

    pub(crate) fn untranslated(&self) -> &Untranslated {
        &self.report
    }

    /// The message with the site's zones.
    fn site_zone_set(&mut self, zones: &Zones, tick: u64) -> Option<Arc<ZoneSet>> {
        let map_id = self.translator.graph().maps.get(zones.map)?.clone();
        let set = vda5050::zone_set::ZoneSet {
            map_id,
            zone_set_id: zones.zone_set_id.clone(),
            zone_set_descriptor: None,
            zones: zones.zones.clone(),
        };
        match self.translator.zone_set_content_from_vda(&set) {
            Ok(set) => Some(Arc::new(set)),
            Err(issue) => {
                let case = (What::SiteZoneSet, Subject::Site);
                let name = || zones.zone_set_id.clone();
                let reason = Reason::Translator(issue);
                self.report.note(case, true, reason, tick, name);
                None
            }
        }
    }

    /// A robot's report as the controller gets it: written as a VDA 5050 state
    /// message and read again; `None` if it is refused on the way.
    pub(crate) fn report(&mut self, report: &FromRobot, tick: u64) -> Option<FromRobot> {
        let case = (What::State, Subject::Robot(report.header.robot));
        match self.translator.state_to_vda(report) {
            Ok(written) => {
                for issue in written.issues {
                    self.count(case, false, issue, tick);
                }
                self.read_state(&written.message, tick)
            }
            Err(issue) => {
                self.count(case, true, issue, tick);
                None
            }
        }
    }

    /// The report that says what the state message `message` says.
    pub(crate) fn read_state(
        &mut self,
        message: &vda5050::state::State,
        tick: u64,
    ) -> Option<FromRobot> {
        let id = RobotId::new(&message.manufacturer, &message.serial_number);
        let (report, issues, refused) = match self.translator.state_from_vda(message) {
            Ok(t) => (Some(t.message), t.issues, false),
            Err(issue) => (None, vec![issue], true),
        };
        for issue in issues {
            self.count_for(What::State, &id, refused, issue, tick);
        }
        report
    }

    /// The controller's message on its way to the robot: written as a VDA 5050
    /// message, which the robot then gets ([`VdaRoute::receive`]).
    pub(crate) fn send(&mut self, world: &mut World, message: &ToRobot) {
        let no = message.header.robot;
        let case = (what(&message.body), Subject::Robot(no));
        let tick = world.tick();
        let written = match self.translator.to_robot_to_vda(message) {
            Ok(written) => written,
            Err(issue) => {
                let error = robot_error(&issue);
                self.count(case, true, issue, tick);
                if let Some(error) = error {
                    refuse(world, no, &message.body, error);
                }
                return;
            }
        };
        for issue in &written.issues {
            self.count(case, false, issue.clone(), tick);
        }
        match (&message.body, &written.message) {
            // actions left out on the way out keep their places among the others
            (ToRobotBody::InstantActions(sent), RobotMessage::InstantActions(m))
                if written.issues.iter().any(|issue| issue.error.is_some()) =>
            {
                let Some((_, arrived)) = self.read_instant_actions(m, tick) else {
                    return;
                };
                let entries = in_places(sent, &written.issues, arrived);
                let _ = world.send_unread(no, Unread::InstantActions(entries));
            }
            _ => self.receive(world, &written.message),
        }
    }

    /// A VDA 5050 message arrives: the robot it names gets what it says, and the
    /// error of what cannot be read.
    pub(crate) fn receive(&mut self, world: &mut World, message: &RobotMessage) {
        let tick = world.tick();
        let (what, id) = match message {
            RobotMessage::Order(m) => (What::Order, (&m.manufacturer, &m.serial_number)),
            RobotMessage::InstantActions(m) => {
                (What::InstantActions, (&m.manufacturer, &m.serial_number))
            }
            RobotMessage::Responses(m) => (What::Responses, (&m.manufacturer, &m.serial_number)),
            RobotMessage::ZoneSet(m) => (What::ZoneSet, (&m.manufacturer, &m.serial_number)),
        };
        let id = RobotId::new(id.0, id.1);
        let read = match message {
            RobotMessage::Order(m) => self.translator.order_from_vda(m),
            RobotMessage::InstantActions(m) => {
                let Some((read, entries)) = self.read_instant_actions(m, tick) else {
                    return;
                };
                let unread = entries
                    .iter()
                    .any(|e| matches!(e, InstantEntry::Unread { .. }));
                if unread {
                    let no = read.header.robot;
                    let _ = world.send_unread(no, Unread::InstantActions(entries));
                } else {
                    let _ = world.send(&read);
                }
                return;
            }
            RobotMessage::Responses(m) => self.translator.responses_from_vda(m),
            RobotMessage::ZoneSet(m) => self.translator.zone_set_from_vda(m),
        };
        match read {
            Ok(Translated { message, issues }) => {
                for issue in issues {
                    self.count_for(what, &id, false, issue, tick);
                }
                // (a robot's answer to an order is not used: it shows in the report)
                let _ = world.send(&message);
            }
            Err(issue) => {
                let error = robot_error(&issue);
                self.count_for(what, &id, true, issue, tick);
                let robot = self.translator.robot_no(&id);
                if let (Some(no), Some(error)) = (robot, error) {
                    let unread = match what {
                        What::Order => Unread::Order(error),
                        What::ZoneSet => Unread::ZoneSet(error),
                        // (responses are refused without an error type)
                        _ => return,
                    };
                    let _ = world.send_unread(no, unread);
                }
            }
        }
    }

    /// An `instantActions` message as a robot reads it: the message with the
    /// actions that can be read, and all entries in their order. `None` if the
    /// message is refused whole (its header), which is counted.
    fn read_instant_actions(
        &mut self,
        message: &vda5050::instant_actions::InstantActions,
        tick: u64,
    ) -> Option<(ToRobot, Vec<InstantEntry>)> {
        let id = RobotId::new(&message.manufacturer, &message.serial_number);
        let read = match self.translator.instant_actions_from_vda(message) {
            Ok(read) => read,
            Err(issue) => {
                self.count_for(What::InstantActions, &id, true, issue, tick);
                return None;
            }
        };
        let ToRobotBody::InstantActions(actions) = &read.message.body else {
            return None; // (cannot happen: instant actions are read as such)
        };
        let mut actions = actions.iter();
        let entries = message
            .actions
            .iter()
            .enumerate()
            .filter_map(
                |(i, a)| match read.issues.iter().find(|issue| issue.entry == Some(i)) {
                    Some(issue) => Some(InstantEntry::Unread {
                        id: a.action_id.as_str().into(),
                        kind: Some(a.action_type.as_str().into()),
                        error: robot_error(issue)?,
                    }),
                    None => actions.next().cloned().map(InstantEntry::Read),
                },
            )
            .collect();
        for issue in read.issues {
            self.count_for(What::InstantActions, &id, false, issue, tick);
        }
        Some((read.message, entries))
    }

    /// Count what the translator says about a message of or to a robot number.
    fn count(&mut self, case: (What, Subject), refused: bool, issue: Issue, tick: u64) {
        let translator = &self.translator;
        let name = || match case.1 {
            Subject::Robot(no) => translator
                .robot_id(no)
                .map_or_else(|| format!("robot number {}", no.slot()), robot_name),
            Subject::Site => String::new(),
        };
        let reason = Reason::Translator(issue);
        self.report.note(case, refused, reason, tick, name);
    }

    /// Count what the translator says about a VDA 5050 message of or to `id`.
    fn count_for(&mut self, what: What, id: &RobotId, refused: bool, issue: Issue, tick: u64) {
        // (a message of a robot without a number is counted under the number no
        // robot has)
        let no = self.translator.robot_no(id);
        let no = no.unwrap_or(RobotNo::new(u32::MAX, u32::MAX));
        let case = (what, Subject::Robot(no));
        let reason = Reason::Translator(issue);
        self.report
            .note(case, refused, reason, tick, || robot_name(id));
    }
}

/// What a message to a robot is.
fn what(body: &ToRobotBody) -> What {
    match body {
        ToRobotBody::Order(_) => What::Order,
        ToRobotBody::InstantActions(_) => What::InstantActions,
        ToRobotBody::Responses(_) => What::Responses,
        ToRobotBody::ZoneSet(_) => What::ZoneSet,
    }
}

/// The error a robot reports for what the translator refused; `None` where VDA
/// 5050 has no error type for it.
fn robot_error(issue: &Issue) -> Option<RobotError> {
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

/// The robot gets the error of a message that was refused whole.
fn refuse(world: &mut World, robot: RobotNo, body: &ToRobotBody, error: RobotError) {
    let unread = match body {
        ToRobotBody::Order(_) => Unread::Order(error),
        ToRobotBody::ZoneSet(_) => Unread::ZoneSet(error),
        // (these are refused whole only for their header, without an error type)
        ToRobotBody::InstantActions(_) | ToRobotBody::Responses(_) => return,
    };
    let _ = world.send_unread(robot, unread);
}

/// All entries of the instant actions `sent`: those left out on the way out, by
/// `issues`, in their places among the entries that `arrived`.
fn in_places(
    sent: &[InstantAction],
    issues: &[Issue],
    arrived: Vec<InstantEntry>,
) -> Vec<InstantEntry> {
    let mut arrived = arrived.into_iter();
    sent.iter()
        .enumerate()
        .filter_map(
            |(i, a)| match issues.iter().find(|issue| issue.entry == Some(i)) {
                Some(issue) => Some(InstantEntry::Unread {
                    id: a.id.clone(),
                    kind: Some(a.kind.action_type()),
                    error: robot_error(issue)?,
                }),
                None => arrived.next(),
            },
        )
        .collect()
}

/// A robot in words.
fn robot_name(id: &RobotId) -> String {
    format!("{}/{}", id.manufacturer, id.serial_number)
}

//! Runs the server's controller against the simulated world. The controller knows
//! no simulator ([`botsim_server::cycle`]) and the simulator no controller; this
//! crate passes the data between them:
//! - [`config`]: the site and the fleet of a world, as the controller is told of
//!   them (a robot's type comes from its factsheet message, S61);
//! - [`controller`], [`with_policy`], [`with_dispatcher`]: a controller for that
//!   site, [`Bridged`] to the world;
//! - [`step`]: one cycle, from the world's states to the messages sent to it.
//!
//! **Translation.** The controller speaks [`botsim_link`]; the world's robots
//! speak VDA 5050. Their messages are translated by `botsim-vda` and nowhere else.
//! Doors and lifts are outside VDA 5050: their numbers, maps and cabin doors are
//! looked up here. What cannot be translated is counted
//! ([`Bridged::untranslated`], see [`report`]):
//! - a state refused whole is as a state that cannot be had: that cycle the
//!   controller only takes the new jobs, so nothing new is released while it lasts;
//! - of a state translated in part the controller sees the rest;
//! - a message to a robot that is refused is not sent (the controller waits as for
//!   a lost one); of instant actions and responses the other entries go;
//! - without its zone set a site with zones has no robot ready for tasks;
//! - a lift state or a command to a lift or door that names what is not there is
//!   left out.
//!
//! It is the one place that does all this, for `botsim` and for tests. It is there
//! only until the server and the simulator are connected through `link`.

pub mod report;

use std::ops::{Deref, DerefMut};
use std::sync::Arc;

use botsim_baseline::baseline_policy::BaselinePolicy;
use botsim_core::lift::{DoorState, LiftSpec, LiftStatus};
use botsim_core::world::World;
use botsim_dispatcher::Dispatcher;
use botsim_link::device::{
    DeviceHeader, DoorCommand, DoorNo, DoorPhase, FromLift, LiftCommand, LiftNo, ToDoor, ToLift,
};
use botsim_link::message::{FromRobot, Millis, ToRobot, ToRobotBody};
use botsim_link::zone_set::ZoneSet;
use botsim_model::job::Job;
use botsim_model::robot::{RobotId, RobotNo};
use botsim_model::zones::Zones;
use botsim_server::controller::Controller;
use botsim_server::cycle::{Config, FleetRobot, Input, Message, Time, robot_no};
use botsim_server::doors::Door;
use botsim_server::lifts::{Lift, LiftStop};
use botsim_server::policy::TrafficPolicy;
use botsim_vda::report::Issue;
use botsim_vda::translator::{RobotMessage, Translator};

use crate::report::{Reason, Subject, Untranslated, What};

/// The site and the fleet of `world`: its robots in the order they were added (one
/// whose factsheet cannot be read is left out), its zones, lifts and doors, its
/// time step and the tick it is at. A zone set that cannot be translated is left
/// out of [`Config::zone_set`].
pub fn config(world: &World) -> Config {
    Adapter::new(world).0
}

/// A controller connected to a world: it is used as the controller it holds (it
/// derefs to it), and it holds what translates between the two.
#[derive(Debug)]
pub struct Bridged<P: TrafficPolicy = BaselinePolicy> {
    controller: Controller<P>,
    adapter: Adapter,
}

impl<P: TrafficPolicy> Deref for Bridged<P> {
    type Target = Controller<P>;

    fn deref(&self) -> &Controller<P> {
        &self.controller
    }
}

impl<P: TrafficPolicy> DerefMut for Bridged<P> {
    fn deref_mut(&mut self) -> &mut Controller<P> {
        &mut self.controller
    }
}

/// The controller that `make` makes of the site and fleet of `world`, connected to
/// that world. `make` must keep the robots, lifts and doors and their order.
pub fn with_controller<P: TrafficPolicy>(
    world: &World,
    make: impl FnOnce(Config) -> Controller<P>,
) -> Bridged<P> {
    let (config, adapter) = Adapter::new(world);
    Bridged {
        controller: make(config),
        adapter,
    }
}

/// [`botsim_baseline::controller()`], with the baseline's traffic rules, for the
/// fleet of `world`.
pub fn controller(world: &World) -> Bridged {
    with_controller(world, |config| botsim_baseline::controller(&config))
}

/// [`Controller::with_policy`] for the fleet of `world`.
pub fn with_policy<P: TrafficPolicy>(world: &World, policy: P) -> Bridged<P> {
    with_controller(world, |config| Controller::with_policy(&config, policy))
}

/// [`Controller::with_dispatcher`] for the fleet of `world`.
pub fn with_dispatcher<P: TrafficPolicy>(
    world: &World,
    policy: P,
    dispatcher: Box<dyn Dispatcher>,
) -> Bridged<P> {
    with_controller(world, |config| {
        Controller::with_dispatcher(&config, policy, dispatcher)
    })
}

impl<P: TrafficPolicy> Bridged<P> {
    /// What could not be translated so far.
    pub fn untranslated(&self) -> &Untranslated {
        &self.adapter.report
    }

    /// The report the controller gets for a robot's state message, read at tick
    /// `tick` of the world; `None` if the message is refused.
    pub fn read_state(&mut self, message: &vda5050::state::State, tick: u64) -> Option<FromRobot> {
        let id = RobotId::new(&message.manufacturer, &message.serial_number);
        self.adapter.state(&id, message, tick)
    }

    /// The state the controller gets for lift `lift` (its place in the world's
    /// list) as `status` describes it, with the robot `rider` in its cabin, at
    /// `time`; `None` if it cannot be written.
    pub fn read_lift(
        &mut self,
        lift: usize,
        status: &LiftStatus,
        rider: Option<&RobotId>,
        time: Millis,
        tick: u64,
    ) -> Option<FromLift> {
        self.adapter.lift(lift, status, rider, (time, tick))
    }

    /// Send one message of the controller to `world`. What the world refuses to
    /// take (a message to something it does not have) is dropped: the controller
    /// learns of a refused order from the robot's state.
    pub fn send(&mut self, world: &mut World, message: &Message) {
        self.adapter.send(world, message);
    }
}

/// One cycle of the controller: the new jobs, the world's time, every robot's
/// state and the states of the lifts go in; the messages are sent to the world in
/// their order, and what the controller holds back (and, if somebody watches,
/// reserves) is published there. Call it once per step, after [`World::step`].
pub fn step<P: TrafficPolicy>(
    bridged: &mut Bridged<P>,
    world: &mut World,
    new_jobs: impl IntoIterator<Item = Job>,
) {
    let Bridged {
        controller,
        adapter,
    } = bridged;
    let tick = world.tick();
    let time = Time {
        tick,
        unix_ms: world.now_ms(),
    };
    // (asking for a state is what moves a delayed state message on, so every robot
    // is asked once in every cycle, whatever becomes of the others' messages)
    let asked: Vec<(&RobotId, Option<vda5050::state::State>)> = controller
        .robots()
        .map(|id| (id, world.state(id).ok()))
        .collect();
    // (if one cannot be had or translated, the list is short and the cycle does
    // nothing)
    let states: Vec<FromRobot> = asked
        .iter()
        .filter_map(|(id, message)| adapter.state(id, message.as_ref()?, tick))
        .collect();
    let at = (Millis(time.unix_ms), tick);
    let lifts: Vec<FromLift> = world
        .lifts()
        .iter()
        .enumerate()
        .filter_map(|(l, status)| {
            let rider = world.lift_rider(&status.id).ok().flatten();
            adapter.lift(l, status, rider, at)
        })
        .collect();
    let input = Input {
        time,
        states: &states,
        lifts: &lifts,
        watched: world.observing(),
    };
    let output = controller.step(&input, new_jobs);
    for message in &output.messages {
        adapter.send(world, message);
    }
    if let Some(held_back) = output.held_back {
        world.publish_held_back(held_back);
    }
    if let Some(reservations) = output.reservations {
        world.publish_reservations(reservations);
    }
}

/// What translates between the world and the controller.
#[derive(Debug)]
struct Adapter {
    /// It knows the layout and the robot numbers.
    translator: Translator,
    /// The lifts, by their numbers.
    lifts: Vec<LiftSpec>,
    /// The ids of the doors, by their numbers.
    doors: Vec<String>,
    /// For each lift, the sequence number of its next state.
    lift_seqs: Vec<u32>,
    report: Untranslated,
}

impl Adapter {
    /// The configuration of a controller for `world`, and the adapter that goes
    /// with it.
    fn new(world: &World) -> (Config, Adapter) {
        let graph = world.graph().clone();
        let robots: Vec<FleetRobot> = world
            .robot_ids()
            .filter_map(|id| FleetRobot::from_factsheet(&world.factsheet(id).ok()?).ok())
            .collect();
        let lifts = world.lift_specs();
        let doors = world.door_specs();
        let tick = world.tick();
        let mut adapter = Adapter {
            translator: Translator::new(Arc::new(graph.clone())),
            doors: doors.iter().map(|d| d.id.clone()).collect(),
            lift_seqs: vec![0; lifts.len()],
            lifts,
            report: Untranslated::default(),
        };
        for (i, robot) in robots.iter().enumerate() {
            let no = robot_no(i);
            if let Err(error) = adapter.translator.add_robot(no, robot.id.clone()) {
                let case = (What::Robot, Subject::Robot(no));
                let name = || robot_name(&robot.id);
                let reason = Reason::Table(error);
                adapter.report.note(case, true, reason, tick, name);
            }
        }
        let zones = world.zones().cloned();
        let zone_set = zones.as_ref().and_then(|z| adapter.site_zone_set(z, tick));
        let config = Config {
            graph,
            robots,
            zones,
            zone_set,
            lifts: adapter
                .lifts
                .iter()
                .map(|l| Lift {
                    id: l.id.clone(),
                    doors: l.doors.clone(),
                    stops: l
                        .stops
                        .iter()
                        .map(|s| LiftStop {
                            node: s.node,
                            door: s.door,
                            waiting: s.waiting,
                        })
                        .collect(),
                })
                .collect(),
            doors: doors
                .into_iter()
                .map(|d| Door {
                    id: d.id,
                    edges: d.edges,
                })
                .collect(),
            step_seconds: world.dt(),
            start_tick: tick,
        };
        (config, adapter)
    }

    /// The message with the site's zones.
    fn site_zone_set(&mut self, zones: &Zones, tick: u64) -> Option<Arc<ZoneSet>> {
        // (with no such map the controller leaves the zones out altogether)
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

    /// The report that says what the state message of the robot `id` says.
    fn state(
        &mut self,
        id: &RobotId,
        message: &vda5050::state::State,
        tick: u64,
    ) -> Option<FromRobot> {
        let (report, issues, refused) = match self.translator.state_from_vda(message) {
            Ok(t) => (Some(t.message), t.issues, false),
            Err(issue) => (None, vec![issue], true),
        };
        if !issues.is_empty() {
            // (a message of a robot without a number is counted under the number
            // no robot has)
            let no = self.translator.robot_no(id);
            let no = no.unwrap_or(RobotNo::new(u32::MAX, u32::MAX));
            let case = (What::State, Subject::Robot(no));
            for issue in issues {
                let reason = Reason::Translator(issue);
                self.report
                    .note(case, refused, reason, tick, || robot_name(id));
            }
        }
        report
    }

    /// The state of lift `l`.
    fn lift(
        &mut self,
        l: usize,
        status: &LiftStatus,
        rider: Option<&RobotId>,
        (time, tick): (Millis, u64),
    ) -> Option<FromLift> {
        let device = LiftNo(u32::try_from(l).unwrap_or(u32::MAX));
        let case = (What::LiftState, Subject::Lift(device));
        let name = || status.id.clone();
        let spec = self.lifts.get(l);
        let found = spec.and_then(|spec| spec.floors.get(status.floor));
        let car_door = u32::try_from(status.door_index).ok();
        let (Some(&map), Some(car_door)) = (found, car_door) else {
            let what = if spec.is_none() {
                "lift"
            } else {
                "floor or cabin door of the lift"
            };
            let reason = Reason::NotThere(what);
            self.report.note(case, true, reason, tick, name);
            return None;
        };
        let occupant = rider.and_then(|id| {
            let no = self.translator.robot_no(id);
            if no.is_none() {
                let reason = Reason::NotThere("robot as the one in the cabin");
                self.report.note(case, false, reason, tick, name);
            }
            no
        });
        let seq = self.lift_seqs.get_mut(l)?;
        let header = DeviceHeader {
            device,
            seq: *seq,
            time,
        };
        *seq = seq.wrapping_add(1);
        Some(FromLift {
            header,
            map,
            car_door,
            door: match status.door {
                DoorState::Closed => DoorPhase::Closed,
                DoorState::Opening => DoorPhase::Opening,
                DoorState::Open => DoorPhase::Open,
                DoorState::Closing => DoorPhase::Closing,
            },
            occupant,
        })
    }

    fn send(&mut self, world: &mut World, message: &Message) {
        match message {
            Message::Robot(message) => self.send_to_robot(world, message),
            Message::Door(message) => self.send_to_door(world, message),
            Message::Lift(message) => self.send_to_lift(world, message),
        }
    }

    fn send_to_robot(&mut self, world: &mut World, message: &ToRobot) {
        let what = match message.body {
            ToRobotBody::Order(_) => What::Order,
            ToRobotBody::InstantActions(_) => What::InstantActions,
            ToRobotBody::Responses(_) => What::Responses,
            ToRobotBody::ZoneSet(_) => What::ZoneSet,
        };
        let no = message.header.robot;
        let case = (what, Subject::Robot(no));
        let tick = world.tick();
        let (translated, issues, refused) = match self.translator.to_robot_to_vda(message) {
            Ok(t) => (Some(t.message), t.issues, false),
            Err(issue) => (None, vec![issue], true),
        };
        for issue in issues {
            self.note_issue(case, refused, issue, tick);
        }
        // (a robot's answer to an order is not used: it shows in the state)
        match translated {
            Some(RobotMessage::Order(m)) => {
                let _ = world.send_order(&m);
            }
            Some(RobotMessage::InstantActions(m)) => {
                let _ = world.send_instant_actions(&m);
            }
            Some(RobotMessage::Responses(m)) => {
                let _ = world.send_responses(&m);
            }
            Some(RobotMessage::ZoneSet(m)) => {
                let _ = world.send_zone_set(&m);
            }
            None => {}
        }
    }

    /// Count what the translator says about a message to a robot.
    fn note_issue(&mut self, case: (What, Subject), refused: bool, issue: Issue, tick: u64) {
        let translator = &self.translator;
        let name = || match case.1 {
            Subject::Robot(no) => translator
                .robot_id(no)
                .map_or_else(|| format!("robot number {}", no.slot()), robot_name),
            _ => String::new(),
        };
        let reason = Reason::Translator(issue);
        self.report.note(case, refused, reason, tick, name);
    }

    fn send_to_door(&mut self, world: &mut World, message: &ToDoor) {
        let device: DoorNo = message.header.device;
        let Some(id) = usize::try_from(device.0)
            .ok()
            .and_then(|d| self.doors.get(d))
        else {
            let case = (What::DoorCommand, Subject::Door(device));
            let name = || format!("door number {}", device.0);
            let reason = Reason::NotThere("door");
            self.report.note(case, true, reason, world.tick(), name);
            return;
        };
        let _ = match message.command {
            DoorCommand::Open => world.open_door(id),
            DoorCommand::Release => world.release_door(id),
        };
    }

    fn send_to_lift(&mut self, world: &mut World, message: &ToLift) {
        let device = message.header.device;
        let case = (What::LiftCommand, Subject::Lift(device));
        let tick = world.tick();
        let Some(spec) = usize::try_from(device.0)
            .ok()
            .and_then(|l| self.lifts.get(l))
        else {
            let name = || format!("lift number {}", device.0);
            let reason = Reason::NotThere("lift");
            self.report.note(case, true, reason, tick, name);
            return;
        };
        match message.command {
            LiftCommand::Release => {
                let _ = world.release_lift(&spec.id);
            }
            LiftCommand::Call { map, car_door } => {
                let map = self.translator.graph().maps.get(map);
                let door = usize::try_from(car_door)
                    .ok()
                    .and_then(|d| spec.doors.get(d));
                let (Some(map), Some(door)) = (map, door) else {
                    let name = || spec.id.clone();
                    let reason = Reason::NotThere("map or cabin door of the lift");
                    self.report.note(case, true, reason, tick, name);
                    return;
                };
                let _ = world.call_lift_door(&spec.id, map, door);
            }
        }
    }
}

/// A robot in words.
fn robot_name(id: &RobotId) -> String {
    format!("{}/{}", id.manufacturer, id.serial_number)
}

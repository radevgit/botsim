//! Connects the server's controller and the simulated world. The controller knows
//! no simulator ([`botsim_server::cycle`]) and the simulator no controller; this
//! module passes the data between them:
//! - [`config`]: the site and the fleet of a world, as the controller is told of
//!   them (a robot's type comes from its factsheet, S61);
//! - [`controller`], [`with_policy`], [`with_dispatcher`]: a controller for that
//!   site, [`Connected`] to the world;
//! - [`step`]: one cycle, from the world's reports to the messages sent to it;
//! - [`collect`]: the reports of a step without a cycle, for the next one;
//! - [`Connected::record`]: what the controller is given in every cycle is
//!   written down, to be replayed without the world (`botsim-replay`).
//!
//! A cycle need not run in every step of the world. Whoever runs them less often
//! calls [`collect`] in each step between: the world's reports are asked for once
//! in every step (that is what moves a delayed one on).
//!
//! Both sides speak [`botsim_link`] and number robots, lifts and doors the same
//! way, in the order they were added to the world: nothing is translated. The
//! site's zone set, which the controller sends to the robots, is made from the
//! world's configuration when the controller is set up.
//!
//! **The route through `vda`** ([`crate::vda_route`]), for tests: the same run
//! with every message to a robot and every report of a robot translated to VDA
//! 5050 and back.

use std::fmt::Debug;
use std::io::Write;
use std::ops::{Deref, DerefMut};
use std::sync::Arc;

use botsim_baseline::baseline_policy::BaselinePolicy;
use botsim_dispatcher::Dispatcher;
use botsim_link::message::{FromRobot, Millis, ToRobot};
use botsim_link::zone_set::ZoneSet;
use botsim_model::job::Job;
use botsim_model::zones::Zones;
use botsim_replay::Recorder;
use botsim_server::controller::Controller;
use botsim_server::cycle::{Config, FleetRobot, Inbox, Message};
use botsim_server::doors::Door;
use botsim_server::lifts::{Lift, LiftStop};
use botsim_server::policy::TrafficPolicy;
use botsim_sim::world::World;
use botsim_vda::translator::RobotMessage;

use crate::vda_route::VdaRoute;
use crate::vda_route::report::Untranslated;

/// The site and the fleet of `world`: its robots, lifts and doors in the order
/// they were added, its zones, and the time it is at. A robot's number is its
/// place in that order; should the factsheet of one not be read, it and the
/// robots after it are left out.
pub fn config(world: &World) -> Config {
    config_with(world, |zones| {
        Some(Arc::new(botsim_vda::zone_set_of_setup(zones)))
    })
}

/// [`config`], with the message of the site's zones made by `zone_set`.
pub(crate) fn config_with(
    world: &World,
    zone_set: impl FnOnce(&Zones) -> Option<Arc<ZoneSet>>,
) -> Config {
    let graph = world.graph().clone();
    let robots: Vec<FleetRobot> = world
        .robot_ids()
        .map_while(|id| FleetRobot::from_factsheet(&world.factsheet(id).ok()?).ok())
        .collect();
    let zones = world.zones().cloned();
    // (with no such map the controller leaves the zones out altogether)
    let zone_set = zones
        .as_ref()
        .filter(|zones| zones.map < graph.maps.len())
        .and_then(zone_set);
    Config {
        graph,
        robots,
        zones,
        zone_set,
        lifts: world
            .lift_specs()
            .into_iter()
            .map(|l| Lift {
                id: l.id,
                doors: l.doors,
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
        doors: world
            .door_specs()
            .into_iter()
            .map(|d| Door {
                id: d.id,
                edges: d.edges,
            })
            .collect(),
        start: Millis(world.now_ms()),
    }
}

/// A controller connected to a world: it is used as the controller it holds (it
/// derefs to it).
#[derive(Debug)]
pub struct Connected<P: TrafficPolicy = BaselinePolicy> {
    pub(crate) controller: Controller<P>,
    /// How many lifts the controller was told of.
    pub(crate) lifts: usize,
    /// On the route through `vda`, what translates.
    pub(crate) route: Option<Box<VdaRoute>>,
    /// What has arrived for the next cycle.
    pub(crate) inbox: Inbox,
    /// Writes down what the controller is given, if the run records it.
    pub(crate) recorder: Option<Recorder<Box<dyn Output>>>,
}

/// Where a recording of the controller's inputs can be written.
pub trait Output: Write + Debug {}

impl<T: Write + Debug> Output for T {}

impl<P: TrafficPolicy> Deref for Connected<P> {
    type Target = Controller<P>;

    fn deref(&self) -> &Controller<P> {
        &self.controller
    }
}

impl<P: TrafficPolicy> DerefMut for Connected<P> {
    fn deref_mut(&mut self) -> &mut Controller<P> {
        &mut self.controller
    }
}

/// The controller that `make` makes of the site and fleet of `world`, connected to
/// that world. `make` must keep the robots, lifts and doors and their order.
pub fn with_controller<P: TrafficPolicy>(
    world: &World,
    make: impl FnOnce(Config) -> Controller<P>,
) -> Connected<P> {
    let config = config(world);
    let lifts = config.lifts.len();
    Connected {
        controller: make(config),
        lifts,
        route: None,
        inbox: Inbox::default(),
        recorder: None,
    }
}

/// [`botsim_baseline::controller()`], with the baseline's traffic rules, for the
/// fleet of `world`.
pub fn controller(world: &World) -> Connected {
    with_controller(world, |config| botsim_baseline::controller(&config))
}

/// [`Controller::with_policy`] for the fleet of `world`.
pub fn with_policy<P: TrafficPolicy>(world: &World, policy: P) -> Connected<P> {
    with_controller(world, |config| Controller::with_policy(&config, policy))
}

/// [`Controller::with_dispatcher`] for the fleet of `world`.
pub fn with_dispatcher<P: TrafficPolicy>(
    world: &World,
    policy: P,
    dispatcher: Box<dyn Dispatcher>,
) -> Connected<P> {
    with_controller(world, |config| {
        Controller::with_dispatcher(&config, policy, dispatcher)
    })
}

impl<P: TrafficPolicy> Connected<P> {
    /// From the next cycle on, [`step`] writes down what the controller is given
    /// (see [`botsim_replay`]). `recorder` must be of the setup the controller was
    /// made from, and no cycle may have run yet.
    pub fn record(&mut self, recorder: Recorder<Box<dyn Output>>) {
        self.recorder = Some(recorder);
    }

    /// End the recording: `None` if there is none, else whether it was written
    /// whole.
    pub fn finish_recording(&mut self) -> Option<Result<(), botsim_replay::Error>> {
        self.recorder.take().map(Recorder::finish)
    }

    /// What the route through `vda` could not translate so far; `None` if the
    /// controller is not on that route, where nothing is translated.
    pub fn untranslated(&self) -> Option<&Untranslated> {
        self.route.as_ref().map(|route| route.untranslated())
    }

    /// Send one message of the controller to `world`. What the world refuses to
    /// take (a message to something it does not have) is dropped: the controller
    /// learns of a refused order from the robot's report.
    pub fn send(&mut self, world: &mut World, message: &Message) {
        match message {
            Message::Robot(message) => self.send_to_robot(world, message),
            // (doors and lifts are outside VDA 5050: the same on both routes)
            Message::Door(message) => {
                let _ = world.command_door(message);
            }
            Message::Lift(message) => {
                let _ = world.command_lift(message);
            }
        }
    }

    fn send_to_robot(&mut self, world: &mut World, message: &ToRobot) {
        match self.route.as_mut() {
            Some(route) => route.send(world, message),
            None => {
                let _ = world.send(message);
            }
        }
    }

    /// On the route through `vda`: hand the VDA 5050 message `message`, which may
    /// be one only another sender can write, to the robot it names, as the route
    /// does with the controller's. `false` if the controller is not on that route.
    pub fn receive(&mut self, world: &mut World, message: &RobotMessage) -> bool {
        let route = self.route.as_mut();
        route.map(|route| route.receive(world, message)).is_some()
    }

    /// On the route through `vda`: the report the controller gets for a robot's
    /// state message, read at tick `tick` of the world; `None` if the message is
    /// refused, or if the controller is not on that route.
    pub fn read_state(&mut self, message: &vda5050::state::State, tick: u64) -> Option<FromRobot> {
        self.route.as_mut()?.read_state(message, tick)
    }
}

/// Take what `world` has to say in this step into the inbox of the next cycle:
/// every robot's report and the states of the lifts. [`step`] does it itself:
/// call this once in each step without a cycle.
pub fn collect<P: TrafficPolicy>(connected: &mut Connected<P>, world: &mut World) {
    let tick = world.tick();
    // (the world makes a report of every robot in every step, which is what moves
    // a delayed one on; the fleet is the robots the controller was told of)
    let mut states = world.reports();
    states.truncate(connected.controller.robots().count());
    if let Some(route) = connected.route.as_mut() {
        // (one that cannot be translated has not arrived: that robot has no new
        // report from it)
        states = states
            .iter()
            .filter_map(|report| route.report(report, tick))
            .collect();
    }
    connected.inbox.robots.append(&mut states);
    let mut lifts = world.lift_reports();
    lifts.truncate(connected.lifts);
    connected.inbox.lifts.append(&mut lifts);
}

/// One cycle of the controller at the world's time: [`collect`], then the new jobs
/// and all that has been collected go in; the messages are sent to the world in
/// their order, and what the controller holds back (and, if somebody watches,
/// reserves) is published there. Call it after [`World::step`], in every step or,
/// with [`collect`] in the others, less often.
pub fn step<P: TrafficPolicy>(
    connected: &mut Connected<P>,
    world: &mut World,
    new_jobs: impl IntoIterator<Item = Job>,
) {
    collect(connected, world);
    let mut inbox = std::mem::take(&mut connected.inbox);
    inbox.jobs.extend(new_jobs);
    inbox.watched = world.observing();
    let now = Millis(world.now_ms());
    let output = match connected.recorder.as_mut() {
        Some(recorder) => recorder.step(&mut connected.controller, now, inbox),
        None => connected.controller.step(now, inbox),
    };
    for message in &output.messages {
        connected.send(world, message);
    }
    world.publish_held_back(output.held_back);
    if let Some(reservations) = output.reservations {
        world.publish_reservations(reservations);
    }
}

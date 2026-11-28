//! Runs the server's controller against the simulated world. The controller knows
//! no simulator ([`botsim_server::cycle`]) and the simulator no controller; this
//! crate passes the data between them:
//! - [`config`]: the site and the fleet of a world, as the controller is told of
//!   them (a robot's type comes from its factsheet, S61);
//! - [`controller`], [`with_policy`], [`with_dispatcher`]: a controller for that
//!   site, [`Bridged`] to the world;
//! - [`step`]: one cycle, from the world's reports to the messages sent to it.
//!
//! Both sides speak [`botsim_link`] and number robots, lifts and doors the same
//! way, in the order they were added to the world: nothing is translated. The
//! site's zone set, which the controller sends to the robots, is made from the
//! world's configuration when the controller is set up.
//!
//! **The route through `vda`** ([`vda_route`]), for tests: the same run with
//! every message to a robot and every report of a robot translated to VDA 5050
//! and back.
//!
//! It is the one place that does this, for `botsim` and for tests. It is there
//! only until `botsim` connects the server and the simulator itself.

pub mod vda_route;

use std::ops::{Deref, DerefMut};
use std::sync::Arc;

use botsim_baseline::baseline_policy::BaselinePolicy;
use botsim_core::world::World;
use botsim_dispatcher::Dispatcher;
use botsim_link::message::{FromRobot, ToRobot};
use botsim_link::zone_set::ZoneSet;
use botsim_model::job::Job;
use botsim_model::zones::Zones;
use botsim_server::controller::Controller;
use botsim_server::cycle::{Config, FleetRobot, Input, Message, Time};
use botsim_server::doors::Door;
use botsim_server::lifts::{Lift, LiftStop};
use botsim_server::policy::TrafficPolicy;
use botsim_vda::translator::RobotMessage;

use crate::vda_route::VdaRoute;
use crate::vda_route::report::Untranslated;

/// The site and the fleet of `world`: its robots, lifts and doors in the order
/// they were added, its zones, its time step and the tick it is at. A robot's
/// number is its place in that order; should the factsheet of one not be read,
/// it and the robots after it are left out.
pub fn config(world: &World) -> Config {
    config_with(world, |zones| {
        Some(Arc::new(botsim_vda::zone_set_of_setup(zones)))
    })
}

/// [`config`], with the message of the site's zones made by `zone_set`.
fn config_with(world: &World, zone_set: impl FnOnce(&Zones) -> Option<Arc<ZoneSet>>) -> Config {
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
        step_seconds: world.dt(),
        start_tick: world.tick(),
    }
}

/// A controller connected to a world: it is used as the controller it holds (it
/// derefs to it).
#[derive(Debug)]
pub struct Bridged<P: TrafficPolicy = BaselinePolicy> {
    controller: Controller<P>,
    /// How many lifts the controller was told of.
    lifts: usize,
    /// On the route through `vda`, what translates.
    route: Option<Box<VdaRoute>>,
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
    let config = config(world);
    let lifts = config.lifts.len();
    Bridged {
        controller: make(config),
        lifts,
        route: None,
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

/// One cycle of the controller: the new jobs, the world's time, every robot's
/// report and the states of the lifts go in; the messages are sent to the world in
/// their order, and what the controller holds back (and, if somebody watches,
/// reserves) is published there. Call it once per step, after [`World::step`].
pub fn step<P: TrafficPolicy>(
    bridged: &mut Bridged<P>,
    world: &mut World,
    new_jobs: impl IntoIterator<Item = Job>,
) {
    let tick = world.tick();
    let time = Time {
        tick,
        unix_ms: world.now_ms(),
    };
    // (the world makes a report of every robot in every cycle, which is what moves
    // a delayed one on; the fleet is the robots the controller was told of)
    let mut states = world.reports();
    states.truncate(bridged.controller.robots().count());
    if let Some(route) = bridged.route.as_mut() {
        // (if one cannot be translated, the list is short and the cycle does
        // nothing)
        states = states
            .iter()
            .filter_map(|report| route.report(report, tick))
            .collect();
    }
    let mut lifts = world.lift_reports();
    lifts.truncate(bridged.lifts);
    let input = Input {
        time,
        states: &states,
        lifts: &lifts,
        watched: world.observing(),
    };
    let output = bridged.controller.step(&input, new_jobs);
    for message in &output.messages {
        bridged.send(world, message);
    }
    if let Some(held_back) = output.held_back {
        world.publish_held_back(held_back);
    }
    if let Some(reservations) = output.reservations {
        world.publish_reservations(reservations);
    }
}

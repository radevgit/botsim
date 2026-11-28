//! Runs the baseline controller against the simulated world. The controller knows
//! no simulator ([`botsim_baseline::cycle`]) and the simulator no controller; this
//! crate passes the data between them:
//! - [`config`]: the site and the fleet of a world, as the controller is told of
//!   them (a robot's type comes from its factsheet message, S61);
//! - [`step`]: one cycle, from the world's states to the messages sent to it.
//!
//! It is the one place that does so, for `botsim` and for tests. It is there only
//! until the server and the simulator are connected through `link`.

use botsim_baseline::controller::Controller;
use botsim_baseline::cycle::{Config, FleetRobot, Input, Message, Output, Time};
use botsim_baseline::doors::Door;
use botsim_baseline::lifts::{Lift, LiftStop};
use botsim_baseline::policy::TrafficPolicy;
use botsim_core::world::World;
use botsim_dispatcher::Dispatcher;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;

/// The site and the fleet of `world`: its robots in the order they were added (one
/// whose factsheet cannot be read is left out), its zones, lifts and doors, its
/// time step and the tick it is at.
pub fn config(world: &World) -> Config {
    Config {
        graph: world.graph().clone(),
        robots: world
            .robot_ids()
            .filter_map(|id| FleetRobot::from_factsheet(&world.factsheet(id).ok()?).ok())
            .collect(),
        zones: world.zones().cloned(),
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

/// [`Controller::new`] for the fleet of `world`.
pub fn controller(world: &World) -> Controller {
    Controller::new(&config(world))
}

/// [`Controller::with_policy`] for the fleet of `world`.
pub fn with_policy<P: TrafficPolicy>(world: &World, policy: P) -> Controller<P> {
    Controller::with_policy(&config(world), policy)
}

/// [`Controller::with_dispatcher`] for the fleet of `world`.
pub fn with_dispatcher<P: TrafficPolicy>(
    world: &World,
    policy: P,
    dispatcher: Box<dyn Dispatcher>,
) -> Controller<P> {
    Controller::with_dispatcher(&config(world), policy, dispatcher)
}

/// One cycle of `controller`: the new jobs, the world's time, every robot's state
/// and the riders of the lifts go in; the messages are sent to the world in their
/// order, and what the controller holds back (and, if somebody watches,
/// reserves) is published there. Call it once per step, after [`World::step`].
///
/// What the world refuses to take (a message to something it does not have) is
/// dropped: the controller learns of a refused order from the robot's state.
pub fn step<P: TrafficPolicy>(
    controller: &mut Controller<P>,
    world: &mut World,
    new_jobs: impl IntoIterator<Item = Job>,
) {
    // (asking for a state is what moves a delayed state message on, so every
    // robot is asked once in every cycle; if one cannot be read, the list is
    // short and the cycle does nothing)
    let ids: Vec<RobotId> = controller.robots().cloned().collect();
    let states: Vec<_> = ids.iter().map_while(|id| world.state(id).ok()).collect();
    let lift_riders: Vec<Option<RobotId>> = world
        .lift_specs()
        .iter()
        .map(|l| world.lift_rider(&l.id).ok().flatten().cloned())
        .collect();
    let input = Input {
        time: Time {
            tick: world.tick(),
            unix_ms: world.now_ms(),
        },
        states: &states,
        lift_riders: &lift_riders,
        watched: world.observing(),
    };
    let output = controller.step(&input, new_jobs);
    deliver(world, output);
}

/// Send the messages of a cycle to the world and publish the rest.
fn deliver(world: &mut World, output: Output) {
    for message in output.messages {
        // (a robot's answer to an order is not used: it shows in the state)
        match message {
            Message::Order(m) => {
                let _ = world.send_order(&m);
            }
            Message::InstantActions(m) => {
                let _ = world.send_instant_actions(&m);
            }
            Message::Responses(m) => {
                let _ = world.send_responses(&m);
            }
            Message::ZoneSet(m) => {
                let _ = world.send_zone_set(&m);
            }
            Message::OpenDoor(door) => {
                let _ = world.open_door(&door);
            }
            Message::ReleaseDoor(door) => {
                let _ = world.release_door(&door);
            }
            Message::CallLift { lift, map, door } => {
                let _ = world.call_lift_door(&lift, &map, &door);
            }
            Message::ReleaseLift(lift) => {
                let _ = world.release_lift(&lift);
            }
        }
    }
    if let Some(held_back) = output.held_back {
        world.publish_held_back(held_back);
    }
    if let Some(reservations) = output.reservations {
        world.publish_reservations(reservations);
    }
}

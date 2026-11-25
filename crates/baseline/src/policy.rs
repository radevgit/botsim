//! The traffic policy: every "who may go now" decision of the controller.
//!
//! The controller is split in two (brief `traffic_policy_slot.md`). The
//! **dispatcher** gives jobs to robots, plans and re-plans their whole routes, sends
//! charging trips, runs lifts and doors, and builds and sends the VDA 5050 messages.
//! The **policy**, behind [`TrafficPolicy`], says how far each route is released and
//! answers RELEASE zone requests. It never plans paths and never talks to a robot.
//!
//! Robots are numbered as in the controller (the order of the world's robots); every
//! list of robots is in that order.
//!
//! **Order of the calls in one cycle:**
//! 1. [`cycle_start`](TrafficPolicy::cycle_start) with all robots;
//! 2. robot by robot: [`release`](TrafficPolicy::release) for a robot that has a
//!    route and can take an update (or [`route_finished`](TrafficPolicy::route_finished)
//!    / [`route_dropped`](TrafficPolicy::route_dropped) when its route ends); after a
//!    new plan for it, [`route_changed`](TrafficPolicy::route_changed);
//! 3. [`after_releases`](TrafficPolicy::after_releases);
//! 4. [`idle`](TrafficPolicy::idle) for every robot without a route;
//! 5. [`zone_answers`](TrafficPolicy::zone_answers);
//! 6. [`place`](TrafficPolicy::place) for every route given out in this cycle;
//! 7. [`reservations`](TrafficPolicy::reservations), if the run is watched.

use std::collections::HashSet;
use std::fmt::Debug;

use botsim_core::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_core::observer::Reservation;
use botsim_core::spec::Shape;
use botsim_core::world::RobotId;
use botsim_core::zones::Zones;
use vda5050::state::State;

use crate::conflicts::{ConflictInput, NodeConflictInput};
use crate::order_builder::{Plan, RELEASE_WINDOW};
use crate::zone_grants::Answer;

/// One robot of the fleet, as far as it does not change.
#[derive(Debug, Clone)]
pub struct PolicyRobot {
    pub id: RobotId,
    /// The series name of its type (the manufacturer is in `id`).
    pub series: String,
    /// Index into [`Setup::types`].
    pub type_idx: usize,
    /// The body outline.
    pub shape: Shape,
    /// It drives in any direction without turning first.
    pub omnidirectional: bool,
}

/// What stays the same during a run.
#[derive(Debug)]
pub struct Setup<'a> {
    pub graph: &'a Graph,
    /// The robot types as `manufacturer/seriesName`.
    pub types: &'a [String],
    pub robots: &'a [PolicyRobot],
    /// The site's zones, if the run has any.
    pub zones: Option<&'a Zones>,
}

/// A robot's route as it stands.
#[derive(Debug, Clone, Copy)]
pub struct RouteView<'a> {
    /// The whole route.
    pub route: &'a Plan,
    /// Index in `route` of the last released node.
    pub released_through: usize,
    /// The order is being cancelled: nothing more will be released.
    pub cancelling: bool,
}

/// All robots in one cycle.
#[derive(Debug)]
pub struct Fleet<'a> {
    /// Simulation step of the cycle.
    pub tick: u64,
    /// Simulated seconds since the cycle before.
    pub elapsed: f64,
    /// The state messages as received.
    pub states: &'a [State],
    /// `None` for a robot without a route.
    pub routes: &'a [Option<RouteView<'a>>],
}

/// The question "how far may this robot's route be released now?".
#[derive(Debug, Clone, Copy)]
pub struct Ask<'a> {
    pub robot: usize,
    pub state: &'a State,
    /// The whole route.
    pub route: &'a Plan,
    /// Index in `route` of the last node the robot has reached.
    pub at: usize,
    /// Index of the last node released so far.
    pub released_through: usize,
    /// The furthest node that may be released now: the policy's
    /// [`window`](TrafficPolicy::window), cut short by lifts and doors.
    pub limit: usize,
    /// Simulated seconds since the cycle before (0 in
    /// [`place`](TrafficPolicy::place)).
    pub elapsed: f64,
}

/// The answer to an [`Ask`].
#[derive(Debug, Clone, PartialEq)]
pub struct Release {
    /// Index of the last node to release: from [`Ask::released_through`] (nothing new)
    /// to [`Ask::limit`].
    pub through: usize,
    /// Ask the dispatcher for another route ("cannot place this route", or the robot
    /// is stuck). The dispatcher changes the route only beyond the released part, and
    /// keeps it if there is no other one.
    pub reroute: Option<Reroute>,
}

/// What a new route should do.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Reroute {
    /// Nodes the new route must not pass.
    pub avoid: HashSet<NodeIdx>,
}

/// Who may go now. The bundled baseline rules are the first implementation.
pub trait TrafficPolicy: Debug {
    /// Once, before the first cycle.
    fn start(&mut self, setup: &Setup);

    /// The edge pairs two robot types must not use at the same time (`botsim-sweep`).
    fn conflicts(&mut self, _edges: &[ConflictInput]) {}

    /// The node-edge pairs; called after [`conflicts`](TrafficPolicy::conflicts).
    fn node_conflicts(&mut self, _nodes: &[NodeConflictInput]) {}

    /// How many nodes ahead of the robot may be released.
    fn window(&self) -> usize {
        RELEASE_WINDOW
    }

    /// The first thing in every cycle, with every robot's progress.
    fn cycle_start(&mut self, _fleet: &Fleet) {}

    /// A robot got a new route; `ask.at` and `ask.released_through` are 0 (it stands
    /// on the first node).
    fn place(&mut self, ask: &Ask) -> Release;

    /// A robot with a route has reported its progress.
    fn release(&mut self, ask: &Ask) -> Release;

    /// The dispatcher changed the unreleased part of a route.
    fn route_changed(&mut self, _robot: usize, _route: &Plan) {}

    /// The robot has done its route and stands on the last node.
    fn route_finished(&mut self, _robot: usize, _route: &Plan) {}

    /// The route is given up (the robot refused the order or an update).
    /// `released_through` is the last node it had been given.
    fn route_dropped(&mut self, _robot: usize, _route: &Plan, _released_through: usize) {}

    /// After all releases of the cycle (the baseline lets robots go first here).
    fn after_releases(&mut self, _fleet: &Fleet) {}

    /// A robot without a route stands on `node`.
    fn idle(&mut self, _robot: usize, _node: NodeIdx) {}

    /// Answers to the RELEASE zone requests the states carry.
    fn zone_answers(&mut self, fleet: &Fleet) -> Vec<Answer>;

    /// The robot that holds a node, for the dispatcher (route load, free chargers).
    fn node_holder(&self, _node: NodeIdx) -> Option<usize> {
        None
    }

    /// The robot that holds an edge, for the dispatcher (route load).
    fn edge_holder(&self, _edge: EdgeIdx) -> Option<usize> {
        None
    }

    /// What is held, for the observer.
    fn reservations(&self) -> Vec<Reservation> {
        Vec::new()
    }

    /// The holder query for waits: the robot that keeps `robot`, standing at the end
    /// of its released route, from the next node of `route`; `None` if no robot
    /// does, or none can be named. It only reports (who waits for whom, deadlocks);
    /// it must not change anything. Asked after the releases of a cycle.
    fn blocker(&self, _robot: usize, _route: &Plan, _released_through: usize) -> Option<usize> {
        None
    }

    /// The same for a RELEASE zone request of `robot` that is not granted: the robots
    /// that keep the zone from it; empty if none can be named.
    fn zone_blockers(&self, _robot: usize) -> Vec<usize> {
        Vec::new()
    }
}

/// A boxed policy is a policy, so that one can be chosen at run time
/// (`Box<dyn TrafficPolicy>`).
impl<T: TrafficPolicy + ?Sized> TrafficPolicy for Box<T> {
    fn start(&mut self, setup: &Setup) {
        (**self).start(setup);
    }
    fn conflicts(&mut self, edges: &[ConflictInput]) {
        (**self).conflicts(edges);
    }
    fn node_conflicts(&mut self, nodes: &[NodeConflictInput]) {
        (**self).node_conflicts(nodes);
    }
    fn window(&self) -> usize {
        (**self).window()
    }
    fn cycle_start(&mut self, fleet: &Fleet) {
        (**self).cycle_start(fleet);
    }
    fn place(&mut self, ask: &Ask) -> Release {
        (**self).place(ask)
    }
    fn release(&mut self, ask: &Ask) -> Release {
        (**self).release(ask)
    }
    fn route_changed(&mut self, robot: usize, route: &Plan) {
        (**self).route_changed(robot, route);
    }
    fn route_finished(&mut self, robot: usize, route: &Plan) {
        (**self).route_finished(robot, route);
    }
    fn route_dropped(&mut self, robot: usize, route: &Plan, released_through: usize) {
        (**self).route_dropped(robot, route, released_through);
    }
    fn after_releases(&mut self, fleet: &Fleet) {
        (**self).after_releases(fleet);
    }
    fn idle(&mut self, robot: usize, node: NodeIdx) {
        (**self).idle(robot, node);
    }
    fn zone_answers(&mut self, fleet: &Fleet) -> Vec<Answer> {
        (**self).zone_answers(fleet)
    }
    fn node_holder(&self, node: NodeIdx) -> Option<usize> {
        (**self).node_holder(node)
    }
    fn edge_holder(&self, edge: EdgeIdx) -> Option<usize> {
        (**self).edge_holder(edge)
    }
    fn reservations(&self) -> Vec<Reservation> {
        (**self).reservations()
    }
    fn blocker(&self, robot: usize, route: &Plan, released_through: usize) -> Option<usize> {
        (**self).blocker(robot, route, released_through)
    }
    fn zone_blockers(&self, robot: usize) -> Vec<usize> {
        (**self).zone_blockers(robot)
    }
}

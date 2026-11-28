//! The traffic policy, the server's slot for the traffic rules: every "who may go
//! now" decision of the controller.
//!
//! A [dispatcher](botsim_dispatcher) gives tasks to robots, with their first
//! routes. The **controller** carries them out: it re-plans routes, runs lifts and
//! doors, and builds and sends the messages ([`botsim_link`]). The **policy**, behind
//! [`TrafficPolicy`] (brief `traffic_policy_slot.md`), says how far each route is
//! released and answers RELEASE zone requests. It never plans paths and never talks
//! to a robot.
//!
//! Robots are numbered as in the controller (the order of
//! [`Config::robots`](crate::cycle::Config::robots)); every list of robots is in
//! that order.
//!
//! **Order of the calls in one cycle:**
//! 0. for what the states show as refused since the cycle before:
//!    [`route_dropped`](TrafficPolicy::route_dropped) for a new order, and
//!    [`route_changed`](TrafficPolicy::route_changed) with the old route for a
//!    refused addition of a job to a parking trip;
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
//!
//! No cycle call is made before every robot has reported (see [`crate::cycle`]);
//! only [`reservations`](TrafficPolicy::reservations) is asked then, of a watched
//! run.
//!
//! **A robot without a new report** ([`Known::new`] is `false`) holds back only
//! itself. The controller asks no [`release`](TrafficPolicy::release) or
//! [`place`](TrafficPolicy::place) for it, and says
//! [`idle`](TrafficPolicy::idle) of one without a route by the report it has kept.
//! **The policy must give such a robot nothing new** (no zone grant or other
//! answer, no node or edge taken over for it) **and take nothing from it**: what
//! it holds stays held.

use std::collections::HashSet;
use std::fmt::Debug;

use botsim_link::ids::RequestId;
use botsim_link::message::Millis;
use botsim_link::response::Grant;
use botsim_link::state::State;
use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_model::reservation::Reservation;
use botsim_model::robot::RobotId;
use botsim_model::spec::Shape;
use botsim_model::zones::Zones;

use crate::conflicts::{ConflictInput, NodeConflictInput, NodeNodeConflictInput};
use crate::cycle::Known;
use crate::order_builder::{Plan, RELEASE_WINDOW};

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
    /// The controller's clock, which does not go back.
    pub now: Millis,
    /// Seconds since the cycle before.
    pub elapsed: f64,
    /// The newest report of every robot, and whether it arrived in this cycle.
    pub robots: &'a [Known],
    /// `None` for a robot without a route.
    pub routes: &'a [Option<RouteView<'a>>],
}

/// The question "how far may this robot's route be released now?".
#[derive(Debug, Clone, Copy)]
pub struct Ask<'a> {
    pub robot: usize,
    /// What the robot reported last.
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
    /// Seconds since the cycle that took the robot's report before, which is what
    /// a time spent waiting adds up (0 in [`place`](TrafficPolicy::place)).
    pub elapsed: f64,
}

/// The answer to an [`Ask`].
#[derive(Debug, Clone, PartialEq)]
pub struct Release {
    /// Index of the last node to release: from [`Ask::released_through`] (nothing new)
    /// to [`Ask::limit`].
    pub through: usize,
    /// Ask the controller for another route ("cannot place this route", or the robot
    /// is stuck). The controller changes the route only beyond the released part, and
    /// keeps it if there is no other one.
    pub reroute: Option<Reroute>,
}

/// What a new route should do.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Reroute {
    /// Nodes the new route must not pass.
    pub avoid: HashSet<NodeIdx>,
}

/// One answer to a RELEASE zone request.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ZoneAnswer {
    pub robot: usize,
    pub request_id: RequestId,
    pub grant: Grant,
}

/// Who may go now. The caller chooses the rules when it makes a controller.
pub trait TrafficPolicy: Debug {
    /// Once, before the first cycle.
    fn start(&mut self, setup: &Setup);

    /// The edge pairs two robot types must not use at the same time (`botsim-space`).
    fn conflicts(&mut self, _edges: &[ConflictInput]) {}

    /// The node-edge pairs; called after [`conflicts`](TrafficPolicy::conflicts).
    fn node_conflicts(&mut self, _nodes: &[NodeConflictInput]) {}

    /// The node-node pairs; called after [`conflicts`](TrafficPolicy::conflicts).
    fn node_node_conflicts(&mut self, _nodes: &[NodeNodeConflictInput]) {}

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

    /// The controller changed the unreleased part of a route (another route, a job
    /// added to a parking trip, or such an addition taken back because the robot
    /// refused it).
    fn route_changed(&mut self, _robot: usize, _route: &Plan) {}

    /// The robot has done its route and stands on the last node.
    fn route_finished(&mut self, _robot: usize, _route: &Plan) {}

    /// The route is given up (the robot refused the order or an update).
    /// `released_through` is the last node it had been given.
    fn route_dropped(&mut self, _robot: usize, _route: &Plan, _released_through: usize) {}

    /// After all releases of the cycle (the baseline lets robots go first here).
    fn after_releases(&mut self, _fleet: &Fleet) {}

    /// A robot without a route stands on `node`, as its newest report says.
    fn idle(&mut self, _robot: usize, _node: NodeIdx) {}

    /// Answers to the RELEASE zone requests the states carry; none for a robot
    /// without a new report.
    fn zone_answers(&mut self, fleet: &Fleet) -> Vec<ZoneAnswer>;

    /// The robot that holds a node, for route load and for the dispatcher (free
    /// chargers and parking places).
    fn node_holder(&self, _node: NodeIdx) -> Option<usize> {
        None
    }

    /// The robot that holds an edge, for route load.
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
    fn node_node_conflicts(&mut self, nodes: &[NodeNodeConflictInput]) {
        (**self).node_node_conflicts(nodes);
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
    fn zone_answers(&mut self, fleet: &Fleet) -> Vec<ZoneAnswer> {
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

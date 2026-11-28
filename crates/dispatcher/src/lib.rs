//! The dispatcher of a fleet server: which robot gets which transport job, and
//! which robot drives to a charger or a parking place.
//!
//! It is a part of its own that another dispatcher can replace: the trait
//! [`Dispatcher`], with [`baseline::BaselineDispatcher`] as the bundled one. It
//! finds routes with `botsim-router` and knows nothing of messages, traffic rules
//! or a simulation: the **controller** that uses it hands it plain data
//! ([`Site`], [`Robot`]) and answers its questions and carries out what it decides
//! ([`Control`]).
//!
//! Robots are numbers, 0 up to the number given to [`Dispatcher::start`], the same
//! in every call; nodes and edges are the numbers of the layout graph.
//!
//! **Calls, in each control cycle:**
//! 1. [`add_job`](Dispatcher::add_job) for every new job;
//! 2. [`arrived`](Dispatcher::arrived) or [`given_up`](Dispatcher::given_up) for
//!    every robot whose task ends;
//! 3. [`stop_charging`](Dispatcher::stop_charging) for every robot;
//! 4. [`dispatch`](Dispatcher::dispatch), once.
//!
//! **Dispatching is serial.** The dispatcher decides one task at a time; the
//! controller has carried it out when [`Control`] returns, so the next decision
//! sees its effect.

use std::collections::HashMap;
use std::fmt::Debug;

use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_model::job::Job;
use botsim_router::types::TypeRouting;

pub mod baseline;

/// What stays the same during a run.
///
/// The parts must fit each other: every list of a [`TypeRouting`] belongs to
/// `graph` (an entry for each edge, `into` for each node), and the rides are edges
/// of it.
#[derive(Debug, Clone, Copy)]
pub struct Site<'a> {
    pub graph: &'a Graph,
    /// What each robot type may drive; [`Robot::type_idx`] is an index into it.
    pub types: &'a [TypeRouting],
    /// Every lift ride with the waiting node of its entrance, `None` if there is
    /// none (see [`botsim_router::lift`]).
    pub rides: &'a HashMap<EdgeIdx, Option<NodeIdx>>,
}

/// A robot as it stands when [`Dispatcher::dispatch`] is called.
#[derive(Debug, Clone, PartialEq)]
pub struct Robot {
    /// Index into [`Site::types`].
    pub type_idx: usize,
    /// The node it stands on, from its reported position; `None` if it is on none.
    pub node: Option<NodeIdx>,
    /// Its reported heading (radians); `None` if it reports no position.
    pub heading: Option<f64>,
    /// Its reported battery charge (percent).
    pub charge: f64,
    /// It may be given work as far as the controller's own rules go (the site's
    /// zone set is enabled, if the site has one).
    pub ready: bool,
    pub work: Work,
}

/// What a robot is doing, as far as the dispatcher is concerned.
#[derive(Debug, Clone, PartialEq)]
pub enum Work {
    /// It has no task. (It may be charging: the dispatcher knows that itself.)
    None,
    /// It is on its way to a parking place and can take an update of its order: the
    /// order is answered and shown by the robot, it is not being cancelled, and the
    /// robot owns no lift or door.
    Parking {
        /// The last released node, where an extension starts.
        from: NodeIdx,
        /// The released edges it has still to drive to get there.
        ahead: Vec<EdgeIdx>,
        /// The edge by which it will arrive at `from`; `None` if `from` is the
        /// start of the trip.
        arrival: Option<EdgeIdx>,
    },
    /// Any other task: it cannot be given one now.
    Busy,
}

/// When a cycle takes place.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cycle {
    /// The controller's clock in steps (a new job of a trip gets it).
    pub tick: u64,
    /// Seconds since the cycle before.
    pub elapsed: f64,
}

/// What a task is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Purpose {
    /// A transport job.
    Job,
    /// To a charger; on arrival the charging starts.
    Charge,
    /// To a free parking place (U13).
    Park,
}

/// A route a robot is to drive, and what it is for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Task {
    /// The transport job. For a trip to a charger or a parking place a stand-in:
    /// from the robot's node to the goal, numbered by the trips of that kind.
    pub job: Job,
    pub purpose: Purpose,
    /// The node the route starts at.
    pub from: NodeIdx,
    /// The edges to drive, one after the other.
    pub edges: Vec<EdgeIdx>,
    /// [`Purpose::Job`]: the first `pickup_at` edges lead to the pickup, the rest
    /// from there to the drop. The route must fit the [`job`](Task::job): its
    /// pickup node after those edges, its drop node at the end. Not used for a
    /// trip.
    pub pickup_at: usize,
}

/// What became of a task given to the controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Answer {
    /// The order is on its way to the robot. If the robot refuses it, the
    /// dispatcher hears of it later ([`Dispatcher::given_up`]).
    Taken,
    /// The controller could not send it. The task is gone: the controller notes a
    /// refused job and gives it to nobody else.
    Refused,
    /// The controller cannot do what was asked (an unknown robot, one that cannot
    /// take the task, a route that does not start at the right node or does not
    /// connect, a job that does not fit its route, or an addition to a trip whose
    /// pickup is the node it starts at). Nothing has happened.
    Invalid,
}

/// When robots are sent to a charger (percent of the charge a robot reports).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChargePolicy {
    /// A free robot below this goes to a charger.
    pub low: f64,
    /// A charging robot gets jobs again from this on.
    pub high: f64,
    /// Below this a robot takes no more jobs and waits for a charger.
    pub critical: f64,
}

/// The figures of a dispatcher that reports use.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Counts {
    /// The parking places of the layout: the interaction nodes of the stations of
    /// type `parking`.
    pub parking_places: usize,
    /// Trips to a parking place started so far, refused ones included.
    pub park_trips: usize,
    /// How often a robot without a job found no free parking place (or none it can
    /// reach) and stayed where it was; counted once until it gets a task again.
    pub no_free_parking: usize,
    /// Trips to a charger started so far, refused ones and charging on the spot
    /// included.
    pub charge_trips: usize,
    /// Seconds that free robots below the low limit have waited for a charger (all
    /// robots together; robots that are busy are not counted).
    pub charge_wait_seconds: f64,
}

/// What a dispatcher may ask and decide, implemented by the controller that calls
/// [`Dispatcher::dispatch`].
///
/// **The controller promises:**
/// - The calls are serial: one at a time, from the one thread that dispatches.
/// - A task is carried out before [`give`](Control::give) or
///   [`extend`](Control::extend) returns: the message is sent, and the traffic
///   rules, lifts and doors know of it (an order that cannot be sent gives back its
///   claims).
///   So the next [`node_holder`](Control::node_holder) and
///   [`route_cost`](Control::route_cost) see the effect.
/// - The [`Robot`] list is **not** updated during a dispatch: the dispatcher itself
///   keeps track of the robots it has given tasks to. Nothing else about them
///   changes while it runs.
/// - What cannot be done is answered with [`Answer::Invalid`], never a crash.
pub trait Control {
    /// The robot that holds the node now, as the traffic rules say; `None` if none
    /// does, or none can be named.
    fn node_holder(&self, node: NodeIdx) -> Option<usize>;

    /// The cost of every edge for a route of `robot` now, with the penalties for
    /// busy places; `None`: use the plain lengths of its type.
    fn route_cost(&self, robot: usize) -> Option<Vec<f64>>;

    /// Give a robot without a task this one.
    fn give(&mut self, robot: usize, task: Task) -> Answer;

    /// Give a job to a robot on its way to a parking place ([`Work::Parking`]):
    /// `task` starts at the trip's last released node, has [`Purpose::Job`] and its
    /// pickup beyond that node ([`Task::pickup_at`] at least 1: a robot does not
    /// run the actions of the node an update starts at). The released part of the
    /// trip stays in front of it.
    fn extend(&mut self, robot: usize, task: Task) -> Answer;

    /// A robot without a task is to start charging where it stands; `trip` is the
    /// number of this charger trip.
    fn charge_here(&mut self, robot: usize, trip: usize);
}

/// Gives jobs to robots and sends them to charge and park.
pub trait Dispatcher: Debug {
    /// Once, before anything else: the layout and the number of robots.
    fn start(&mut self, graph: &Graph, robots: usize);

    /// Send robots to chargers, with these limits (by default none is sent).
    /// Returns the number of charger nodes of the layout (0: nothing will happen).
    fn set_charging(&mut self, policy: ChargePolicy) -> usize;

    /// A new transport job.
    fn add_job(&mut self, job: Job);

    /// Jobs not yet given to a robot, oldest first.
    fn pending(&self) -> Box<dyn Iterator<Item = &Job> + '_>;

    /// A robot reports `charge` percent: `true` if it was charging and is to stop
    /// now (the controller tells it to). It is free for work again.
    fn stop_charging(&mut self, robot: usize, charge: f64) -> bool;

    /// The robot has done its task and stands on the last node. After a
    /// [`Purpose::Charge`] trip it starts charging now.
    fn arrived(&mut self, robot: usize, purpose: Purpose);

    /// The robot's task is given up: it refused the order or an update of it.
    fn given_up(&mut self, robot: usize, purpose: Purpose);

    /// Give tasks to the robots, one at a time, through `control`.
    fn dispatch(&mut self, site: &Site, robots: &[Robot], cycle: Cycle, control: &mut dyn Control);

    /// The figures so far.
    fn counts(&self) -> Counts;
}

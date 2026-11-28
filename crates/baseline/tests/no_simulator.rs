//! The controller runs on data alone: a site and states written by hand, no
//! simulator.

use std::cell::RefCell;
use std::rc::Rc;

use botsim_baseline::baseline_policy::BaselinePolicy;
use botsim_dispatcher::baseline::BaselineDispatcher;
use botsim_dispatcher::{
    Answer, ChargePolicy, Control, Counts, Cycle, Dispatcher, Purpose, Robot, Site, Task, Work,
};
use botsim_link::message::{FromRobot, Millis, ToRobotBody};
use botsim_model::errors::ErrorType;
use botsim_model::graph::{Graph, NodeIdx};
use botsim_model::job::Job;
use botsim_server::controller::Controller;
use botsim_server::cycle::{Config, FleetRobot, Inbox, Message, Outbox};
use botsim_server::order_builder::Plan;
use botsim_server::policy::{Ask, Fleet, Release, Setup, TrafficPolicy, ZoneAnswer};
use serde_json::{Value, json};
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;

mod common;
use common::graph;
mod translate;

// the nodes of the line A → B → C → D → E
const A: NodeIdx = 0;
const B: NodeIdx = 1;
const C: NodeIdx = 2;
const E: NodeIdx = 4;
const NAMES: [&str; 5] = ["A", "B", "C", "D", "E"];

fn line() -> Graph {
    graph(
        &[
            ("A", 0.0, 0.0),
            ("B", 10.0, 0.0),
            ("C", 20.0, 0.0),
            ("D", 30.0, 0.0),
            ("E", 40.0, 0.0),
        ],
        &[("A", "B"), ("B", "C"), ("C", "D"), ("D", "E")],
    )
}

fn config() -> Config {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    Config {
        graph: line(),
        robots: vec![FleetRobot::from_factsheet(&f).unwrap()],
        zones: None,
        zone_set: None,
        lifts: Vec::new(),
        doors: Vec::new(),
        start: Millis(0),
    }
}

/// What a robot says: the order it has, the node it stands on and its errors.
struct Report {
    order: &'static str,
    update: u32,
    /// Plan index of the node, and the node.
    at: (u32, usize),
    errors: Vec<Value>,
    cancel: Option<&'static str>,
}

impl Report {
    fn new(order: &'static str, update: u32, step: u32, node: NodeIdx) -> Report {
        Report {
            order,
            update,
            at: (step, node),
            errors: Vec::new(),
            cancel: None,
        }
    }

    /// The report that says what the robot's state message says.
    fn state(&self) -> FromRobot {
        let (step, node) = self.at;
        let actions: Vec<Value> = self
            .cancel
            .iter()
            .map(|id| json!({"actionId": id, "actionType": "cancelOrder", "actionStatus": "FINISHED"}))
            .collect();
        let config = config();
        let robot = &config.robots[0].id;
        let message = serde_json::from_value(json!({
            "headerId": 0,
            "timestamp": "2026-11-20T10:00:00.000Z",
            "version": "3.0.1",
            "manufacturer": robot.manufacturer,
            "serialNumber": robot.serial_number,
            "orderId": self.order,
            "orderUpdateId": self.update,
            "lastNodeId": NAMES[node],
            "lastNodeSequenceId": 2 * step,
            "nodeStates": [],
            "edgeStates": [],
            "mobileRobotPosition": {"x": 10.0 * node as f64, "y": 0.0, "theta": 0.0,
                "mapId": "M", "localized": true},
            "driving": false,
            "actionStates": [],
            "instantActionStates": actions,
            "powerSupply": {"stateOfCharge": 100.0, "charging": false},
            "operatingMode": "AUTOMATIC",
            "errors": self.errors,
            "safetyState": {"activeEmergencyStop": "NONE", "fieldViolation": false},
        }))
        .unwrap();
        let translator = translate::translator(&config.graph, robot);
        let read = translator.state_from_vda(&message).unwrap();
        assert!(read.issues.is_empty(), "{:?}", read.issues);
        read.message
    }
}

/// An error that names an order, or only an edge.
fn error(error_type: &str, key: &str, value: &str) -> Value {
    json!({"errorType": error_type, "errorLevel": "WARNING",
        "errorReferences": [{"referenceKey": key, "referenceValue": value}]})
}

/// One cycle at `tick` with the robot's report.
fn cycle<P: TrafficPolicy>(
    c: &mut Controller<P>,
    tick: u64,
    report: &Report,
    jobs: impl IntoIterator<Item = Job>,
) -> Outbox {
    let inbox = Inbox {
        robots: vec![report.state()],
        jobs: jobs.into_iter().collect(),
        watched: true,
        ..Inbox::default()
    };
    c.step(Millis(50 * tick as i64), inbox)
}

/// The orders sent, as VDA 5050 `order` messages.
fn orders(out: &Outbox) -> Vec<Order> {
    let config = config();
    out.messages
        .iter()
        .filter_map(|m| match m {
            Message::Robot(m) if matches!(m.body, ToRobotBody::Order(_)) => Some(m),
            _ => None,
        })
        .map(|m| translate::vda_order(&config.graph, &config.robots[0].id, m))
        .collect()
}

/// The ids of the instant actions sent.
fn actions(out: &Outbox) -> Vec<&str> {
    out.messages
        .iter()
        .filter_map(|m| match m {
            Message::Robot(m) => match &m.body {
                ToRobotBody::InstantActions(actions) => Some(actions),
                _ => None,
            },
            _ => None,
        })
        .flatten()
        .map(|a| a.id.as_str())
        .collect()
}

fn released(order: &Order) -> Vec<&str> {
    order
        .nodes
        .iter()
        .filter(|n| n.released)
        .map(|n| n.node_id.as_str())
        .collect()
}

fn job(id: u64, pickup: NodeIdx, drop: NodeIdx) -> Job {
    Job {
        id,
        pickup,
        drop,
        created: Millis(0),
    }
}

#[test]
fn the_controller_runs_a_job_from_states_written_by_hand() {
    let mut c = botsim_baseline::controller(&config());
    let out = cycle(&mut c, 0, &Report::new("", 0, 0, A), [job(1, A, C)]);
    let sent = orders(&out);
    assert_eq!(sent.len(), 1);
    assert_eq!(
        (sent[0].order_id.as_str(), sent[0].order_update_id),
        ("job-1", 0)
    );
    assert_eq!(sent[0].timestamp, "1970-01-01T00:00:00.000Z");
    assert_eq!(released(&sent[0]), ["A", "B", "C"]);
    assert_eq!(out.held_back, vec![None]);
    assert_eq!(out.reservations.as_ref().map(Vec::len), Some(1));
    // the robot has not answered: nothing to say
    let out = cycle(&mut c, 1, &Report::new("", 0, 0, A), []);
    assert!(out.messages.is_empty());
    assert_eq!(c.busy(), 1);
    // it arrives
    let out = cycle(&mut c, 2, &Report::new("job-1", 0, 2, C), []);
    assert!(out.messages.is_empty());
    assert_eq!(c.completed().len(), 1);
    assert_eq!(c.completed()[0].finished, Millis(100));
    assert_eq!(c.busy(), 0);
}

#[test]
fn a_robot_without_any_report_gets_nothing() {
    let mut c = botsim_baseline::controller(&config());
    let inbox = Inbox {
        jobs: vec![job(1, A, C)],
        watched: true,
        ..Inbox::default()
    };
    let out = c.step(Millis(0), inbox);
    assert!(out.messages.is_empty());
    assert_eq!(out.held_back, vec![None]);
    assert_eq!(out.reservations, Some(Vec::new()));
    assert_eq!(c.pending().count(), 1);
    // a time that goes back is no crash
    let report = Report::new("", 0, 0, A);
    cycle(&mut c, 5, &report, []);
    cycle(&mut c, 1, &report, []);
    assert_eq!(c.busy(), 1);
}

/// The baseline's rules, noting what the controller says about routes that change
/// or are given up.
#[derive(Debug)]
struct Noting {
    rules: BaselinePolicy,
    calls: Vec<RouteCall>,
}

#[derive(Debug, PartialEq)]
enum RouteCall {
    /// The route now ends at this plan index.
    Changed(usize),
    /// Given up: the plan index of the route's end and of the last released node.
    Dropped(usize, usize),
}

impl TrafficPolicy for Noting {
    fn start(&mut self, setup: &Setup) {
        self.rules.start(setup);
    }
    fn cycle_start(&mut self, fleet: &Fleet) {
        self.rules.cycle_start(fleet);
    }
    fn place(&mut self, ask: &Ask) -> Release {
        self.rules.place(ask)
    }
    fn release(&mut self, ask: &Ask) -> Release {
        self.rules.release(ask)
    }
    fn route_changed(&mut self, robot: usize, route: &Plan) {
        self.calls.push(RouteCall::Changed(route.last()));
        self.rules.route_changed(robot, route);
    }
    fn route_dropped(&mut self, robot: usize, route: &Plan, released_through: usize) {
        self.calls
            .push(RouteCall::Dropped(route.last(), released_through));
        self.rules.route_dropped(robot, route, released_through);
    }
    fn idle(&mut self, robot: usize, node: NodeIdx) {
        self.rules.idle(robot, node);
    }
    fn zone_answers(&mut self, fleet: &Fleet) -> Vec<ZoneAnswer> {
        self.rules.zone_answers(fleet)
    }
}

/// A controller of the site with [`Noting`] rules.
fn noting(dispatcher: Box<dyn Dispatcher>) -> Controller<Noting> {
    let config = config();
    let policy = Noting {
        rules: BaselinePolicy::new(&config.graph),
        calls: Vec::new(),
    };
    Controller::with_dispatcher(&config, policy, dispatcher)
}

#[test]
fn a_refused_update_takes_its_release_back() {
    let mut c = noting(Box::new(BaselineDispatcher::new()));
    cycle(&mut c, 0, &Report::new("", 0, 0, A), [job(1, A, E)]);
    let out = cycle(&mut c, 1, &Report::new("job-1", 0, 1, B), []);
    assert_eq!(orders(&out)[0].order_update_id, 1);
    assert_eq!(released(&orders(&out)[0]), ["C", "D"]);
    // not shown yet, and no error: nothing is taken back
    cycle(&mut c, 2, &Report::new("job-1", 0, 1, B), []);
    let mut refused = Report::new("job-1", 0, 1, B);
    refused.errors = vec![error("VALIDATION_FAILURE", "orderId", "job-1")];
    let out = cycle(&mut c, 3, &refused, []);
    assert_eq!(actions(&out), ["cancel-job-1"]);
    assert!(orders(&out).is_empty());
    // the robot stands, but its state does not list the cancel yet
    cycle(&mut c, 4, &refused, []);
    assert_eq!((c.busy(), c.rejected().len()), (1, 0));
    refused.cancel = Some("cancel-job-1");
    cycle(&mut c, 5, &refused, []);
    assert_eq!((c.busy(), c.rejected().len()), (0, 1));
    assert_eq!(c.rejected()[0].reason, ErrorType::ValidationFailure);
    // released through C, as before the refused update, not D
    assert_eq!(c.policy().calls, [RouteCall::Dropped(4, 2)]);
}

#[test]
fn an_accepted_update_makes_a_later_orientation_error_new() {
    // while two updates are on their way the robot reports an orientation error
    // for the last edge; accepting the first of them clears its errors, so the
    // error it reports then is a new one, from the second
    let mut c = botsim_baseline::controller(&config());
    cycle(&mut c, 0, &Report::new("", 0, 0, A), [job(1, A, E)]);
    let orientation = || vec![error("ORIENTATION_UNREACHABLE", "edgeId", "D-E")];
    let report = |update, step, node| {
        let mut r = Report::new("job-1", update, step, node);
        r.errors = orientation();
        r
    };
    let out = cycle(&mut c, 1, &report(0, 1, B), []);
    assert_eq!(orders(&out)[0].order_update_id, 1);
    let out = cycle(&mut c, 2, &report(0, 2, C), []);
    assert_eq!(orders(&out)[0].order_update_id, 2);
    assert!(actions(&out).is_empty(), "the error was there before");
    let out = cycle(&mut c, 3, &report(1, 2, C), []);
    assert_eq!(actions(&out), ["cancel-job-1"]);
    let mut stopped = report(1, 2, C);
    stopped.cancel = Some("cancel-job-1");
    cycle(&mut c, 4, &stopped, []);
    assert_eq!(c.rejected()[0].reason, ErrorType::OrientationUnreachable);
}

/// Sends the robot to park at C and then adds a job to the trip.
#[derive(Debug)]
struct ParkThenJob {
    given_up: Rc<RefCell<Vec<Purpose>>>,
}

impl Dispatcher for ParkThenJob {
    fn start(&mut self, _graph: &Graph, _robots: usize) {}
    fn set_charging(&mut self, _policy: ChargePolicy) -> usize {
        0
    }
    fn add_job(&mut self, _job: Job) {}
    fn pending(&self) -> Box<dyn Iterator<Item = &Job> + '_> {
        Box::new(std::iter::empty())
    }
    fn stop_charging(&mut self, _robot: usize, _charge: f64) -> bool {
        false
    }
    fn arrived(&mut self, _robot: usize, _purpose: Purpose) {}
    fn given_up(&mut self, _robot: usize, purpose: Purpose) {
        self.given_up.borrow_mut().push(purpose);
    }
    fn counts(&self) -> Counts {
        Counts::default()
    }
    fn dispatch(&mut self, site: &Site, robots: &[Robot], cycle: Cycle, c: &mut dyn Control) {
        let edges = |from: usize, to: usize| (from..to).collect::<Vec<_>>();
        assert_eq!(site.graph.edges.len(), 4);
        match (cycle.now.0 / 50, &robots[0].work) {
            (0, Work::None) => {
                let task = Task {
                    job: job(0, A, C),
                    purpose: Purpose::Park,
                    from: A,
                    edges: edges(0, 2),
                    pickup_at: 0,
                };
                assert_eq!(c.give(0, task), Answer::Taken);
            }
            (1, Work::Parking { .. }) => {
                let add = |pickup, drop, pickup_at| Task {
                    job: job(7, pickup, drop),
                    purpose: Purpose::Job,
                    from: C,
                    edges: edges(2, 4),
                    pickup_at,
                };
                // the pickup where the update starts would be lost; the job
                // must fit the route
                assert_eq!(c.extend(0, add(C, E, 0)), Answer::Invalid);
                assert_eq!(c.extend(0, add(B, E, 1)), Answer::Invalid);
                assert_eq!(c.extend(0, add(E - 1, C, 1)), Answer::Invalid);
                assert_eq!(c.extend(0, add(E - 1, E, 1)), Answer::Taken);
            }
            _ => {}
        }
    }
}

#[test]
fn a_refused_addition_to_a_trip_gives_the_trip_its_old_route_back() {
    let given_up = Rc::new(RefCell::new(Vec::new()));
    let dispatcher = ParkThenJob {
        given_up: given_up.clone(),
    };
    let mut c = noting(Box::new(dispatcher));
    cycle(&mut c, 0, &Report::new("", 0, 0, A), []);
    let out = cycle(&mut c, 1, &Report::new("park-0", 0, 0, A), []);
    assert_eq!(
        orders(&out).last().unwrap().nodes.last().unwrap().node_id,
        "E"
    );
    let mut refused = Report::new("park-0", 0, 2, C);
    refused.errors = vec![error("NO_ROUTE_TO_TARGET", "orderId", "park-0")];
    let out = cycle(&mut c, 2, &refused, []);
    assert_eq!(actions(&out), ["cancel-park-0"]);
    // standing at C, the end of the old route, it is not taken to have arrived
    cycle(&mut c, 3, &refused, []);
    assert!(given_up.borrow().is_empty());
    assert_eq!(out.held_back, vec![None]);
    refused.cancel = Some("cancel-park-0");
    cycle(&mut c, 4, &refused, []);
    assert_eq!(*given_up.borrow(), [Purpose::Job]);
    assert_eq!(c.rejected()[0].job.id, 7);
    assert!(c.completed().is_empty());
    // the job's route to E, then the trip's own route to C again
    assert_eq!(
        c.policy().calls,
        [
            RouteCall::Changed(4),
            RouteCall::Changed(2),
            RouteCall::Dropped(2, 2)
        ]
    );
}

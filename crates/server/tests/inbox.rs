//! The cycle's inbox and time, on hand-written data: which report counts, the
//! start before every robot has reported, a robot that stops reporting, as the
//! rules and the dispatcher are told of it, and times that stand, jump or go back.

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;

use botsim_dispatcher::{
    Answer, ChargePolicy, Control, Counts, Cycle, Dispatcher, Purpose, Robot, Site, Task, Work,
};
use botsim_link::ids::ZoneSetId;
use botsim_link::message::{FromRobot, Millis, ToRobotBody};
use botsim_link::zone_set::ZoneSet;
use botsim_model::graph::{Graph, NodeIdx};
use botsim_model::job::Job;
use botsim_model::robot::RobotNo;
use botsim_model::zones::Zones;
use botsim_server::controller::Controller;
use botsim_server::cycle::{Inbox, Message, Outbox};
use botsim_server::policy::{Ask, Fleet, Release, Setup, TrafficPolicy, ZoneAnswer};
use serde_json::json;

mod common;
use common::graph;
#[allow(dead_code)] // (not every part of the helper is used here)
mod hand;
use hand::{config, job, orders, report};

/// A line A → B → C → D → E and, away from it, F → G.
fn site() -> (Graph, [NodeIdx; 7]) {
    let g = graph(
        &[
            ("A", 0.0, 0.0),
            ("B", 10.0, 0.0),
            ("C", 20.0, 0.0),
            ("D", 30.0, 0.0),
            ("E", 40.0, 0.0),
            ("F", 0.0, 50.0),
            ("G", 10.0, 50.0),
        ],
        &[("A", "B"), ("B", "C"), ("C", "D"), ("D", "E"), ("F", "G")],
    );
    let nodes = ["A", "B", "C", "D", "E", "F", "G"].map(|id| g.node_idx(id).unwrap());
    (g, nodes)
}

/// `report` as made at `ms` with the sequence number `seq`.
fn made(mut report: FromRobot, ms: i64, seq: u32) -> FromRobot {
    report.header.time = Millis(ms);
    report.header.seq = seq;
    report
}

/// A robot without an order, standing on `node`.
fn idle(g: &Graph, robot: usize, node: NodeIdx) -> FromRobot {
    report(g, robot, "", &[node], 0, false)
}

/// What the rules were told in one question.
#[derive(Debug, Clone, PartialEq)]
struct Asked {
    robot: usize,
    elapsed: f64,
}

#[derive(Debug, Default)]
struct Told {
    /// For every cycle: which robots had a new report.
    new: Vec<Vec<bool>>,
    placed: Vec<usize>,
    released: Vec<Asked>,
    idle: Vec<(usize, NodeIdx)>,
    zone_calls: usize,
}

/// Rules that let every robot go as far as its window and note what they are told.
#[derive(Debug, Default)]
struct Open(Rc<RefCell<Told>>);

impl TrafficPolicy for Open {
    fn start(&mut self, _setup: &Setup) {}

    fn cycle_start(&mut self, fleet: &Fleet) {
        let new = fleet.robots.iter().map(|k| k.new).collect();
        self.0.borrow_mut().new.push(new);
    }

    fn place(&mut self, ask: &Ask) -> Release {
        self.0.borrow_mut().placed.push(ask.robot);
        Release {
            through: ask.limit,
            reroute: None,
        }
    }

    fn release(&mut self, ask: &Ask) -> Release {
        self.0.borrow_mut().released.push(Asked {
            robot: ask.robot,
            elapsed: ask.elapsed,
        });
        Release {
            through: ask.limit,
            reroute: None,
        }
    }

    fn idle(&mut self, robot: usize, node: NodeIdx) {
        self.0.borrow_mut().idle.push((robot, node));
    }

    fn zone_answers(&mut self, _fleet: &Fleet) -> Vec<ZoneAnswer> {
        self.0.borrow_mut().zone_calls += 1;
        Vec::new()
    }
}

fn controller(g: &Graph) -> (Controller<Open>, Rc<RefCell<Told>>) {
    let told = Rc::new(RefCell::new(Told::default()));
    let c = Controller::with_policy(&config(g.clone()), Open(told.clone()));
    (c, told)
}

fn step<P: TrafficPolicy>(
    c: &mut Controller<P>,
    ms: i64,
    robots: Vec<FromRobot>,
    jobs: Vec<Job>,
) -> Outbox {
    let inbox = Inbox {
        robots,
        jobs,
        ..Inbox::default()
    };
    c.step(Millis(ms), inbox)
}

#[test]
fn nothing_goes_to_anyone_until_every_robot_has_reported() {
    let (g, [a, _, c, _, _, f, gg]) = site();
    let (mut ctl, told) = controller(&g);
    // only the first robot reports, in several cycles, with jobs for both
    let jobs = vec![job(1, a, c), job(2, f, gg)];
    let out = step(&mut ctl, 0, vec![idle(&g, 0, a)], jobs);
    assert!(out.messages.is_empty());
    assert_eq!(out.held_back, vec![None, None]);
    assert_eq!(out.reservations, None, "nobody watches");
    for ms in [50, 100, 60_000] {
        let inbox = Inbox {
            robots: vec![made(idle(&g, 0, a), ms, 0)],
            watched: true,
            ..Inbox::default()
        };
        let out = ctl.step(Millis(ms), inbox);
        assert!(out.messages.is_empty());
        assert_eq!(out.held_back, vec![None, None]);
        // (the rules have been told of nobody: what they would say of themselves)
        assert_eq!(out.reservations, Some(Vec::new()));
    }
    assert_eq!((ctl.pending().count(), ctl.busy()), (2, 0));
    {
        let told = told.borrow();
        assert!(told.new.is_empty() && told.placed.is_empty() && told.idle.is_empty());
        assert_eq!(told.zone_calls, 0);
    }

    // the second robot reports, the first does not in this cycle: the cycle runs,
    // and only the second gets something
    let out = step(&mut ctl, 60_050, vec![idle(&g, 1, f)], vec![]);
    assert_eq!(orders(&out), [(1, 0, vec![f, gg])]);
    assert_eq!(ctl.pending().count(), 1);
    assert_eq!(told.borrow().new, [[false, true]]);
    // (the rules hear where the first stands, by the report kept)
    assert_eq!(told.borrow().idle, [(0, a), (1, f)]);

    // the first again: its job
    let out = step(
        &mut ctl,
        60_100,
        vec![made(idle(&g, 0, a), 60_100, 0)],
        vec![],
    );
    assert_eq!(orders(&out), [(0, 0, vec![a, g.edges[0].end, c])]);
}

#[test]
fn the_zone_set_waits_for_every_robot_too_and_goes_only_to_those_that_report() {
    let (g, [a, .., f, _]) = site();
    let set = serde_json::from_value(json!({"mapId": "M", "zoneSetId": "zs",
        "zones": [{"zoneId": "z", "zoneType": "BLOCKED", "vertices": [
            {"x": 90.0, "y": 5.0}, {"x": 92.0, "y": 5.0}, {"x": 92.0, "y": 7.0}]}]}))
    .unwrap();
    let mut config = config(g.clone());
    config.zones = Some(Zones::new(&g, set).unwrap());
    config.zone_set = Some(Arc::new(ZoneSet {
        id: ZoneSetId::new("zs"),
        map: 0,
        descriptor: None,
        zones: Vec::new(),
    }));
    let mut ctl = Controller::with_policy(&config, Open::default());
    let sent = |out: &Outbox| -> Vec<u32> {
        out.messages
            .iter()
            .map(|m| match m {
                Message::Robot(m) if matches!(m.body, ToRobotBody::ZoneSet(_)) => {
                    m.header.robot.slot()
                }
                other => panic!("a zone set expected: {other:?}"),
            })
            .collect()
    };
    assert!(sent(&step(&mut ctl, 0, vec![idle(&g, 1, f)], vec![])).is_empty());
    let out = step(&mut ctl, 50, vec![idle(&g, 0, a)], vec![]);
    assert_eq!(sent(&out), [0]);
    let out = step(&mut ctl, 100, vec![made(idle(&g, 1, f), 100, 0)], vec![]);
    assert_eq!(sent(&out), [1]);
}

/// A controller whose first robot has job 1 from A to E and is released to C; the
/// second stands on F.
fn on_its_way() -> (Graph, [NodeIdx; 7], Controller<Open>, Rc<RefCell<Told>>) {
    let (g, nodes) = site();
    let [a, b, c, _, e, f, _] = nodes;
    let (mut ctl, told) = controller(&g);
    let reports = vec![idle(&g, 0, a), idle(&g, 1, f)];
    let out = step(&mut ctl, 0, reports, vec![job(1, a, e)]);
    assert_eq!(orders(&out), [(0, 0, vec![a, b, c])]);
    (g, nodes, ctl, told)
}

/// The first robot on job 1, at step `at` of its route A … E, standing.
fn at_step(g: &Graph, nodes: &[NodeIdx; 7], at: usize) -> FromRobot {
    report(g, 0, "job-1", &nodes[..5], at, false)
}

#[test]
fn of_several_reports_of_a_robot_the_newest_counts_in_any_order() {
    let all_orders: [[usize; 3]; 6] = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    for order in all_orders {
        let (g, nodes, mut ctl, _) = on_its_way();
        let [_, _, c, d, e, ..] = nodes;
        // at step 0, 1 and 2, made at 10, 20 and 30 ms
        let reports = order
            .iter()
            .map(|&at| made(at_step(&g, &nodes, at), 10 * (at as i64 + 1), 0))
            .collect();
        let out = step(&mut ctl, 50, reports, vec![]);
        assert_eq!(orders(&out), [(0, 1, vec![c, d, e])], "order {order:?}");
    }
}

#[test]
fn of_equal_times_the_sequence_number_and_then_the_place_in_the_list_say_the_newest() {
    // the higher sequence number, wherever it is
    for swap in [false, true] {
        let (g, nodes, mut ctl, _) = on_its_way();
        let [_, _, c, d, e, ..] = nodes;
        let mut reports = vec![
            made(at_step(&g, &nodes, 1), 30, 4),
            made(at_step(&g, &nodes, 2), 30, 5),
        ];
        if swap {
            reports.reverse();
        }
        let out = step(&mut ctl, 50, reports, vec![]);
        assert_eq!(orders(&out), [(0, 1, vec![c, d, e])], "swap {swap}");
    }
    // of the very same header the later one
    for (first, second, released) in [(1, 2, 4), (2, 1, 3)] {
        let (g, nodes, mut ctl, _) = on_its_way();
        let reports = vec![
            made(at_step(&g, &nodes, first), 30, 5),
            made(at_step(&g, &nodes, second), 30, 5),
        ];
        let out = step(&mut ctl, 50, reports, vec![]);
        assert_eq!(orders(&out), [(0, 1, nodes[2..=released].to_vec())]);
    }
}

#[test]
fn an_older_report_is_not_looked_at_and_the_same_one_again_counts() {
    let (g, nodes, mut ctl, told) = on_its_way();
    step(
        &mut ctl,
        50,
        vec![made(at_step(&g, &nodes, 2), 30, 5)],
        vec![],
    );
    // older by time, and by sequence number: the arrival they tell of is not seen
    for (ms, seq) in [(29, 9), (30, 4), (i64::MIN, u32::MAX)] {
        let old = made(at_step(&g, &nodes, 4), ms, seq);
        let out = step(&mut ctl, 100, vec![old], vec![]);
        assert!(out.messages.is_empty());
        assert_eq!(ctl.completed().len(), 0);
        assert_eq!(told.borrow().new.last().unwrap(), &[false, false]);
    }
    // the same header as the one kept: it has arrived
    let again = made(at_step(&g, &nodes, 4), 30, 5);
    step(&mut ctl, 150, vec![again], vec![]);
    assert_eq!(ctl.completed().len(), 1);
    assert_eq!(told.borrow().new.last().unwrap(), &[true, false]);
}

#[test]
fn reports_of_robots_the_fleet_does_not_have_are_not_looked_at() {
    let (g, [a, .., f, _]) = site();
    let (mut ctl, told) = controller(&g);
    let stranger = |no: RobotNo| {
        let mut r = idle(&g, 0, a);
        r.header.robot = no;
        r
    };
    // beyond the fleet, and a later generation of a robot's slot: none of them is
    // the second robot's report
    let strangers = vec![
        stranger(RobotNo::new(7, 0)),
        stranger(RobotNo::new(u32::MAX, u32::MAX)),
        stranger(RobotNo::new(1, 1)),
    ];
    let mut reports = vec![idle(&g, 0, a)];
    reports.extend(strangers.clone());
    let out = step(&mut ctl, 0, reports, vec![job(1, a, g.edges[0].end)]);
    assert!(out.messages.is_empty());
    assert!(told.borrow().new.is_empty(), "still waiting for the second");
    let mut reports = strangers;
    reports.push(idle(&g, 1, f));
    reports.push(made(idle(&g, 0, a), 50, 0));
    let out = step(&mut ctl, 50, reports, vec![]);
    assert_eq!(orders(&out).len(), 1);
    assert_eq!(out.held_back.len(), 2);
}

#[test]
fn a_silent_robot_gets_nothing_new_and_counts_the_whole_time_when_it_reports_again() {
    let (g, nodes, mut ctl, told) = on_its_way();
    let [.., f, _] = nodes;
    let other = |ms| made(idle(&g, 1, f), ms, 0);
    step(
        &mut ctl,
        1_000,
        vec![made(at_step(&g, &nodes, 0), 1_000, 0)],
        vec![],
    );
    let asked = told.borrow().released.len();
    // the first is silent for a minute, in which the second reports
    for ms in [2_000, 30_000, 61_000] {
        let out = step(&mut ctl, ms, vec![other(ms)], vec![]);
        assert!(out.messages.is_empty(), "{:?}", out.messages);
        assert_eq!(told.borrow().released.len(), asked, "no release asked");
    }
    // it reports again, standing at A: the rules and the waiting count the minute
    step(
        &mut ctl,
        61_500,
        vec![made(at_step(&g, &nodes, 0), 61_500, 0)],
        vec![],
    );
    let last = told.borrow().released.last().unwrap().clone();
    assert_eq!(
        last,
        Asked {
            robot: 0,
            elapsed: 60.5
        }
    );
    step(
        &mut ctl,
        62_000,
        vec![made(at_step(&g, &nodes, 4), 62_000, 0)],
        vec![],
    );
    // (0 to 1 s, then 1 s to 61.5 s)
    assert_eq!(ctl.completed()[0].waiting_seconds, 61.5);
}

#[test]
fn a_time_that_stands_or_goes_back_counts_nothing() {
    let (g, nodes, mut ctl, told) = on_its_way();
    let waits = |ms: i64, seq| made(at_step(&g, &nodes, 0), ms, seq);
    // (the report's own time goes on: it is the cycle's that does not)
    step(&mut ctl, 1_000, vec![waits(1, 0)], vec![]);
    step(&mut ctl, 1_000, vec![waits(2, 0)], vec![]);
    step(&mut ctl, 500, vec![waits(3, 0)], vec![]);
    step(&mut ctl, -70_000, vec![waits(4, 0)], vec![]);
    step(&mut ctl, 2_000, vec![waits(5, 0)], vec![]);
    let elapsed: Vec<f64> = told.borrow().released.iter().map(|a| a.elapsed).collect();
    assert_eq!(elapsed, [1.0, 0.0, 0.0, 0.0, 1.0]);
    // done in a cycle whose time is before the one that gave the job
    step(
        &mut ctl,
        -5,
        vec![made(at_step(&g, &nodes, 4), 6, 0)],
        vec![],
    );
    let done = &ctl.completed()[0];
    assert_eq!(done.waiting_seconds, 2.0);
    assert_eq!((done.assigned, done.finished), (Millis(0), Millis(2_000)));
}

#[test]
fn times_that_jump_to_the_ends_of_their_range_are_no_crash() {
    let (g, nodes) = site();
    let [a, _, _, _, e, f, _] = nodes;
    let (mut ctl, _) = controller(&g);
    ctl.set_wait_weight(1.0);
    let reports = vec![
        made(idle(&g, 0, a), i64::MIN, 0),
        made(idle(&g, 1, f), i64::MIN, u32::MAX),
    ];
    step(&mut ctl, i64::MIN, reports, vec![job(1, a, e)]);
    for (n, ms) in [i64::MAX, i64::MIN, 0, i64::MAX, -1]
        .into_iter()
        .enumerate()
    {
        let r = made(at_step(&g, &nodes, 0), i64::MAX, n as u32);
        step(&mut ctl, ms, vec![r], vec![]);
    }
    let last = made(at_step(&g, &nodes, 4), i64::MAX, u32::MAX);
    step(&mut ctl, i64::MIN, vec![last], vec![]);
    let done = &ctl.completed()[0];
    assert!(done.finished >= done.assigned);
    assert!(done.waiting_seconds.is_finite() && done.waiting_seconds >= 0.0);
}

#[derive(Debug, Default)]
struct Seen {
    robots: Vec<Vec<Robot>>,
    cycles: Vec<Cycle>,
    answers: Vec<Answer>,
    asked_to_stop: Vec<usize>,
    jobs: usize,
}

/// A dispatcher that notes what it is shown and tries to move and charge the
/// second robot in every dispatch.
#[derive(Debug, Default)]
struct Pushing(Rc<RefCell<Seen>>);

impl Dispatcher for Pushing {
    fn start(&mut self, _graph: &Graph, _robots: usize) {}
    fn set_charging(&mut self, _policy: ChargePolicy) -> usize {
        0
    }
    fn add_job(&mut self, _job: Job) {
        self.0.borrow_mut().jobs += 1;
    }
    fn pending(&self) -> Box<dyn Iterator<Item = &Job> + '_> {
        Box::new(std::iter::empty())
    }
    fn stop_charging(&mut self, robot: usize, _charge: f64) -> bool {
        self.0.borrow_mut().asked_to_stop.push(robot);
        true
    }
    fn arrived(&mut self, _robot: usize, _purpose: Purpose) {}
    fn given_up(&mut self, _robot: usize, _purpose: Purpose) {}
    fn counts(&self) -> Counts {
        Counts::default()
    }

    fn dispatch(&mut self, site: &Site, robots: &[Robot], cycle: Cycle, c: &mut dyn Control) {
        let node = |n: &str| site.graph.node_idx(n).unwrap();
        c.charge_here(1, 0);
        let task = Task {
            job: Job {
                id: 9,
                pickup: node("F"),
                drop: node("G"),
                created: cycle.now,
            },
            purpose: Purpose::Job,
            from: node("F"),
            edges: vec![site.graph.edge_idx("F-G").unwrap()],
            pickup_at: 0,
        };
        let answer = c.give(1, task);
        let mut seen = self.0.borrow_mut();
        seen.answers.push(answer);
        seen.robots.push(robots.to_vec());
        seen.cycles.push(cycle);
    }
}

#[test]
fn the_dispatcher_sees_a_silent_robot_as_not_ready_and_can_give_it_nothing() {
    let (g, [a, .., f, _]) = site();
    let seen = Rc::new(RefCell::new(Seen::default()));
    let dispatcher = Box::new(Pushing(seen.clone()));
    let mut ctl = Controller::with_dispatcher(&config(g.clone()), Open::default(), dispatcher);
    // the jobs reach the dispatcher before every robot has reported, a dispatch
    // does not take place
    step(&mut ctl, 0, vec![idle(&g, 1, f)], vec![job(1, a, f)]);
    assert_eq!(seen.borrow().jobs, 1);
    assert!(seen.borrow().cycles.is_empty() && seen.borrow().asked_to_stop.is_empty());

    // the second is silent in the first cycle that runs
    let out = step(&mut ctl, 1_000, vec![idle(&g, 0, a)], vec![]);
    {
        let seen = seen.borrow();
        assert_eq!(seen.answers, [Answer::Invalid]);
        assert_eq!(seen.asked_to_stop, [0]);
        let [first, second] = &seen.robots[0][..] else {
            panic!("two robots: {:?}", seen.robots);
        };
        assert!(first.ready && !second.ready);
        assert_eq!((first.elapsed, second.elapsed), (1.0, 0.0));
        assert_eq!((second.node, &second.work), (Some(f), &Work::None));
        assert_eq!(seen.cycles[0].now, Millis(1_000));
    }
    // (only the first is told to stop charging; nothing goes to the second)
    let to: Vec<u32> = out
        .messages
        .iter()
        .map(|m| match m {
            Message::Robot(m) => m.header.robot.slot(),
            other => panic!("to a robot: {other:?}"),
        })
        .collect();
    assert_eq!(to, [0]);

    // it reports: charged on the spot, then given the task, counting its own time
    step(
        &mut ctl,
        3_500,
        vec![made(idle(&g, 1, f), 3_500, 0)],
        vec![],
    );
    assert_eq!(seen.borrow().answers.last(), Some(&Answer::Taken));
    assert_eq!(seen.borrow().robots[1][1].elapsed, 3.5);
    assert_eq!(seen.borrow().cycles[1].elapsed, 2.5);

    // silent with a task: busy
    step(
        &mut ctl,
        4_000,
        vec![made(idle(&g, 0, a), 4_000, 0)],
        vec![],
    );
    assert_eq!(seen.borrow().robots[2][1].work, Work::Busy);
}

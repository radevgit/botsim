//! A robot that stops reporting under the baseline's rules: what it holds stays
//! held, and it is given nothing. On hand-written data.

use botsim_baseline::zone_grants::{RobotView, ZoneGrants};
use botsim_link::device::DoorCommand;
use botsim_link::ids::{RequestId, ZoneId, ZoneSetId};
use botsim_link::message::{FromRobot, Millis};
use botsim_link::response::Grant::{Granted, Queued};
use botsim_link::state::{RequestStatus, ZoneRequest, ZoneRequestType};
use botsim_model::graph::{Graph, NodeIdx};
use botsim_model::job::Job;
use botsim_model::waiting::{HeldBack, WaitReason};
use botsim_model::zones::Zones;
use botsim_server::controller::Controller;
use botsim_server::cycle::{Inbox, Message, Outbox};
use botsim_server::doors::Door;
use botsim_server::policy::{TrafficPolicy, ZoneAnswer};
use serde_json::json;

mod common;
use common::graph;
#[allow(dead_code)] // (not every part of the helper is used here)
#[path = "../../server/tests/hand/mod.rs"]
mod hand;
use hand::{config, job, orders, report};

/// A → M → B and C → M → D cross at M.
fn crossing() -> (Graph, [NodeIdx; 5]) {
    let g = graph(
        &[
            ("A", -10.0, 0.0),
            ("M", 0.0, 0.0),
            ("B", 10.0, 0.0),
            ("C", 0.0, -10.0),
            ("D", 0.0, 10.0),
        ],
        &[("A", "M"), ("M", "B"), ("C", "M"), ("M", "D")],
    );
    let nodes = ["A", "M", "B", "C", "D"].map(|id| g.node_idx(id).unwrap());
    (g, nodes)
}

fn at(mut report: FromRobot, ms: i64) -> FromRobot {
    report.header.time = Millis(ms);
    report
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
        watched: true,
        ..Inbox::default()
    };
    c.step(Millis(ms), inbox)
}

fn waits_for(robot: usize) -> Option<HeldBack> {
    Some(HeldBack {
        reason: WaitReason::Release,
        blockers: vec![robot],
    })
}

#[test]
fn a_silent_robot_keeps_its_nodes_while_the_other_waits_for_them() {
    let (g, [a, m, b, c, d]) = crossing();
    let mut ctl = botsim_baseline::controller(&config(g.clone()));
    let idle = vec![
        report(&g, 0, "", &[a], 0, false),
        report(&g, 1, "", &[c], 0, false),
    ];
    let out = step(&mut ctl, 0, idle, vec![job(1, a, b), job(2, c, d)]);
    assert_eq!(orders(&out), [(0, 0, vec![a, m, b]), (1, 0, vec![c])]);
    let held = out.reservations.unwrap();

    // the first falls silent; the second waits, however long
    let second = |ms| at(report(&g, 1, "job-2", &[c, m, d], 0, false), ms);
    for ms in [50, 1_000, 20_000, 40_000, 3_600_000] {
        let out = step(&mut ctl, ms, vec![second(ms)], vec![]);
        assert!(out.messages.is_empty(), "{ms}: {:?}", out.messages);
        assert_eq!(out.held_back, vec![None, waits_for(0)]);
        assert_eq!(out.reservations.unwrap(), held, "nothing is given back");
    }
    assert_eq!(ctl.reroutes(), 0);

    // it reports from its goal: it gives back the way, and the second goes
    let ms = 3_600_050;
    let done = at(report(&g, 0, "job-1", &[a, m, b], 2, false), ms);
    let out = step(&mut ctl, ms, vec![done, second(ms)], vec![]);
    assert_eq!(orders(&out), [(1, 1, vec![c, m, d])]);
    assert_eq!(ctl.completed().len(), 1);
}

#[test]
fn a_silent_robot_is_not_given_what_has_become_free() {
    let (g, [a, m, b, c, d]) = crossing();
    let mut ctl = botsim_baseline::controller(&config(g.clone()));
    let idle = vec![
        report(&g, 0, "", &[a], 0, false),
        report(&g, 1, "", &[c], 0, false),
    ];
    step(&mut ctl, 0, idle, vec![job(1, a, b), job(2, c, d)]);
    let second = at(report(&g, 1, "job-2", &[c, m, d], 0, false), 50);
    step(&mut ctl, 50, vec![second], vec![]);

    // the second is silent; the first arrives and gives back the crossing
    let done = at(report(&g, 0, "job-1", &[a, m, b], 2, false), 100);
    let out = step(&mut ctl, 100, vec![done], vec![]);
    assert!(out.messages.is_empty(), "{:?}", out.messages);
    assert_eq!(ctl.completed().len(), 1);
    let held = out.reservations.unwrap();
    let out = step(&mut ctl, 150, vec![], vec![]);
    assert!(out.messages.is_empty());
    assert_eq!(out.reservations.unwrap(), held);

    let second = at(report(&g, 1, "job-2", &[c, m, d], 0, false), 200);
    let out = step(&mut ctl, 200, vec![second], vec![]);
    assert_eq!(orders(&out), [(1, 1, vec![c, m, d])]);
}

#[test]
fn a_silent_robot_keeps_its_door_which_the_other_waits_for() {
    let (g, [a, m, b, c, d]) = crossing();
    let mut config = config(g.clone());
    config.doors = vec![Door {
        id: "door".into(),
        edges: vec![g.edge_idx("A-M").unwrap()],
    }];
    let mut ctl = botsim_baseline::controller(&config);
    let idle = vec![
        report(&g, 0, "", &[a], 0, false),
        report(&g, 1, "", &[c], 0, false),
    ];
    step(&mut ctl, 0, idle, vec![job(1, a, b), job(2, c, d)]);
    let door = |out: &Outbox| -> Vec<DoorCommand> {
        let of = |m: &Message| match m {
            Message::Door(m) => Some(m.command),
            _ => None,
        };
        out.messages.iter().filter_map(of).collect()
    };
    for ms in [50, 10_000, 3_600_000] {
        let second = at(report(&g, 1, "job-2", &[c, m, d], 0, false), ms);
        let out = step(&mut ctl, ms, vec![second], vec![]);
        assert_eq!(door(&out), [DoorCommand::Open]);
        assert_eq!(out.messages.len(), 1, "only the door: {:?}", out.messages);
    }
}

fn request(id: &str) -> ZoneRequest {
    ZoneRequest {
        id: RequestId::new(id),
        kind: ZoneRequestType::Access,
        zone: ZoneId::new("a"),
        zone_set: ZoneSetId::new("zs"),
        status: RequestStatus::Requested,
    }
}

/// A robot far from the zone, with a request for it.
fn asks(id: &str, new: bool) -> RobotView {
    RobotView {
        requests: vec![request(id)],
        position: Some((0, 50.0, 50.0)),
        new,
    }
}

fn answer(robot: usize, id: &str, grant: botsim_link::response::Grant) -> ZoneAnswer {
    ZoneAnswer {
        robot,
        request_id: RequestId::new(id),
        grant,
    }
}

#[test]
fn a_silent_robot_keeps_its_zone_grant_and_is_not_answered() {
    let (g, _) = crossing();
    let set = serde_json::from_value(json!({"mapId": "M", "zoneSetId": "zs",
        "zones": [{"zoneId": "a", "zoneType": "RELEASE", "releaseLossBehavior": "STOP",
            "vertices": [{"x": 90.0, "y": 5.0}, {"x": 92.0, "y": 5.0},
                         {"x": 92.0, "y": 7.0}]}]}))
    .unwrap();
    let mut grants = ZoneGrants::new(Zones::new(&g, set).unwrap());
    // a free zone is not granted to a silent robot, which is not told to wait
    // either
    assert_eq!(grants.decide(&[asks("r0", false)]), []);
    // one behind it gets the zone meanwhile
    let both = [asks("r0", false), asks("r1", true)];
    assert_eq!(grants.decide(&both), [answer(1, "r1", Granted)]);
    // that one falls silent and keeps its grant; the first, back, waits
    let both = [asks("r0", true), asks("r1", false)];
    assert_eq!(grants.decide(&both), [answer(0, "r0", Queued)]);
    assert_eq!(grants.decide(&both), []);
    assert_eq!(grants.granted(), [(1, 0)]);
    // the second has left: the first, silent again, is not granted; then it is
    let left = |new| RobotView {
        requests: Vec::new(),
        position: Some((0, 50.0, 50.0)),
        new,
    };
    assert_eq!(grants.decide(&[asks("r0", false), left(true)]), []);
    assert!(grants.granted().is_empty());
    let first = grants.decide(&[asks("r0", true), left(true)]);
    assert_eq!(first, [answer(0, "r0", Granted)]);
}

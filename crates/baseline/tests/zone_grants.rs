use std::cell::RefCell;
use std::rc::Rc;

use botsim_baseline::controller::Controller;
use botsim_baseline::zone_grants::{Answer, RobotView, ZoneGrants};
use botsim_core::actions::ActionTimes;
use botsim_core::graph::Graph;
use botsim_core::jobs::{Job, JobGenerator};
use botsim_core::observer::{Observer, Snapshot};
use botsim_core::spec::RobotSpec;
use botsim_core::world::{RobotId, World};
use botsim_core::zones::Zones;
use vda5050::factsheet::Factsheet;
use vda5050::responses::GrantType::{Granted, Queued};
use vda5050::state::{RequestStatus, ZoneRequest, ZoneRequestType};
use vda5050::zone_set::ZoneSet;

mod common;
use common::graph;

const DT: f64 = 0.05;

fn ring() -> Graph {
    graph(
        &[
            ("N0", 0.0, 0.0),
            ("N1", 20.0, 0.0),
            ("N2", 20.0, 20.0),
            ("N3", 0.0, 20.0),
        ],
        &[("N0", "N1"), ("N1", "N2"), ("N2", "N3"), ("N3", "N0")],
    )
}

/// Two RELEASE zones on map M: "a" (x 0 … 10) and "b" (x 20 … 30), each 10 × 10.
fn zones(g: &Graph) -> Zones {
    let square = |id: &str, x0: f64, x1: f64| {
        format!(
            r#"{{"zoneId":"{id}","zoneType":"RELEASE","vertices":[{{"x":{x0},"y":-5}},{{"x":{x1},"y":-5}},{{"x":{x1},"y":5}},{{"x":{x0},"y":5}}]}}"#
        )
    };
    let set: ZoneSet = serde_json::from_str(&format!(
        r#"{{"mapId":"M","zoneSetId":"zs","zones":[{},{}]}}"#,
        square("a", 0.0, 10.0),
        square("b", 20.0, 30.0)
    ))
    .unwrap();
    Zones::new(g, set).unwrap()
}

fn request(id: &str, zone: &str) -> ZoneRequest {
    ZoneRequest {
        request_id: id.into(),
        request_type: ZoneRequestType::Access,
        zone_id: zone.into(),
        zone_set_id: "zs".into(),
        request_status: RequestStatus::Requested,
        trajectory: None,
    }
}

/// A robot far from every zone, with the given requests.
fn robot(requests: Vec<ZoneRequest>) -> RobotView {
    RobotView {
        requests,
        position: Some((0, 50.0, 50.0)),
    }
}

fn inside(zone_x: f64, requests: Vec<ZoneRequest>) -> RobotView {
    RobotView {
        requests,
        position: Some((0, zone_x, 0.0)),
    }
}

fn answer(robot: usize, id: &str, grant: vda5050::responses::GrantType) -> Answer {
    Answer {
        robot,
        request_id: id.into(),
        grant,
    }
}

fn grants() -> ZoneGrants {
    ZoneGrants::new(zones(&ring()))
}

// ---- the rules on their own ----

#[test]
fn a_free_zone_is_granted_at_once_and_only_once() {
    let mut g = grants();
    let r = vec![robot(vec![request("req-1", "a")])];
    assert_eq!(g.decide(&r), vec![answer(0, "req-1", Granted)]);
    assert!(
        g.decide(&r).is_empty(),
        "no second answer to the same request"
    );
    assert_eq!(g.granted(), vec![(0, 0)]);
}

#[test]
fn a_second_robot_is_queued_once_and_granted_after_the_first_has_left() {
    let mut g = grants();
    let mut r = vec![
        robot(vec![request("req-1", "a")]),
        robot(vec![request("req-1", "a")]),
    ];
    assert_eq!(
        g.decide(&r),
        vec![answer(0, "req-1", Granted), answer(1, "req-1", Queued)]
    );
    assert!(g.decide(&r).is_empty(), "queued is said once");

    // robot 0 is inside the zone: robot 1 still waits
    r[0] = inside(5.0, vec![request("req-1", "a")]);
    assert!(g.decide(&r).is_empty());

    // robot 0 has left: its request is gone — now robot 1 gets the zone
    r[0] = robot(vec![]);
    assert_eq!(g.decide(&r), vec![answer(1, "req-1", Granted)]);
}

#[test]
fn first_come_first_served_in_the_order_requests_were_first_seen() {
    let mut g = grants();
    // robot 2 asks first (and is granted), then robot 0, then robot 1
    let mut r = vec![
        robot(vec![]),
        robot(vec![]),
        robot(vec![request("req-1", "a")]),
    ];
    assert_eq!(g.decide(&r), vec![answer(2, "req-1", Granted)]);
    r[0] = robot(vec![request("req-1", "a")]);
    assert_eq!(g.decide(&r), vec![answer(0, "req-1", Queued)]);
    r[1] = robot(vec![request("req-1", "a")]);
    assert_eq!(g.decide(&r), vec![answer(1, "req-1", Queued)]);

    // robot 2 leaves: robot 0 was first, it goes first
    r[2] = robot(vec![]);
    assert_eq!(g.decide(&r), vec![answer(0, "req-1", Granted)]);
    r[0] = robot(vec![]);
    assert_eq!(g.decide(&r), vec![answer(1, "req-1", Granted)]);
}

#[test]
fn a_robot_standing_in_the_zone_without_a_grant_blocks_it() {
    let mut g = grants();
    let r = vec![
        inside(5.0, vec![]), // parked inside zone "a"
        robot(vec![request("req-1", "a")]),
    ];
    assert_eq!(g.decide(&r), vec![answer(1, "req-1", Queued)]);
    // it drives out
    let r = vec![robot(vec![]), robot(vec![request("req-1", "a")])];
    assert_eq!(g.decide(&r), vec![answer(1, "req-1", Granted)]);
}

#[test]
fn zones_are_independent_and_a_waiting_request_does_not_hold_back_others() {
    let mut g = grants();
    let r = vec![
        robot(vec![request("req-1", "a")]),
        robot(vec![request("req-1", "a")]), // waits for robot 0
        robot(vec![request("req-1", "b")]), // another zone: free
    ];
    assert_eq!(
        g.decide(&r),
        vec![
            answer(0, "req-1", Granted),
            answer(1, "req-1", Queued),
            answer(2, "req-1", Granted)
        ]
    );
}

#[test]
fn a_robot_may_hold_a_second_zone_next_to_the_first() {
    let mut g = grants();
    let r = vec![robot(vec![request("req-1", "a"), request("req-2", "b")])];
    assert_eq!(
        g.decide(&r),
        vec![answer(0, "req-1", Granted), answer(0, "req-2", Granted)]
    );
}

#[test]
fn other_zone_sets_and_unknown_zones_are_ignored() {
    let mut g = grants();
    let mut other_set = request("req-1", "a");
    other_set.zone_set_id = "elsewhere".into();
    let unknown = request("req-2", "nope");
    let r = vec![robot(vec![other_set, unknown])];
    assert!(g.decide(&r).is_empty());
}

// ---- with the controller and the world ----

#[derive(Debug)]
struct Recorder(Rc<RefCell<Vec<Snapshot>>>);

impl Observer for Recorder {
    fn on_layout(&mut self, _: &Graph) {}

    fn on_step(&mut self, s: &Snapshot) {
        self.0.borrow_mut().push(s.clone());
    }
}

/// Three robots on the ring with the two zones, random jobs; returns the
/// number of jobs done and the snapshots.
fn busy_ring(zones_on: bool) -> (usize, Vec<Snapshot>) {
    let g = ring();
    let z = zones(&g);
    let nodes: Vec<_> = (0..4)
        .map(|i| g.node_idx(&format!("N{i}")).unwrap())
        .collect();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let starts: Vec<_> = ["N0", "N1", "N3"]
        .iter()
        .map(|n| g.node_idx(n).unwrap())
        .collect();
    let mut w = World::new(g, DT, 0).unwrap();
    if zones_on {
        w.set_zones(z);
    }
    for (i, n) in starts.into_iter().enumerate() {
        let id = RobotId::new("ACME", &format!("R-{}", i + 1));
        w.add_robot(
            id,
            spec.clone(),
            ActionTimes::new(1.0).with("pick", 5.0).with("drop", 5.0),
            n,
            0.0,
        )
        .unwrap();
    }
    let log = Rc::new(RefCell::new(Vec::new()));
    // every step: a robot may wait for the zone for only a few steps
    w.set_observer(Box::new(Recorder(log.clone())));
    let mut c = Controller::new(&w);
    let mut gen_ = JobGenerator::new(5, &nodes, 240.0, DT).unwrap();
    for t in 0..24_000u64 {
        let new: Vec<Job> = gen_.step(t).into_iter().collect();
        c.step(&mut w, new);
        w.step();
    }
    let snaps = log.take();
    (c.completed().len(), snaps)
}

#[test]
fn never_two_robots_inside_one_release_zone_and_the_work_gets_done() {
    let (done, snaps) = busy_ring(true);
    assert!(done > 20, "{done} jobs done");

    let g = ring();
    let z = zones(&g);
    let mut times_inside = 0;
    let mut waited_for_zone = 0;
    for s in &snaps {
        for zi in 0..z.zones.len() {
            let n = s
                .robots
                .iter()
                .filter(|r| {
                    r.pose.map == z.map && Zones::contains(&z.zones[zi], r.pose.x, r.pose.y)
                })
                .count();
            assert!(n <= 1, "{n} robots inside zone {zi} at tick {}", s.tick);
            times_inside += n;
        }
        waited_for_zone += s
            .robots
            .iter()
            .filter(|r| r.status == botsim_core::observer::RobotStatus::WaitingForZone)
            .count();
    }
    assert!(times_inside > 100, "the zones were really used");
    eprintln!("{done} jobs, {waited_for_zone} robot-snapshots waiting for a zone");
    assert!(
        waited_for_zone > 0,
        "robots had to wait for the zone at least once"
    );
}

#[test]
fn the_same_run_gives_the_same_result() {
    let (a, sa) = busy_ring(true);
    let (b, sb) = busy_ring(true);
    assert_eq!(a, b);
    assert_eq!(sa, sb);
}

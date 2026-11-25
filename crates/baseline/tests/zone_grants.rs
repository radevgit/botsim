use std::cell::RefCell;
use std::rc::Rc;

use botsim_baseline::controller::Controller;
use botsim_baseline::zone_grants::{Answer, RobotView, ZoneGrants};
use botsim_core::actions::ActionTimes;
use botsim_core::graph::Graph;
use botsim_core::jobs::{Job, JobGenerator};
use botsim_core::observer::{Observer, Snapshot};
use botsim_core::spec::RobotSpec;
use botsim_core::waiting::{Wait, WaitReason};
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
    busy_ring_with(zones_on, false, 0)
}

/// Like [`busy_ring`]; with `by_message` the robots get the zone set from the
/// controller (a `zoneSet` message and `enableZoneSet`, S62) instead of having it
/// from the start; `delay` is the message delay in steps.
fn busy_ring_with(zones_on: bool, by_message: bool, delay: u64) -> (usize, Vec<Snapshot>) {
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
    if zones_on && by_message {
        w.set_site_zones(z);
    } else if zones_on {
        w.set_zones(z);
    }
    w.set_message_delay(delay);
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
fn a_robot_that_waits_for_a_zone_names_the_robot_that_has_it() {
    let mut grants = ZoneGrants::new(zones(&ring()));
    // robot 0 gets zone a, robot 2 stands in it without a grant; robot 1 waits
    let robots = [
        robot(vec![request("r0", "a")]),
        robot(vec![request("r1", "a")]),
        robot(vec![]),
    ];
    grants.decide(&robots);
    assert_eq!(grants.blockers(1), vec![0]);
    assert_eq!(grants.blockers(0), Vec::<usize>::new());
    let robots = [
        robot(vec![request("r0", "a")]),
        robot(vec![request("r1", "a")]),
        inside(5.0, vec![]),
    ];
    grants.decide(&robots);
    assert_eq!(grants.blockers(1), vec![0, 2]);
}

#[test]
fn in_a_run_zone_waits_are_named_with_their_blocker() {
    // the ring, and a node Z beside it inside zone b, where a robot stands for good
    let g = graph(
        &[
            ("N0", 0.0, 0.0),
            ("N1", 20.0, 0.0),
            ("N2", 20.0, 20.0),
            ("N3", 0.0, 20.0),
            ("Z", 27.0, 3.0),
        ],
        &[("N0", "N1"), ("N1", "N2"), ("N2", "N3"), ("N3", "N0")],
    );
    let node = |n: &str| g.node_idx(n).unwrap();
    let (n0, n2, beside) = (node("N0"), node("N2"), node("Z"));
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let z = zones(&g);
    let mut w = World::new(g, DT, 0).unwrap();
    w.set_zones(z);
    for (i, n) in [n0, beside].into_iter().enumerate() {
        let id = RobotId::new("ACME", &format!("R-{}", i + 1));
        w.add_robot(id, spec.clone(), ActionTimes::new(1.0), n, 0.0)
            .unwrap();
    }
    let mut c = Controller::new(&w);
    // R-1 is to drive from N0 through zone b to N2
    let mut jobs = Some(vec![Job {
        id: 1,
        pickup: n0,
        drop: n2,
        created_tick: 0,
    }]);
    for _ in 0..(60.0 / DT) as usize {
        c.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    assert!(c.completed().is_empty(), "R-2 never leaves the zone");
    let expected = Wait {
        reason: WaitReason::Zone,
        blocker: Some(1),
    };
    assert_eq!(w.waits(), [Some(expected), None]);
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

/// The most robots inside one zone at any snapshot, and how often a zone was used.
fn most_inside(snaps: &[Snapshot]) -> (usize, usize) {
    let g = ring();
    let z = zones(&g);
    let (mut most, mut inside) = (0, 0);
    for s in snaps {
        for zone in &z.zones {
            let n = s
                .robots
                .iter()
                .filter(|r| r.pose.map == z.map && Zones::contains(zone, r.pose.x, r.pose.y))
                .count();
            most = most.max(n);
            inside += n;
        }
    }
    (most, inside)
}

#[test]
fn zones_sent_by_message_keep_the_same_guarantee_with_and_without_a_delay() {
    for delay in [0, 10, 40] {
        let (done, snaps) = busy_ring_with(true, true, delay);
        assert!(done > 10, "delay {delay}: {done} jobs done");
        let (most, inside) = most_inside(&snaps);
        assert_eq!(most, 1, "delay {delay}: never two robots in one zone");
        assert!(inside > 50, "delay {delay}: the zones were used");
    }
}

#[test]
fn robots_get_no_jobs_before_their_zone_set_is_enabled() {
    // the world has the site's zones, the robots have none; the controller sends
    // and enables the set first, so the first job starts a few cycles in (more
    // with a delay), and the robots have the set enabled by then
    let first_assignment = |zones_by_message: bool, delay: u64| {
        let g = ring();
        let z = zones(&g);
        let n0 = g.node_idx("N0").unwrap();
        let job = Job {
            id: 1,
            pickup: n0,
            drop: g.node_idx("N2").unwrap(),
            created_tick: 0,
        };
        let f: Factsheet =
            serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
        let mut w = World::new(g, DT, 0).unwrap();
        if zones_by_message {
            w.set_site_zones(z);
        } else {
            w.set_zones(z);
        }
        w.set_message_delay(delay);
        let id = RobotId::new("ACME", "R-1");
        w.add_robot(
            id.clone(),
            RobotSpec::from_factsheet(&f).unwrap(),
            ActionTimes::new(1.0),
            n0,
            0.0,
        )
        .unwrap();
        let mut c = Controller::new(&w);
        let mut jobs = Some(vec![job]);
        for _ in 0..400 {
            c.step(&mut w, jobs.take().unwrap_or_default());
            w.step();
            if c.busy() == 1 {
                // the robot's own copy of the zone set is enabled by now
                let state = w.state(&id).unwrap();
                let sets = state.zone_sets.unwrap_or_default();
                return (
                    w.tick(),
                    sets.len(),
                    sets.iter()
                        .all(|s| s.zone_set_status == vda5050::state::EnabledStatus::Enabled),
                );
            }
        }
        panic!("the job was never given out");
    };
    let (preset, n, enabled) = first_assignment(false, 0);
    assert_eq!((n, enabled), (1, true));
    let (instant, n, enabled) = first_assignment(true, 0);
    assert_eq!((n, enabled), (1, true));
    let (late, n, enabled) = first_assignment(true, 10);
    assert_eq!((n, enabled), (1, true));
    assert!(instant > preset, "{instant} against {preset}");
    assert!(late > instant + 20, "{late} against {instant}");
}

#[test]
fn the_same_run_gives_the_same_result() {
    let (a, sa) = busy_ring(true);
    let (b, sb) = busy_ring(true);
    assert_eq!(a, b);
    assert_eq!(sa, sb);
}

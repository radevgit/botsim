use std::cell::RefCell;
use std::rc::Rc;

use botsim_core::actions::ActionTimes;
use botsim_core::graph::Graph;
use botsim_core::observer::{Observer, RobotStatus, Snapshot};
use botsim_core::robot::Reception::Accepted;
use botsim_core::spec::{RobotSpec, Shape};
use botsim_core::world::{RobotId, World};
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

const DT: f64 = 0.05;

/// What a recorder saw: the layout once, and every snapshot.
#[derive(Debug, Default)]
struct Log {
    layouts: Vec<(usize, usize, usize)>, // maps, nodes, edges
    snapshots: Vec<Snapshot>,
}

/// A small consumer that collects everything (the test for the port).
#[derive(Debug)]
struct Recorder(Rc<RefCell<Log>>);

impl Observer for Recorder {
    fn on_layout(&mut self, layout: &Graph) {
        self.0.borrow_mut().layouts.push((
            layout.maps.len(),
            layout.nodes.len(),
            layout.edges.len(),
        ));
    }

    fn on_step(&mut self, snapshot: &Snapshot) {
        self.0.borrow_mut().snapshots.push(snapshot.clone());
    }
}

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn id(n: &str) -> RobotId {
    RobotId::new("ACME", n)
}

/// R-1 on A and R-2 on C, every action 2 s.
fn world() -> World {
    let g = graph();
    let (a, c) = (g.node_idx("A").unwrap(), g.node_idx("C").unwrap());
    let mut w = World::new(g, DT, 0).unwrap();
    for (n, node) in [("R-1", a), ("R-2", c)] {
        w.add_robot(id(n), spec(), ActionTimes::new(2.0), node, 0.0)
            .unwrap();
    }
    w
}

fn observed(w: &mut World) -> Rc<RefCell<Log>> {
    let log = Rc::new(RefCell::new(Log::default()));
    w.set_observer(Box::new(Recorder(log.clone())));
    log
}

/// R-1: A → B → C, only A and B released (C is the horizon), a HARD action at B.
fn order() -> Order {
    serde_json::from_str(
        r#"{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"A","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"B","sequenceId":2,"released":true,
                  "actions":[{"actionId":"x","actionType":"pick","blockingType":"HARD"}]},
                 {"nodeId":"C","sequenceId":4,"released":false,"actions":[]}],
        "edges":[{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]},
                 {"edgeId":"B-C","sequenceId":3,"released":false,"actions":[]}]}"#,
    )
    .unwrap()
}

#[test]
fn the_layout_comes_once_and_a_snapshot_after_every_step() {
    let mut w = world();
    let log = observed(&mut w);
    assert_eq!(
        log.borrow().layouts,
        vec![(1, 3, 3)],
        "at once, when attached"
    );
    assert!(log.borrow().snapshots.is_empty());

    for _ in 0..25 {
        w.step();
    }
    let l = log.borrow();
    assert_eq!(l.layouts.len(), 1, "the layout is not sent again");
    assert_eq!(l.snapshots.len(), 25);
    for (i, s) in l.snapshots.iter().enumerate() {
        assert_eq!(s.tick, i as u64 + 1);
        assert!((s.time - (i as f64 + 1.0) * DT).abs() < 1e-9);
        assert_eq!(s.robots.len(), 2);
    }
}

#[test]
fn a_snapshot_describes_every_robot() {
    let mut w = world();
    let log = observed(&mut w);
    w.step();
    let s = log.borrow().snapshots[0].clone();

    let r = &s.robots[0];
    assert_eq!(r.id, id("R-1"));
    assert_eq!(
        (r.pose.x, r.pose.y, r.pose.heading, r.pose.map),
        (0.0, 0.0, 0.0, 0)
    );
    assert!(matches!(r.shape, Shape::Rect { .. }));
    assert_eq!(r.status, RobotStatus::Idle);
    assert_eq!(r.speed, 0.0);
    assert_eq!(r.order_id, "");
    assert!(r.released_nodes.is_empty() && r.planned_nodes.is_empty());

    let c = w.graph().node_idx("C").unwrap();
    assert_eq!(s.robots[1].id, id("R-2"));
    assert_eq!((s.robots[1].pose.x, s.robots[1].pose.y), (10.0, 5.0));
    assert_eq!(c, 2);
    // nothing published, nobody blocked, nobody overlapping
    assert!(s.reservations.is_empty() && s.blocking.is_empty());
    assert!(s.overlapping.is_empty() && s.deadlocks.is_empty());
}

#[test]
fn status_route_and_speed_follow_the_robot_through_an_order() {
    let mut w = world();
    let log = observed(&mut w);
    assert_eq!(w.send_order(&order()), Ok(Accepted));
    for _ in 0..1500 {
        w.step();
    }
    let l = log.borrow();
    let g = graph();
    let (b, c) = (g.node_idx("B").unwrap(), g.node_idx("C").unwrap());
    let (ab, bc) = (g.edge_idx("A-B").unwrap(), g.edge_idx("B-C").unwrap());

    let first = &l.snapshots[0].robots[0];
    assert_eq!(first.order_id, "o1");
    // ahead: B and the edge A-B released, C and B-C planned (the horizon)
    assert_eq!(first.released_nodes, vec![b]);
    assert_eq!(first.released_edges, vec![ab]);
    assert_eq!(first.planned_nodes, vec![c]);
    assert_eq!(first.planned_edges, vec![bc]);
    assert_eq!(first.last_node, Some(g.node_idx("A").unwrap()));

    // it goes through Driving, Acting (the action at B), WaitingForRelease
    let statuses: Vec<RobotStatus> = l.snapshots.iter().map(|s| s.robots[0].status).collect();
    let pos = |st| statuses.iter().position(|s| *s == st);
    let (driving, acting, waiting) = (
        pos(RobotStatus::Driving).unwrap(),
        pos(RobotStatus::Acting).unwrap(),
        pos(RobotStatus::WaitingForRelease).unwrap(),
    );
    assert!(
        driving < acting && acting < waiting,
        "{driving} {acting} {waiting}"
    );
    assert!(l.snapshots[driving].robots[0].speed > 0.0);
    assert_eq!(l.snapshots[acting].robots[0].speed, 0.0);

    // at the end it stands on B, with C still planned
    let last = &l.snapshots.last().unwrap().robots[0];
    assert_eq!(last.last_node, Some(b));
    assert_eq!(last.planned_nodes, vec![c]);
    assert!(last.released_nodes.is_empty());
    assert_eq!((last.pose.x, last.pose.y), (10.0, 0.0));
}

#[test]
fn a_robot_stopped_by_a_disturbance_is_held() {
    let mut w = world();
    w.enable_random_stops(
        4,
        botsim_core::disturbances::StopSpec {
            per_hour: 1800.0,
            min_seconds: 5.0,
            max_seconds: 10.0,
        },
    )
    .unwrap();
    let log = observed(&mut w);
    w.send_order(&order()).unwrap();
    for _ in 0..1500 {
        w.step();
    }
    assert!(
        log.borrow()
            .snapshots
            .iter()
            .any(|s| s.robots[0].status == RobotStatus::Held),
        "no held robot seen"
    );
}

#[test]
fn robots_that_overlap_are_listed() {
    let g = graph();
    let a = g.node_idx("A").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    for n in ["R-1", "R-2"] {
        w.add_robot(id(n), spec(), ActionTimes::new(1.0), a, 0.0)
            .unwrap();
    }
    let log = observed(&mut w);
    w.step();
    assert_eq!(log.borrow().snapshots[0].overlapping, vec![(0, 1)]);
}

#[test]
fn the_same_run_gives_the_same_snapshots_and_observing_changes_nothing() {
    let run = |observe: bool| {
        let mut w = world();
        let log = observe.then(|| observed(&mut w));
        w.send_order(&order()).unwrap();
        for _ in 0..800 {
            w.step();
        }
        let poses: Vec<_> = [id("R-1"), id("R-2")]
            .iter()
            .map(|r| {
                let p = w.pose(r).unwrap();
                (p.x, p.y, p.heading)
            })
            .collect();
        let snaps = log.map(|l| l.borrow().snapshots.clone());
        (poses, w.collisions().len(), snaps)
    };
    let (p1, c1, s1) = run(true);
    let (p2, c2, s2) = run(true);
    let (p3, c3, none) = run(false);
    assert_eq!(s1, s2, "same run, same snapshots");
    assert!(none.is_none());
    // with or without an observer: the same result
    assert_eq!((p1.clone(), c1), (p3, c3));
    assert_eq!((p1, c1), (p2, c2));
}

#[test]
fn the_world_knows_whether_it_is_observed() {
    let mut w = world();
    assert!(!w.observing());
    let _ = observed(&mut w);
    assert!(w.observing());
}

fn observed_at(w: &mut World, seconds: f64) -> Rc<RefCell<Log>> {
    let log = Rc::new(RefCell::new(Log::default()));
    w.set_observer_at(Box::new(Recorder(log.clone())), seconds)
        .unwrap();
    log
}

#[test]
fn a_snapshot_interval_works_like_a_frame_rate() {
    let mut w = world();
    let log = observed_at(&mut w, 0.5); // dt is 0.05: every 10th step
    assert_eq!(log.borrow().layouts.len(), 1);
    for _ in 0..35 {
        w.step();
    }
    let l = log.borrow();
    let ticks: Vec<u64> = l.snapshots.iter().map(|s| s.tick).collect();
    assert_eq!(ticks, vec![10, 20, 30]);
    assert!((l.snapshots[2].time - 1.5).abs() < 1e-9);
    assert_eq!(l.layouts.len(), 1, "still only one layout");
}

#[test]
fn an_interval_is_rounded_to_whole_steps_and_is_at_least_one_step() {
    let ticks_for = |seconds: f64| {
        let mut w = world();
        let log = observed_at(&mut w, seconds);
        for _ in 0..30 {
            w.step();
        }
        let t: Vec<u64> = log.borrow().snapshots.iter().map(|s| s.tick).collect();
        t
    };
    assert_eq!(
        ticks_for(0.26),
        vec![5, 10, 15, 20, 25, 30],
        "0.26 s ≈ 5 steps"
    );
    assert_eq!(
        ticks_for(0.001).len(),
        30,
        "shorter than a step: every step"
    );
    assert_eq!(ticks_for(0.05).len(), 30);
    assert_eq!(ticks_for(100.0).len(), 0, "longer than the run: none");
}

#[test]
fn a_bad_interval_is_refused_and_the_interval_does_not_change_the_run() {
    let mut w = world();
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let r = w.set_observer_at(
            Box::new(Recorder(Rc::new(RefCell::new(Log::default())))),
            bad,
        );
        assert!(
            matches!(
                r,
                Err(botsim_core::world::WorldError::BadObserverInterval(_))
            ),
            "{bad}"
        );
        assert!(!w.observing(), "a refused observer is not attached");
    }

    // the result of a run is the same for every interval
    let run = |seconds: Option<f64>| {
        let mut w = world();
        if let Some(s) = seconds {
            observed_at(&mut w, s);
        }
        w.send_order(&order()).unwrap();
        for _ in 0..800 {
            w.step();
        }
        let p = w.pose(&id("R-1")).unwrap();
        (p.x, p.y, p.heading)
    };
    assert_eq!(run(None), run(Some(0.05)));
    assert_eq!(run(None), run(Some(2.0)));
}

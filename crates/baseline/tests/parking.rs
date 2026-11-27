//! Idle robots go to a free parking place and leave it for a job (U13). Parking
//! places are the interaction nodes of the stations of type `parking`.

use botsim_baseline::controller::Controller;
use botsim_core::actions::ActionTimes;
use botsim_core::jobs::Job;
use botsim_core::world::World;
use botsim_model::graph::{Graph, GraphStation};
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;

mod common;
use common::graph;

const DT: f64 = 0.05;

fn id(n: &str) -> RobotId {
    RobotId::new("ACME", n)
}

/// A one-way ring A → B → C → D → A with one-way sidings B → P → C and, if
/// `two`, D → Q → A; P (and Q) are parking places. Robots on `starts`.
fn setup(two: bool, starts: &[&str]) -> (World, Controller) {
    let mut nodes = vec![
        ("A", 0.0, 0.0),
        ("B", 20.0, 0.0),
        ("C", 20.0, 20.0),
        ("D", 0.0, 20.0),
        ("P", 26.0, 10.0),
    ];
    let mut edges = vec![
        ("A", "B"),
        ("B", "C"),
        ("C", "D"),
        ("D", "A"),
        ("B", "P"),
        ("P", "C"),
    ];
    if two {
        nodes.push(("Q", -6.0, 10.0));
        edges.push(("D", "Q"));
        edges.push(("Q", "A"));
    }
    let mut g: Graph = graph(&nodes, &edges);
    let places: &[&str] = if two { &["P", "Q"] } else { &["P"] };
    let station = GraphStation {
        id: "PARK".into(),
        name: None,
        station_type: Some("Parking".into()),
        interaction_nodes: places.iter().map(|p| g.node_idx(p).unwrap()).collect(),
    };
    g.stations.push(station);
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let starts: Vec<usize> = starts.iter().map(|n| g.node_idx(n).unwrap()).collect();
    let mut w = World::new(g, DT, 0).unwrap();
    for (k, &node) in starts.iter().enumerate() {
        w.add_robot(
            id(&format!("R{}", k + 1)),
            spec.clone(),
            ActionTimes::new(1.0),
            node,
            0.0,
        )
        .unwrap();
    }
    let ctl = Controller::new(&w);
    (w, ctl)
}

fn run(w: &mut World, ctl: &mut Controller, seconds: f64) {
    for _ in 0..(seconds / DT) as usize {
        ctl.step(w, []);
        w.step();
    }
}

/// Is robot `r` standing on node `n`?
fn on(w: &World, r: &str, n: &str) -> bool {
    let node = &w.graph().nodes[w.graph().node_idx(n).unwrap()];
    let p = w.pose(&id(r)).unwrap();
    (p.x - node.x).hypot(p.y - node.y) < 0.1
}

#[test]
fn an_idle_robot_goes_to_a_parking_place_leaves_it_for_a_job_and_comes_back() {
    let (mut w, mut ctl) = setup(false, &["A"]);
    assert_eq!(ctl.parking_places(), 1);
    run(&mut w, &mut ctl, 60.0);
    assert!(on(&w, "R1", "P"));
    assert_eq!((ctl.park_trips(), ctl.no_free_parking()), (1, 0));
    // a parking trip is neither a job nor busy with one
    assert!(ctl.completed().is_empty() && ctl.rejected().is_empty());
    assert_eq!(ctl.busy(), 0);

    let (d, a) = (
        w.graph().node_idx("D").unwrap(),
        w.graph().node_idx("A").unwrap(),
    );
    let job = Job {
        id: 1,
        pickup: d,
        drop: a,
        created_tick: w.tick(),
    };
    ctl.step(&mut w, [job]);
    assert_eq!(ctl.busy(), 1, "it left the place at once");
    run(&mut w, &mut ctl, 120.0);
    assert_eq!(ctl.completed().len(), 1);
    assert!(on(&w, "R1", "P"), "idle again, parked again");
    assert_eq!((ctl.park_trips(), ctl.no_free_parking()), (2, 0));
}

/// Run until a job is completed (at most 200 s); the actions R1 reported as
/// finished on the way, and its order id the last cycle before.
fn run_one_job(w: &mut World, ctl: &mut Controller) -> (Vec<String>, String) {
    let (mut finished, mut order) = (Vec::new(), String::new());
    for _ in 0..(200.0 / DT) as usize {
        let state = w.state(&id("R1")).unwrap();
        for a in &state.action_states {
            let done = a.action_status == vda5050::state::ActionStatus::Finished;
            if done && !finished.contains(&a.action_id) {
                finished.push(a.action_id.clone());
            }
        }
        order = state.order_id;
        ctl.step(w, []);
        if !ctl.completed().is_empty() {
            break;
        }
        w.step();
    }
    (finished, order)
}

fn job(w: &World, pickup: &str, drop: &str) -> Job {
    let node = |n: &str| w.graph().node_idx(n).unwrap();
    Job {
        id: 1,
        pickup: node(pickup),
        drop: node(drop),
        created_tick: w.tick(),
    }
}

fn did(finished: &[String], action: &str) -> bool {
    finished.iter().any(|a| a.starts_with(action))
}

#[test]
fn a_robot_on_its_way_to_a_place_takes_a_job_by_extending_its_order() {
    let (mut w, mut ctl) = setup(false, &["A"]);
    run(&mut w, &mut ctl, 2.0);
    assert_eq!(w.state(&id("R1")).unwrap().order_id, "park-0");
    assert!(!on(&w, "R1", "P"), "still on its way");

    let j = job(&w, "D", "A");
    ctl.step(&mut w, [j]);
    assert_eq!((ctl.busy(), ctl.pending().count()), (1, 0), "taken at once");
    assert!(w.state(&id("R1")).unwrap().order_update_id >= 1);

    let (finished, order) = run_one_job(&mut w, &mut ctl);
    assert_eq!(ctl.completed().len(), 1, "{:?}", ctl.rejected());
    assert_eq!(order, "park-0", "the same order all the way");
    assert!(
        did(&finished, "pick") && did(&finished, "drop"),
        "{finished:?}"
    );
    // idle again, so to the place again
    run(&mut w, &mut ctl, 60.0);
    assert!(on(&w, "R1", "P"));
    assert_eq!((ctl.park_trips(), ctl.no_free_parking()), (2, 0));
}

#[test]
fn a_pickup_at_the_last_released_node_is_not_added_to_the_order() {
    let (mut w, mut ctl) = setup(false, &["A"]);
    run(&mut w, &mut ctl, 2.0);
    // released: A, B and P
    let p = w.graph().node_idx("P").unwrap();
    assert_eq!(ctl.reservations().node_holder(p), Some(0));

    let j = job(&w, "P", "D");
    ctl.step(&mut w, [j]);
    assert_eq!((ctl.busy(), ctl.pending().count()), (0, 1), "it waits");
    let (finished, _) = run_one_job(&mut w, &mut ctl);
    assert_eq!(ctl.completed().len(), 1, "{:?}", ctl.rejected());
    assert!(
        did(&finished, "pick") && did(&finished, "drop"),
        "{finished:?}"
    );
}

#[test]
fn a_parked_robot_holds_only_its_place_so_others_can_pass() {
    let (mut w, mut ctl) = setup(false, &["A"]);
    run(&mut w, &mut ctl, 60.0);
    assert!(on(&w, "R1", "P"));
    // the way it came, A → B → P, is free again; B is on the ring
    let node = |n: &str| w.graph().node_idx(n).unwrap();
    let (a, b, p) = (node("A"), node("B"), node("P"));
    assert_eq!(ctl.reservations().node_holder(a), None);
    assert_eq!(ctl.reservations().node_holder(b), None);
    assert_eq!(ctl.reservations().node_holder(p), Some(0));
    assert_eq!(ctl.reservations().held_by(0), (1, 0), "one node, no edge");
}

#[test]
fn a_robot_on_a_parking_place_stays_there() {
    let (mut w, mut ctl) = setup(false, &["P"]);
    run(&mut w, &mut ctl, 30.0);
    assert!(on(&w, "R1", "P"));
    assert_eq!((ctl.park_trips(), ctl.no_free_parking()), (0, 0));
}

#[test]
fn without_a_free_place_the_robot_stays_and_is_counted_once() {
    // R2 starts on the only place
    let (mut w, mut ctl) = setup(false, &["A", "P"]);
    run(&mut w, &mut ctl, 60.0);
    assert!(on(&w, "R1", "A") && on(&w, "R2", "P"));
    assert_eq!((ctl.park_trips(), ctl.no_free_parking()), (0, 1));
}

#[test]
fn each_robot_gets_its_own_place_the_nearest_pair_first() {
    // R1 on A is nearer to P (via B) than R2 on C; R2 is nearer to Q (via D)
    let (mut w, mut ctl) = setup(true, &["A", "C"]);
    run(&mut w, &mut ctl, 90.0);
    assert!(on(&w, "R1", "P"), "R1 to P");
    assert!(on(&w, "R2", "Q"), "R2 to Q");
    assert_eq!((ctl.park_trips(), ctl.no_free_parking()), (2, 0));
}

#[test]
fn a_layout_without_parking_stations_leaves_idle_robots_where_they_are() {
    let (mut w, _) = setup(false, &["A"]);
    // the same world, a controller that sees no parking: take the station away
    let mut g = w.graph().clone();
    g.stations.clear();
    let spec = w.spec(&id("R1")).unwrap().clone();
    let a = g.node_idx("A").unwrap();
    w = World::new(g, DT, 0).unwrap();
    w.add_robot(id("R1"), spec, ActionTimes::new(1.0), a, 0.0)
        .unwrap();
    let mut ctl = Controller::new(&w);
    run(&mut w, &mut ctl, 30.0);
    assert!(on(&w, "R1", "A"));
    assert_eq!((ctl.parking_places(), ctl.park_trips()), (0, 0));
}

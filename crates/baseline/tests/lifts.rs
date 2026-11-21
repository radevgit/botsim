//! The baseline controller uses a lift (S47), on the official two-level layout:
//! floor 1 (`Map_Z-Level_1`) has N1 and N2, floor 2 (`Map_Z-Level_2`) has N102
//! and N101; N2→N102 and N102→N2 are the rides, N2 and N102 the stops.

use botsim_baseline::controller::Controller;
use botsim_core::actions::ActionTimes;
use botsim_core::graph::Graph;
use botsim_core::jobs::Job;
use botsim_core::lift::{DoorState, LiftSpec, LiftStop};
use botsim_core::spec::RobotSpec;
use botsim_core::world::{RobotId, World};
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

const DT: f64 = 0.05;
const LEVEL_1: &str = "Map_Z-Level_1";
const LEVEL_2: &str = "Map_Z-Level_2";

fn graph() -> Graph {
    // in the official example N101 is only 0.4 m from N102, less than a robot is
    // long; move it 6 m away so that a robot can stand on each
    let text = include_str!("data/two_levels.lif.json").replace("\"x\": 12.0", "\"x\": 18.4");
    let lif: Lif = serde_json::from_str(&text).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn id(n: usize) -> RobotId {
    RobotId::new("ACME", &format!("R-{n}"))
}

/// Robots R-1, R-2, … on the given nodes; one lift "L1" with a stop on N2 and on
/// N102 (unless `stops` is false).
fn world(starts: &[&str], stops: bool) -> World {
    let stops: &[(&str, Option<&str>)] = if stops {
        &[("N2", None), ("N102", None)]
    } else {
        &[]
    };
    build(graph(), [LEVEL_1, LEVEL_2], starts, stops)
}

/// Like [`world`] on layout `g` whose floors are the maps `floors`; `stops` are
/// (node, waiting node), the first on floor 1 and the second on floor 2.
fn build(g: Graph, floors: [&str; 2], starts: &[&str], stops: &[(&str, Option<&str>)]) -> World {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let node = |n: &str| g.node_idx(n).unwrap();
    let stops: Vec<_> = stops
        .iter()
        .enumerate()
        .map(|(floor, (n, w))| LiftStop {
            node: node(n),
            floor,
            door: 0,
            waiting: w.map(node),
        })
        .collect();
    let floors: Vec<_> = floors.iter().map(|m| g.map_idx(m).unwrap()).collect();
    let nodes: Vec<_> = starts.iter().map(|n| node(n)).collect();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_lift(LiftSpec {
        id: "L1".into(),
        floors,
        doors: vec!["front".into()],
        stops,
        start_floor: 0,
        travel_seconds_per_floor: 5.0,
        door_open_seconds: 2.0,
        door_close_seconds: 2.0,
        door_hold_seconds: None,
    })
    .unwrap();
    for (i, n) in nodes.into_iter().enumerate() {
        let times = ActionTimes::new(1.0).with("pick", 5.0).with("drop", 5.0);
        w.add_robot(id(i + 1), spec.clone(), times, n, 0.0).unwrap();
    }
    w
}

fn job(g: &Graph, id: u64, pickup: &str, drop: &str) -> Job {
    Job {
        id,
        pickup: g.node_idx(pickup).unwrap(),
        drop: g.node_idx(drop).unwrap(),
        created_tick: 0,
    }
}

fn map_of(w: &World, n: usize) -> String {
    w.graph().maps[w.pose(&id(n)).unwrap().map].clone()
}

#[test]
fn a_robot_rides_the_lift_for_a_job_and_the_lift_is_let_go_afterwards() {
    let mut w = world(&["N1"], true);
    let mut c = Controller::new(&w);
    let jobs = vec![job(&graph(), 1, "N1", "N101")];
    let mut jobs = Some(jobs);
    let mut rode = false;
    for _ in 0..8000 {
        c.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
        rode |= w.lift_rider("L1").unwrap() == Some(&id(1));
    }
    assert!(rode, "the robot was in the cabin");
    assert_eq!(c.completed().len(), 1);
    assert_eq!(map_of(&w, 1), LEVEL_2);
    assert!(w.collisions().is_empty());
    // the door was released after the robot drove off: closed, nobody inside
    let st = w.lift_status("L1").unwrap();
    assert_eq!((st.floor, st.door), (1, DoorState::Closed));
    assert_eq!(w.lift_rider("L1").unwrap(), None);
}

#[test]
fn a_robot_that_starts_on_the_entrance_rides_too() {
    // job from N2 (the stop) to N101: the ride is the first edge
    let mut w = world(&["N2"], true);
    let mut c = Controller::new(&w);
    let mut jobs = Some(vec![job(&graph(), 1, "N2", "N101")]);
    for _ in 0..8000 {
        c.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    assert_eq!(c.completed().len(), 1);
    assert_eq!(map_of(&w, 1), LEVEL_2);
}

#[test]
fn the_second_robot_waits_before_the_entrance_until_the_lift_is_free() {
    // R-1 (job N2 → N101) starts on the entrance and takes the lift; R-2 (job
    // N1 → N102) is one node before it
    let g = graph();
    let mut w = world(&["N2", "N1"], true);
    let mut c = Controller::new(&w);
    let mut jobs = Some(vec![job(&g, 1, "N2", "N101"), job(&g, 2, "N1", "N102")]);

    let mut seen_waiting = false;
    for _ in 0..16000 {
        c.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
        // while R-1 is in the cabin, R-2 stands on N1 (x = 0), not on the entrance
        if w.lift_rider("L1").unwrap() == Some(&id(1)) {
            let p = w.pose(&id(2)).unwrap();
            seen_waiting = true;
            assert!(p.x.abs() < 1e-9, "R-2 left its waiting node: x = {}", p.x);
        }
    }
    assert!(seen_waiting);
    assert_eq!(c.completed().len(), 2, "both jobs are done");
    assert_eq!(map_of(&w, 1), LEVEL_2);
    assert_eq!(map_of(&w, 2), LEVEL_2);
    assert!(w.collisions().is_empty(), "{:?}", w.collisions());
    assert!(w.deadlocks().is_empty());
}

#[test]
fn a_job_that_comes_back_down_uses_the_lift_twice() {
    // pick up on the upper floor, drop on the lower floor: up and down in one job
    // (the robot starts on N1; N101 is the pickup; the lower floor is a dead end
    // after N2, so the drop is the entrance N2)
    let g = graph();
    let mut w = world(&["N1"], true);
    let mut c = Controller::new(&w);
    let mut jobs = Some(vec![job(&g, 1, "N101", "N2")]);
    for _ in 0..16000 {
        c.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    assert_eq!(c.completed().len(), 1, "{:?}", c.rejected());
    assert_eq!(map_of(&w, 1), LEVEL_1);
    let p = w.pose(&id(1)).unwrap();
    assert!((p.x - 11.0).abs() < 1e-9 && p.y.abs() < 1e-9);
    assert_eq!(w.lift_status("L1").unwrap().door, DoorState::Closed);
    assert_eq!(w.lift_rider("L1").unwrap(), None);
}

#[test]
fn a_lift_ride_works_with_delayed_messages() {
    let g = graph();
    let mut w = world(&["N1"], true);
    w.set_message_delay(10);
    let mut c = Controller::new(&w);
    let mut jobs = Some(vec![job(&g, 1, "N1", "N101")]);
    for _ in 0..12000 {
        c.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    assert_eq!(c.completed().len(), 1, "{:?}", c.rejected());
    assert_eq!(map_of(&w, 1), LEVEL_2);
    assert!(w.collisions().is_empty());
    assert_eq!(w.lift_rider("L1").unwrap(), None);
}

#[test]
fn a_job_that_needs_a_lift_without_stops_is_refused_by_the_robot() {
    let g = graph();
    let mut w = world(&["N1"], false);
    let mut c = Controller::new(&w);
    let mut jobs = Some(vec![job(&g, 1, "N1", "N101")]);
    for _ in 0..200 {
        c.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    assert_eq!(c.rejected().len(), 1);
    assert!(c.completed().is_empty());
    assert_eq!(c.busy(), 0);
}

#[test]
fn the_same_run_gives_the_same_result() {
    let run = || {
        let g = graph();
        let mut w = world(&["N2", "N1"], true);
        let mut c = Controller::new(&w);
        let mut jobs = Some(vec![job(&g, 1, "N2", "N101"), job(&g, 2, "N1", "N102")]);
        let mut done = Vec::new();
        for _ in 0..16000 {
            c.step(&mut w, jobs.take().unwrap_or_default());
            w.step();
        }
        for d in c.completed() {
            done.push((d.job.id, d.finished_tick, d.waiting_seconds.to_bits()));
        }
        done
    };
    assert_eq!(run(), run());
}

/// A two-way layout: on each floor a lane G0 – GJ – G2 (G2 is the lift entrance,
/// a dead end) and a lay-by GW off the junction GJ (floor 2: U0, UJ, U2, UW).
/// The rides are G2→U2 and U2→G2.
fn two_way() -> Graph {
    let node = |id: &str, map: &str, x: f64, y: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"{map}","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]}}]}}"#
        )
    };
    let edge = |a: &str, b: &str| {
        format!(
            r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}",
            "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}],
            "orientationType":"TANGENTIAL","mobileRobotOrientations":[0.0]}}]}}"#
        )
    };
    let floor = |p: &str, map: &str| -> (Vec<String>, Vec<String>) {
        let n = |s: &str| format!("{p}{s}");
        let nodes = vec![
            node(&n("0"), map, 0.0, 0.0),
            node(&n("J"), map, 10.0, 0.0),
            node(&n("2"), map, 20.0, 0.0),
            node(&n("W"), map, 10.0, 6.0),
        ];
        let edges = ["0", "2", "W"]
            .iter()
            .flat_map(|s| [edge(&n("J"), &n(s)), edge(&n(s), &n("J"))])
            .collect();
        (nodes, edges)
    };
    let (gn, mut ge) = floor("G", "L1");
    let (un, mut ue) = floor("U", "L2");
    ge.push(edge("G2", "U2"));
    ue.push(edge("U2", "G2"));
    let layout = |id: &str, n: Vec<String>, e: Vec<String>| {
        format!(
            r#"{{"originId":"O{id}","layouts":[{{"layoutId":"{id}","layoutVersion":"1",
            "nodes":[{}],"edges":[{}]}}]}}"#,
            n.join(","),
            e.join(",")
        )
    };
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{},{}]}}"#,
        layout("1", gn, ge),
        layout("2", un, ue)
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn two_way_world(waiting: bool) -> World {
    let stops = if waiting {
        [("G2", Some("GW")), ("U2", Some("UW"))]
    } else {
        [("G2", None), ("U2", None)]
    };
    build(two_way(), ["L1", "L2"], &["G0", "U0"], &stops)
}

fn edge_ids(g: &Graph, edges: &[usize]) -> Vec<String> {
    edges.iter().map(|&e| g.edges[e].id.clone()).collect()
}

#[test]
fn a_route_to_the_lift_goes_through_the_waiting_node() {
    use botsim_baseline::lifts::LiftDesk;

    let g = two_way();
    let n = |s: &str| g.node_idx(s).unwrap();
    let all = vec![true; g.edges.len()]; // every edge may be used

    let w = two_way_world(true);
    let desk = LiftDesk::new(&w);
    // G0 → U0: to the lay-by, back, to the entrance, the ride, on the upper floor
    // (no detour there: the way after the ride has no ride in it)
    let r = desk.route(&all, n("G0"), n("U0")).unwrap();
    assert_eq!(
        edge_ids(&g, &r),
        [
            "G0-GJ", "GJ-GW", "GW-GJ", "GJ-G2", "G2-U2", "U2-UJ", "UJ-U0"
        ]
    );
    // a robot that starts on the entrance cannot wait outside: no detour
    let r = desk.route(&all, n("G2"), n("U0")).unwrap();
    assert_eq!(edge_ids(&g, &r), ["G2-U2", "U2-UJ", "UJ-U0"]);
    // a robot that starts on the lay-by has passed it already
    let r = desk.route(&all, n("GW"), n("U2")).unwrap();
    assert_eq!(edge_ids(&g, &r), ["GW-GJ", "GJ-G2", "G2-U2"]);
    // no ride, no detour
    let r = desk.route(&all, n("G0"), n("G2")).unwrap();
    assert_eq!(edge_ids(&g, &r), ["G0-GJ", "GJ-G2"]);

    // without a waiting node the route is the shortest one
    let w = two_way_world(false);
    let desk = LiftDesk::new(&w);
    let r = desk.route(&all, n("G0"), n("U0")).unwrap();
    assert_eq!(
        edge_ids(&g, &r),
        ["G0-GJ", "GJ-G2", "G2-U2", "U2-UJ", "UJ-U0"]
    );
}

/// R-1 goes up and R-2 comes down at the same time; returns (jobs done,
/// collisions, deadlocks).
fn opposite_jobs(waiting: bool) -> (usize, usize, usize) {
    let g = two_way();
    let mut w = two_way_world(waiting);
    let mut c = Controller::new(&w);
    let mut jobs = Some(vec![job(&g, 1, "G0", "U0"), job(&g, 2, "U0", "G0")]);
    for _ in 0..20000 {
        c.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    (
        c.completed().len(),
        w.collisions().len(),
        w.deadlocks().len(),
    )
}

#[test]
fn with_a_lay_by_robots_going_opposite_ways_both_get_through() {
    assert_eq!(opposite_jobs(true), (2, 0, 0));
}

#[test]
fn without_a_lay_by_on_a_two_way_lane_the_robots_block_each_other() {
    // the waiting robot stands on the lane the other one must leave by
    let (done, collisions, _) = opposite_jobs(false);
    assert!(done < 2, "done {done}");
    assert_eq!(collisions, 0);
}

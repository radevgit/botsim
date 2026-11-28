//! The baseline controller with a car-like robot (S70): on a one-way loop with
//! rounded corners it does its jobs; on a layout with a sharp corner the robot
//! refuses the order, and the baseline (which does not plan around what a robot
//! cannot turn) does not get the job done.

use botsim::connect;
use botsim::connect::Connected as Controller;
use botsim_model::errors::ErrorType;
use botsim_model::graph::Graph;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_sim::actions::ActionTimes;
use botsim_sim::world::World;
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

#[path = "../../sim/tests/vda_port/mod.rs"]
mod vda_port;
use vda_port::VdaWorld;

const DT: f64 = 0.05;
/// The weight of the middle control point of a quarter circle.
const W: f64 = std::f64::consts::FRAC_1_SQRT_2;

/// A one-way loop (anticlockwise) around a 20 m × 20 m rectangle:
/// N0(0,0) N1(10,0) N2(15,5) N3(15,15) N4(10,20) N5(0,20) N6(-5,15) N7(-5,5).
/// Straight edges N0→N1, N2→N3, N4→N5, N6→N7; with `rounded` the corners are
/// quarter circles of radius 5 (N1→N2, N3→N4, N5→N6, N7→N0), else straight lines
/// (the robot would have to turn 45 degrees at each node).
fn layout(rounded: bool) -> Graph {
    layout_with(if rounded { 4 } else { 0 })
}

/// The first `rounded` corners (in the order of the loop) are rounded, the others sharp.
fn layout_with(rounded: usize) -> Graph {
    let types = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let nodes = [
        (0.0, 0.0),
        (10.0, 0.0),
        (15.0, 5.0),
        (15.0, 15.0),
        (10.0, 20.0),
        (0.0, 20.0),
        (-5.0, 15.0),
        (-5.0, 5.0),
    ];
    // the corner point each arc bends around
    let corners = [(15.0, 0.0), (15.0, 20.0), (-5.0, 20.0), (-5.0, 0.0)];
    let node_json: Vec<String> = nodes
        .iter()
        .enumerate()
        .map(|(i, (x, y))| {
            format!(
                r#"{{"nodeId":"N{i}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
                "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
            )
        })
        .collect();
    let edge_json: Vec<String> = (0..8)
        .map(|i| {
            let j = (i + 1) % 8;
            let (a, b) = (nodes[i], nodes[j]);
            let arc = if i % 2 == 1 && i / 2 < rounded {
                let c = corners[i / 2];
                format!(
                    r#","trajectory":{{"degree":2,"controlPoints":[
                    {{"x":{},"y":{}}},{{"x":{},"y":{},"weight":{W}}},{{"x":{},"y":{}}}]}}"#,
                    a.0, a.1, c.0, c.1, b.0, b.1
                )
            } else {
                String::new()
            };
            format!(
                r#"{{"edgeId":"N{i}-N{j}","startNodeId":"N{i}","endNodeId":"N{j}",
                "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":{types}{arc}}}]}}"#
            )
        })
        .collect();
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{}],"edges":[{}]}}]}}]}}"#,
        node_json.join(","),
        edge_json.join(",")
    );
    Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap()).unwrap()
}

/// One robot on N0 facing east; the jobs around the loop; (jobs done, jobs given
/// back as unserviceable) after 400 s.
fn run(rounded: bool, kinematics: &str) -> (usize, usize) {
    let g = layout(rounded);
    let text = include_str!("data/diff_robot.factsheet.json").replace("DIFFERENTIAL", kinematics);
    let f: Factsheet = serde_json::from_str(&text).unwrap();
    let n = |i: usize| g.node_idx(&format!("N{i}")).unwrap();
    let jobs: Vec<Job> = [(0, 3), (2, 6), (5, 1), (4, 0)]
        .iter()
        .enumerate()
        .map(|(k, &(p, d))| Job {
            id: k as u64 + 1,
            pickup: n(p),
            drop: n(d),
            created_tick: 0,
        })
        .collect();
    let start = n(0);
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(
        RobotId::new("ACME", "R-1"),
        RobotSpec::from_factsheet(&f).unwrap(),
        ActionTimes::new(1.0).with("pick", 2.0).with("drop", 2.0),
        start,
        0.0,
    )
    .unwrap();
    let mut c = connect::controller(&w);
    let mut jobs = Some(jobs);
    for _ in 0..8000 {
        connect::step(&mut c, &mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    assert!(w.collisions().is_empty());
    (c.completed().len(), c.rejected().len())
}

#[test]
fn a_car_like_robot_does_its_jobs_on_a_loop_with_rounded_corners() {
    let (done, rejected) = run(true, "THREE_WHEEL");
    assert_eq!((done, rejected), (4, 0));
}

/// A layout from nodes `(id, x, y)` and edges `(from, to, extra JSON)`; every node
/// and edge allows type ACME X1.
fn custom(nodes: &[(&str, f64, f64)], edges: &[(&str, &str, String)]) -> Graph {
    let types = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let node_json: Vec<String> = nodes
        .iter()
        .map(|(id, x, y)| {
            format!(
                r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
                "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
            )
        })
        .collect();
    let edge_json: Vec<String> = edges
        .iter()
        .map(|(a, b, extra)| {
            format!(
                r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}",
                "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":{types}{extra}}}]}}"#
            )
        })
        .collect();
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{}],"edges":[{}]}}]}}]}}"#,
        node_json.join(","),
        edge_json.join(",")
    );
    Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap()).unwrap()
}

/// The trajectory JSON of a quarter circle from `a` through the corner `c` to `b`.
fn arc(a: (f64, f64), c: (f64, f64), b: (f64, f64)) -> String {
    format!(
        r#","trajectory":{{"degree":2,"controlPoints":[
        {{"x":{},"y":{}}},{{"x":{},"y":{},"weight":{W}}},{{"x":{},"y":{}}}]}}"#,
        a.0, a.1, c.0, c.1, b.0, b.1
    )
}

/// One robot of the given kinematics on `start` facing east; the jobs (pickup, drop);
/// the world after 400 s with the controller, and the nodes the robot has reached.
fn run_custom(
    g: Graph,
    kinematics: &str,
    tune: impl Fn(&mut RobotSpec),
    start: &str,
    jobs: &[(&str, &str)],
) -> (World, Controller, Vec<String>) {
    let text = include_str!("data/diff_robot.factsheet.json").replace("DIFFERENTIAL", kinematics);
    let f: Factsheet = serde_json::from_str(&text).unwrap();
    let mut spec = RobotSpec::from_factsheet(&f).unwrap();
    tune(&mut spec);
    let jobs: Vec<Job> = jobs
        .iter()
        .enumerate()
        .map(|(k, &(p, d))| Job {
            id: k as u64 + 1,
            pickup: g.node_idx(p).unwrap(),
            drop: g.node_idx(d).unwrap(),
            created_tick: 0,
        })
        .collect();
    let start = g.node_idx(start).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let id = RobotId::new("ACME", "R-1");
    w.add_robot(
        id.clone(),
        spec,
        ActionTimes::new(1.0).with("pick", 2.0).with("drop", 2.0),
        start,
        0.0,
    )
    .unwrap();
    let mut c = connect::controller(&w);
    let mut jobs = Some(jobs);
    let mut reached: Vec<String> = Vec::new();
    for _ in 0..8000 {
        connect::step(&mut c, &mut w, jobs.take().unwrap_or_default());
        w.step();
        let at = w.state(&id).unwrap().last_node_id;
        if reached.last() != Some(&at) {
            reached.push(at);
        }
    }
    assert!(w.collisions().is_empty());
    (w, c, reached)
}

#[test]
fn a_refused_order_update_cancels_the_order_and_the_job_is_given_up() {
    // The baseline plans around turns, but does not know the limit on how tight a
    // curve may be: N0 → N1 → N2 → N3 on a line, then a curve of radius 0.5 m
    // (N3 → N4, east to north) and on north to N5. The robot's lowest speed is 0.5 m/s
    // and its angular speed 0.5 rad/s, so the smallest radius it can drive is 1 m.
    // The curve is the 4th edge: not in the first order (two edges), but in the
    // update, which the robot refuses (it used to stop the whole run with a panic).
    let g = custom(
        &[
            ("N0", 0.0, 0.0),
            ("N1", 5.0, 0.0),
            ("N2", 10.0, 0.0),
            ("N3", 15.0, 0.0),
            ("N4", 15.5, 0.5),
            ("N5", 15.5, 5.5),
        ],
        &[
            ("N0", "N1", String::new()),
            ("N1", "N2", String::new()),
            ("N2", "N3", String::new()),
            ("N3", "N4", arc((15.0, 0.0), (15.5, 0.0), (15.5, 0.5))),
            ("N4", "N5", String::new()),
        ],
    );
    let tune = |s: &mut RobotSpec| {
        s.min_speed = 0.5;
        s.limits.max_angular_speed = 0.5;
    };
    let (w, c, _) = run_custom(g, "THREE_WHEEL", tune, "N0", &[("N0", "N5")]);
    assert!(c.completed().is_empty());
    assert_eq!(c.rejected().len(), 1);
    assert_eq!(c.rejected()[0].job.id, 1);
    assert_eq!(c.rejected()[0].reason, ErrorType::NoRouteToTarget);
    // the robot has stopped on a node, short of the curve
    let id = RobotId::new("ACME", "R-1");
    let mut w = w;
    assert!(!w.state(&id).unwrap().driving);
    let p = w.pose(&id).unwrap();
    assert!(p.x > 0.0 && p.x <= 15.0 && p.y == 0.0, "({}, {})", p.x, p.y);
}

#[test]
fn on_a_loop_with_sharp_corners_a_car_like_robot_gets_no_job_and_a_differential_robot_all() {
    // 45 degree turns at every node: a robot that turns on the spot does them all
    assert_eq!(run(false, "DIFFERENTIAL"), (4, 0));
    // for the car-like robot there is no route at all: the baseline does not send an
    // order it would refuse; the jobs wait
    let g = layout(false);
    let jobs = [("N0", "N3"), ("N2", "N6"), ("N5", "N1"), ("N4", "N0")];
    let (_, c, _) = run_custom(g, "THREE_WHEEL", |_| {}, "N0", &jobs);
    assert_eq!((c.completed().len(), c.rejected().len()), (0, 0));
    assert_eq!(c.pending().count(), 4);
}

#[test]
fn the_baseline_costs_a_curved_edge_by_the_length_of_the_curve() {
    // A (0,0) to E (20,10) two ways. Over B: a quarter circle of radius 10 to B
    // (10,10), 15.7 m long on a chord of 14.1 m, then straight on to E: 25.7 m (24.1
    // m if the curve counted as its chord). Over C (12.5,0): 12.5 + 12.5 = 25 m. The
    // route over C is the shorter one; costed by the chord the baseline would take B.
    let g = custom(
        &[
            ("A", 0.0, 0.0),
            ("B", 10.0, 10.0),
            ("C", 12.5, 0.0),
            ("E", 20.0, 10.0),
        ],
        &[
            ("A", "B", arc((0.0, 0.0), (10.0, 0.0), (10.0, 10.0))),
            ("B", "E", String::new()),
            ("A", "C", String::new()),
            ("C", "E", String::new()),
        ],
    );
    let (_, c, reached) = run_custom(g, "DIFFERENTIAL", |_| {}, "A", &[("A", "E")]);
    assert_eq!(c.completed().len(), 1, "{:?}", c.rejected());
    assert!(reached.contains(&"C".to_string()), "{reached:?}");
    assert!(!reached.contains(&"B".to_string()), "{reached:?}");
}

#[test]
fn the_baseline_plans_around_a_node_where_the_layout_allows_no_rotation() {
    // Same idea for a robot that turns on the spot: the direct edge B → E leaves B
    // 18° off the heading, and the layout allows no rotation at its start; so the
    // robot goes on to D (where a 45° turn to E is allowed).
    let layout = |bar: &str| {
        custom(
            &[
                ("A", 0.0, 0.0),
                ("B", 10.0, 0.0),
                ("D", 20.0, 0.0),
                ("E", 25.0, 5.0),
            ],
            &[
                ("A", "B", String::new()),
                ("B", "E", bar.to_string()),
                ("B", "D", String::new()),
                ("D", "E", String::new()),
            ],
        )
    };
    let jobs = [("A", "E")];
    let free = run_custom(layout(""), "DIFFERENTIAL", |_| {}, "A", &jobs);
    assert_eq!(free.1.completed().len(), 1);
    assert!(!free.2.contains(&"D".to_string()), "{:?}", free.2);
    let barred = run_custom(
        layout(r#","rotationAtStartNodeAllowed":"NONE""#),
        "DIFFERENTIAL",
        |_| {},
        "A",
        &jobs,
    );
    assert_eq!(barred.1.completed().len(), 1, "{:?}", barred.1.rejected());
    assert!(barred.2.contains(&"D".to_string()), "{:?}", barred.2);
}

#[test]
fn the_baseline_takes_the_longer_route_that_a_car_like_robot_can_drive() {
    // A → B → D on a line (east). From B the direct edge to E (25, 5) leaves 18°
    // off the heading: a differential robot turns there and takes it (15.8 m). A
    // car-like robot cannot: it goes on to D and takes the curve to E, which arrives
    // heading north (10 + 7.1 m, longer).
    let layout = || {
        custom(
            &[
                ("A", 0.0, 0.0),
                ("B", 10.0, 0.0),
                ("D", 20.0, 0.0),
                ("E", 25.0, 5.0),
            ],
            &[
                ("A", "B", String::new()),
                ("B", "E", String::new()),
                ("B", "D", String::new()),
                ("D", "E", arc((20.0, 0.0), (25.0, 0.0), (25.0, 5.0))),
            ],
        )
    };
    let jobs = [("A", "E")];
    let (_, c, reached) = run_custom(layout(), "DIFFERENTIAL", |_| {}, "A", &jobs);
    assert_eq!(c.completed().len(), 1);
    assert!(!reached.contains(&"D".to_string()), "{reached:?}");
    let (_, c, reached) = run_custom(layout(), "THREE_WHEEL", |_| {}, "A", &jobs);
    assert_eq!(c.completed().len(), 1, "{:?}", c.rejected());
    assert!(reached.contains(&"D".to_string()), "{reached:?}");
}

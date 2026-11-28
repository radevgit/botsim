//! A deadlock with a safety link (S77): robot O has the way released along a long
//! edge S → Q that passes the node P where robot X stands and waits for Q. O is
//! stopped by X in its protective field, X waits for O's node: neither can move.
//! After a few seconds X, which is right in front of O and heads the same way, takes
//! the node over and goes first.

use botsim_bridge as bridge;
use botsim_bridge::Bridged as Controller;
use botsim_core::actions::ActionTimes;
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

const DT: f64 = 0.05;

/// S(-10,0), P(0,0), Q(10,0), R(20,0): S → Q is one long edge that passes P, then
/// P → Q and Q → R. With `back`, also R → ... nothing: a line.
fn graph() -> Graph {
    let types = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let node = |id: &str, x: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":0.0}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
        )
    };
    let edge = |a: &str, b: &str| {
        format!(
            r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}",
            "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
        )
    };
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{},{}],"edges":[{},{},{}]}}]}}]}}"#,
        node("S", -10.0),
        node("P", 0.0),
        node("Q", 10.0),
        node("R", 20.0),
        edge("S", "Q"),
        edge("P", "Q"),
        edge("Q", "R")
    );
    Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap()).unwrap()
}

fn run(seconds: f64) -> (World, Controller) {
    let g = graph();
    let n = |id: &str| g.node_idx(id).unwrap();
    let (s, p, q, r) = (n("S"), n("P"), n("Q"), n("R"));
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let times = ActionTimes::new(1.0).with("pick", 2.0).with("drop", 2.0);
    // O is added first: it is given its job first and has the way released first
    w.add_robot(
        RobotId::new("ACME", "O"),
        spec.clone(),
        times.clone(),
        s,
        0.0,
    )
    .unwrap();
    w.add_robot(RobotId::new("ACME", "X"), spec, times, p, 0.0)
        .unwrap();
    w.set_safety_stop(true);
    w.set_deadlock_timeout(20.0);
    let jobs = vec![
        Job {
            id: 1,
            pickup: s,
            drop: q,
            created_tick: 0,
        },
        Job {
            id: 2,
            pickup: p,
            drop: r,
            created_tick: 0,
        },
    ];
    let mut c = bridge::controller(&w);
    let mut jobs = Some(jobs);
    for _ in 0..(seconds / DT) as usize {
        bridge::step(&mut c, &mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    (w, c)
}

/// A lane change (brief update): robot O drives S(-10,0) → Q(-2,-8) on a quarter
/// arc; robot X stands on P(-6,-0.5) beside the start of the curve and goes
/// P → Q → R → T. Both go on to Q and R; X heads about 55° across O's heading.
fn crossing_graph() -> Graph {
    let types = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let node = |id: &str, x: f64, y: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
        )
    };
    let edge = |a: &str, b: &str, arc: &str| {
        format!(
            r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}",
            "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":{types}{arc}}}]}}"#
        )
    };
    let arc = format!(
        r#","trajectory":{{"degree":2,"controlPoints":[
        {{"x":-10.0,"y":0.0}},{{"x":-2.0,"y":0.0,"weight":{w}}},{{"x":-2.0,"y":-8.0}}]}}"#
    );
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{},{},{}],"edges":[{},{},{},{}]}}]}}]}}"#,
        node("S", -10.0, 0.0),
        node("P", -6.0, -0.5),
        node("Q", -2.0, -8.0),
        node("R", -2.0, -16.0),
        node("T", -2.0, -24.0),
        edge("S", "Q", &arc),
        edge("P", "Q", ""),
        edge("Q", "R", ""),
        edge("R", "T", "")
    );
    Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap()).unwrap()
}

#[test]
fn a_robot_heading_across_the_curve_of_a_stopped_robot_goes_first() {
    let g = crossing_graph();
    let n = |id: &str| g.node_idx(id).unwrap();
    let (s, p, r, t) = (n("S"), n("P"), n("R"), n("T"));
    let (o, x) = (RobotId::new("ACME", "O"), RobotId::new("ACME", "X"));
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let times = ActionTimes::new(1.0).with("pick", 2.0).with("drop", 2.0);
    w.add_robot(o, spec.clone(), times.clone(), s, 0.0).unwrap();
    // X already faces Q (no turn on the spot needed, which would sweep into O)
    w.add_robot(x, spec, times, p, (-7.5f64).atan2(4.0))
        .unwrap();
    w.set_safety_stop(true);
    w.set_deadlock_timeout(20.0);
    let job = |id, pickup, drop| Job {
        id,
        pickup,
        drop,
        created_tick: 0,
    };
    let mut jobs = Some(vec![job(1, s, r), job(2, p, t)]);
    let mut c = bridge::controller(&w);
    for _ in 0..(200.0 / DT) as usize {
        bridge::step(&mut c, &mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    let at = |name: &str| {
        let p = w.pose(&RobotId::new("ACME", name)).unwrap();
        format!("{name} ({:.1},{:.1}) {:.2}", p.x, p.y, p.heading)
    };
    let dbg = format!(
        "{} {} gw={} dl={:?} {:?}",
        at("O"),
        at("X"),
        c.give_ways(),
        w.deadlocks(),
        c.rejected()
    );
    assert_eq!(c.completed().len(), 2, "{dbg}");
    assert!(c.give_ways() >= 1, "nobody gave way");
    assert!(w.deadlocks().is_empty(), "{:?}", w.deadlocks());
    assert!(w.collisions().is_empty());
}

#[test]
fn the_robot_in_front_goes_first_and_both_jobs_are_done() {
    let (w, c) = run(200.0);
    assert_eq!(c.completed().len(), 2, "{:?}", c.rejected());
    assert!(c.give_ways() >= 1, "nobody gave way");
    assert!(w.deadlocks().is_empty(), "{:?}", w.deadlocks());
    assert!(w.collisions().is_empty());
}

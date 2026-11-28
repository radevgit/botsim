//! The baseline plans routes for each robot type separately (S55): a robot is only
//! given edges its type may use, and jobs go to robots that can do them.

use botsim_baseline::controller::Controller;
use botsim_bridge as bridge;
use botsim_core::actions::ActionTimes;
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

const DT: f64 = 0.05;

fn types(list: &[&str]) -> String {
    let t: Vec<String> = list
        .iter()
        .map(|s| format!(r#"{{"manufacturer":"ACME","seriesName":"{s}"}}"#))
        .collect();
    format!("[{}]", t.join(","))
}

/// ```text
///       E          A → B → C → D      B → C only for X1;
///      ↗ ↘                           the way round B → E → C is for both
///     B     C
/// ```
/// A(0,0) B(5,0) C(10,0) D(15,0) E(7.5,5). Every node allows both types.
fn layout(round: bool) -> Graph {
    let node = |id: &str, x: f64, y: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{}}}]}}"#,
            types(&["X1", "Y2"])
        )
    };
    let edge = |a: &str, b: &str, allowed: &[&str]| {
        format!(
            r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}","mobileRobotTypeEdgeProperties":[
            {{"mobileRobotTypes":{},"orientationType":"TANGENTIAL","mobileRobotOrientations":[0.0]}}]}}"#,
            types(allowed)
        )
    };
    let mut edges = vec![
        edge("A", "B", &["X1", "Y2"]),
        edge("B", "C", &["X1"]),
        edge("C", "D", &["X1", "Y2"]),
    ];
    if round {
        edges.push(edge("B", "E", &["X1", "Y2"]));
        edges.push(edge("E", "C", &["X1", "Y2"]));
    }
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{},{},{}],"edges":[{}]}}]}}]}}"#,
        node("A", 0.0, 0.0),
        node("B", 5.0, 0.0),
        node("C", 10.0, 0.0),
        node("D", 15.0, 0.0),
        node("E", 7.5, 5.0),
        edges.join(",")
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn spec(series: &str) -> RobotSpec {
    let text =
        include_str!("data/diff_robot.factsheet.json").replace("\"X1\"", &format!("\"{series}\""));
    let f: Factsheet = serde_json::from_str(&text).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn id(n: usize) -> RobotId {
    RobotId::new("ACME", &format!("R-{n}"))
}

/// Robots R-1, R-2, … (series, start node).
fn world(g: Graph, robots: &[(&str, &str)]) -> World {
    let starts: Vec<_> = robots.iter().map(|(_, n)| g.node_idx(n).unwrap()).collect();
    let mut w = World::new(g, DT, 0).unwrap();
    for (i, ((series, _), n)) in robots.iter().zip(starts).enumerate() {
        let times = ActionTimes::new(1.0).with("pick", 2.0).with("drop", 2.0);
        w.add_robot(id(i + 1), spec(series), times, n, 0.0).unwrap();
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

/// Run one job for `seconds`; returns whether robot R-n passed near E (7.5, 5).
fn run(w: &mut World, c: &mut Controller, jobs: Vec<Job>, seconds: f64) -> Vec<bool> {
    let mut jobs = Some(jobs);
    let n = w.robot_ids().count();
    let mut via_e = vec![false; n];
    for _ in 0..(seconds / DT) as u64 {
        bridge::step(c, w, jobs.take().unwrap_or_default());
        w.step();
        for (i, seen) in via_e.iter_mut().enumerate() {
            let p = w.pose(&id(i + 1)).unwrap();
            *seen |= (p.x - 7.5).hypot(p.y - 5.0) < 0.3;
        }
    }
    via_e
}

#[test]
fn each_type_gets_a_route_over_the_edges_it_may_use() {
    // X1 takes B → C, Y2 must go round by E
    for (series, goes_round) in [("X1", false), ("Y2", true)] {
        let g = layout(true);
        let j = job(&g, 1, "A", "D");
        let mut w = world(g, &[(series, "A")]);
        let mut c = bridge::controller(&w);
        let via_e = run(&mut w, &mut c, vec![j], 120.0);
        assert_eq!(c.completed().len(), 1, "{series}");
        assert_eq!(via_e[0], goes_round, "{series}");
        let p = w.pose(&id(1)).unwrap();
        assert_eq!((p.x, p.y), (15.0, 0.0), "{series}");
    }
}

#[test]
fn a_job_goes_to_the_robot_that_can_do_it_even_if_another_is_nearer() {
    // without the way round only X1 can get from A to D; both robots stand on A
    // (equally near) and the Y2 robot was added first, so it would win a tie
    let g = layout(false);
    let j = job(&g, 1, "A", "D");
    let mut w = world(g, &[("Y2", "A"), ("X1", "A")]);
    let mut c = bridge::controller(&w);
    run(&mut w, &mut c, vec![j], 120.0);
    let done = c.completed();
    assert_eq!(done.len(), 1, "{:?}", c.rejected());
    assert_eq!(done[0].robot, id(2), "the X1 robot did it");
    assert!(c.rejected().is_empty());
}

#[test]
fn a_job_no_robot_type_can_do_waits_and_is_not_refused() {
    // only a Y2 robot, no way round: the job cannot be done
    let g = layout(false);
    let j = job(&g, 1, "A", "D");
    let mut w = world(g, &[("Y2", "A")]);
    let mut c = bridge::controller(&w);
    run(&mut w, &mut c, vec![j], 30.0);
    assert!(c.completed().is_empty());
    assert!(c.rejected().is_empty());
    assert_eq!(c.pending().count(), 1);
    assert_eq!(c.busy(), 0);
}

#[test]
fn the_same_run_gives_the_same_result() {
    let once = || {
        let g = layout(true);
        let jobs = vec![job(&g, 1, "A", "D"), job(&g, 2, "A", "C")];
        let mut w = world(g, &[("Y2", "A"), ("X1", "B")]);
        let mut c = bridge::controller(&w);
        run(&mut w, &mut c, jobs, 120.0);
        c.completed()
            .iter()
            .map(|d| (d.job.id, d.robot.serial_number.clone(), d.finished_tick))
            .collect::<Vec<_>>()
    };
    assert_eq!(once(), once());
}

//! Route with load: two lanes of the same length lead from A to B. The first robot
//! takes one; with a load weight the second, whose job starts while the first holds
//! its lane, takes the empty one. Without it both use the same lane (the shortest
//! route is the same every time).

use botsim::connect;
use botsim_model::graph::Graph;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_model::time::Millis;
use botsim_sim::actions::ActionTimes;
use botsim_sim::world::World;
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

const DT: f64 = 0.05;

/// C(-5,0) → A(0,0), then two lanes A → U(20,10) → B(40,0) and A → L(20,-10) → B.
fn graph() -> Graph {
    let types = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let node = |id: &str, x: f64, y: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
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
        "nodes":[{},{},{},{},{}],"edges":[{},{},{},{},{}]}}]}}]}}"#,
        node("C", -5.0, 0.0),
        node("A", 0.0, 0.0),
        node("U", 20.0, 10.0),
        node("L", 20.0, -10.0),
        node("B", 40.0, 0.0),
        edge("C", "A"),
        edge("A", "U"),
        edge("A", "L"),
        edge("U", "B"),
        edge("L", "B")
    );
    Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap()).unwrap()
}

/// Robot 0 (at A) and robot 1 (at C) both go to B; robot 1's job starts 2 s later.
/// Returns which of the lane nodes (U, L) each robot held at some time in 60 s:
/// `[robot][lane]`.
fn lanes_used(weight: f64) -> [[bool; 2]; 2] {
    let g = graph();
    let n = |id: &str| g.node_idx(id).unwrap();
    let (c, a, b, u, l) = (n("C"), n("A"), n("B"), n("U"), n("L"));
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let times = ActionTimes::new(1.0).with("pick", 2.0).with("drop", 2.0);
    w.add_robot(
        RobotId::new("ACME", "R0"),
        spec.clone(),
        times.clone(),
        a,
        0.0,
    )
    .unwrap();
    w.add_robot(RobotId::new("ACME", "R1"), spec, times, c, 0.0)
        .unwrap();
    w.set_safety_stop(true);
    let mut ctl = connect::controller(&w);
    ctl.set_load_weight(weight);
    let job = |id, pickup, created| Job {
        id,
        pickup,
        drop: b,
        created: Millis(created),
    };
    let mut used = [[false; 2]; 2];
    for tick in 0..(60.0 / DT) as u64 {
        let new = match tick {
            0 => vec![job(1, a, 0)],
            40 => vec![job(2, c, 2000)],
            _ => Vec::new(),
        };
        connect::step(&mut ctl, &mut w, new);
        w.step();
        let r = ctl.policy().table();
        for (lane, node) in [u, l].into_iter().enumerate() {
            if let Some(h) = r.node_holder(node) {
                used[h][lane] = true;
            }
        }
    }
    used
}

#[test]
fn without_a_load_weight_both_robots_use_the_same_lane() {
    let used = lanes_used(0.0);
    assert_eq!(used[0], used[1], "{used:?}");
    assert!(used[0][0] != used[0][1], "{used:?}");
}

#[test]
fn with_a_load_weight_the_second_robot_takes_the_empty_lane() {
    let used = lanes_used(5.0);
    assert!(
        used[0][0] != used[0][1] && used[1][0] != used[1][1],
        "{used:?}"
    );
    assert_ne!(used[0], used[1], "{used:?}");
}

//! Route with load, waiting average: the upper lane A → U1 → U2 → B is a little
//! shorter than the lower lane A → L → B, but a parked robot Z stands on U2, so the
//! first robot R0 waits on U1 for ever. A robot R1 that gets its job a minute later
//! takes the lower lane if the waiting average counts, else it queues behind R0.

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
        "nodes":[{},{},{},{},{},{},{}],"edges":[{},{},{},{},{},{}]}}]}}]}}"#,
        node("C", -5.0, 0.0),
        node("A", 0.0, 0.0),
        node("U1", 10.0, 4.75),
        node("U2", 20.0, 9.5),
        node("L", 20.0, -10.0),
        node("B", 40.0, 0.0),
        node("Z", 20.0, 14.0),
        edge("C", "A"),
        edge("A", "U1"),
        edge("U1", "U2"),
        edge("U2", "B"),
        edge("A", "L"),
        edge("L", "B")
    );
    Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap()).unwrap()
}

/// Did robot 1 hold the lower lane node L within 8 s of the start of its job?
fn second_robot_takes_the_lower_lane(wait_weight: f64) -> bool {
    let g = graph();
    let n = |id: &str| g.node_idx(id).unwrap();
    let (c, a, b, l, u2) = (n("C"), n("A"), n("B"), n("L"), n("U2"));
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    let times = ActionTimes::new(1.0).with("pick", 2.0).with("drop", 2.0);
    for (name, node) in [("R0", a), ("R1", c), ("Z", u2)] {
        w.add_robot(
            RobotId::new("ACME", name),
            spec.clone(),
            times.clone(),
            node,
            0.0,
        )
        .unwrap();
    }
    w.set_safety_stop(true);
    let mut ctl = bridge::controller(&w);
    ctl.set_wait_weight(wait_weight);
    let job = |id, pickup, created_tick| Job {
        id,
        pickup,
        drop: b,
        created_tick,
    };
    let mut took_l = false;
    for tick in 0..(120.0 / DT) as u64 {
        let new = match tick {
            0 => vec![job(1, a, 0)],
            1200 => vec![job(2, c, 1200)],
            _ => Vec::new(),
        };
        bridge::step(&mut ctl, &mut w, new);
        w.step();
        // (only in the 8 s after the job starts: after 10 s of waiting the re-route
        // takes the lower lane in any case)
        if (1200..1360).contains(&tick) {
            took_l |= ctl.policy().table().node_holder(l) == Some(1);
        }
    }
    took_l
}

#[test]
fn without_a_wait_weight_the_second_robot_queues_in_the_shorter_lane() {
    assert!(!second_robot_takes_the_lower_lane(0.0));
}

#[test]
fn with_a_wait_weight_the_second_robot_avoids_the_lane_where_robots_wait() {
    assert!(second_robot_takes_the_lower_lane(5.0));
}

//! The baseline controller with a curved edge (S69): orders do not change, the
//! robot follows the curve, and drives through where the curves join.

use botsim_baseline::controller::Controller;
use botsim_core::actions::ActionTimes;
use botsim_core::jobs::Job;
use botsim_core::world::{RobotId, World};
use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

const DT: f64 = 0.05;

/// A(0,0) –[quarter circle]→ B(5,5) → C(5,12): the arc ends going north and B → C
/// goes north.
fn layout() -> Graph {
    let types = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let node = |id: &str, x: f64, y: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
        )
    };
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{}],"edges":[
          {{"edgeId":"A-B","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[
            {{"mobileRobotTypes":{types},"trajectory":{{"degree":2,"controlPoints":[
              {{"x":0.0,"y":0.0}},{{"x":5.0,"y":0.0,"weight":0.7071067811865476}},{{"x":5.0,"y":5.0}}]}}}}]}},
          {{"edgeId":"B-C","startNodeId":"B","endNodeId":"C","mobileRobotTypeEdgeProperties":[
            {{"mobileRobotTypes":{types}}}]}}]}}]}}]}}"#,
        node("A", 0.0, 0.0),
        node("B", 5.0, 5.0),
        node("C", 5.0, 12.0)
    );
    Graph::from_lif(&serde_json::from_str::<Lif>(&json).unwrap()).unwrap()
}

/// One robot on A facing east, a job A → C; returns (finish tick, the lowest speed
/// the robot had while it was between 1 m before and 1 m after B).
fn run(pass_through: bool) -> (u64, f64) {
    let g = layout();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let a = g.node_idx("A").unwrap();
    let job = Job {
        id: 1,
        pickup: a,
        drop: g.node_idx("C").unwrap(),
        created_tick: 0,
    };
    let mut w = World::new(g, DT, 0).unwrap();
    let id = RobotId::new("ACME", "R-1");
    w.add_robot(
        id.clone(),
        RobotSpec::from_factsheet(&f).unwrap(),
        ActionTimes::new(1.0).with("pick", 2.0).with("drop", 2.0),
        a,
        0.0,
    )
    .unwrap();
    w.set_pass_through(pass_through);
    let mut c = Controller::new(&w);
    let mut jobs = Some(vec![job]);
    let mut lowest = f64::MAX;
    for _ in 0..6000 {
        c.step(&mut w, jobs.take().unwrap_or_default());
        w.step();
        let p = w.pose(&id).unwrap();
        if (p.x - 5.0).abs() < 1e-6 && (4.0..6.0).contains(&p.y)
            || (p.x - 5.0).abs() < 1e-3 && (6.0..6.9).contains(&p.y)
        {
            lowest = lowest.min(w.snapshot().robots[0].speed.abs());
        }
    }
    assert_eq!(c.completed().len(), 1, "{:?}", c.rejected());
    let p = w.pose(&id).unwrap();
    assert_eq!((p.x, p.y), (5.0, 12.0));
    assert!(w.collisions().is_empty());
    (c.completed()[0].finished_tick, lowest)
}

#[test]
fn a_job_over_a_curved_edge_is_done_and_the_robot_drives_through_the_smooth_node() {
    let (stopping, _) = run(false);
    let (through, lowest) = run(true);
    // at B the arc and B → C both point north: no stop and no turn
    assert!(lowest > 1.0, "slowest near B: {lowest}");
    assert!(through < stopping, "through {through}, stopping {stopping}");
}

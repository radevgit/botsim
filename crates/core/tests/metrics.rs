use botsim_core::actions::ActionTimes;
use botsim_core::robot::Robot;
use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

mod read_state;
use read_state::is_waiting;
mod vda_port;
use vda_port::{Header, VdaRobot};

const DT: f64 = 0.05;

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn header() -> Header {
    Header {
        header_id: 1,
        timestamp: "2026-11-20T10:00:00.000Z".into(),
        version: "3.0.1".into(),
        manufacturer: "ACME".into(),
        serial_number: "R-001".into(),
    }
}

fn waiting(r: &Robot) -> bool {
    is_waiting(&r.state(&graph(), &header()))
}

#[test]
fn a_robot_is_waiting_only_when_still_and_not_busy_with_an_action() {
    let g = graph();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let mut r = Robot::new(
        &g,
        RobotSpec::from_factsheet(&f).unwrap(),
        ActionTimes::new(2.0),
        g.node_idx("A").unwrap(),
        0.0,
    )
    .unwrap();
    assert!(waiting(&r), "standing idle");

    // A→B, a HARD action at B (2 s), nothing at the end
    let order: Order = serde_json::from_str(
        r#"{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"A","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"B","sequenceId":2,"released":true,
                  "actions":[{"actionId":"a","actionType":"pick","blockingType":"HARD"}]}],
        "edges":[{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]}]}"#,
    )
    .unwrap();
    r.receive_order(&g, &order);
    let (mut driving, mut acting, mut other) = (0, 0, 0);
    for _ in 0..2000 {
        r.step(DT);
        let s = r.state(&g, &header());
        let busy_action = s
            .action_states
            .iter()
            .any(|a| a.action_status == vda5050::state::ActionStatus::Running);
        match (s.driving, busy_action, is_waiting(&s)) {
            (true, _, false) => driving += 1,
            (false, true, false) => acting += 1,
            (false, false, true) => other += 1,
            bad => panic!("unexpected {bad:?}"),
        }
    }
    assert!(driving > 100, "driving {driving}");
    assert_eq!(acting, 40, "2 s of action at 50 ms");
    assert!(other > 0, "finished: standing still again");
}

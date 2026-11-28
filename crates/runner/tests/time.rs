//! The controller's messages carry the time of the world.

use std::sync::Arc;

use botsim_model::graph::Graph;
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_model::time::Millis;
use botsim_server::cycle::{Inbox, Message, robot_no};
use botsim_sim::actions::ActionTimes;
use botsim_sim::world::World;
use botsim_vda::translator::{RobotMessage, Translator};
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

fn graph() -> Graph {
    let lif: Lif =
        serde_json::from_str(include_str!("../../sim/tests/data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

#[test]
fn the_timestamp_made_from_the_time_of_a_cycle_is_the_worlds() {
    let f: Factsheet = serde_json::from_str(include_str!(
        "../../sim/tests/data/diff_robot.factsheet.json"
    ))
    .unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let id = RobotId::new("ACME", "R-001");
    // steps that are no whole number of milliseconds, and a start that is not on a
    // whole second
    for (dt, start) in [(0.05, 0), (0.0333, 1_763_632_800_123), (0.0125, -7)] {
        let g = graph();
        let (a, b) = (g.node_idx("A").unwrap(), g.node_idx("B").unwrap());
        let mut w = World::new(g, dt, start).unwrap();
        w.add_robot(id.clone(), spec.clone(), ActionTimes::new(1.0), a, 0.0)
            .unwrap();
        for steps in [0, 1, 2, 3, 7, 1000, 12345] {
            while w.tick() < steps {
                w.step();
            }
            // a controller that has just started sends an order at once
            let mut c = botsim::connect::controller(&w);
            let job = Job {
                id: 1,
                pickup: a,
                drop: b,
                created: Millis(w.now_ms()),
            };
            let inbox = Inbox {
                robots: w.reports(),
                jobs: vec![job],
                ..Inbox::default()
            };
            let out = c.step(Millis(w.now_ms()), inbox);
            let [Message::Robot(message)] = &out.messages[..] else {
                panic!("one message expected: {:?}", out.messages);
            };
            let mut translator = Translator::new(Arc::new(w.graph().clone()));
            translator.add_robot(robot_no(0), id.clone()).unwrap();
            let written = translator.to_robot_to_vda(message).unwrap().message;
            let RobotMessage::Order(order) = written else {
                panic!("an order expected: {written:?}");
            };
            assert_eq!(order.timestamp, w.timestamp(), "dt {dt}, step {steps}");
        }
    }
}

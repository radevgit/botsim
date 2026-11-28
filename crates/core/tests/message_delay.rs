//! Message delay between the controller and the robots (S58).

use botsim_core::actions::ActionTimes;
use botsim_core::drive::Phase;
use botsim_core::robot::Reception;
use botsim_core::world::World;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::instant_actions::InstantActions;
use vda5050::order::Order;
use vdma_lif::Lif;

mod vda_port;
use vda_port::VdaWorld;

const DT: f64 = 0.05;

fn id() -> RobotId {
    RobotId::new("ACME", "R-001")
}

/// Layout A(0,0) – B(10,0) – C(10,5); robot R-001 on A; messages take `delay` steps.
fn world(delay: u64) -> World {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    let g = Graph::from_lif(&lif).unwrap();
    let a = g.node_idx("A").unwrap();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(
        id(),
        RobotSpec::from_factsheet(&f).unwrap(),
        ActionTimes::new(1.0),
        a,
        0.0,
    )
    .unwrap();
    w.set_message_delay(delay);
    w
}

fn order(from: &str, edge: &str, to: &str) -> Order {
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001","orderId":"o1","orderUpdateId":0,
        "nodes":[{{"nodeId":"{from}","sequenceId":0,"released":true,"actions":[]}},
                 {{"nodeId":"{to}","sequenceId":2,"released":true,"actions":[]}}],
        "edges":[{{"edgeId":"{edge}","sequenceId":1,"released":true,"actions":[]}}]}}"#
    );
    serde_json::from_str(&json).unwrap()
}

fn cancel() -> InstantActions {
    serde_json::from_str(
        r#"{"headerId":2,"timestamp":"2026-11-21T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001",
        "actions":[{"actionType":"cancelOrder","actionId":"c1","blockingType":"NONE"}]}"#,
    )
    .unwrap()
}

#[test]
fn without_a_delay_everything_is_instant() {
    let mut w = world(0);
    assert_eq!(w.message_delay(), 0);
    assert_eq!(
        w.send_order(&order("A", "A-B", "B")),
        Ok(Reception::Accepted)
    );
    assert_ne!(
        w.phase(&id()),
        Ok(Phase::Finished),
        "the robot has the order"
    );
    assert_eq!(w.state(&id()).unwrap().order_id, "o1");
}

#[test]
fn an_order_arrives_after_the_delay() {
    let delay = 10;
    let mut w = world(delay);
    assert_eq!(
        w.send_order(&order("A", "A-B", "B")),
        Ok(Reception::InTransit)
    );
    // on its way: for `delay` steps the robot knows nothing
    for step in 0..delay {
        assert_eq!(w.phase(&id()), Ok(Phase::Finished), "step {step}");
        w.step();
    }
    assert_eq!(w.phase(&id()), Ok(Phase::Finished));
    // the next step delivers it; the robot has the order now
    w.step();
    assert_ne!(w.phase(&id()), Ok(Phase::Finished));
}

#[test]
fn the_controller_reads_states_that_are_a_delay_old() {
    let delay = 8;
    let mut w = world(delay);
    w.send_order(&order("A", "A-B", "B")).unwrap();
    // R-001 is turning/driving as soon as the order is there; find the first step
    // at which the truth says so and the first step at which the state says so
    let mut truth_at = None;
    let mut seen_at = None;
    for step in 1..200u64 {
        w.step();
        let truth = w.phase(&id()).unwrap() != Phase::Finished;
        let seen = !w.state(&id()).unwrap().order_id.is_empty();
        if truth && truth_at.is_none() {
            truth_at = Some(step);
        }
        if seen && seen_at.is_none() {
            seen_at = Some(step);
        }
    }
    let (truth_at, seen_at) = (truth_at.unwrap(), seen_at.unwrap());
    assert_eq!(seen_at - truth_at, delay, "the state is {delay} steps old");
}

#[test]
fn an_instant_action_arrives_after_the_delay() {
    let delay = 10;
    let mut w = world(delay);
    w.send_order(&order("A", "A-B", "B")).unwrap();
    for _ in 0..(delay + 40) {
        w.step();
    }
    let x_before = w.pose(&id()).unwrap().x;
    // cancel: the robot goes on for `delay` more steps
    w.send_instant_actions(&cancel()).unwrap();
    let mut still_driving = 0;
    for _ in 0..(delay + 400) {
        w.step();
        if w.state(&id()).unwrap().driving {
            still_driving += 1;
        }
    }
    let x_after = w.pose(&id()).unwrap().x;
    assert!(
        x_after > x_before + 0.1,
        "it kept going: {x_before} -> {x_after}"
    );
    assert!(still_driving > 0);
    // and then it stopped at the next node (a cancelled order ends there)
    assert_eq!(w.phase(&id()), Ok(Phase::Finished));
}

#[test]
fn a_refused_order_shows_in_the_state_one_delay_after_it_arrived() {
    let delay = 6;
    let mut w = world(delay);
    // the robot is on A, the order starts on B: refused (START_NODE_OUT_OF_RANGE)
    assert_eq!(
        w.send_order(&order("B", "B-C", "C")),
        Ok(Reception::InTransit)
    );
    let mut first_seen = None;
    for step in 1..100u64 {
        w.step();
        if first_seen.is_none() && !w.state(&id()).unwrap().errors.is_empty() {
            first_seen = Some(step);
        }
    }
    // it arrives at step `delay + 1`; the state with the error is made then and
    // read `delay` steps later
    assert_eq!(first_seen, Some(2 * delay + 1));
}

#[test]
fn a_delayed_run_is_repeatable() {
    let run = || {
        let mut w = world(7);
        w.send_order(&order("A", "A-B", "B")).unwrap();
        (0..400)
            .map(|_| {
                w.step();
                let s = w.state(&id()).unwrap();
                (w.pose(&id()).unwrap().x.to_bits(), s.driving, s.order_id)
            })
            .collect::<Vec<_>>()
    };
    assert_eq!(run(), run());
}

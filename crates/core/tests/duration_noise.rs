use botsim_core::actions::{ActionSpec, ActionTimes};
use botsim_core::disturbances::DisturbanceError;
use botsim_core::graph::Graph;
use botsim_core::jobs::Rng;
use botsim_core::robot::Reception::Accepted;
use botsim_core::spec::RobotSpec;
use botsim_core::world::{RobotId, World, WorldError};
use vda5050::common::BlockingType;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vda5050::state::ActionStatus::Running;
use vdma_lif::Lif;

const DT: f64 = 0.05;

fn pick() -> ActionSpec {
    ActionSpec {
        action_id: "a".into(),
        action_type: "pick".into(),
        blocking: BlockingType::Hard,
    }
}

fn durations(t: &mut ActionTimes, n: usize) -> Vec<f64> {
    (0..n).map(|_| t.plan(&pick()).duration).collect()
}

#[test]
fn without_noise_every_action_takes_its_base_time() {
    let mut t = ActionTimes::new(1.0).with("pick", 4.0);
    assert!(durations(&mut t, 20).iter().all(|d| *d == 4.0));
    // variation 0 is the same
    let mut t = ActionTimes::new(1.0)
        .with("pick", 4.0)
        .with_noise(Rng::new(1), 0.0);
    assert!(durations(&mut t, 20).iter().all(|d| *d == 4.0));
}

#[test]
fn noise_adds_between_zero_and_the_variation_and_spreads() {
    let mut t = ActionTimes::new(1.0)
        .with("pick", 10.0)
        .with_noise(Rng::new(1), 0.2);
    let d = durations(&mut t, 2000);
    assert!(d.iter().all(|x| (10.0..12.0).contains(x)), "{d:?}");
    let mean = d.iter().sum::<f64>() / d.len() as f64;
    assert!((mean - 11.0).abs() < 0.1, "mean {mean}");
    assert!(d.iter().any(|x| *x < 10.3) && d.iter().any(|x| *x > 11.7));
}

#[test]
fn every_order_gets_its_own_stream_that_does_not_depend_on_earlier_orders() {
    let master = |seed| ActionTimes::new(1.0).with_noise(Rng::new(seed), 0.5);

    // order 1 uses 1 action in one run and 7 in another; order 2 is the same in both
    let (mut a, mut b) = (master(9), master(9));
    let mut a1 = a.for_next_order();
    let mut b1 = b.for_next_order();
    durations(&mut a1, 1);
    durations(&mut b1, 7);
    let a2 = durations(&mut a.for_next_order(), 3);
    let b2 = durations(&mut b.for_next_order(), 3);
    assert_eq!(a2, b2);

    // two orders of one robot do not repeat each other
    let mut m = master(9);
    let first = durations(&mut m.for_next_order(), 3);
    let second = durations(&mut m.for_next_order(), 3);
    assert_ne!(first, second);
    // and another seed differs
    assert_ne!(first, durations(&mut master(10).for_next_order(), 3));
}

#[test]
fn without_noise_the_times_for_the_next_order_are_a_plain_copy() {
    let mut t = ActionTimes::new(2.0).with("pick", 3.0);
    assert_eq!(t.for_next_order(), t);
}

// ---- a robot and a world ----

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn id(n: &str) -> RobotId {
    RobotId::new("ACME", n)
}

/// A→B with a HARD "pick" at B; `order_id` and `update` as given.
fn order(serial: &str, order_id: &str, from: &str, edge: &str, to: &str) -> Order {
    serde_json::from_str(&format!(
        r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"{serial}","orderId":"{order_id}","orderUpdateId":0,
        "nodes":[{{"nodeId":"{from}","sequenceId":0,"released":true,"actions":[]}},
                 {{"nodeId":"{to}","sequenceId":2,"released":true,
                   "actions":[{{"actionId":"pick","actionType":"pick","blockingType":"HARD"}}]}}],
        "edges":[{{"edgeId":"{edge}","sequenceId":1,"released":true,"actions":[]}}]}}"#
    ))
    .unwrap()
}

/// A world with R-1 on A and R-2 on B; every action takes 2 s.
fn world(variation: Option<f64>, enable_first: bool) -> World {
    let g = graph();
    let (a, b) = (g.node_idx("A").unwrap(), g.node_idx("B").unwrap());
    let mut w = World::new(g, DT, 0).unwrap();
    if let (Some(v), true) = (variation, enable_first) {
        w.enable_duration_variation(5, v).unwrap();
    }
    for (n, node) in [("R-1", a), ("R-2", b)] {
        w.add_robot(id(n), spec(), ActionTimes::new(2.0), node, 0.0)
            .unwrap();
    }
    if let (Some(v), false) = (variation, enable_first) {
        w.enable_duration_variation(5, v).unwrap();
    }
    w
}

/// How many ticks the robot's "pick" runs.
fn pick_ticks(w: &mut World, robot: &str) -> u32 {
    let mut n = 0;
    for _ in 0..6000 {
        w.step();
        let s = w.state(&id(robot)).unwrap();
        if s.action_states
            .iter()
            .any(|a| a.action_id == "pick" && a.action_status == Running)
        {
            n += 1;
        }
    }
    n
}

#[test]
fn a_robot_acts_between_the_base_time_and_the_base_time_plus_the_variation() {
    let mut plain = world(None, true);
    assert_eq!(
        plain.send_order(&order("R-1", "o1", "A", "A-B", "B")),
        Ok(Accepted)
    );
    assert_eq!(pick_ticks(&mut plain, "R-1"), 40, "2 s at 50 ms");

    // 50 % variation: 2 s … 3 s = 40 … 60 ticks; look at 20 different orders
    let mut w = world(Some(0.5), true);
    let mut seen = Vec::new();
    // R-2 on B goes B→C and back is not possible (one-way); use two robots, one order each
    for (robot, from, edge, to) in [("R-1", "A", "A-B", "B"), ("R-2", "B", "B-C", "C")] {
        assert_eq!(
            w.send_order(&order(robot, "o", from, edge, to)),
            Ok(Accepted)
        );
        seen.push(pick_ticks(&mut w, robot));
    }
    assert!(seen.iter().all(|t| (40..=61).contains(t)), "{seen:?}");
    assert!(seen.iter().any(|t| *t > 40), "some extra time: {seen:?}");
}

#[test]
fn the_same_seed_gives_the_same_durations_whenever_the_variation_is_switched_on() {
    let run = |enable_first| {
        let mut w = world(Some(0.5), enable_first);
        w.send_order(&order("R-1", "o", "A", "A-B", "B")).unwrap();
        pick_ticks(&mut w, "R-1")
    };
    assert_eq!(run(true), run(true));
    assert_eq!(
        run(true),
        run(false),
        "robots added before or after enabling"
    );
}

#[test]
fn bad_variations_are_refused() {
    let mut w = World::new(graph(), DT, 0).unwrap();
    for v in [-0.1, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            w.enable_duration_variation(1, v),
            Err(WorldError::Disturbance(
                DisturbanceError::BadDurationVariation(_)
            ))
        ));
    }
}

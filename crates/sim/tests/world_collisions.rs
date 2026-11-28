use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_sim::actions::ActionTimes;
use botsim_sim::world::World;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

mod vda_port;
use vda_port::VdaWorld;

const DT: f64 = 0.05;

fn spec() -> RobotSpec {
    // 1.2 m long, 0.8 m wide
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn id(serial: &str) -> RobotId {
    RobotId::new("ACME", serial)
}

/// Layout A(0,0) – B(10,0) – C(10,5); robots (serial, node).
fn world(robots: &[(&str, &str)]) -> World {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    let g = Graph::from_lif(&lif).unwrap();
    let nodes: Vec<_> = robots.iter().map(|(_, n)| g.node_idx(n).unwrap()).collect();
    let mut w = World::new(g, DT, 0).unwrap();
    for ((serial, _), n) in robots.iter().zip(nodes) {
        w.add_robot(id(serial), spec(), ActionTimes::new(1.0), n, 0.0)
            .unwrap();
    }
    w
}

fn go(w: &mut World, serial: &str, from: &str, edge: &str, to: &str, order_id: &str) {
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"{serial}","orderId":"{order_id}","orderUpdateId":0,
        "nodes":[{{"nodeId":"{from}","sequenceId":0,"released":true,"actions":[]}},
                 {{"nodeId":"{to}","sequenceId":2,"released":true,"actions":[]}}],
        "edges":[{{"edgeId":"{edge}","sequenceId":1,"released":true,"actions":[]}}]}}"#
    );
    let o: Order = serde_json::from_str(&json).unwrap();
    assert_eq!(w.send_order(&o), Ok(botsim_sim::robot::Reception::Accepted));
}

#[test]
fn robots_far_apart_do_not_collide() {
    let mut w = world(&[("R-1", "A"), ("R-2", "B")]);
    for _ in 0..200 {
        w.step();
    }
    assert!(w.collisions().is_empty());
}

#[test]
fn driving_into_a_standing_robot_is_one_collision_at_the_right_moment() {
    let mut w = world(&[("R-1", "A"), ("R-2", "B")]);
    go(&mut w, "R-1", "A", "A-B", "B", "o1");
    let mut x_at_hit = None;
    for _ in 0..3000 {
        w.step();
        if x_at_hit.is_none() && !w.collisions().is_empty() {
            x_at_hit = Some(w.pose(&id("R-1")).unwrap().x);
        }
    }
    // both are 1.2 m long and face east: they touch at a centre distance of 1.2,
    // i.e. at x = 8.8; the overlap is seen on the first step past it (≤ 1.5 m/s × dt)
    let x = x_at_hit.expect("no collision found");
    assert!(
        (8.8..=8.8 + 1.5 * DT).contains(&x),
        "first overlap at x = {x}"
    );
    // R-1 ends on top of R-2 but that is still the same single contact
    assert!((w.pose(&id("R-1")).unwrap().x - 10.0).abs() < 1e-9);
    let c = w.collisions();
    assert_eq!(c.len(), 1, "{c:?}");
    assert_eq!((&c[0].a, &c[0].b), (&id("R-1"), &id("R-2")));
    assert!(c[0].tick > 0 && c[0].tick < w.tick());
}

#[test]
fn a_new_contact_after_separating_is_a_new_collision() {
    let mut w = world(&[("R-1", "A"), ("R-2", "B"), ("R-3", "C")]);
    go(&mut w, "R-1", "A", "A-B", "B", "o1");
    for _ in 0..3000 {
        w.step();
    }
    assert_eq!(w.collisions().len(), 1, "R-1 on R-2");

    // R-1 drives on to C, where R-3 stands: it leaves R-2, then meets R-3
    go(&mut w, "R-1", "B", "B-C", "C", "o2");
    for _ in 0..3000 {
        w.step();
    }
    let c = w.collisions();
    assert_eq!(c.len(), 2, "{c:?}");
    assert_eq!((&c[1].a, &c[1].b), (&id("R-1"), &id("R-3")));
    assert!(c[1].tick > c[0].tick);
}

#[test]
fn robots_standing_on_each_other_from_the_start_collide_at_the_first_step() {
    let mut w = world(&[("R-1", "A"), ("R-2", "A")]);
    assert!(
        w.collisions().is_empty(),
        "nothing is checked before a step"
    );
    w.step();
    assert_eq!(w.collisions().len(), 1);
    assert_eq!(w.collisions()[0].tick, 1);
    for _ in 0..50 {
        w.step();
    }
    assert_eq!(
        w.collisions().len(),
        1,
        "not again while they stay overlapped"
    );
}

#[test]
fn robots_on_different_maps_never_collide() {
    // two_levels: make N102 (level 2) sit exactly on N2 (level 1)
    let text = include_str!("data/two_levels.lif.json")
        .replace("\"x\": 12.4", "\"x\": 11.0")
        .replace("\"y\": 3.4", "\"y\": 0.0");
    let lif: Lif = serde_json::from_str(&text).unwrap();
    let g = Graph::from_lif(&lif).unwrap();
    let (n2, n102) = (g.node_idx("N2").unwrap(), g.node_idx("N102").unwrap());
    assert_ne!(g.nodes[n2].map, g.nodes[n102].map);
    assert_eq!(
        (g.nodes[n2].x, g.nodes[n2].y),
        (g.nodes[n102].x, g.nodes[n102].y)
    );

    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(id("R-1"), spec(), ActionTimes::new(1.0), n2, 0.0)
        .unwrap();
    w.add_robot(id("R-2"), spec(), ActionTimes::new(1.0), n102, 0.0)
        .unwrap();
    for _ in 0..10 {
        w.step();
    }
    assert!(w.collisions().is_empty());
}

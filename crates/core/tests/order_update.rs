use botsim_core::actions::ActionTimes;
use botsim_core::drive::{DriveError, Driver, Phase};
use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

mod vda_port;
use vda_port::route_from_order;

const DT: f64 = 0.05;

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

/// `nodes`: (id, sequenceId, released); `edges`: (id, sequenceId, released).
fn order(update_id: u32, nodes: &[(&str, u32, bool)], edges: &[(&str, u32, bool)]) -> Order {
    let n: Vec<String> = nodes
        .iter()
        .map(|(id, s, r)| {
            format!(r#"{{"nodeId":"{id}","sequenceId":{s},"released":{r},"actions":[]}}"#)
        })
        .collect();
    let e: Vec<String> = edges
        .iter()
        .map(|(id, s, r)| {
            format!(r#"{{"edgeId":"{id}","sequenceId":{s},"released":{r},"actions":[]}}"#)
        })
        .collect();
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001","orderId":"o1","orderUpdateId":{update_id},
        "nodes":[{}],"edges":[{}]}}"#,
        n.join(","),
        e.join(",")
    );
    serde_json::from_str(&json).unwrap()
}

/// Order A→B→C, only A-B released (B-C and C are the horizon).
fn first_order() -> Order {
    order(
        0,
        &[("A", 0, true), ("B", 2, true), ("C", 4, false)],
        &[("A-B", 1, true), ("B-C", 3, false)],
    )
}

/// The update that releases B-C and C.
fn update() -> Order {
    order(1, &[("B", 2, true), ("C", 4, true)], &[("B-C", 3, true)])
}

fn run_until_waiting(d: &mut Driver) -> usize {
    let mut steps = 0;
    while d.phase() != Phase::Waiting && d.phase() != Phase::Finished {
        d.step(DT);
        steps += 1;
        assert!(steps < 10_000);
    }
    steps
}

fn run_to_finish(d: &mut Driver) -> usize {
    let mut steps = 0;
    while !d.is_finished() {
        d.step(DT);
        steps += 1;
        assert!(steps < 10_000);
    }
    steps
}

#[test]
fn waits_at_the_end_of_the_released_base_until_an_update_arrives() {
    let g = graph();
    let r = route_from_order(&g, &first_order()).unwrap();
    assert!(r.has_horizon);
    let mut d = Driver::from_order_route(&g, &spec(), 0.0, &r, ActionTimes::new(0.0)).unwrap();

    run_until_waiting(&mut d);
    assert_eq!(d.phase(), Phase::Waiting);
    let p = d.pose();
    assert!((p.x - 10.0).abs() < 1e-9 && p.y.abs() < 1e-9, "stands at B");
    assert_eq!(d.speed(), 0.0);

    // it keeps waiting, not moving, not finished
    for _ in 0..200 {
        d.step(DT);
    }
    assert_eq!(d.phase(), Phase::Waiting);
    assert!(!d.is_finished());
    assert_eq!(d.pose(), p);

    // the update arrives: it drives on to C and finishes
    let u = route_from_order(&g, &update()).unwrap();
    assert!(!u.has_horizon);
    d.extend(&g, &u).unwrap();
    assert_ne!(d.phase(), Phase::Waiting);
    run_to_finish(&mut d);
    let p = d.pose();
    assert!((p.x - 10.0).abs() < 1e-9 && (p.y - 5.0).abs() < 1e-9);
}

#[test]
fn an_update_while_still_driving_gives_the_same_run_as_a_fully_released_order() {
    let g = graph();

    // reference: everything released from the start
    let full = order(
        0,
        &[("A", 0, true), ("B", 2, true), ("C", 4, true)],
        &[("A-B", 1, true), ("B-C", 3, true)],
    );
    let r = route_from_order(&g, &full).unwrap();
    let mut d = Driver::from_order_route(&g, &spec(), 0.0, &r, ActionTimes::new(0.0)).unwrap();
    let reference = run_to_finish(&mut d);

    // update arrives after 3 s, while the robot is still on A-B
    let r = route_from_order(&g, &first_order()).unwrap();
    let mut d = Driver::from_order_route(&g, &spec(), 0.0, &r, ActionTimes::new(0.0)).unwrap();
    for _ in 0..60 {
        d.step(DT);
    }
    assert_eq!(d.phase(), Phase::Driving);
    d.extend(&g, &route_from_order(&g, &update()).unwrap())
        .unwrap();
    let rest = run_to_finish(&mut d);
    assert_eq!(60 + rest, reference);
}

#[test]
fn an_update_that_does_not_start_at_the_decision_point_is_refused() {
    let g = graph();
    let r = route_from_order(&g, &first_order()).unwrap();
    let mut d = Driver::from_order_route(&g, &spec(), 0.0, &r, ActionTimes::new(0.0)).unwrap();
    run_until_waiting(&mut d);

    // starts at C, but the base ends at B
    let wrong = order(1, &[("C", 4, true), ("B", 6, true)], &[("C-B", 5, true)]);
    let err = d
        .extend(&g, &route_from_order(&g, &wrong).unwrap())
        .unwrap_err();
    assert_eq!(
        err,
        DriveError::NotContinuation {
            expected: "B".into(),
            found: "C".into()
        }
    );
    // unchanged: still waiting at B, and the right update still works
    assert_eq!(d.phase(), Phase::Waiting);
    d.extend(&g, &route_from_order(&g, &update()).unwrap())
        .unwrap();
    run_to_finish(&mut d);
}

#[test]
fn an_update_can_release_only_a_bit_more_and_keep_a_horizon() {
    // three-step chain A→B→C→... needs only the two edges we have: release B-C
    // but keep the order open (horizon after C) — robot waits again at C
    let g = graph();
    let r = route_from_order(&g, &first_order()).unwrap();
    let mut d = Driver::from_order_route(&g, &spec(), 0.0, &r, ActionTimes::new(0.0)).unwrap();
    run_until_waiting(&mut d);

    // C-B is the horizon edge of this update; B is released again at the end
    let u = order(
        1,
        &[("B", 2, true), ("C", 4, true), ("B", 6, false)],
        &[("B-C", 3, true), ("C-B", 5, false)],
    );
    let u = route_from_order(&g, &u).unwrap();
    assert!(u.has_horizon);
    d.extend(&g, &u).unwrap();
    run_until_waiting(&mut d);
    assert_eq!(d.phase(), Phase::Waiting);
    let p = d.pose();
    assert!(
        (p.x - 10.0).abs() < 1e-9 && (p.y - 5.0).abs() < 1e-9,
        "waits at C"
    );
}

#[test]
fn an_order_without_a_horizon_finishes_and_one_with_an_empty_base_waits_at_once() {
    let g = graph();
    let full = order(0, &[("A", 0, true), ("B", 2, true)], &[("A-B", 1, true)]);
    let r = route_from_order(&g, &full).unwrap();
    let mut d = Driver::from_order_route(&g, &spec(), 0.0, &r, ActionTimes::new(0.0)).unwrap();
    run_until_waiting(&mut d);
    assert_eq!(d.phase(), Phase::Finished);

    // only the first node released: nothing to drive, waits for an update
    let empty_base = order(0, &[("A", 0, true), ("B", 2, false)], &[("A-B", 1, false)]);
    let r = route_from_order(&g, &empty_base).unwrap();
    let d = Driver::from_order_route(&g, &spec(), 0.0, &r, ActionTimes::new(0.0)).unwrap();
    assert_eq!(d.phase(), Phase::Waiting);
}

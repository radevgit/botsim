use botsim_core::actions::ActionTimes;
use botsim_core::disturbances::{DisturbanceError, StopGenerator, StopSpec};
use botsim_core::robot::Reception::Accepted;
use botsim_core::world::{World, WorldError};
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

mod vda_port;
use vda_port::VdaWorld;

const DT: f64 = 0.05;

fn spec(per_hour: f64) -> StopSpec {
    StopSpec {
        per_hour,
        min_seconds: 5.0,
        max_seconds: 15.0,
    }
}

// ---- the generator ----

#[test]
fn stops_follow_the_rate_and_the_duration_range() {
    // 36 stops per hour while moving: mean 36 in one hour of 50 ms steps
    let mut g = StopGenerator::new(1, 0, spec(36.0), DT).unwrap();
    let stops: Vec<f64> = (0..72_000).filter_map(|_| g.step()).collect();
    assert!((12..=60).contains(&stops.len()), "{} stops", stops.len());
    assert!(stops.iter().all(|s| (5.0..=15.0).contains(s)));
    let (lo, hi) = (
        stops.iter().copied().fold(f64::MAX, f64::min),
        stops.iter().copied().fold(f64::MIN, f64::max),
    );
    assert!(lo < 7.0 && hi > 13.0, "durations spread over {lo} .. {hi}");

    // rate 0: never
    let mut g = StopGenerator::new(1, 0, spec(0.0), DT).unwrap();
    assert!((0..72_000).all(|_| g.step().is_none()));
}

#[test]
fn a_fixed_duration_is_allowed_and_seeds_and_robots_make_other_stops() {
    let fixed = StopSpec {
        per_hour: 3600.0,
        min_seconds: 7.0,
        max_seconds: 7.0,
    };
    let mut g = StopGenerator::new(1, 0, fixed, DT).unwrap();
    assert!((0..2000).filter_map(|_| g.step()).all(|s| s == 7.0));

    let trace = |seed, robot| {
        let mut g = StopGenerator::new(seed, robot, spec(600.0), DT).unwrap();
        (0..5000).map(|_| g.step()).collect::<Vec<_>>()
    };
    assert_eq!(trace(1, 0), trace(1, 0));
    assert_ne!(trace(1, 0), trace(2, 0), "other seed");
    assert_ne!(trace(1, 0), trace(1, 1), "other robot");
}

#[test]
fn bad_settings_are_refused() {
    let bad_rate = |r| StopGenerator::new(1, 0, spec(r), DT).unwrap_err();
    for r in [-1.0, f64::NAN, f64::INFINITY] {
        assert!(matches!(bad_rate(r), DisturbanceError::BadStopRate(_)));
    }
    // more than one stop per step on average
    assert!(matches!(bad_rate(1.0e6), DisturbanceError::BadStopRate(_)));

    for (min, max) in [
        (-1.0, 5.0),
        (10.0, 5.0),
        (f64::NAN, 5.0),
        (1.0, f64::INFINITY),
    ] {
        let s = StopSpec {
            per_hour: 1.0,
            min_seconds: min,
            max_seconds: max,
        };
        assert!(matches!(
            StopGenerator::new(1, 0, s, DT),
            Err(DisturbanceError::BadStopDuration { .. })
        ));
    }
}

// ---- in the world ----

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn factsheet_spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn id(n: &str) -> RobotId {
    RobotId::new("ACME", n)
}

/// Robots R-1 on A and R-2 on B, stops enabled with `per_hour` before or after adding them.
fn world(per_hour: f64, enable_first: bool) -> World {
    let g = graph();
    let (a, b) = (g.node_idx("A").unwrap(), g.node_idx("B").unwrap());
    let mut w = World::new(g, DT, 0).unwrap();
    if enable_first {
        w.enable_random_stops(3, spec(per_hour)).unwrap();
    }
    for (n, node) in [("R-1", a), ("R-2", b)] {
        w.add_robot(id(n), factsheet_spec(), ActionTimes::new(1.0), node, 0.0)
            .unwrap();
    }
    if !enable_first {
        w.enable_random_stops(3, spec(per_hour)).unwrap();
    }
    w
}

/// R-1: A → B → C.
fn send_job(w: &mut World) {
    let o: Order = serde_json::from_str(
        r#"{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"A","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"B","sequenceId":2,"released":true,"actions":[]},
                 {"nodeId":"C","sequenceId":4,"released":true,"actions":[]}],
        "edges":[{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]},
                 {"edgeId":"B-C","sequenceId":3,"released":true,"actions":[]}]}"#,
    )
    .unwrap();
    // R-2 stands on B: send the order to a world without that robot there is not
    // possible, so R-1 drives alone; R-2 is not on the route's way (it is on B!)
    assert_eq!(w.send_order(&o), Ok(Accepted));
}

fn finish_tick(w: &mut World) -> u64 {
    for _ in 0..30_000 {
        w.step();
        let s = w.state(&id("R-1")).unwrap();
        if s.last_node_id == "C" && !s.driving {
            return w.tick();
        }
    }
    panic!("R-1 did not reach C");
}

#[test]
fn robots_that_stand_still_are_never_stopped() {
    let mut w = world(3600.0, true);
    for _ in 0..12_000 {
        w.step(); // ten minutes, nobody has an order
    }
    assert_eq!(w.random_stops(), 0);
}

#[test]
fn a_moving_robot_is_stopped_and_arrives_later_and_the_world_counts_the_stops() {
    let plain = {
        let mut w = world(0.0, true);
        send_job(&mut w);
        finish_tick(&mut w)
    };
    let mut w = world(1800.0, true); // on average one stop every 2 s of driving
    send_job(&mut w);
    let disturbed = finish_tick(&mut w);
    assert!(w.random_stops() >= 1, "{} stops", w.random_stops());
    // each stop lasts at least 5 s
    assert!(
        (disturbed - plain) as f64 * DT >= 5.0 * w.random_stops() as f64 * 0.5,
        "{plain} → {disturbed}"
    );
}

#[test]
fn robots_added_after_enabling_get_stops_too_and_the_run_is_repeatable() {
    let run = |first| {
        let mut w = world(1800.0, first);
        send_job(&mut w);
        let t = finish_tick(&mut w);
        (t, w.random_stops())
    };
    let (t, n) = run(true);
    assert!(n >= 1);
    assert_eq!((t, n), run(true), "same seed, same stops");
    assert_eq!(
        (t, n),
        run(false),
        "enabled after adding the robots: the same"
    );
}

#[test]
fn enabling_with_bad_numbers_is_an_error() {
    let mut w = World::new(graph(), DT, 0).unwrap();
    assert!(matches!(
        w.enable_random_stops(1, spec(-5.0)),
        Err(WorldError::Disturbance(DisturbanceError::BadStopRate(_)))
    ));
}

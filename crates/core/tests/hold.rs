use botsim_core::actions::ActionTimes;
use botsim_core::graph::Graph;
use botsim_core::metrics::is_waiting;
use botsim_core::robot::Robot;
use botsim_core::spec::RobotSpec;
use botsim_core::state_report::Header;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vda5050::state::State;
use vdma_lif::Lif;

const DT: f64 = 0.05;

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn robot() -> Robot {
    let g = graph();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    Robot::new(
        &g,
        RobotSpec::from_factsheet(&f).unwrap(),
        ActionTimes::new(2.0),
        g.node_idx("A").unwrap(),
        0.0,
    )
    .unwrap()
}

/// A→B→C (10 m east, then 5 m north), fully released; no actions.
fn order() -> Order {
    serde_json::from_str(
        r#"{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"A","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"B","sequenceId":2,"released":true,"actions":[]},
                 {"nodeId":"C","sequenceId":4,"released":true,"actions":[]}],
        "edges":[{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]},
                 {"edgeId":"B-C","sequenceId":3,"released":true,"actions":[]}]}"#,
    )
    .unwrap()
}

fn state(r: &Robot) -> State {
    r.state(
        &graph(),
        &Header {
            header_id: 1,
            timestamp: "2026-11-20T10:00:00.000Z".into(),
            version: "3.0.1".into(),
            manufacturer: "ACME".into(),
            serial_number: "R-001".into(),
        },
    )
}

/// Ticks until the order is done (nothing ahead, not moving), with an optional
/// hold of `seconds` after `after` ticks.
fn ticks_to_finish(hold: Option<(u64, f64)>) -> u64 {
    let g = graph();
    let mut r = robot();
    r.receive_order(&g, &order());
    let mut t = 0;
    loop {
        if let Some((after, seconds)) = hold
            && t == after
        {
            r.hold(seconds);
        }
        r.step(DT);
        t += 1;
        let s = state(&r);
        if s.node_states.is_empty() && !s.driving {
            return t;
        }
        assert!(t < 10_000, "never finished");
    }
}

#[test]
fn a_hold_while_driving_brakes_stands_still_and_then_goes_on() {
    let g = graph();
    let mut r = robot();
    r.receive_order(&g, &order());
    for _ in 0..60 {
        r.step(DT); // 3 s: well on the way to B at speed
    }
    let v_before = state(&r).velocity.unwrap().vx.unwrap();
    assert!(v_before > 0.5);

    r.hold(5.0);
    let mut x_prev = r.pose().x;
    let mut braking = 0;
    // it brakes without going backwards and without a jump in speed
    let mut v_prev = v_before;
    loop {
        r.step(DT);
        let s = state(&r);
        assert!(s.safety_state.field_violation, "held: field violation");
        assert!(
            !is_waiting(&s),
            "a held robot is not waiting for the controller"
        );
        let v = s.velocity.unwrap().vx.unwrap();
        assert!(v <= v_prev + 1e-12 && v >= 0.0);
        assert!(
            v_prev - v <= 0.8 * DT + 1e-9,
            "decelerates at most at 0.8 m/s²"
        );
        assert!(r.pose().x >= x_prev);
        x_prev = r.pose().x;
        v_prev = v;
        braking += 1;
        if !s.driving {
            break;
        }
    }
    assert!(braking > 5, "braking takes time: {braking} steps");

    // now stopped: it does not move for the rest of the hold
    let x_stopped = r.pose().x;
    let mut ticks_held = braking;
    while state(&r).safety_state.field_violation {
        assert!(!state(&r).driving);
        r.step(DT);
        ticks_held += 1;
    }
    assert_eq!(r.pose().x, x_stopped);
    // the hold lasted 5 s from the call (100 ticks)
    assert!((99..=101).contains(&ticks_held), "{ticks_held}");
    // and the robot drives on
    r.step(DT);
    assert!(state(&r).driving && !state(&r).safety_state.field_violation);
}

#[test]
fn a_hold_delays_the_order_by_at_least_its_length() {
    let plain = ticks_to_finish(None);
    let held = ticks_to_finish(Some((60, 5.0)));
    let delay = (held - plain) as f64 * DT;
    // 5 s standing, plus braking and accelerating again
    assert!(delay >= 5.0, "delay {delay} s");
    assert!(delay < 5.0 + 6.0, "delay {delay} s");
}

#[test]
fn a_hold_while_turning_freezes_the_heading() {
    let g = graph();
    let mut r = robot();
    r.receive_order(&g, &order());
    // drive to B and wait until the turn towards C has started
    while state(&r).velocity.unwrap().omega.unwrap() == 0.0 {
        r.step(DT);
        assert!(r.pose().x <= 10.0 + 1e-9, "turn not found");
    }
    r.step(DT);
    let h0 = r.pose().heading;
    r.hold(3.0);
    for _ in 0..50 {
        r.step(DT);
        let s = state(&r);
        assert_eq!(r.pose().heading, h0, "no turning while held");
        assert_eq!(s.velocity.unwrap().omega, Some(0.0));
        assert!(s.safety_state.field_violation && !s.driving);
    }
    // after the hold it turns on
    for _ in 0..30 {
        r.step(DT);
    }
    assert_ne!(r.pose().heading, h0);
    assert!(!state(&r).safety_state.field_violation);
}

#[test]
fn a_hold_has_no_effect_on_a_robot_that_stands_still() {
    let mut r = robot(); // idle on A
    r.hold(5.0);
    assert!(!state(&r).safety_state.field_violation);
    assert!(
        is_waiting(&state(&r)),
        "idle and not held: waiting by the rule (the caller decides if it has a job)"
    );
    // a hold runs down with time even if nothing moves: after 5.1 s of standing
    // still, a new order is driven at once
    for _ in 0..102 {
        r.step(DT);
        assert!(!state(&r).safety_state.field_violation);
    }
    let g = graph();
    r.receive_order(&g, &order());
    r.step(DT);
    let s = state(&r);
    assert!(s.driving && !s.safety_state.field_violation);
}

#[test]
fn a_longer_hold_is_kept_and_a_shorter_one_does_not_shorten_it() {
    let g = graph();
    let mut r = robot();
    r.receive_order(&g, &order());
    for _ in 0..40 {
        r.step(DT);
    }
    r.hold(4.0);
    r.hold(1.0); // does not shorten
    let mut ticks = 0;
    while {
        r.step(DT);
        ticks += 1;
        state(&r).safety_state.field_violation
    } {}
    assert!(ticks >= 79, "{ticks} steps held, expected about 80");
}

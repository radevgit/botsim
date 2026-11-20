use botsim_core::motion::{Along, EdgeGoal, advance};
use botsim_core::spec::Limits;

const DT: f64 = 0.05;

fn limits() -> Limits {
    Limits {
        max_speed: 1.5,
        max_acceleration: 0.5,
        max_deceleration: 0.8,
        max_angular_speed: 1.0,
    }
}

/// Run until arrival; return all states (including the start) and the time taken.
fn run(start: Along, goal: EdgeGoal, limits: &Limits) -> (Vec<Along>, f64) {
    let mut states = vec![start];
    let mut st = start;
    for _ in 0..100_000 {
        let (next, arrived) = advance(st, goal, limits, DT);
        states.push(next);
        st = next;
        if arrived {
            return (states.clone(), (states.len() - 1) as f64 * DT);
        }
    }
    panic!("did not arrive");
}

/// Speed and position stay within limits at every step. The last step is
/// excluded from the deceleration check (speed is set to `v_end` on arrival).
fn check_invariants(states: &[Along], goal: EdgeGoal, l: &Limits) {
    let eps = 1e-9;
    for w in states.windows(2) {
        let (a, b) = (w[0], w[1]);
        assert!(b.s >= a.s, "position went backwards");
        assert!(b.s <= goal.length + eps, "overshoot");
        // Above the limit is only allowed while still slowing down towards it.
        assert!(
            b.v <= goal.v_limit.min(l.max_speed) + eps || b.v <= a.v,
            "too fast"
        );
        assert!(
            b.v - a.v <= l.max_acceleration * DT + eps,
            "accelerates too hard"
        );
    }
    for w in states[..states.len() - 1].windows(2) {
        assert!(
            w[0].v - w[1].v <= l.max_deceleration * DT + eps,
            "brakes too hard: {} -> {}",
            w[0].v,
            w[1].v
        );
    }
}

#[test]
fn long_edge_trapezoid_and_stop() {
    let goal = EdgeGoal {
        length: 10.0,
        v_limit: 1.5,
        v_end: 0.0,
    };
    let (states, t) = run(Along { s: 0.0, v: 0.0 }, goal, &limits());
    check_invariants(&states, goal, &limits());

    // Analytic: 3 s accelerating (2.25 m) + 6.34375 m cruising + 1.875 s braking (1.40625 m).
    let expected = 3.0 + 6.34375 / 1.5 + 1.875;
    assert!((t - expected).abs() < 0.25, "t = {t}, expected {expected}");

    let last = states.last().unwrap();
    assert_eq!((last.s, last.v), (10.0, 0.0));
    // The speed just before the stop is small (no big jump at the snap).
    assert!(states[states.len() - 2].v < 0.1);
    // It reached the speed limit.
    let vmax = states.iter().map(|s| s.v).fold(0.0, f64::max);
    assert!((vmax - 1.5).abs() < 1e-9);
}

#[test]
fn short_edge_triangle_never_reaches_limit() {
    let goal = EdgeGoal {
        length: 1.0,
        v_limit: 1.5,
        v_end: 0.0,
    };
    let (states, t) = run(Along { s: 0.0, v: 0.0 }, goal, &limits());
    check_invariants(&states, goal, &limits());

    // Analytic peak speed sqrt(2 L / (1/a + 1/d)) = 0.784 m/s, time v/a + v/d = 2.55 s.
    let vmax = states.iter().map(|s| s.v).fold(0.0, f64::max);
    assert!(vmax < 0.8 && vmax > 0.7, "peak {vmax}");
    assert!((t - 2.55).abs() < 0.25, "t = {t}");
}

#[test]
fn edge_limit_lower_than_robot_limit() {
    let goal = EdgeGoal {
        length: 10.0,
        v_limit: 0.6,
        v_end: 0.0,
    };
    let (states, _) = run(Along { s: 0.0, v: 0.0 }, goal, &limits());
    check_invariants(&states, goal, &limits());
    let vmax = states.iter().map(|s| s.v).fold(0.0, f64::max);
    assert!((vmax - 0.6).abs() < 1e-9);
}

#[test]
fn drives_on_at_speed_into_next_edge() {
    let goal = EdgeGoal {
        length: 10.0,
        v_limit: 1.5,
        v_end: 1.0,
    };
    let (states, _) = run(Along { s: 0.0, v: 0.0 }, goal, &limits());
    check_invariants(&states, goal, &limits());
    let last = states.last().unwrap();
    assert_eq!((last.s, last.v), (10.0, 1.0));
    // Speed just before arrival is close to v_end.
    assert!((states[states.len() - 2].v - 1.0).abs() < 0.1);
}

#[test]
fn slows_down_when_started_faster_than_the_limit() {
    let goal = EdgeGoal {
        length: 10.0,
        v_limit: 0.5,
        v_end: 0.0,
    };
    let (states, _) = run(Along { s: 0.0, v: 1.5 }, goal, &limits());
    check_invariants(&states, goal, &limits());
    // Needs (1.5 - 0.5) / 0.8 = 1.25 s at the deceleration limit before it is within the limit.
    let first_ok = states.iter().position(|s| s.v <= 0.5 + 1e-9).unwrap();
    assert!(((first_ok as f64) * DT - 1.25).abs() < 0.1);
}

#[test]
fn already_at_the_end() {
    let goal = EdgeGoal {
        length: 2.0,
        v_limit: 1.0,
        v_end: 0.0,
    };
    let (st, arrived) = advance(Along { s: 2.0, v: 0.0 }, goal, &limits(), DT);
    assert!(arrived);
    assert_eq!((st.s, st.v), (2.0, 0.0));
}

mod rotation {
    use botsim_core::motion::{angle_diff, normalize_angle, rotate};
    use std::f64::consts::{FRAC_PI_2, PI};

    const DT: f64 = 0.05;

    fn turn(mut h: f64, target: f64, rate: f64) -> (f64, usize) {
        for i in 1..=10_000 {
            let (nh, done) = rotate(h, target, rate, DT);
            h = nh;
            if done {
                return (h, i);
            }
        }
        panic!("did not arrive");
    }

    #[test]
    fn normalises_angles() {
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-12);
        assert!((normalize_angle(-PI) - PI).abs() < 1e-12);
        assert!((normalize_angle(2.5 * PI) - FRAC_PI_2).abs() < 1e-12);
        assert!((normalize_angle(-2.5 * PI) + FRAC_PI_2).abs() < 1e-12);
        assert_eq!(normalize_angle(0.3), 0.3);
    }

    #[test]
    fn takes_the_shortest_way() {
        // 170° -> -170° is 20° counter-clockwise, not 340° clockwise.
        let from = 170f64.to_radians();
        let to = (-170f64).to_radians();
        assert!((angle_diff(from, to) - 20f64.to_radians()).abs() < 1e-12);
        let (h, _) = rotate(from, to, 1.0, DT);
        assert!(angle_diff(from, h) > 0.0);
        // and the other direction
        let (h, _) = rotate(to, from, 1.0, DT);
        assert!(angle_diff(to, h) < 0.0);
    }

    #[test]
    fn time_matches_angle_over_rate() {
        // 90° at 1 rad/s: 1.5708 s -> 32 steps of 0.05 s.
        let (h, steps) = turn(0.0, FRAC_PI_2, 1.0);
        assert_eq!(steps, 32);
        assert!((h - FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn never_overshoots_and_stops_exactly_on_target() {
        let (h, _) = turn(0.1, -2.9, 0.7);
        assert!((h - normalize_angle(-2.9)).abs() < 1e-12);
        // already there: arrives at once
        assert_eq!(rotate(1.0, 1.0, 0.5, DT), (1.0, true));
        // target given as a different turn of the same angle
        let (h, done) = rotate(0.0, 2.0 * PI, 0.5, DT);
        assert!(done && h.abs() < 1e-12);
    }
}

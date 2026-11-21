use std::f64::consts::{FRAC_PI_2, PI};

use botsim_core::drive::{DriveError, Driver, Phase, RouteEdge};
use botsim_core::graph::Graph;
use botsim_core::orientation::OrientationError;
use botsim_core::spec::{Kinematics, RobotSpec};
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

const DT: f64 = 0.05;

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_layout(&lif.origins[0].layouts[0]).unwrap()
}

fn spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

/// Edges A-B, B-C of the test layout.
fn route(g: &Graph) -> Vec<RouteEdge> {
    vec![
        g.edge_idx("A-B").unwrap().into(),
        g.edge_idx("B-C").unwrap().into(),
    ]
}

/// Time of one straight edge with a stop at the end, from the analytic
/// trapezoid/triangle profile (a = 0.5, d = 0.8, vmax = 1.5).
fn edge_time(l: f64) -> f64 {
    let (a, d, vmax) = (0.5, 0.8, 1.5);
    let ramp = vmax * vmax / (2.0 * a) + vmax * vmax / (2.0 * d);
    if l >= ramp {
        vmax / a + vmax / d + (l - ramp) / vmax
    } else {
        let v = (2.0 * l / (1.0 / a + 1.0 / d)).sqrt();
        v / a + v / d
    }
}

#[test]
fn drives_a_to_b_to_c() {
    let g = graph();
    let mut d = Driver::new(&g, &spec(), g.node_idx("A").unwrap(), 0.0, &route(&g)).unwrap();

    let mut steps = 0;
    let mut turned_at_b = false;
    let mut last = d.pose();
    let mut last_phase = d.phase();
    while !d.is_finished() {
        d.step(DT);
        steps += 1;
        assert!(steps < 10_000, "never finished");
        assert!(d.speed() <= 1.5 + 1e-9);

        let p = d.pose();
        if d.phase() == Phase::Rotating {
            // turning happens at B with the robot standing still; the step that
            // ends the drive and starts the turn still moved, so skip that one
            assert_eq!(d.speed(), 0.0);
            assert!((p.x - 10.0).abs() < 1e-9 && p.y.abs() < 1e-9);
            if last_phase == Phase::Rotating {
                assert_eq!((p.x, p.y), (last.x, last.y));
            }
            turned_at_b = true;
        }
        last = p;
        last_phase = d.phase();
    }

    assert!(turned_at_b);
    let p = d.pose();
    assert!((p.x - 10.0).abs() < 1e-9 && (p.y - 5.0).abs() < 1e-9);
    assert!((p.heading - FRAC_PI_2).abs() < 1e-9);
    assert_eq!(d.speed(), 0.0);

    // two edges + a 90° turn at 1 rad/s
    let expected = edge_time(10.0) + FRAC_PI_2 + edge_time(5.0);
    let t = steps as f64 * DT;
    assert!((t - expected).abs() < 0.5, "t = {t}, expected {expected}");
}

#[test]
fn turns_first_if_the_start_heading_is_wrong() {
    let g = graph();
    let mut d = Driver::new(&g, &spec(), g.node_idx("A").unwrap(), PI, &route(&g)).unwrap();
    assert_eq!(d.phase(), Phase::Rotating);
    d.step(DT);
    assert_eq!((d.pose().x, d.pose().y), (0.0, 0.0));
    assert!(d.pose().heading.abs() < PI); // moved away from π
}

#[test]
fn empty_route_is_finished_at_once() {
    let g = graph();
    let d = Driver::new(&g, &spec(), g.node_idx("A").unwrap(), 0.0, &[]).unwrap();
    assert!(d.is_finished());
}

#[test]
fn rejects_unconnected_route_and_unsupported_kinematics() {
    let g = graph();
    let start = g.node_idx("A").unwrap();
    // B-C does not start at A
    let err = Driver::new(
        &g,
        &spec(),
        start,
        0.0,
        &[g.edge_idx("B-C").unwrap().into()],
    )
    .unwrap_err();
    assert_eq!(err, DriveError::NotConnected { edge: "B-C".into() });

    // a car-like robot needs curved paths, which are not modelled (S63: the
    // omnidirectional robot is, see omnidirectional.rs)
    let mut s = spec();
    s.kinematics = Kinematics::ThreeWheel;
    let err = Driver::new(&g, &s, start, 0.0, &route(&g)).unwrap_err();
    assert_eq!(
        err,
        DriveError::UnsupportedKinematics(Kinematics::ThreeWheel)
    );
}

#[test]
fn pose_has_the_map_of_the_start_node_and_map_transitions_are_rejected() {
    let g = graph();
    let d = Driver::new(&g, &spec(), g.node_idx("A").unwrap(), 0.0, &route(&g)).unwrap();
    assert_eq!(g.maps[d.pose().map], "M");

    // same layout with node C on another map
    let mut lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    lif.origins[0].layouts[0].nodes[2].map_id = "M2".into();
    let g2 = Graph::from_layout(&lif.origins[0].layouts[0]).unwrap();
    let err = Driver::new(&g2, &spec(), g2.node_idx("A").unwrap(), 0.0, &route(&g2)).unwrap_err();
    assert_eq!(err, DriveError::MapTransition { edge: "B-C".into() });
}

/// Drive A-B (10 m east) once; return the number of steps, the heading at the
/// end, whether the robot ever turned on the spot, and the final pose.
fn drive_a_b(start_heading: f64, orientation: Option<f64>) -> (usize, bool, f64) {
    let g = graph();
    let ab = g.edge_idx("A-B").unwrap();
    let re = match orientation {
        Some(o) => RouteEdge::tangential(ab, o),
        None => RouteEdge::new(ab),
    };
    let mut d = Driver::new(&g, &spec(), g.node_idx("A").unwrap(), start_heading, &[re]).unwrap();
    let mut steps = 0;
    let mut turned = d.phase() == Phase::Rotating;
    while !d.is_finished() {
        d.step(DT);
        steps += 1;
        turned |= d.phase() == Phase::Rotating;
        assert!(steps < 10_000);
    }
    let p = d.pose();
    assert!((p.x - 10.0).abs() < 1e-9 && p.y.abs() < 1e-9);
    (steps, turned, p.heading)
}

#[test]
fn backwards_edge_with_robot_facing_backwards_needs_no_turn() {
    // robot faces west (π); the edge goes east; orientation π = drive backwards
    let (steps_back, turned, heading) = drive_a_b(PI, Some(PI));
    assert!(!turned, "must not rotate");
    assert!(
        (heading.abs() - PI).abs() < 1e-9,
        "still faces west: {heading}"
    );

    // same time as a normal forward drive of the same edge
    let (steps_fwd, turned_fwd, _) = drive_a_b(0.0, Some(0.0));
    assert!(!turned_fwd);
    assert_eq!(steps_back, steps_fwd);
}

#[test]
fn forward_edge_with_robot_facing_backwards_turns_first() {
    let (steps, turned, heading) = drive_a_b(PI, Some(0.0));
    assert!(turned);
    assert!(heading.abs() < 1e-9, "faces east at the end: {heading}");
    // 180° at 1 rad/s = about 3.14 s extra
    let (steps_fwd, _, _) = drive_a_b(0.0, Some(0.0));
    let extra = (steps - steps_fwd) as f64 * DT;
    assert!((extra - PI).abs() < 0.1, "extra {extra}");
}

#[test]
fn no_orientation_means_forwards_even_if_the_robot_faces_backwards() {
    // decision S13
    let (_, turned, heading) = drive_a_b(PI, None);
    assert!(turned);
    assert!(heading.abs() < 1e-9);
}

#[test]
fn backwards_robot_turns_at_node_towards_the_backwards_heading() {
    // A→B east, then B→C north driven backwards: at B the robot must end up facing south
    let g = graph();
    let route = [
        RouteEdge::new(g.edge_idx("A-B").unwrap()),
        RouteEdge::tangential(g.edge_idx("B-C").unwrap(), PI),
    ];
    let mut d = Driver::new(&g, &spec(), g.node_idx("A").unwrap(), 0.0, &route).unwrap();
    while !d.is_finished() {
        d.step(DT);
    }
    let p = d.pose();
    assert!((p.x - 10.0).abs() < 1e-9 && (p.y - 5.0).abs() < 1e-9);
    assert!(
        (p.heading + FRAC_PI_2).abs() < 1e-9,
        "faces south: {}",
        p.heading
    );
}

#[test]
fn unreachable_orientation_is_reported_with_the_edge() {
    let g = graph();
    let route = [RouteEdge::tangential(g.edge_idx("A-B").unwrap(), FRAC_PI_2)];
    let err = Driver::new(&g, &spec(), g.node_idx("A").unwrap(), 0.0, &route).unwrap_err();
    assert_eq!(
        err,
        DriveError::Orientation {
            edge: "A-B".into(),
            error: OrientationError::OnlyForwardsOrBackwards {
                orientation: FRAC_PI_2
            }
        }
    );
}

#[test]
fn zero_length_and_non_finite_edges_are_rejected() {
    // C on top of B: edge B-C has zero length
    let mut lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    lif.origins[0].layouts[0].nodes[2].node_position.x = 10.0;
    lif.origins[0].layouts[0].nodes[2].node_position.y = 0.0;
    let g = Graph::from_lif(&lif).unwrap();
    let err = Driver::new(&g, &spec(), g.node_idx("A").unwrap(), 0.0, &route(&g)).unwrap_err();
    assert_eq!(err, DriveError::ZeroLengthEdge { edge: "B-C".into() });

    // coordinates so large that the length overflows to infinity
    let mut lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    lif.origins[0].layouts[0].nodes[2].node_position.x = f64::MAX;
    lif.origins[0].layouts[0].nodes[2].node_position.y = f64::MAX;
    let g = Graph::from_lif(&lif).unwrap();
    assert!(g.edges[g.edge_idx("B-C").unwrap()].length.is_infinite());
    let err = Driver::new(&g, &spec(), g.node_idx("A").unwrap(), 0.0, &route(&g)).unwrap_err();
    assert_eq!(err, DriveError::ZeroLengthEdge { edge: "B-C".into() });
}

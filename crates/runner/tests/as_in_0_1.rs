//! Inputs that botsim 0.1 ran, that a controller connected through VDA 5050
//! cannot be given, and that a direct connection runs again as 0.1 did: nothing
//! is translated. (A zone set with an empty id: `botsim`, `tests/untranslated.rs`.)

use botsim_model::angle::normalize_angle;
use botsim_model::graph::{EdgeOrientation, Graph};
use botsim_model::job::Job;
use botsim_model::spec::RobotSpec;
use botsim_sim::actions::ActionTimes;
use botsim_sim::world::World;

mod setup;
use setup::{DT, graph, id, line, robot_spec};

/// The robot of [`robot_spec`] with the kinematics `kinematics`, as its factsheet
/// says it (from which the controller learns it).
fn spec(kinematics: &str) -> RobotSpec {
    let mut factsheet = robot_spec().factsheet;
    let text = serde_json::to_string(&factsheet)
        .unwrap()
        .replace("DIFFERENTIAL", kinematics);
    factsheet = serde_json::from_str(&text).unwrap();
    RobotSpec::from_factsheet(&factsheet).unwrap()
}

/// The robot ACME/R-001 on the first node of `g` does a job from `pickup` to
/// `drop`; the world after 120 s.
fn job_done(g: Graph, spec: RobotSpec, start_ms: i64, pickup: &str, drop: &str) -> World {
    let node = |n: &str| g.node_idx(n).unwrap();
    let job = Job {
        id: 1,
        pickup: node(pickup),
        drop: node(drop),
        created_tick: 0,
    };
    let mut w = World::new(g, DT, start_ms).unwrap();
    w.add_robot(id(1), spec, ActionTimes::new(1.0), 0, 0.0)
        .unwrap();
    let mut c = botsim::connect::controller(&w);
    let mut jobs = Some(vec![job]);
    for _ in 0..(120.0 / DT) as usize {
        botsim::connect::step(&mut c, &mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    assert_eq!((c.completed().len(), c.rejected().len()), (1, 0));
    let state = w.reports().remove(0).state;
    assert!(state.errors.is_empty(), "{:?}", state.errors);
    w
}

#[test]
fn a_run_past_the_year_9999_goes_on() {
    // the year 11476, which a VDA 5050 timestamp cannot say
    let w = job_done(line(), robot_spec(), 300_000_000_000_000, "N1", "N2");
    assert_eq!(w.pose(&id(1)).unwrap().x, 20.0);
}

#[test]
fn a_node_with_an_empty_id_is_a_node_like_any_other() {
    let g = graph(
        &[
            ("N0", "M", 0.0, 0.0),
            ("", "M", 10.0, 0.0),
            ("N2", "M", 20.0, 0.0),
            ("N3", "M", 30.0, 0.0),
        ],
        &[("N0", ""), ("", "N2"), ("N2", "N3")],
    );
    let empty = g.node_idx("").unwrap();
    let mut w = job_done(g, robot_spec(), 0, "", "N3");
    assert_eq!(w.pose(&id(1)).unwrap().x, 30.0);
    // and it is reported as one
    let node = w.reports().remove(0).state.route.last_node().unwrap().node;
    assert_ne!(node, empty);
    assert_eq!(w.graph().nodes[node].id, "N3");
}

#[test]
fn an_edge_orientation_outside_the_range_of_a_message_goes_as_it_is() {
    let mut g = line();
    for edge in &mut g.edges {
        for props in &mut edge.types {
            props.orientation = Some(EdgeOrientation::Tangential(4.0));
        }
    }
    let w = job_done(g, spec("OMNIDIRECTIONAL"), 0, "N1", "N2");
    // 4 rad from the direction of the edges, which run along x
    assert_eq!(w.pose(&id(1)).unwrap().heading, normalize_angle(4.0));
}

//! The shipped example `scenarios/mixed_fleet.toml`: two robot types on one
//! layout (S56). It must stay working, and the per-type rules of the layout must
//! hold in the run.

use std::cell::RefCell;
use std::path::Path;
use std::rc::Rc;

use botsim::scenario::Scenario;
use botsim::summary;
use botsim_core::graph::Graph;
use botsim_core::observer::{Observer, Snapshot};

#[derive(Debug)]
struct Recorder(Rc<RefCell<Vec<Snapshot>>>);

impl Observer for Recorder {
    fn on_layout(&mut self, _: &Graph) {}
    fn on_step(&mut self, s: &Snapshot) {
        self.0.borrow_mut().push(s.clone());
    }
}

#[test]
fn the_summary_names_each_robots_type_and_counts_its_finished_jobs() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    let text = std::fs::read_to_string(dir.join("mixed_fleet.toml")).unwrap();
    let s = Scenario::from_toml(&text).unwrap();
    let r = summary::run(&s, s.build_world(&dir).unwrap()).unwrap();
    let types: Vec<_> = r
        .robots
        .iter()
        .map(|b| (b.serial_number.as_str(), b.series_name.as_str()))
        .collect();
    assert_eq!(types, [("X-001", "X1"), ("X-002", "X1"), ("Y-001", "Y2")]);
    // every finished job was done by exactly one robot, and every robot did some
    let total: usize = r.robots.iter().map(|b| b.jobs_done).sum();
    assert_eq!(total, r.jobs.done);
    assert!(r.robots.iter().all(|b| b.jobs_done > 0), "{:?}", r.robots);

    // without a [jobs] section nothing is done
    let no_jobs = text.split("[jobs]").next().unwrap();
    let s = Scenario::from_toml(no_jobs).unwrap();
    let r = summary::run(&s, s.build_world(&dir).unwrap()).unwrap();
    assert!(r.robots.iter().all(|b| b.jobs_done == 0));
}

#[test]
fn a_mixed_fleet_keeps_to_what_its_types_may_use() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    let path = dir.join("mixed_fleet.toml");
    let s = Scenario::from_toml(&std::fs::read_to_string(&path).unwrap()).unwrap();
    let mut world = s.build_world(&dir).unwrap();
    let log = Rc::new(RefCell::new(Vec::new()));
    world
        .set_observer_at(Box::new(Recorder(log.clone())), 0.5)
        .unwrap();
    let r = summary::run(&s, world).unwrap();

    assert_eq!(r.collision_count, 0);
    assert_eq!(r.deadlock_count, 0);
    assert_eq!(r.jobs.rejected, 0);
    assert!(r.jobs.done >= 40, "{:?}", r.jobs);

    // robots in the order of the scenario: X-001, X-002, Y-001
    let log = log.borrow();
    assert!(log.len() > 1000);
    let on_cross_aisle =
        |p: &botsim_core::drive::Pose| (p.x - 20.0).abs() < 1e-6 && p.y > 0.5 && p.y < 19.5;
    let on_ramp =
        |p: &botsim_core::drive::Pose| (p.x - 40.0).abs() < 1e-6 && p.y > 0.5 && p.y < 19.5;
    let mut x_on_cross_aisle = false;
    let mut y_top_speed_on_ramp: f64 = 0.0;
    let mut x_top_speed_on_ramp: f64 = 0.0;
    for snap in log.iter() {
        for (i, robot) in snap.robots.iter().enumerate() {
            let is_y = i == 2;
            if is_y {
                assert!(
                    !on_cross_aisle(&robot.pose),
                    "the forklift is on the cross aisle at {:.1} s",
                    snap.time
                );
                if on_ramp(&robot.pose) {
                    y_top_speed_on_ramp = y_top_speed_on_ramp.max(robot.speed.abs());
                }
            } else {
                x_on_cross_aisle |= on_cross_aisle(&robot.pose);
                if on_ramp(&robot.pose) {
                    x_top_speed_on_ramp = x_top_speed_on_ramp.max(robot.speed.abs());
                }
            }
        }
    }
    assert!(x_on_cross_aisle, "the small robots use the cross aisle");
    assert!(
        y_top_speed_on_ramp > 0.3 && y_top_speed_on_ramp <= 0.5 + 1e-6,
        "the forklift's speed on the ramp: {y_top_speed_on_ramp}"
    );
    assert!(
        x_top_speed_on_ramp > 1.0,
        "the small robots are not limited on the ramp: {x_top_speed_on_ramp}"
    );
}

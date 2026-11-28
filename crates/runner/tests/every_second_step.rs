//! A server cycle in every second step of the world only: the inbox holds the
//! reports of both steps, and the jobs of both. The jobs of the example
//! scenarios are done, without collisions.

use std::path::{Path, PathBuf};

use botsim::connect::{self, Connected};
use botsim::scenario::Scenario;
use botsim::traffic_setup;
use botsim_model::job::Job;
use botsim_model::time::Millis;
use botsim_server::policy::TrafficPolicy;
use botsim_sim::world::World;

fn file(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(name)
}

fn load(name: &str) -> Scenario {
    Scenario::load(&file(name)).unwrap()
}

fn world(scenario: &Scenario) -> World {
    scenario.build_world(&file("")).unwrap()
}

/// What became of a run.
struct Outcome {
    created: usize,
    done: usize,
    /// Jobs a robot still has at the end.
    busy: usize,
}

/// Run `scenario` in `world` with a cycle in every second step: jobs appear for
/// `jobs_seconds`, and the run ends when all are done, or after `seconds`. No
/// robots collide or lock each other, no job is refused or left without a robot.
fn run<P: TrafficPolicy>(
    scenario: &Scenario,
    mut world: World,
    mut c: Connected<P>,
    jobs_seconds: f64,
    seconds: f64,
) -> Outcome {
    let mut generator = scenario.job_generator(world.graph()).unwrap().unwrap();
    c.set_load_weight(scenario.route_load_weight);
    let steps = |s: f64| (s / scenario.time_step).round() as u64;
    let mut waiting: Vec<Job> = Vec::new();
    let mut created = 0;
    while world.tick() < steps(seconds) {
        if world.tick() < steps(jobs_seconds) {
            waiting.extend(generator.step(Millis(world.now_ms())));
        }
        if world.tick().is_multiple_of(2) {
            created += waiting.len();
            connect::step(&mut c, &mut world, waiting.drain(..));
        } else {
            connect::collect(&mut c, &mut world);
        }
        world.step();
        if world.tick() > steps(jobs_seconds) && c.completed().len() == created {
            break;
        }
    }
    assert!(world.collisions().is_empty(), "{:?}", world.collisions());
    assert!(world.deadlocks().is_empty(), "{:?}", world.deadlocks());
    assert!(c.rejected().is_empty(), "{:?}", c.rejected());
    assert_eq!(c.pending().count(), 0);
    let outcome = Outcome {
        created,
        done: c.completed().len(),
        busy: c.busy(),
    };
    assert_eq!(outcome.done + outcome.busy, created);
    outcome
}

/// The scenarios of two robots have no parking place: after the last job a free
/// robot stays where it is, and may stand in the way of the other's job (with a
/// cycle in every step too).
fn all_done_but_one_a_free_robot_blocks(outcome: Outcome, at_least: usize) {
    assert!(outcome.busy <= 1, "{} jobs unfinished", outcome.busy);
    assert!(
        outcome.done > at_least,
        "{} of {} jobs done",
        outcome.done,
        outcome.created
    );
}

#[test]
fn the_jobs_on_a_ring_with_zones_are_done() {
    let s = load("ring_jobs.toml");
    let w = world(&s);
    let c = connect::controller(&w);
    all_done_but_one_a_free_robot_blocks(run(&s, w, c, 1200.0, 1800.0), 10);
}

#[test]
fn the_jobs_on_a_ring_are_done_with_a_message_delay() {
    let mut s = load("ring_jobs.toml");
    s.message_delay = Some(0.5);
    let w = world(&s);
    let c = connect::controller(&w);
    all_done_but_one_a_free_robot_blocks(run(&s, w, c, 1200.0, 1800.0), 10);
}

#[test]
fn the_jobs_over_a_lift_are_done() {
    let s = load("lift_jobs.toml");
    let w = world(&s);
    let c = connect::controller(&w);
    all_done_but_one_a_free_robot_blocks(run(&s, w, c, 1200.0, 2400.0), 5);
}

#[test]
fn the_jobs_of_twelve_robots_under_the_traffic_rules_are_done() {
    let s = load("m1_short.toml");
    let w = world(&s);
    let adapter = traffic_setup::adapter(&s, &file("m1_short.toml")).unwrap();
    let c = connect::with_policy(&w, adapter);
    let outcome = run(&s, w, c, 900.0, 2400.0);
    assert_eq!(outcome.busy, 0, "of {} jobs", outcome.created);
    assert!(outcome.done > 10, "{} jobs done", outcome.done);
}

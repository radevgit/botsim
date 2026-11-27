//! Scale test: many robots on a big one-way grid, measured in real time (S52).
//!
//! ```sh
//! cargo run --release -p botsim --example scale -- [robots] [side] [sim_seconds] [jobs_per_robot_per_hour] [observer_seconds]
//! ```
//!
//! Defaults: 100 robots, a 30 × 30 grid, 60 simulated seconds, 20 jobs per robot
//! per hour, no observer (`observer_seconds` > 0 attaches an observer that gets a
//! snapshot every that many simulated seconds). The layout is a Manhattan grid with one-way streets (even rows run
//! east, odd rows west, even columns run south, odd columns north; `side` must be
//! even), 6 m between nodes. The example writes the layout and a scenario file
//! into `target/scale/`, runs them like the CLI does and prints the real time.

use std::cell::Cell;
use std::fmt::Write as _;
use std::path::Path;
use std::rc::Rc;
use std::time::Instant;

use botsim::scenario::Scenario;
use botsim::summary;
use botsim_core::observer::{Observer, Snapshot};
use botsim_model::graph::Graph;

const FACTSHEET: &str = include_str!("../../../scenarios/diff_robot.factsheet.json");

/// An observer that counts the snapshots it gets.
#[derive(Debug)]
struct Counter(Rc<Cell<u64>>);

impl Observer for Counter {
    fn on_layout(&mut self, _: &Graph) {}
    fn on_step(&mut self, s: &Snapshot) {
        // touch the data, so that building it cannot be optimised away
        std::hint::black_box(s);
        self.0.set(self.0.get() + 1);
    }
}

fn node(r: usize, c: usize) -> String {
    format!("N{r}_{c}")
}

fn layout(side: usize) -> String {
    let mut nodes = Vec::new();
    for r in 0..side {
        for c in 0..side {
            nodes.push(format!(
                r#"{{"nodeId":"{}","mapId":"M","nodePosition":{{"x":{},"y":{}}},
                "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]}}]}}"#,
                node(r, c),
                c as f64 * 6.0,
                r as f64 * 6.0
            ));
        }
    }
    let edge = |a: String, b: String| {
        format!(
            r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}",
            "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}],
            "orientationType":"TANGENTIAL","mobileRobotOrientations":[0.0]}}]}}"#
        )
    };
    // The outer streets form a clockwise ring (top row east, right column south,
    // bottom row west, left column north); without it two corners would be dead ends.
    let last = side - 1;
    let mut edges = Vec::new();
    for r in 0..side {
        for c in 0..side - 1 {
            // even rows run east, odd rows west (the first and last row: the ring)
            edges.push(if r % 2 == 0 {
                edge(node(r, c), node(r, c + 1))
            } else {
                edge(node(r, c + 1), node(r, c))
            });
        }
    }
    for c in 0..side {
        for r in 0..side - 1 {
            // even columns run south (increasing r), odd columns north; the last
            // column south and the first column north (the ring)
            let south = if c == 0 {
                false
            } else if c == last {
                true
            } else {
                c % 2 == 0
            };
            edges.push(if south {
                edge(node(r, c), node(r + 1, c))
            } else {
                edge(node(r + 1, c), node(r, c))
            });
        }
    }
    format!(
        r#"{{"metaInformation":{{"lifId":"grid","creator":"scale","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{}],"edges":[{}]}}]}}]}}"#,
        nodes.join(","),
        edges.join(",")
    )
}

fn main() {
    let arg = |i: usize, default: f64| {
        std::env::args()
            .nth(i)
            .map_or(default, |s| s.parse().expect("a number"))
    };
    let robots = arg(1, 100.0) as usize;
    let side = arg(2, 30.0) as usize;
    let seconds = arg(3, 60.0);
    let rate = arg(4, 20.0);
    // simulated seconds between observer snapshots; 0 = no observer
    let observer_every = arg(5, 0.0);
    // 1 = robots drive through nodes (the default of the scenario), 0 = they stop
    // at every node
    let drive_through = arg(6, 1.0) != 0.0;
    // 1 = end the run when the first deadlock is reported and print where its
    // robots stand
    let stop_at_deadlock = arg(7, 0.0) != 0.0;
    assert!(side >= 2 && side.is_multiple_of(2), "side must be even");
    assert!(robots <= side * side, "more robots than nodes");

    // robots spread evenly over the nodes
    let stride = side * side / robots;
    let starts: Vec<String> = (0..robots)
        .map(|i| {
            let k = i * stride;
            format!("\"{}\"", node(k / side, k % side))
        })
        .collect();
    let mut job_nodes = String::new();
    for r in 0..side {
        for c in 0..side {
            write!(job_nodes, "\"{}\",", node(r, c)).unwrap();
        }
    }

    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/scale");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("grid.lif.json"), layout(side)).unwrap();
    std::fs::write(dir.join("diff_robot.factsheet.json"), FACTSHEET).unwrap();
    let toml = format!(
        r#"
name = "scale"
seed = 1
duration = {seconds}
time_step = 0.05
start_time = 2026-11-21T10:00:00Z
lif = "grid.lif.json"
drive_through_nodes = {drive_through}
stop_at_first_deadlock = {stop_at_deadlock}

[[robot]]
manufacturer = "ACME"
serial_prefix = "R"
factsheet = "diff_robot.factsheet.json"
start_nodes = [{}]
action_time = 1.0

[robot.action_times]
pick = 5.0
drop = 5.0

[jobs]
rate_per_hour = {}
nodes = [{job_nodes}]
"#,
        starts.join(","),
        rate * robots as f64
    );

    let t = Instant::now();
    let scenario = Scenario::from_toml(&toml).unwrap();
    let mut world = scenario.build_world(&dir).unwrap();
    // an observer that only counts (what a viewer would pay at least)
    let observed = Rc::new(Cell::new(0_u64));
    if observer_every > 0.0 {
        world
            .set_observer_at(Box::new(Counter(observed.clone())), observer_every)
            .unwrap();
    }
    let built = t.elapsed();
    let t = Instant::now();
    let r = summary::run_with(&scenario, world, true).unwrap();
    let run = t.elapsed();

    let steps = r.ticks as f64;
    let timing = r.timing.as_ref().unwrap();
    println!(
        "robots {robots}, grid {side}x{side} ({} nodes), {seconds} s simulated = {} steps",
        side * side,
        r.ticks
    );
    println!("build {:.2?}", built);
    println!(
        "run {:.2?}  = {:.3} ms per step ({:.1}x real time)",
        run,
        run.as_secs_f64() * 1e3 / steps,
        seconds / run.as_secs_f64()
    );
    println!(
        "controller {:.1} ms total, {:.3} ms per step (max {:.3} ms); the rest is the world",
        timing.controller_total_ms,
        timing.controller_total_ms / steps,
        timing.controller_max_us.unwrap_or(0.0) / 1e3
    );
    if observer_every > 0.0 {
        println!(
            "observer every {observer_every} s: {} snapshots",
            observed.get()
        );
    }
    println!(
        "jobs created {} done {} in progress {} waiting {} rejected {}; collisions {}, deadlocks {}",
        r.jobs.created,
        r.jobs.done,
        r.jobs.in_progress,
        r.jobs.waiting_for_robot,
        r.jobs.rejected,
        r.collision_count,
        r.deadlock_count
    );
    println!("re-routes: {}", r.jobs.rerouted);
    // how many robots are in each deadlock (2 = a swap, more = a longer cycle)
    let mut sizes = std::collections::BTreeMap::new();
    for d in &r.deadlocks {
        *sizes.entry(d.robots.len()).or_insert(0_usize) += 1;
    }
    println!("deadlocks by number of robots in the cycle: {sizes:?}");
    if stop_at_deadlock && let Some(d) = r.deadlocks.first() {
        println!(
            "the first deadlock, at {:.1} s: robots (x, y) and the node they last reached:",
            d.detected_time
        );
        for name in &d.robots {
            let robot = r
                .robots
                .iter()
                .find(|b| format!("{}/{}", b.manufacturer, b.serial_number) == *name)
                .expect("a robot of the deadlock");
            println!(
                "  {name}: ({:.1}, {:.1}) last node {}",
                robot.x, robot.y, robot.last_node_id
            );
        }
    }
}

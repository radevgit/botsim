//! Recordings that say what cannot be: changed bytes under check values made to
//! fit, and reports and jobs with numbers no site has. The replay reports a
//! difference or an error; it does not panic.

use std::io::{self, Write};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use botsim::scenario::Scenario;
use botsim::summary::{self, InputRecording};
use botsim::traffic_setup;
use botsim_link::state::{Ahead, LastNode, Route};
use botsim_model::checksum::Checksum;
use botsim_replay::{Cycle, Reader, Rules, Setup, Writer, replay};

/// A recording written into memory.
#[derive(Debug, Clone, Default)]
struct Memory(Arc<Mutex<Vec<u8>>>);

impl Write for Memory {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        self.0.lock().unwrap().extend_from_slice(bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

fn scenarios() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

/// The first `seconds` of an example scenario, recorded.
fn recording(name: &str, traffic: bool, seconds: f64) -> Vec<u8> {
    let file = scenarios().join(name);
    let mut scenario = Scenario::load(&file).unwrap();
    scenario.duration = seconds;
    // jobs from the start
    scenario.jobs.as_mut().unwrap().rate_per_hour = 3600.0;
    if traffic {
        // (the traffic module answers no zone request yet)
        scenario.zone_set = None;
    }
    let world = scenario.build_world(&scenarios()).unwrap();
    let lif = traffic_setup::layout_text(&scenario, &file).unwrap();
    let rules = if traffic {
        traffic_setup::rules(&scenario, &file, &lif).unwrap()
    } else {
        Rules::Baseline
    };
    let memory = Memory::default();
    let input = InputRecording {
        lif,
        rules,
        output: Box::new(memory.clone()),
    };
    let (_, written) = summary::run_recorded(&scenario, world, false, input).unwrap();
    written.unwrap();
    memory.0.lock().unwrap().clone()
}

/// Replay `file`; a panic fails the test with `what` was done to it.
fn survives(file: &[u8], what: impl Fn() -> String) {
    let result = catch_unwind(AssertUnwindSafe(|| {
        let _ = replay(file);
    }));
    assert!(result.is_ok(), "panic: {}", what());
}

/// Where the content of each record of `file` is: (start of the record, length of
/// the content).
fn records(file: &[u8]) -> Vec<(usize, usize)> {
    // the header: 12 bytes, the version with its length, the check value
    let mut at = 12 + 1 + usize::from(file[12]) + 8;
    let mut found = Vec::new();
    while at < file.len() {
        let length = u32::from_le_bytes(file[at + 1..at + 5].try_into().unwrap()) as usize;
        found.push((at, length));
        at += 5 + length + 8;
    }
    found
}

/// Make the check value of the record at `at` fit what the record says now.
fn fit(file: &mut [u8], at: usize, length: usize) {
    use std::hash::Hasher;
    let end = at + 5 + length;
    let mut sum = Checksum::new();
    sum.write(&file[at..at + 5]);
    sum.write(&file[at + 5..end]);
    let check = sum.finish().to_le_bytes();
    file[end..end + 8].copy_from_slice(&check);
}

/// Change bytes of the content of the records, one at a time, make the check value
/// fit and replay. In long records only every `stride`-th byte.
fn damage_bytes(file: &[u8]) {
    replay(file).unwrap();
    let all = records(file);
    for (number, &(at, length)) in all.iter().enumerate() {
        // the setup, the first cycles and some later ones
        if number > 12 && number % 9 != 0 {
            continue;
        }
        // (a setup is long and costs a controller each time)
        let tries = if number == 0 { 40 } else { 60 };
        let stride = (length / tries).max(1);
        // (the file up to this record is all the replay gets to see)
        let upto = at + 5 + length + 8;
        let mut changed = file[..upto].to_vec();
        for byte in (at + 5..at + 5 + length).step_by(stride) {
            let was = changed[byte];
            for new in [was ^ 1, was ^ 0x80, 0xFF, 0, 0x7F] {
                if new == was {
                    continue;
                }
                changed[byte] = new;
                fit(&mut changed, at, length);
                survives(&changed, || {
                    format!("record {number}, byte {} set to {new}", byte - at - 5)
                });
            }
            changed[byte] = was;
        }
    }
}

#[test]
fn changed_bytes_under_fitting_check_values_do_not_panic() {
    damage_bytes(&recording("ring_jobs.toml", false, 8.0));
    damage_bytes(&recording("lift_jobs.toml", false, 8.0));
    damage_bytes(&recording("ring_jobs.toml", true, 8.0));
}

fn read(file: &[u8]) -> (Setup, Vec<Cycle>) {
    let mut reader = Reader::new(file).unwrap();
    let setup = reader.setup().clone();
    let mut cycles = Vec::new();
    while let Some(cycle) = reader.next_cycle().unwrap() {
        cycles.push(cycle);
    }
    (setup, cycles)
}

/// A change of the inputs of one cycle.
type Change = (&'static str, fn(&mut Cycle));

/// `route` with its last node at `step`, if it can say so.
fn at_step(route: &Route, step: u32) -> Option<Route> {
    let (last, ahead, released) = route.parts();
    let node = last?.node;
    Route::new(Some(LastNode { node, step }), ahead.to_vec(), released).ok()
}

/// What no site has.
const IMPOSSIBLE: [Change; 8] = [
    ("a node far along the route", |c| {
        for r in &mut c.inbox.robots {
            if let Some(route) = at_step(&r.state.route, r.state.last_step() + 5) {
                r.state.route = route;
            }
        }
    }),
    ("a node very far along the route", |c| {
        for r in &mut c.inbox.robots {
            if let Some(route) = at_step(&r.state.route, 1_000_000_000) {
                r.state.route = route;
            }
        }
    }),
    ("nodes and edges the layout does not have", |c| {
        for r in &mut c.inbox.robots {
            let last = Some(LastNode {
                node: usize::MAX,
                step: r.state.last_step(),
            });
            let ahead = vec![Ahead {
                edge: usize::MAX,
                node: usize::MAX,
            }];
            r.state.route = Route::new(last, ahead, 1).unwrap();
        }
    }),
    ("an enormous position", |c| {
        for r in &mut c.inbox.robots {
            if let Some(p) = &mut r.state.position {
                (p.x, p.y, p.theta) = (1e308, -1e308, 1e308);
            }
        }
    }),
    ("a position that is no number", |c| {
        for r in &mut c.inbox.robots {
            if let Some(p) = &mut r.state.position {
                (p.map, p.x, p.y, p.theta) = (usize::MAX, f64::NAN, f64::INFINITY, f64::NAN);
            }
            r.state.charge = f64::NAN;
        }
    }),
    ("jobs outside the layout", |c| {
        let mut job = botsim_model::job::Job {
            id: u64::MAX,
            pickup: usize::MAX,
            drop: 0,
            created: c.now,
        };
        c.inbox.jobs.push(job);
        (job.pickup, job.drop) = (0, usize::MAX);
        c.inbox.jobs.push(job);
    }),
    ("lifts the site does not have", |c| {
        for lift in &mut c.inbox.lifts {
            lift.header.device.0 = u32::MAX;
            lift.map = usize::MAX;
            lift.car_door = u32::MAX;
        }
    }),
    ("the end of time", |c| c.now.0 = i64::MAX),
];

#[test]
fn reports_and_jobs_that_cannot_be_do_not_panic() {
    let files = [
        ("baseline", recording("ring_jobs.toml", false, 20.0)),
        ("lifts", recording("lift_jobs.toml", false, 20.0)),
        ("traffic", recording("ring_jobs.toml", true, 20.0)),
    ];
    for (name, file) in files {
        let (setup, cycles) = read(&file);
        // in one cycle at a time, early and late
        for k in (0..cycles.len()).step_by(13) {
            for (what, change) in IMPOSSIBLE {
                let mut changed = cycles[..=k].to_vec();
                change(&mut changed[k]);
                let mut bytes = Vec::new();
                let mut writer = Writer::new(&mut bytes, &setup).unwrap();
                changed.iter().for_each(|c| writer.cycle(c).unwrap());
                writer.finish().unwrap();
                survives(&bytes, || format!("{name}: cycle {k}: {what}"));
            }
        }
    }
}

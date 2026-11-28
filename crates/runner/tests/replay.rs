//! A run's server inputs are recorded and replayed without the simulator: every
//! cycle gives the same messages and leaves the same state, under both traffic
//! rules, with zones, a message delay, doors, lifts, charging, refused orders and
//! a robot that stops reporting. A changed input is reported at its cycle.

use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Output as Finished};
use std::sync::{Arc, Mutex};

use botsim::connect;
use botsim::scenario::Scenario;
use botsim::summary::{self, InputRecording, Summary};
use botsim::traffic_setup;
use botsim_dispatcher::ChargePolicy;
use botsim_model::time::Millis;
use botsim_replay::{Cycle, Dispatch, Reader, Recorder, Rules, Setup, Writer, replay};
use botsim_server::cycle::Inbox;
use botsim_server::policy::TrafficPolicy;
use botsim_sim::world::World;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Policy {
    Baseline,
    Traffic,
}

const BOTH: [Policy; 2] = [Policy::Baseline, Policy::Traffic];

/// A recording written into memory.
#[derive(Debug, Clone, Default)]
struct Memory(Arc<Mutex<Vec<u8>>>);

impl Memory {
    fn bytes(&self) -> Vec<u8> {
        self.0.lock().unwrap().clone()
    }
}

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

/// A new folder holding the files of the example scenarios.
fn folder(name: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join("replay")
        .join(name);
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    for entry in fs::read_dir(scenarios()).unwrap() {
        let path = entry.unwrap().path();
        if path.extension().is_some_and(|e| e == "json") {
            fs::copy(&path, dir.join(path.file_name().unwrap())).unwrap();
        }
    }
    dir
}

/// The example scenario `name`, its text changed by `edit`, in a folder of its own.
fn scenario_file(test: &str, name: &str, edit: impl FnOnce(String) -> String) -> PathBuf {
    let dir = folder(test);
    let text = fs::read_to_string(scenarios().join(name)).unwrap();
    let file = dir.join(name);
    fs::write(&file, edit(text)).unwrap();
    file
}

/// `text` with the run lasting `seconds`.
fn lasting(text: &str, seconds: u32) -> String {
    text.lines()
        .map(|line| {
            if line.starts_with("duration") {
                format!("duration = {seconds}.0\n")
            } else {
                format!("{line}\n")
            }
        })
        .collect()
}

/// `text` without its zones: the traffic module answers no zone request yet, so
/// under its rules the robots would stop at the first zone.
fn without_zones(text: &str) -> String {
    text.lines()
        .filter(|line| !line.starts_with("zone_set"))
        .map(|line| format!("{line}\n"))
        .collect()
}

/// `text` with `lines` among the settings of the scenario itself.
fn with_setting(text: &str, lines: &str) -> String {
    format!("{lines}\n{text}")
}

fn load(file: &Path) -> (Scenario, World) {
    let scenario = Scenario::load(file).unwrap();
    let world = scenario.build_world(file.parent().unwrap()).unwrap();
    (scenario, world)
}

fn rules(policy: Policy, scenario: &Scenario, file: &Path, lif: &str) -> Rules {
    match policy {
        Policy::Baseline => Rules::Baseline,
        Policy::Traffic => traffic_setup::rules(scenario, file, lif).unwrap(),
    }
}

/// Run the scenario with its server inputs recorded; the summary and the file.
fn recorded(file: &Path, policy: Policy) -> (Summary, Vec<u8>) {
    let (scenario, world) = load(file);
    let lif = traffic_setup::layout_text(&scenario, file).unwrap();
    let memory = Memory::default();
    let recording = InputRecording {
        rules: rules(policy, &scenario, file, &lif),
        lif,
        output: Box::new(memory.clone()),
    };
    let (summary, written) = summary::run_recorded(&scenario, world, false, recording).unwrap();
    written.unwrap();
    (summary, memory.bytes())
}

/// The same run without a recording.
fn unrecorded(file: &Path, policy: Policy) -> Summary {
    let (scenario, world) = load(file);
    match policy {
        Policy::Baseline => summary::run(&scenario, world),
        Policy::Traffic => {
            let adapter = traffic_setup::adapter(&scenario, file).unwrap();
            summary::run_with_policy(&scenario, world, Box::new(adapter))
        }
    }
    .unwrap()
}

/// The replay of `file` gives every cycle as recorded; how many there are.
fn replays(file: &[u8]) -> u64 {
    let outcome = replay(file).unwrap();
    assert!(outcome.is_same(), "{outcome:?}");
    outcome.cycles
}

/// Record the scenario under `policy`, check that recording changes nothing and
/// that the replay is the same; the summary and the file.
fn record_and_replay(file: &Path, policy: Policy) -> (Summary, Vec<u8>) {
    let (summary, bytes) = recorded(file, policy);
    assert_eq!(summary, unrecorded(file, policy), "{policy:?}");
    // (a cycle in every step and one at the end)
    assert_eq!(replays(&bytes), summary.ticks + 1, "{policy:?}");
    (summary, bytes)
}

#[test]
fn zones_and_a_message_delay_replay() {
    for policy in BOTH {
        let name = format!("delay-{policy:?}");
        let file = scenario_file(&name, "ring_jobs.toml", |t| {
            with_setting(&lasting(&t, 400), "message_delay = 0.5")
        });
        let (summary, _) = record_and_replay(&file, policy);
        assert!(summary.zone_set.is_some());
        assert!(summary.jobs.created > 0, "{:?}", summary.jobs);
        if policy == Policy::Baseline {
            assert!(summary.jobs.done > 0, "{:?}", summary.jobs);
        }
    }
}

/// A door on the first edge of the ring, `width` metres wide.
fn door(width: f64) -> String {
    let lif = fs::read_to_string(scenarios().join("ring.lif.json")).unwrap();
    let lif: serde_json::Value = serde_json::from_str(&lif).unwrap();
    let edge = lif["origins"][0]["layouts"][0]["edges"][0]["edgeId"]
        .as_str()
        .unwrap()
        .to_string();
    format!(
        "\n[[door]]\nid = \"D\"\nedges = [\"{edge}\"]\npassage_width = {width}\n\
         open_seconds = 2.0\nclose_seconds = 2.0\n"
    )
}

#[test]
fn a_door_replays() {
    for policy in BOTH {
        let name = format!("door-{policy:?}");
        let file = scenario_file(&name, "ring_jobs.toml", |t| {
            without_zones(&lasting(&t, 300)) + &door(3.0)
        });
        let (summary, _) = record_and_replay(&file, policy);
        assert_eq!(summary.doors.len(), 1);
        assert_eq!(summary.jobs.rejected, 0);
        assert!(summary.jobs.done > 0, "{policy:?}: {:?}", summary.jobs);
    }
}

#[test]
fn refused_orders_replay() {
    for policy in BOTH {
        // the robots refuse an order through a door narrower than they are
        let name = format!("refused-{policy:?}");
        let file = scenario_file(&name, "ring_jobs.toml", |t| {
            without_zones(&lasting(&t, 300)) + &door(0.2)
        });
        let (summary, _) = record_and_replay(&file, policy);
        assert!(summary.jobs.rejected > 0, "{:?}", summary.jobs);
    }
}

#[test]
fn lifts_replay() {
    for policy in BOTH {
        let name = format!("lift-{policy:?}");
        let file = scenario_file(&name, "lift_jobs.toml", |t| lasting(&t, 400));
        let (summary, _) = record_and_replay(&file, policy);
        assert_eq!(summary.lifts.len(), 1);
        assert!(summary.jobs.created > 0, "{:?}", summary.jobs);
    }
}

#[test]
fn charging_and_parking_of_twelve_robots_replay() {
    for policy in BOTH {
        let name = format!("charging-{policy:?}");
        let file = scenario_file(&name, "m1_short.toml", |t| {
            let batteries = "battery_wh = 100.0\ndrive_wh_per_m = 0.2\nstart_charge = [10.0, 40.0]";
            let t = t.replace("\nfactsheet = ", &format!("\n{batteries}\nfactsheet = "));
            with_setting(&lasting(&t, 200), "charger_power_w = 1500.0")
        });
        // one of the parking places becomes a charger
        let lif = file.with_file_name("m1.lif.json");
        let text = fs::read_to_string(&lif).unwrap();
        let mut json: serde_json::Value = serde_json::from_str(&text).unwrap();
        let stations = json["origins"][0]["layouts"][0]["stations"]
            .as_array_mut()
            .unwrap();
        let parking = stations
            .iter_mut()
            .find(|s| s["stationType"] == "parking")
            .unwrap();
        parking["stationType"] = "charger".into();
        fs::write(&lif, json.to_string()).unwrap();

        let (summary, _) = record_and_replay(&file, policy);
        let battery = summary.battery.unwrap();
        assert!(battery.charge_trips > 0, "{battery:?}");
        assert!(summary.parking.is_some_and(|p| p.trips > 0));
    }
}

/// The ring with two robots, its cycles run by hand: robot 2 reports for the first
/// time after 5 s, and robot 1 is silent from 60 s to 120 s. The file.
fn silent_robot(test: &str, policy: Policy) -> Vec<u8> {
    let name = format!("silent-{test}-{policy:?}");
    let file = scenario_file(&name, "ring_jobs.toml", |t| match policy {
        Policy::Baseline => t,
        Policy::Traffic => without_zones(&t),
    });
    let (scenario, mut world) = load(&file);
    let lif = traffic_setup::layout_text(&scenario, &file).unwrap();
    let setup = Setup {
        rules: rules(policy, &scenario, &file, &lif),
        lif,
        config: connect::config(&world),
        dispatch: Dispatch::Baseline,
        calls: Vec::new(),
    };
    let memory = Memory::default();
    let mut recorder = Recorder::new(memory.clone(), &setup).unwrap();
    let controller = botsim_replay::controller(&setup).unwrap();
    let mut connected = connect::with_controller(&world, |_| controller);
    let mut generator = scenario.job_generator(world.graph()).unwrap().unwrap();
    for tick in 0..4000u64 {
        let mut robots = world.reports();
        robots.retain(|r| match r.header.robot.index() {
            0 => !(1200..2400).contains(&tick),
            _ => tick >= 100,
        });
        let inbox = Inbox {
            robots,
            lifts: world.lift_reports(),
            jobs: generator.step(Millis(world.now_ms())).into_iter().collect(),
            // somebody watches now and then
            watched: tick % 7 == 0,
        };
        let now = Millis(world.now_ms());
        let outbox = recorder.step(&mut *connected, now, inbox);
        for message in &outbox.messages {
            connected.send(&mut world, message);
        }
        world.step();
    }
    assert!(
        !connected.completed().is_empty(),
        "{policy:?}: {} busy, {} waiting, {:?}",
        connected.busy(),
        connected.pending().count(),
        connected.rejected()
    );
    recorder.finish().unwrap();
    memory.bytes()
}

#[test]
fn a_robot_that_stops_reporting_replays() {
    for policy in BOTH {
        assert_eq!(replays(&silent_robot("replays", policy)), 4000);
    }
}

#[test]
fn the_same_run_gives_the_same_file() {
    // (every controller gets new hashed collections, with keys of their own)
    for policy in BOTH {
        let first = silent_robot("twice", policy);
        assert_eq!(silent_robot("twice", policy), first, "{policy:?}");
    }
}

#[test]
fn the_state_checksum_covers_the_rules_and_the_dispatcher() {
    let file = scenarios().join("ring_jobs.toml");
    let (scenario, world) = load(&file);
    let lif = traffic_setup::layout_text(&scenario, &file).unwrap();
    for policy in BOTH {
        let setup = Setup {
            rules: rules(policy, &scenario, &file, &lif),
            lif: lif.clone(),
            config: connect::config(&world),
            dispatch: Dispatch::Baseline,
            calls: Vec::new(),
        };
        let mut controller = botsim_replay::controller(&setup).unwrap();
        let at_start = controller.state_checksum();
        assert_eq!(controller.state_checksum(), at_start);
        // only the rules change: they hear of a robot standing on a node
        controller.policy_mut().idle(0, 0);
        let with_a_robot = controller.state_checksum();
        assert_ne!(with_a_robot, at_start, "{policy:?}");
        // only the dispatcher changes
        controller.set_charging(ChargePolicy {
            low: 25.0,
            high: 80.0,
            critical: 10.0,
        });
        assert_ne!(controller.state_checksum(), with_a_robot, "{policy:?}");
    }
}

/// The setup and the cycles of a recording.
fn read(file: &[u8]) -> (Setup, Vec<Cycle>) {
    let mut reader = Reader::new(file).unwrap();
    let setup = reader.setup().clone();
    let mut cycles = Vec::new();
    while let Some(cycle) = reader.next_cycle().unwrap() {
        cycles.push(cycle);
    }
    (setup, cycles)
}

fn write(setup: &Setup, cycles: &[Cycle]) -> Vec<u8> {
    let mut bytes = Vec::new();
    let mut writer = Writer::new(&mut bytes, setup).unwrap();
    for cycle in cycles {
        writer.cycle(cycle).unwrap();
    }
    writer.finish().unwrap();
    bytes
}

/// A change of the inputs of one cycle.
type Change = fn(&mut Cycle);

#[test]
fn a_changed_input_is_reported_at_its_cycle() {
    for policy in BOTH {
        let file = silent_robot("changed", policy);
        let (setup, cycles) = read(&file);
        // what is rewritten without a change is the file
        assert_eq!(write(&setup, &cycles), file);

        // a cycle in the middle with both robots, and one that brings a job
        let both = (3000..).find(|&k| cycles[k].inbox.robots.len() == 2);
        let both = both.unwrap();
        let job = cycles.iter().position(|c| !c.inbox.jobs.is_empty());
        let job = job.unwrap();
        let changes: [(&str, usize, Change); 6] = [
            ("charge", both, |c| c.inbox.robots[1].state.charge -= 0.5),
            ("position", both, |c| {
                let position = c.inbox.robots[0].state.position.as_mut();
                position.unwrap().x += 1e-9;
            }),
            ("report left out", both, |c| {
                c.inbox.robots.pop();
            }),
            ("time", both, |c| c.now.0 += 1),
            ("job", job, |c| c.inbox.jobs[0].id += 1000),
            ("watched", both, |c| c.inbox.watched = !c.inbox.watched),
        ];
        for (what, k, change) in changes {
            let mut changed = cycles.clone();
            change(&mut changed[k]);
            let outcome = replay(&write(&setup, &changed)[..]).unwrap();
            let difference = outcome.difference.expect(what);
            assert_eq!(difference.cycle, k as u64 + 1, "{policy:?}: {what}");
            assert_eq!(outcome.cycles, difference.cycle);
            assert!(difference.messages || difference.state);
        }

        // another job queue: found when the job comes, or later
        let mut changed = cycles.clone();
        changed[job].inbox.jobs.clear();
        let outcome = replay(&write(&setup, &changed)[..]).unwrap();
        assert!(outcome.difference.unwrap().cycle > job as u64);
    }
}

fn cli(args: &[&str]) -> Finished {
    Command::new(env!("CARGO_BIN_EXE_botsim"))
        .args(args)
        .output()
        .unwrap()
}

fn json(output: &Finished) -> serde_json::Value {
    serde_json::from_slice(&output.stdout).unwrap_or_else(|e| panic!("{e}: {output:?}"))
}

#[test]
fn the_command_line_records_and_replays() {
    for (policy, name) in [(Policy::Baseline, "baseline"), (Policy::Traffic, "traffic")] {
        let file = scenario_file(&format!("cli-{name}"), "ring_jobs.toml", |t| {
            lasting(&t, 200)
        });
        let scenario = file.to_str().unwrap();
        let recording = file.with_file_name("run.inputs");
        let recording_text = recording.to_str().unwrap();

        let plain = cli(&["--policy", name, scenario]);
        assert!(plain.status.success(), "{plain:?}");
        let with = cli(&[
            "--policy",
            name,
            "--record-inputs",
            recording_text,
            scenario,
        ]);
        assert!(with.status.success(), "{with:?}");
        assert_eq!(with.stdout, plain.stdout);
        // another process gives the same file as this one
        assert_eq!(fs::read(&recording).unwrap(), recorded(&file, policy).1);

        // the replay needs no other file
        let alone = folder(&format!("alone-{name}")).join("run.inputs");
        fs::rename(&recording, &alone).unwrap();
        fs::remove_dir_all(file.parent().unwrap()).unwrap();
        let alone_text = alone.to_str().unwrap();
        let replayed = cli(&["--replay", alone_text]);
        assert_eq!(replayed.status.code(), Some(0), "{replayed:?}");
        assert!(replayed.stderr.is_empty(), "{replayed:?}");
        let report = json(&replayed);
        assert_eq!(report["cycles"], 4001);
        assert_eq!(report["same"], true);
        assert_eq!(report["complete"], true);
        assert!(report.get("first_difference").is_none());

        // a changed input: exit 3
        let bytes = fs::read(&alone).unwrap();
        let (setup, mut cycles) = read(&bytes);
        cycles[100].inbox.robots[0].state.charge -= 1.0;
        fs::write(&alone, write(&setup, &cycles)).unwrap();
        let differs = cli(&["--replay", alone_text]);
        assert_eq!(differs.status.code(), Some(3), "{differs:?}");
        let report = json(&differs);
        assert_eq!(report["same"], false);
        assert_eq!(report["first_difference"]["cycle"], 101);
        assert_eq!(report["first_difference"]["state"], true);

        // cut off: what is whole is replayed, then the error
        fs::write(&alone, &bytes[..bytes.len() / 2]).unwrap();
        let cut = cli(&["--replay", alone_text]);
        assert_eq!(cut.status.code(), Some(1), "{cut:?}");
        let report = json(&cut);
        assert_eq!(report["same"], true);
        assert_eq!(report["complete"], false);
        assert!(report["cycles"].as_u64().unwrap() > 100);
        assert!(String::from_utf8_lossy(&cut.stderr).contains("cut off"));

        // another botsim version: replayed, and said
        let mut other = Vec::new();
        let mut writer = Writer::with_version(&mut other, &setup, "0.0.9").unwrap();
        let (_, cycles) = read(&bytes);
        cycles[..50].iter().for_each(|c| writer.cycle(c).unwrap());
        writer.finish().unwrap();
        fs::write(&alone, other).unwrap();
        let older = cli(&["--replay", alone_text]);
        assert_eq!(older.status.code(), Some(0), "{older:?}");
        assert_eq!(json(&older)["recorded_with"], "0.0.9");
        assert!(String::from_utf8_lossy(&older.stderr).contains("0.0.9"));
    }
}

#[test]
fn the_command_line_reports_what_it_cannot_replay_or_record() {
    let dir = folder("cli-errors");
    // foreign and missing files: exit 1, nothing on stdout
    for file in [
        dir.join("ring.lif.json"),
        dir.join("nothing-here"),
        dir.clone(),
    ] {
        let o = cli(&["--replay", file.to_str().unwrap()]);
        assert_eq!(o.status.code(), Some(1), "{o:?}");
        assert!(o.stdout.is_empty(), "{o:?}");
        assert!(String::from_utf8_lossy(&o.stderr).starts_with("error: "));
    }
    // wrong command lines: exit 2
    for args in [
        &["--replay"][..],
        &["--replay", "a", "b"],
        &["--timing", "--replay", "a"],
        &["--replay", "--timing"],
        &["--record-inputs"],
        &["--record-inputs", "a"],
        &["--record", "a", "--record-inputs", "a", "s.toml"],
    ] {
        let o = cli(args);
        assert_eq!(o.status.code(), Some(2), "{args:?}");
        assert!(String::from_utf8_lossy(&o.stderr).contains("usage"));
    }
    // a scenario without jobs has no server: nothing is created
    let scenario = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/two_robots.toml");
    let file = dir.join("none.inputs");
    let o = cli(&[
        "--record-inputs",
        file.to_str().unwrap(),
        scenario.to_str().unwrap(),
    ]);
    assert_eq!(o.status.code(), Some(1), "{o:?}");
    assert!(String::from_utf8_lossy(&o.stderr).contains("[jobs]"));
    assert!(!file.exists());
    // a file that cannot be written
    let o = cli(&[
        "--record-inputs",
        dir.join("no/such/folder").to_str().unwrap(),
        scenarios().join("ring_jobs.toml").to_str().unwrap(),
    ]);
    assert_eq!(o.status.code(), Some(1), "{o:?}");
    assert!(String::from_utf8_lossy(&o.stderr).contains("cannot write"));
}

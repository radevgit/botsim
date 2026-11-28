//! The recording file: the setup comes back, a hand-made run replays, and a file
//! that is cut off, changed, foreign or made of what cannot be gives an error.

use std::io::{self, Write};

use botsim_dispatcher::ChargePolicy;
use botsim_link::message::{FromRobot, Header, Millis};
use botsim_link::state::{EmergencyStop, LastNode, OperatingMode, Position, Route, State};
use botsim_link::zone_set::{Zone, ZoneKind, ZoneSet};
use botsim_model::graph::Graph;
use botsim_model::job::Job;
use botsim_model::robot::{RobotId, RobotNo};
use botsim_model::spec::{Kinematics, Shape};
use botsim_model::zones::Zones;
use botsim_replay::file::{FORMAT, MAGIC};
use botsim_replay::{
    Call, Dispatch, Error, Reader, Recorder, Rules, Setup, VERSION, Writer, controller, replay,
};
use botsim_server::conflicts::{ConflictInput, NodeConflictInput, NodeNodeConflictInput};
use botsim_server::cycle::{Config, FleetRobot, Inbox};
use botsim_server::doors::Door;
use botsim_server::lifts::{Lift, LiftStop};
use botsim_space::{MapOptions, TypeRef};
use vdma_lif::Lif;

/// The line A → B → C, one map, for the robot type ACME/X1.
fn lif() -> String {
    let types = r#"[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let node = |id: &str, x: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":0.0}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
        )
    };
    let edge = |a: &str, b: &str| {
        format!(
            r#"{{"edgeId":"{a}{b}","startNodeId":"{a}","endNodeId":"{b}",
            "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":{types}}}]}}"#
        )
    };
    format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-20T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{}],"edges":[{},{}]}}]}}]}}"#,
        node("A", 0.0),
        // (a number that must be read back bit for bit)
        node("B", 0.1 + 0.2 + 5.0),
        node("C", 10.0),
        edge("A", "B"),
        edge("B", "C"),
    )
}

fn graph(lif: &str) -> Graph {
    Graph::from_lif(&serde_json::from_str::<Lif>(lif).unwrap()).unwrap()
}

/// A setup that uses every part there is.
fn full(rules: Rules) -> Setup {
    let lif = lif();
    let graph = graph(&lif);
    let robot = |serial: &str, kinematics, shape| FleetRobot {
        id: RobotId::new("ACME", serial),
        series: "X1".into(),
        kinematics,
        shape,
    };
    let rect = Shape::Rect {
        cx: 0.1,
        cy: -0.0,
        half_length: 0.5,
        half_width: 0.4,
    };
    let circle = Shape::Circle {
        cx: 0.0,
        cy: 0.0,
        radius: 0.4,
    };
    let zones = serde_json::from_str(
        r#"[{"zoneId":"Z","zoneType":"RELEASE","releaseLossBehavior":"STOP",
        "vertices":[{"x":4.0,"y":-1.0},{"x":6.0,"y":-1.0},{"x":5.0,"y":1.0}]}]"#,
    )
    .unwrap();
    let pair = || vec![("AB".to_string(), "BC".to_string())];
    Setup {
        config: Config {
            robots: vec![
                robot("R1", Kinematics::Differential, rect),
                robot("R2", Kinematics::Omnidirectional, circle),
                robot("R3", Kinematics::ThreeWheel, rect),
            ],
            zones: Some(Zones {
                map: 0,
                zone_set_id: "set".into(),
                zones,
            }),
            zone_set: Some(
                ZoneSet {
                    id: "set".into(),
                    map: 0,
                    descriptor: Some("zones".into()),
                    zones: vec![Zone {
                        id: "Z".into(),
                        descriptor: None,
                        vertices: vec![(4.0, -1.0), (6.0, -1.0), (5.0, 1.0)],
                        kind: ZoneKind::Release { loss: None },
                    }],
                }
                .into(),
            ),
            lifts: vec![Lift {
                id: "L".into(),
                doors: vec!["front".into()],
                stops: vec![LiftStop {
                    node: 2,
                    door: 0,
                    waiting: Some(1),
                }],
            }],
            doors: vec![Door {
                id: "D".into(),
                edges: vec![1],
            }],
            start: Millis(1_000),
            graph,
        },
        lif,
        rules,
        dispatch: Dispatch::Baseline,
        calls: vec![
            Call::LoadWeight(5.0),
            Call::WaitWeight(0.5),
            Call::Charging(ChargePolicy {
                low: 25.0,
                high: 80.0,
                critical: 10.0,
            }),
            Call::Conflicts(vec![ConflictInput {
                type_a: "ACME/X1".into(),
                type_b: "ACME/X1".into(),
                edges: pair(),
            }]),
            Call::NodeConflicts(vec![NodeConflictInput {
                type_node: "ACME/X1".into(),
                type_edge: "ACME/X1".into(),
                pairs: pair(),
            }]),
            Call::NodeNodeConflicts(vec![NodeNodeConflictInput {
                type_a: "ACME/X1".into(),
                type_b: "ACME/X1".into(),
                pairs: pair(),
            }]),
        ],
    }
}

fn traffic() -> Rules {
    Rules::Traffic {
        types: vec![TypeRef::test_rect(
            "ACME/X1",
            Kinematics::Differential,
            1.0,
            0.8,
        )],
        options: MapOptions::default(),
    }
}

/// Robot `i` standing on node `node` with everything idle.
fn report(i: u32, node: usize, x: f64, time: Millis) -> FromRobot {
    FromRobot {
        header: Header {
            robot: RobotNo::new(i, 0),
            seq: 0,
            time,
        },
        state: State {
            order_id: None,
            update_id: 0,
            route: Route::new(Some(LastNode { node, step: 0 }), Vec::new(), 0).unwrap(),
            position: Some(Position {
                map: 0,
                x,
                y: 0.0,
                theta: 0.0,
                localized: true,
            }),
            velocity: None,
            driving: false,
            field_violation: false,
            emergency_stop: EmergencyStop::None,
            operating_mode: OperatingMode::Automatic,
            charge: 50.0,
            charging: false,
            actions: Vec::new(),
            instant_actions: Vec::new(),
            zone_requests: None,
            zone_sets: Vec::new(),
            errors: Vec::new(),
        },
    }
}

/// A job from B to C.
fn job(time: Millis) -> Job {
    Job {
        id: 1,
        pickup: 1,
        drop: 2,
        created: time,
    }
}

const CYCLES: u64 = 6;

/// A run of a few cycles written into `out`.
fn record_into(out: impl Write, setup: &Setup) -> Result<(), Error> {
    let mut recorder = Recorder::new(out, setup)?;
    let mut ctl = controller(setup)?;
    for k in 0..CYCLES as i64 {
        let now = Millis(1_000 + 50 * k);
        let robots = vec![
            report(0, 0, 0.0, now),
            report(1, 1, 0.1 + 0.2 + 5.0, now),
            report(2, 2, 10.0, now),
        ];
        let inbox = Inbox {
            robots,
            lifts: Vec::new(),
            jobs: if k == 2 { vec![job(now)] } else { Vec::new() },
            watched: k % 2 == 1,
        };
        recorder.step(&mut ctl, now, inbox);
    }
    recorder.finish()
}

fn recording(setup: &Setup) -> Vec<u8> {
    let mut bytes = Vec::new();
    record_into(&mut bytes, setup).unwrap();
    bytes
}

#[test]
fn a_run_with_every_part_of_a_setup_replays() {
    for rules in [Rules::Baseline, traffic()] {
        let setup = full(rules);
        let file = recording(&setup);
        let outcome = replay(&file[..]).unwrap();
        assert!(outcome.is_same(), "{outcome:?}");
        assert_eq!(outcome.cycles, CYCLES);
        assert_eq!(outcome.recorded_with, VERSION);

        // what is read, written again, is the file: nothing of the setup or of a
        // cycle is lost
        let mut reader = Reader::new(&file[..]).unwrap();
        let read = reader.setup().clone();
        assert_eq!(read.config.graph, setup.config.graph);
        assert_eq!(read.calls, setup.calls);
        let mut again = Vec::new();
        let mut writer = Writer::new(&mut again, &read).unwrap();
        while let Some(cycle) = reader.next_cycle().unwrap() {
            writer.cycle(&cycle).unwrap();
        }
        writer.finish().unwrap();
        assert_eq!(again, file);
        // and the end stays the end
        assert!(reader.next_cycle().unwrap().is_none());
    }
}

#[test]
fn a_file_cut_off_anywhere_is_replayed_as_far_as_it_is_whole() {
    let file = recording(&full(Rules::Baseline));
    let mut most = 0;
    for length in 0..file.len() {
        match replay(&file[..length]) {
            Err(Error::Foreign) => assert!(length < MAGIC.len(), "{length}"),
            Err(Error::CutOff) => {}
            Err(other) => panic!("{length}: {other}"),
            Ok(outcome) => {
                assert!(outcome.difference.is_none(), "{length}");
                assert!(
                    matches!(outcome.broken, Some(Error::CutOff)),
                    "{length}: {outcome:?}"
                );
                assert!(outcome.cycles >= most);
                most = outcome.cycles;
            }
        }
    }
    // without the end record only, every cycle is there
    assert_eq!(most, CYCLES);
}

#[test]
fn a_file_with_any_byte_changed_is_not_taken() {
    for rules in [Rules::Baseline, traffic()] {
        let file = recording(&full(rules));
        // the format number: bytes 8 to 12
        let mut changed = file.clone();
        for at in 0..file.len() {
            for bit in [0x01, 0x80] {
                changed[at] ^= bit;
                match replay(&changed[..]) {
                    Err(Error::Foreign) => assert!(at < 8, "{at}"),
                    Err(Error::Format(_)) => assert!((8..12).contains(&at), "{at}"),
                    // (a longer length reads as a file cut off)
                    Err(Error::CutOff | Error::Damaged(_)) => {}
                    Err(other) => panic!("{at}: {other}"),
                    Ok(outcome) => {
                        assert!(outcome.difference.is_none(), "{at}");
                        assert!(
                            matches!(outcome.broken, Some(Error::CutOff | Error::Damaged(_))),
                            "{at}: {outcome:?}"
                        );
                    }
                }
                changed[at] ^= bit;
            }
        }
    }
}

#[test]
fn a_foreign_file_another_format_and_bytes_after_the_end_are_refused() {
    for foreign in [&b""[..], b"BOTSIM", b"{\"type\":\"layout\"}\n", &[0; 1000]] {
        assert!(
            matches!(replay(foreign), Err(Error::Foreign)),
            "{foreign:?}"
        );
    }
    let file = recording(&full(Rules::Baseline));
    let mut other = file.clone();
    other[8..12].copy_from_slice(&(FORMAT + 1).to_le_bytes());
    let error = replay(&other[..]).unwrap_err();
    assert!(matches!(error, Error::Format(n) if n == FORMAT + 1));
    assert!(error.to_string().contains("format 2"), "{error}");

    let mut longer = file.clone();
    longer.push(0);
    let outcome = replay(&longer[..]).unwrap();
    assert_eq!(outcome.cycles, CYCLES);
    assert!(matches!(outcome.broken, Some(Error::Damaged(_))));
    assert!(!outcome.is_same());
}

#[test]
fn a_file_of_another_version_is_replayed_and_says_so() {
    let setup = full(Rules::Baseline);
    let mut reader_bytes = recording(&setup);
    let mut reader = Reader::new(&reader_bytes[..]).unwrap();
    let mut other = Vec::new();
    let mut writer = Writer::with_version(&mut other, &setup, "9.9.9").unwrap();
    while let Some(cycle) = reader.next_cycle().unwrap() {
        writer.cycle(&cycle).unwrap();
    }
    writer.finish().unwrap();
    let outcome = replay(&other[..]).unwrap();
    assert_eq!(outcome.recorded_with, "9.9.9");
    assert!(outcome.is_same(), "{outcome:?}");

    // a version too long for the header
    reader_bytes.clear();
    let long = "9".repeat(256);
    let result = Writer::with_version(&mut reader_bytes, &setup, &long);
    assert!(matches!(result, Err(Error::TooLarge)));
}

/// Whether the file of the setup `change` makes of a good one is refused.
fn refused(rules: Rules, change: impl FnOnce(&mut Setup)) -> String {
    let mut setup = full(rules);
    change(&mut setup);
    let mut bytes = Vec::new();
    Writer::new(&mut bytes, &setup).unwrap().finish().unwrap();
    replay(&bytes[..]).unwrap_err().to_string()
}

#[test]
fn a_setup_of_what_cannot_be_is_refused() {
    let b = || Rules::Baseline;
    refused(b(), |s| s.config.lifts[0].stops[0].node = 3);
    refused(b(), |s| {
        s.config.lifts[0].stops[0].waiting = Some(usize::MAX)
    });
    refused(b(), |s| s.config.zones.as_mut().unwrap().map = 1);
    refused(b(), |s| {
        s.config.zones.as_mut().unwrap().zones[0].vertices.clear();
    });
    refused(b(), |s| {
        s.config.robots[0].shape = Shape::Circle {
            cx: 0.0,
            cy: 0.0,
            radius: f64::NAN,
        };
    });
    let with = |change: fn(&mut TypeRef, &mut MapOptions)| {
        let Rules::Traffic {
            mut types,
            mut options,
        } = traffic()
        else {
            unreachable!();
        };
        change(&mut types[0], &mut options);
        Rules::Traffic { types, options }
    };
    refused(with(|_, o| o.sample_step = 0.0), |_| {});
    refused(with(|_, o| o.sample_step = 1e-300), |_| {});
    refused(with(|_, o| o.clearance = f64::INFINITY), |_| {});
    refused(with(|t, _| t.length = 1e300), |_| {});
}

#[test]
fn a_layout_text_that_is_not_the_graph_is_not_written() {
    let mut setup = full(Rules::Baseline);
    setup.config.graph.nodes[0].x += 1.0;
    let mut bytes = Vec::new();
    assert!(matches!(
        Writer::new(&mut bytes, &setup),
        Err(Error::Setup(_))
    ));
    setup.lif = "no layout".into();
    assert!(matches!(
        Writer::new(&mut bytes, &setup),
        Err(Error::Setup(_))
    ));
}

/// Takes `room` bytes, then fails.
struct Full {
    room: usize,
}

impl Write for Full {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        if bytes.len() > self.room {
            return Err(io::Error::other("the disk is full"));
        }
        self.room -= bytes.len();
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[test]
fn a_failure_to_write_is_reported_and_the_controller_goes_on() {
    let setup = full(Rules::Baseline);
    let whole = recording(&setup).len();
    // fails in the header, in the setup, in a cycle, and in the end
    for room in [0, 40, whole - 100, whole - 1] {
        let result = record_into(Full { room }, &setup);
        assert!(
            matches!(&result, Err(Error::Io(e)) if e.to_string() == "the disk is full"),
            "{room}: {result:?}"
        );
    }
    record_into(Full { room: whole }, &setup).unwrap();
}

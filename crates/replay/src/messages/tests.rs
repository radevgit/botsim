//! Every kind of message comes back as it was written, bit for bit.

use std::hash::{Hash, Hasher};

use botsim_model::checksum::Checksum;

use super::*;

/// Equal by the bits of every number (`==` holds no number that is none equal to
/// itself, and −0 equal to 0).
fn bits(value: &impl Hash) -> u64 {
    let mut sum = Checksum::new();
    value.hash(&mut sum);
    sum.finish()
}

fn plain() -> State {
    State {
        order_id: None,
        update_id: 0,
        route: Route::default(),
        position: None,
        velocity: None,
        driving: false,
        field_violation: false,
        emergency_stop: EmergencyStop::None,
        operating_mode: OperatingMode::Automatic,
        charge: 100.0,
        charging: false,
        actions: Vec::new(),
        instant_actions: Vec::new(),
        zone_requests: None,
        zone_sets: Vec::new(),
        errors: Vec::new(),
    }
}

fn actions() -> Vec<ActionState> {
    use ActionStatus::*;
    [
        Waiting,
        Initializing,
        Running,
        Paused,
        Retriable,
        Finished,
        Failed,
    ]
    .into_iter()
    .enumerate()
    .map(|(i, status)| ActionState {
        id: ActionId::new(format!("a-{i}")),
        kind: (i % 2 == 0).then(|| ActionType::new("pick")),
        status,
    })
    .collect()
}

fn requests() -> Vec<ZoneRequest> {
    use RequestStatus::*;
    [Requested, Queued, Granted, Revoked, Expired]
        .into_iter()
        .enumerate()
        .map(|(i, status)| ZoneRequest {
            id: RequestId::new(format!("q-{i}")),
            kind: if i % 2 == 0 {
                ZoneRequestType::Access
            } else {
                ZoneRequestType::Replanning
            },
            zone: ZoneId::new("z"),
            zone_set: ZoneSetId::new(""),
            status,
        })
        .collect()
}

fn errors() -> Vec<Error> {
    use ErrorLevel::*;
    let mut errors: Vec<Error> = ErrorType::ALL
        .into_iter()
        .map(|kind| Error {
            kind: ErrorKind::Predefined(kind),
            level: kind.level(),
            references: Vec::new(),
            description: None,
            hint: None,
        })
        .collect();
    for level in [Warning, Urgent, Critical, Fatal] {
        errors.push(Error {
            kind: ErrorKind::Other("BATTERY_EMPTY".into()),
            level,
            references: vec![("orderId".into(), "job-1".into()), ("".into(), "ü".into())],
            description: Some("text".into()),
            hint: Some("".into()),
        });
    }
    errors
}

/// States that differ from each other in every field, one after the other.
fn states() -> Vec<State> {
    use OperatingMode::*;
    let mut states = vec![plain()];
    let mut state = plain();
    let mut next = |change: &mut dyn FnMut(&mut State)| {
        change(&mut state);
        states.push(state.clone());
    };
    next(&mut |s| s.order_id = Some(OrderId::new("job-1")));
    next(&mut |s| s.update_id = u32::MAX);
    next(&mut |s| {
        let last = Some(LastNode { node: 3, step: 2 });
        let ahead = vec![Ahead { edge: 4, node: 5 }, Ahead { edge: 0, node: 0 }];
        s.route = Route::new(last, ahead, 1).unwrap();
    });
    next(&mut |s| {
        s.position = Some(Position {
            map: 1,
            x: 1.5,
            y: -0.0,
            theta: f64::NAN,
            localized: true,
        });
    });
    // nothing changes at all
    next(&mut |_| {});
    // one number of the position, to one that `==` cannot tell apart
    next(&mut |s| s.position.as_mut().unwrap().y = 0.0);
    next(&mut |s| s.position.as_mut().unwrap().x = f64::INFINITY);
    next(&mut |s| s.position.as_mut().unwrap().theta = -f64::NAN);
    next(&mut |s| s.position.as_mut().unwrap().map = usize::MAX);
    next(&mut |s| s.position.as_mut().unwrap().localized = false);
    next(&mut |s| s.position = None);
    next(&mut |s| {
        s.velocity = Some(Velocity {
            vx: 0.1,
            vy: 0.0,
            omega: -3.0,
        });
    });
    next(&mut |s| s.velocity.as_mut().unwrap().vy = -0.0);
    next(&mut |s| s.velocity = None);
    next(&mut |s| s.driving = true);
    next(&mut |s| s.field_violation = true);
    next(&mut |s| s.emergency_stop = EmergencyStop::Manual);
    next(&mut |s| s.emergency_stop = EmergencyStop::Remote);
    for mode in [Startup, Semiautomatic, Intervened, Manual, Service, TeachIn] {
        next(&mut |s| s.operating_mode = mode);
    }
    next(&mut |s| s.charging = true);
    next(&mut |s| s.charge = f64::MIN);
    next(&mut |s| s.actions = actions());
    next(&mut |s| s.instant_actions = actions());
    next(&mut |s| s.zone_requests = Some(Vec::new()));
    next(&mut |s| s.zone_requests = Some(requests()));
    next(&mut |s| {
        s.zone_sets = vec![ZoneSetInfo {
            id: ZoneSetId::new("set"),
            map: 2,
            enabled: true,
        }];
    });
    next(&mut |s| s.errors = errors());
    next(&mut |s| *s = plain());
    states
}

#[test]
fn reports_come_back_as_written_one_after_the_other() {
    let now = Millis(1_000_000);
    // two robots in turn, with times and sequence numbers that jump and wrap
    let reports: Vec<FromRobot> = states()
        .into_iter()
        .enumerate()
        .flat_map(|(i, state)| {
            [0u32, 1].map(|slot| FromRobot {
                header: Header {
                    robot: RobotNo::new(slot * u32::MAX, slot),
                    seq: [0, u32::MAX, 1 << 31, 7][i % 4].wrapping_add(i as u32),
                    time: Millis([now.0, i64::MIN, i64::MAX, 0][i % 4]),
                },
                state: state.clone(),
            })
        })
        .collect();
    let mut written = Previous::default();
    let mut read = Previous::default();
    for report in &reports {
        let mut e = Encoder::default();
        write_report(&mut e, report, now, &mut written);
        let mut d = Decoder::new(&e.bytes);
        let back = read_report(&mut d, now, &mut read).unwrap();
        d.end().unwrap();
        assert_eq!(bits(&back), bits(report), "{report:?}");
    }
}

#[test]
fn a_report_without_news_is_short() {
    let now = Millis(5000);
    let mut state = plain();
    state.errors = errors();
    state.actions = actions();
    let mut report = FromRobot {
        header: Header {
            robot: RobotNo::new(3, 0),
            seq: 8,
            time: now,
        },
        state,
    };
    let mut previous = Previous::default();
    let mut first = Encoder::default();
    write_report(&mut first, &report, now, &mut previous);
    report.header.seq = 9;
    let mut second = Encoder::default();
    write_report(&mut second, &report, now, &mut previous);
    // robot (2), sequence, time and mask
    assert_eq!(second.bytes.len(), 5);
    assert!(first.bytes.len() > 100);
}

#[test]
fn a_first_report_must_say_everything() {
    // robot 0, sequence, time, a mask with no field
    let mut d = Decoder::new(&[0, 0, 0, 0, 0]);
    let e = read_report(&mut d, Millis(0), &mut Previous::default()).unwrap_err();
    assert_eq!(e, NO_BEFORE);
}

#[test]
fn lifts_and_jobs_come_back() {
    use DoorPhase::*;
    let now = Millis(-5);
    for (i, door) in [Closed, Opening, Open, Closing].into_iter().enumerate() {
        let lift = FromLift {
            header: DeviceHeader {
                device: LiftNo(u32::MAX),
                seq: i as u32,
                time: Millis(i64::MAX),
            },
            map: usize::MAX,
            car_door: 3,
            door,
            occupant: (i % 2 == 0).then(|| RobotNo::new(1, 2)),
        };
        let job = Job {
            id: u64::MAX,
            pickup: i,
            drop: usize::MAX,
            created: Millis(i64::MIN),
        };
        let mut e = Encoder::default();
        write_lift(&mut e, &lift, now);
        write_job(&mut e, &job, now);
        let mut d = Decoder::new(&e.bytes);
        assert_eq!(read_lift(&mut d, now), Ok(lift));
        assert_eq!(read_job(&mut d, now), Ok(job));
        d.end().unwrap();
    }
}

#[test]
fn every_kind_of_zone_comes_back() {
    use Blocking as B;
    let action = |blocking| ZoneAction {
        kind: ActionType::new("beep"),
        descriptor: Some("d".into()),
        blocking,
        parameters: Some(vec![
            ("a".into(), serde_json::json!(0.1 + 0.2)),
            (
                "b".into(),
                serde_json::json!({"x": [1, null, "y", true, -1.5e300]}),
            ),
        ]),
        retriable: Some(false),
    };
    let loss = [
        None,
        Some(LossBehavior::Stop),
        Some(LossBehavior::Continue),
        Some(LossBehavior::Evacuate),
    ];
    let mut kinds = vec![
        ZoneKind::Blocked,
        ZoneKind::LineGuided,
        ZoneKind::CoordinatedReplanning,
        ZoneKind::SpeedLimit {
            maximum_speed: f64::NAN,
        },
        ZoneKind::Action {
            entry: vec![action(B::None), action(B::Soft)],
            during: vec![action(B::Single)],
            exit: vec![action(B::Hard)],
        },
        ZoneKind::Priority { factor: 0.5 },
        ZoneKind::Penalty { factor: -0.0 },
    ];
    kinds.extend(loss.map(|loss| ZoneKind::Release { loss }));
    kinds.extend(
        [Directed::Soft, Directed::Restricted, Directed::Strict].map(|limitation| {
            ZoneKind::Directed {
                direction: 1.0,
                limitation,
            }
        }),
    );
    kinds.extend(
        [Bidirected::Soft, Bidirected::Restricted].map(|limitation| ZoneKind::Bidirected {
            direction: -1.0,
            limitation,
        }),
    );
    let set = ZoneSet {
        id: ZoneSetId::new("set"),
        map: 1,
        descriptor: None,
        zones: kinds
            .into_iter()
            .map(|kind| Zone {
                id: ZoneId::new("z"),
                descriptor: None,
                vertices: vec![(0.0, -0.0), (1.0, f64::EPSILON)],
                kind,
            })
            .collect(),
    };
    let mut e = Encoder::default();
    write_zone_set(&mut e, &set);
    let mut d = Decoder::new(&e.bytes);
    let back = read_zone_set(&mut d).unwrap();
    d.end().unwrap();
    assert_eq!(bits(&back), bits(&set));
}

//! A small layout, a translator with one robot, and messages that use every part
//! and every value of the `link` types.

#![allow(dead_code)] // each test file uses some of this

use std::sync::Arc;

use botsim_link::ids::{ActionId, ActionType, OrderId, RequestId, ZoneId, ZoneSetId};
use botsim_link::instant::{InstantAction, InstantKind};
use botsim_link::message::{FromRobot, Header, Millis, ToRobot, ToRobotBody};
use botsim_link::order::{Action, Blocking, Leg, NodeAction, Order, OrientationType};
use botsim_link::response::{Grant, Response};
use botsim_link::state::*;
use botsim_link::zone_set::{
    Bidirected, Directed, LossBehavior, Zone, ZoneAction, ZoneKind, ZoneSet,
};
use botsim_model::errors::ErrorType;
use botsim_model::graph::Graph;
use botsim_model::robot::{RobotId, RobotNo};
use botsim_vda::translator::{RobotMessage, Translator};
use serde_json::json;
use vdma_lif::Lif;

pub const ROBOT: RobotNo = RobotNo::new(3, 1);
/// 2026-11-20T10:00:00.250Z
pub const TIME: Millis = Millis(1_795_168_800_250);

/// Nodes A, B, C (numbers 0, 1, 2) and edges A-B, B-C, C-B (0, 1, 2) on map M (0).
pub fn graph() -> Arc<Graph> {
    let lif: Lif = serde_json::from_str(include_str!("../data/three_nodes.lif.json")).unwrap();
    Arc::new(Graph::from_lif(&lif).unwrap())
}

pub fn translator() -> Translator {
    let mut t = Translator::new(graph());
    t.add_robot(ROBOT, RobotId::new("ACME", "R-001")).unwrap();
    t
}

pub fn header(seq: u32) -> Header {
    Header {
        robot: ROBOT,
        seq,
        time: TIME,
    }
}

pub fn to_robot(body: ToRobotBody) -> ToRobot {
    ToRobot {
        header: header(7),
        body,
    }
}

pub fn vda_order(t: &Translator, order: Order) -> vda5050::order::Order {
    match translate(t, ToRobotBody::Order(order)) {
        RobotMessage::Order(o) => o,
        other => panic!("{other:?}"),
    }
}

/// Translate a message to a robot that has nothing to report.
pub fn translate(t: &Translator, body: ToRobotBody) -> RobotMessage {
    let out = t.to_robot_to_vda(&to_robot(body)).unwrap();
    assert_eq!(out.issues, Vec::new());
    out.message
}

pub fn action(id: &str, blocking: Blocking) -> Action {
    Action {
        id: ActionId::new(id),
        kind: ActionType::new("pick"),
        blocking,
    }
}

pub fn leg(edge: usize, end: usize) -> Leg {
    Leg {
        edge,
        end,
        orientation: None,
        orientation_type: None,
    }
}

/// A -> B -> C with `released` legs.
pub fn order(first_step: u32, released: u32) -> Order {
    Order::new(
        OrderId::new("job-5"),
        if first_step == 0 { 0 } else { 2 },
        first_step,
        0,
        vec![leg(0, 1), leg(1, 2)],
        released,
        Vec::new(),
    )
    .unwrap()
}

pub fn orders() -> Vec<Order> {
    let legs = vec![
        Leg {
            edge: 0,
            end: 1,
            orientation: Some(0.5),
            orientation_type: Some(OrientationType::Tangential),
        },
        Leg {
            edge: 1,
            end: 2,
            orientation: Some(-std::f64::consts::PI),
            orientation_type: Some(OrientationType::Global),
        },
        // a type without an angle, and an angle without a type
        Leg {
            edge: 2,
            end: 1,
            orientation: None,
            orientation_type: Some(OrientationType::Global),
        },
        Leg {
            edge: 1,
            end: 2,
            orientation: Some(std::f64::consts::PI),
            orientation_type: None,
        },
    ];
    let actions = [
        Blocking::None,
        Blocking::Soft,
        Blocking::Single,
        Blocking::Hard,
    ]
    .into_iter()
    .enumerate()
    .map(|(i, blocking)| NodeAction {
        node: [0, 2, 2, 4][i],
        action: action(&format!("a-{i}"), blocking),
    })
    .collect();
    let full = Order::new(OrderId::new("job-1"), 3, 5, 0, legs, 2, actions).unwrap();
    let only_start = Order::new(OrderId::new("o"), 0, 0, 2, Vec::new(), 0, Vec::new()).unwrap();
    vec![full, only_start, order(0, 0), order(0, 2), order(4, 1)]
}

pub fn instant_actions() -> Vec<InstantAction> {
    [
        InstantKind::CancelOrder { order_id: None },
        InstantKind::CancelOrder {
            order_id: Some(OrderId::new("job-5")),
        },
        InstantKind::EnableZoneSet {
            zone_set: ZoneSetId::new("zs"),
        },
        InstantKind::DeleteZoneSet {
            zone_set: ZoneSetId::new("zs"),
        },
        InstantKind::StartCharging,
        InstantKind::StopCharging,
        InstantKind::Other(ActionType::new("beep")),
    ]
    .into_iter()
    .enumerate()
    .map(|(i, kind)| InstantAction {
        id: ActionId::new(format!("i-{i}")),
        kind,
    })
    .collect()
}

pub fn responses() -> Vec<Response> {
    [
        Grant::Granted,
        Grant::Queued,
        Grant::Revoked,
        Grant::Rejected,
    ]
    .into_iter()
    .enumerate()
    .map(|(i, grant)| Response {
        request: RequestId::new(format!("r-{i}")),
        grant,
        lease_expiry: (i == 0).then_some(Millis(TIME.0 + 60_000)),
    })
    .collect()
}

pub fn zone(id: &str, kind: ZoneKind) -> Zone {
    Zone {
        id: ZoneId::new(id),
        descriptor: None,
        vertices: vec![(0.0, 0.0), (1.5, 0.0), (1.5, 2.25)],
        kind,
    }
}

pub fn zone_set(zones: Vec<Zone>) -> ZoneSet {
    ZoneSet {
        id: ZoneSetId::new("zs"),
        map: 0,
        descriptor: None,
        zones,
    }
}

/// Every zone type with every value of its parameters.
pub fn full_zone_set() -> ZoneSet {
    let plain_action = ZoneAction {
        kind: ActionType::new("beep"),
        descriptor: None,
        blocking: Blocking::None,
        parameters: None,
        retriable: None,
    };
    let full_action = ZoneAction {
        kind: ActionType::new("lights"),
        descriptor: Some("on".into()),
        blocking: Blocking::Hard,
        parameters: Some(vec![
            ("number".into(), json!(103.2)),
            ("flag".into(), json!(true)),
            ("text".into(), json!("a")),
            ("list".into(), json!(["a", 1])),
            ("object".into(), json!({"x": 1})),
        ]),
        retriable: Some(false),
    };
    let mut kinds = vec![
        ZoneKind::Blocked,
        ZoneKind::LineGuided,
        ZoneKind::Release { loss: None },
        ZoneKind::CoordinatedReplanning,
        ZoneKind::SpeedLimit { maximum_speed: 0.5 },
        ZoneKind::Action {
            entry: vec![plain_action, full_action],
            during: Vec::new(),
            exit: Vec::new(),
        },
        ZoneKind::Priority { factor: 0.0 },
        ZoneKind::Penalty { factor: 1.0 },
    ];
    for loss in [
        LossBehavior::Stop,
        LossBehavior::Continue,
        LossBehavior::Evacuate,
    ] {
        kinds.push(ZoneKind::Release { loss: Some(loss) });
    }
    for limitation in [Directed::Soft, Directed::Restricted, Directed::Strict] {
        kinds.push(ZoneKind::Directed {
            direction: 7.5,
            limitation,
        });
    }
    for limitation in [Bidirected::Soft, Bidirected::Restricted] {
        kinds.push(ZoneKind::Bidirected {
            direction: -1.0,
            limitation,
        });
    }
    let mut zones: Vec<Zone> = kinds
        .into_iter()
        .enumerate()
        .map(|(i, kind)| zone(&format!("z-{i}"), kind))
        .collect();
    zones[0].descriptor = Some("a zone".into());
    ZoneSet {
        descriptor: Some("all".into()),
        ..zone_set(zones)
    }
}

pub fn to_robot_messages() -> Vec<ToRobot> {
    let mut bodies: Vec<ToRobotBody> = orders().into_iter().map(ToRobotBody::Order).collect();
    bodies.push(ToRobotBody::InstantActions(instant_actions()));
    bodies.push(ToRobotBody::InstantActions(Vec::new()));
    bodies.push(ToRobotBody::Responses(responses()));
    bodies.push(ToRobotBody::Responses(Vec::new()));
    bodies.push(ToRobotBody::ZoneSet(Arc::new(full_zone_set())));
    bodies.push(ToRobotBody::ZoneSet(Arc::new(zone_set(Vec::new()))));
    bodies.into_iter().map(to_robot).collect()
}

/// A report of a robot that never had an order and says as little as it may.
pub fn idle_state() -> State {
    State {
        order_id: None,
        update_id: 0,
        route: Route::new(None, Vec::new(), 0).unwrap(),
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

pub fn action_states() -> Vec<ActionState> {
    [
        ActionStatus::Waiting,
        ActionStatus::Initializing,
        ActionStatus::Running,
        ActionStatus::Paused,
        ActionStatus::Retriable,
        ActionStatus::Finished,
        ActionStatus::Failed,
    ]
    .into_iter()
    .enumerate()
    .map(|(i, status)| ActionState {
        id: ActionId::new(format!("a-{i}")),
        kind: (i > 0).then(|| ActionType::new("pick")),
        status,
    })
    .collect()
}

pub fn zone_requests() -> Vec<ZoneRequest> {
    [
        RequestStatus::Requested,
        RequestStatus::Queued,
        RequestStatus::Granted,
        RequestStatus::Revoked,
        RequestStatus::Expired,
    ]
    .into_iter()
    .enumerate()
    .map(|(i, status)| ZoneRequest {
        id: RequestId::new(format!("r-{i}")),
        kind: if i == 0 {
            ZoneRequestType::Replanning
        } else {
            ZoneRequestType::Access
        },
        zone: ZoneId::new("z"),
        zone_set: ZoneSetId::new("zs"),
        status,
    })
    .collect()
}

pub fn errors() -> Vec<Error> {
    let levels = [
        ErrorLevel::Warning,
        ErrorLevel::Urgent,
        ErrorLevel::Critical,
        ErrorLevel::Fatal,
    ];
    let mut errors: Vec<Error> = ErrorType::ALL
        .into_iter()
        .map(ErrorKind::Predefined)
        .chain([ErrorKind::Other("BATTERY_EMPTY".into())])
        .enumerate()
        .map(|(i, kind)| Error {
            kind,
            level: levels[i % levels.len()],
            references: Vec::new(),
            description: None,
            hint: None,
        })
        .collect();
    errors[0].references = vec![
        ("orderId".into(), "job-5".into()),
        ("orderUpdateId".into(), "2".into()),
    ];
    errors[0].description = Some("what".into());
    errors[0].hint = Some("how".into());
    errors
}

pub fn full_state() -> State {
    State {
        order_id: Some(OrderId::new("job-5")),
        update_id: 2,
        route: Route::new(
            Some(LastNode { node: 0, step: 3 }),
            vec![Ahead { edge: 0, node: 1 }, Ahead { edge: 1, node: 2 }],
            1,
        )
        .unwrap(),
        position: Some(Position {
            map: 0,
            x: 0.1,
            y: -2.5,
            theta: -3.1,
            localized: false,
        }),
        velocity: Some(Velocity {
            vx: 1.25,
            vy: -0.1,
            omega: 0.3,
        }),
        driving: true,
        field_violation: true,
        emergency_stop: EmergencyStop::Manual,
        operating_mode: OperatingMode::Semiautomatic,
        charge: 55.5,
        charging: true,
        actions: action_states(),
        instant_actions: action_states(),
        zone_requests: Some(zone_requests()),
        zone_sets: vec![
            ZoneSetInfo {
                id: ZoneSetId::new("zs"),
                map: 0,
                enabled: true,
            },
            ZoneSetInfo {
                id: ZoneSetId::new("old"),
                map: 0,
                enabled: false,
            },
        ],
        errors: errors(),
    }
}

pub fn states() -> Vec<State> {
    let mut states = vec![idle_state(), full_state()];
    // an empty list of zone requests is not the same as none
    states.push(State {
        zone_requests: Some(Vec::new()),
        charge: 0.0,
        ..idle_state()
    });
    for emergency_stop in [EmergencyStop::Manual, EmergencyStop::Remote] {
        states.push(State {
            emergency_stop,
            ..idle_state()
        });
    }
    for operating_mode in [
        OperatingMode::Startup,
        OperatingMode::Intervened,
        OperatingMode::Manual,
        OperatingMode::Service,
        OperatingMode::TeachIn,
    ] {
        states.push(State {
            operating_mode,
            ..idle_state()
        });
    }
    states
}

pub fn from_robot(state: State) -> FromRobot {
    FromRobot {
        header: header(9),
        state,
    }
}

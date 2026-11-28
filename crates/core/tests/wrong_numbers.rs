//! No message, however wrong its numbers, makes the simulator panic: it is an
//! error of the world, a refusal of the robot, or taken.

use botsim_core::actions::ActionTimes;
use botsim_core::door::DoorSpec;
use botsim_core::lift::{LiftSpec, LiftStop};
use botsim_core::robot::{InstantEntry, Reception, Unread};
use botsim_core::world::{World, WorldError};
use botsim_link::device::{DeviceHeader, DoorCommand, DoorNo, LiftCommand, LiftNo, ToDoor, ToLift};
use botsim_link::ids::{ActionId, ActionType, OrderId, RequestId, ZoneId, ZoneSetId};
use botsim_link::instant::{InstantAction, InstantKind};
use botsim_link::message::{Header, Millis, ToRobot, ToRobotBody};
use botsim_link::order::{Action, Blocking, Leg, NodeAction, Order, OrientationType};
use botsim_link::response::{Grant, Response};
use botsim_link::state::ErrorKind;
use botsim_link::zone_set::{Zone, ZoneKind, ZoneSet};
use botsim_model::errors::ErrorType;
use botsim_model::graph::Graph;
use botsim_model::robot::{RobotId, RobotNo};
use botsim_model::spec::{Kinematics, RobotSpec};
use std::sync::Arc;
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

const DT: f64 = 0.05;

const ROBOT: RobotNo = RobotNo::new(0, 0);

/// A (0,0) → B (10,0) → C (10,5) on map M, a door on A-B, a lift between M and a
/// second map that stops at A, and one robot on A.
fn world(kinematics: Kinematics, delay: u64) -> World {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    let mut g = Graph::from_lif(&lif).unwrap();
    g.maps.push("M2".into());
    let a = g.node_idx("A").unwrap();
    let ab = g.edge_idx("A-B").unwrap();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let mut spec = RobotSpec::from_factsheet(&f).unwrap();
    spec.kinematics = kinematics;
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_door(DoorSpec {
        id: "d".into(),
        edges: vec![ab],
        passage_width: None,
        open_seconds: 0.5,
        close_seconds: 0.5,
        hold_seconds: None,
    })
    .unwrap();
    w.add_lift(LiftSpec {
        id: "l".into(),
        floors: vec![0, 1],
        doors: vec!["front".into()],
        stops: vec![LiftStop {
            node: a,
            floor: 0,
            door: 0,
            waiting: None,
        }],
        start_floor: 0,
        travel_seconds_per_floor: 1.0,
        door_open_seconds: 0.5,
        door_close_seconds: 0.5,
        door_hold_seconds: None,
    })
    .unwrap();
    w.add_robot(
        RobotId::new("ACME", "R-001"),
        spec,
        ActionTimes::new(1.0),
        a,
        0.0,
    )
    .unwrap();
    w.set_message_delay(delay);
    w
}

fn to_robot(robot: RobotNo, body: ToRobotBody) -> ToRobot {
    ToRobot {
        header: Header {
            robot,
            seq: u32::MAX,
            time: Millis(i64::MIN),
        },
        body,
    }
}

fn leg(edge: usize, end: usize, orientation: Option<f64>) -> Leg {
    Leg {
        edge,
        end,
        orientation,
        orientation_type: orientation.map(|_| OrientationType::Tangential),
    }
}

fn order(id: &str, update: u32, first: u32, start: usize, legs: Vec<Leg>) -> ToRobotBody {
    let released = legs.len() as u32;
    let actions = vec![NodeAction {
        node: released,
        action: Action {
            id: ActionId::new(""),
            kind: ActionType::new(""),
            blocking: Blocking::Hard,
        },
    }];
    let order = Order::new(
        OrderId::new(id),
        update,
        first,
        start,
        legs,
        released,
        actions,
    );
    ToRobotBody::Order(order.unwrap())
}

/// Numbers that make no sense, and orders that are only odd.
fn orders() -> Vec<ToRobotBody> {
    let last = u32::MAX / 2;
    let mut orders = vec![
        order("o", 0, 0, usize::MAX, Vec::new()),
        order("o", 0, 0, 0, vec![leg(usize::MAX, 1, None)]),
        order("o", 0, 0, 0, vec![leg(0, usize::MAX, None)]),
        order("o", 0, 0, 2, vec![leg(0, 1, None)]),
        order("o", 0, last, 0, Vec::new()),
        order("o", u32::MAX, last - 1, 0, vec![leg(0, 1, None)]),
        order("", 0, 0, 0, Vec::new()),
    ];
    for angle in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, f64::MAX, -1e300] {
        for kind in [OrientationType::Tangential, OrientationType::Global] {
            let mut odd = leg(0, 1, Some(angle));
            odd.orientation_type = Some(kind);
            orders.push(order("angle", 0, 0, 0, vec![odd]));
        }
    }
    // updates of whatever order is there, with the highest numbers
    for id in ["o", "angle", ""] {
        orders.push(order(id, u32::MAX, last - 1, 1, vec![leg(1, 2, None)]));
        orders.push(order(id, u32::MAX, 0, 1, vec![leg(1, 2, None)]));
    }
    orders
}

fn zone_sets() -> Vec<ToRobotBody> {
    let square = |s: f64| vec![(-s, -s), (s, -s), (s, s), (-s, s)];
    let wild = [f64::NAN, f64::INFINITY, -f64::MAX, 1e308];
    let mut corners = vec![Vec::new(), vec![(0.0, 0.0)], square(1e308), square(1.0)];
    corners.extend(wild.map(|v| vec![(v, v), (-v, v), (v, -v)]));
    let mut kinds = vec![ZoneKind::Release { loss: None }, ZoneKind::Blocked];
    for v in wild {
        kinds.push(ZoneKind::SpeedLimit { maximum_speed: v });
        kinds.push(ZoneKind::Priority { factor: v });
        kinds.push(ZoneKind::Penalty { factor: v });
    }
    let mut sets = Vec::new();
    for (i, vertices) in corners.iter().enumerate() {
        for (j, kind) in kinds.iter().enumerate() {
            let zone = Zone {
                id: ZoneId::new("z"),
                descriptor: None,
                vertices: vertices.clone(),
                kind: kind.clone(),
            };
            let zones = if j == 0 {
                vec![zone.clone(), zone]
            } else {
                vec![zone]
            };
            sets.push((format!("{i}-{j}"), 0, zones));
        }
    }
    sets.push(("far".into(), usize::MAX, Vec::new()));
    sets.into_iter()
        .map(|(id, map, zones)| {
            ToRobotBody::ZoneSet(Arc::new(ZoneSet {
                id: ZoneSetId::new(id),
                map,
                descriptor: None,
                zones,
            }))
        })
        .collect()
}

fn instant_actions() -> ToRobotBody {
    let mut kinds = vec![
        InstantKind::StartCharging,
        InstantKind::StopCharging,
        InstantKind::Other(ActionType::new("")),
        InstantKind::CancelOrder { order_id: None },
        InstantKind::CancelOrder {
            order_id: Some(OrderId::new("")),
        },
    ];
    // every zone set sent: enable it, then try to delete it
    for i in 0..8 {
        for j in 0..18 {
            let zone_set = ZoneSetId::new(format!("{i}-{j}"));
            kinds.push(InstantKind::EnableZoneSet {
                zone_set: zone_set.clone(),
            });
            kinds.push(InstantKind::DeleteZoneSet { zone_set });
        }
    }
    let actions = kinds.into_iter().map(|kind| InstantAction {
        id: ActionId::new(""),
        kind,
    });
    ToRobotBody::InstantActions(actions.collect())
}

fn responses() -> ToRobotBody {
    let responses = [i64::MIN, -1, 0, i64::MAX]
        .into_iter()
        .flat_map(|lease| {
            ["", "req-1", "req-18446744073709551615"].map(|id| Response {
                request: RequestId::new(id),
                grant: Grant::Granted,
                lease_expiry: Some(Millis(lease)),
            })
        })
        .collect();
    ToRobotBody::Responses(responses)
}

#[test]
fn a_message_to_a_robot_with_wrong_numbers_does_not_panic() {
    for kinematics in [Kinematics::Differential, Kinematics::Omnidirectional] {
        for delay in [0, 3] {
            let mut w = world(kinematics, delay);
            let mut bodies = zone_sets();
            bodies.push(instant_actions());
            bodies.extend(orders());
            bodies.push(responses());
            for body in bodies {
                w.send(&to_robot(ROBOT, body)).unwrap();
                for _ in 0..5 {
                    w.step();
                    w.reports();
                }
            }
            // what is taken is driven on
            for _ in 0..1000 {
                w.step();
                w.reports();
            }
            w.send(&to_robot(ROBOT, responses())).unwrap();
            for _ in 0..10 {
                w.step();
            }
            w.snapshot();
            w.wait_totals();
        }
    }
}

#[test]
fn an_unread_message_with_any_content_is_taken() {
    let mut w = world(Kinematics::Differential, u64::MAX);
    let error = botsim_core::errors::RobotError {
        error_type: ErrorType::ValidationFailure,
        references: Vec::new(),
        description: None,
    };
    for unread in [
        Unread::Order(error.clone()),
        Unread::ZoneSet(error.clone()),
        Unread::InstantActions(Vec::new()),
        Unread::InstantActions(vec![InstantEntry::Unread {
            id: ActionId::new(""),
            kind: None,
            error,
        }]),
    ] {
        // (the longest delay there is: on its way for ever)
        assert_eq!(w.send_unread(ROBOT, unread), Ok(()));
    }
    w.step();
}

#[test]
fn numbers_that_name_nothing_are_errors_of_the_world() {
    let mut w = world(Kinematics::Differential, 0);
    for robot in [
        RobotNo::new(1, 0),
        RobotNo::new(u32::MAX, u32::MAX),
        RobotNo::new(0, 1),
    ] {
        let unknown = WorldError::UnknownRobotNo(robot);
        let message = to_robot(robot, responses());
        assert_eq!(w.send(&message), Err(unknown.clone()));
        let unread = Unread::InstantActions(Vec::new());
        assert_eq!(w.send_unread(robot, unread), Err(unknown.clone()));
        assert_eq!(w.report(robot), Err(unknown));
    }
    let header = |device| DeviceHeader {
        device,
        seq: u32::MAX,
        time: Millis(i64::MAX),
    };
    let door = ToDoor {
        header: header(DoorNo(u32::MAX)),
        command: DoorCommand::Open,
    };
    assert_eq!(
        w.command_door(&door),
        Err(WorldError::UnknownDoorNo(DoorNo(u32::MAX)))
    );
    let lift = |no, command| ToLift {
        header: DeviceHeader {
            device: LiftNo(no),
            seq: u32::MAX,
            time: Millis(i64::MAX),
        },
        command,
    };
    assert!(
        w.command_lift(&lift(u32::MAX, LiftCommand::Release))
            .is_err()
    );
    for (map, car_door) in [(usize::MAX, 0), (0, u32::MAX)] {
        let call = LiftCommand::Call { map, car_door };
        assert!(w.command_lift(&lift(0, call)).is_err());
    }
    w.step();
    assert_eq!(w.door_reports().len(), 1);
}

#[test]
fn an_order_with_numbers_outside_the_layout_is_refused_by_the_robot() {
    let mut w = world(Kinematics::Differential, 0);
    let message = to_robot(ROBOT, order("o", 0, 0, 0, vec![leg(99, 1, None)]));
    assert_eq!(
        w.send(&message),
        Ok(Some(Reception::Rejected(ErrorType::NoRouteToTarget)))
    );
    let errors = w.report(ROBOT).unwrap().state.errors;
    assert_eq!(
        errors[0].kind,
        ErrorKind::Predefined(ErrorType::NoRouteToTarget)
    );
}

#[test]
fn a_zone_set_the_robot_cannot_take_is_refused_with_the_matching_error() {
    let mut w = world(Kinematics::Differential, 0);
    let zone = |id: &str, corners: usize| Zone {
        id: ZoneId::new(id),
        descriptor: None,
        vertices: [(0.0, 5.0), (1.0, 5.0), (1.0, 6.0)][..corners].to_vec(),
        kind: ZoneKind::Blocked,
    };
    let set = |id: &str, map, zones| {
        let body = ToRobotBody::ZoneSet(Arc::new(ZoneSet {
            id: ZoneSetId::new(id),
            map,
            descriptor: None,
            zones,
        }));
        to_robot(ROBOT, body)
    };
    w.send(&set("map", 7, Vec::new())).unwrap();
    w.send(&set("twice", 0, vec![zone("z", 3), zone("z", 3)]))
        .unwrap();
    w.send(&set("corners", 0, vec![zone("z", 2)])).unwrap();
    // an empty id is a good one, as in botsim 0.1
    w.send(&set("", 0, vec![zone("", 3)])).unwrap();
    w.send(&set("", 0, vec![zone("", 3)])).unwrap();
    let state = w.report(ROBOT).unwrap().state;
    let errors: Vec<_> = state.errors.iter().map(|e| e.kind.clone()).collect();
    assert_eq!(
        errors,
        [
            ErrorType::UnknownMapId,
            ErrorType::ValidationFailure,
            ErrorType::ValidationFailure,
            ErrorType::DuplicateZoneSet,
        ]
        .map(ErrorKind::Predefined)
    );
    assert_eq!(state.zone_sets.len(), 1);
}

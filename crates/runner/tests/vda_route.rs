//! On the route through `vda`, what the translator refuses of a message to a robot
//! shows in that robot's errors: each kind of refusal, with its error type and
//! references, kept as long as the robot keeps its own refusals.

use std::sync::Arc;

use botsim::{connect::Connected, vda_route};
use botsim_link::ids::{ActionId, ActionType, OrderId, RequestId, ZoneId, ZoneSetId};
use botsim_link::instant::{InstantAction, InstantKind};
use botsim_link::message::{Header, Millis, ToRobot, ToRobotBody};
use botsim_link::order::{Action, Blocking, Leg, NodeAction, Order};
use botsim_link::response::{Grant, Response};
use botsim_link::state::{ActionStatus, ErrorKind, State};
use botsim_link::zone_set::{Zone, ZoneAction, ZoneKind, ZoneSet};
use botsim_model::errors::ErrorType::{self, *};
use botsim_server::cycle::{Message, robot_no};
use botsim_sim::actions::ActionTimes;
use botsim_sim::world::World;
use botsim_vda::translator::RobotMessage;
use serde_json::{Value, json};

mod setup;
use setup::{DT, id, line, robot_spec};

/// 2026-11-20T10:00:00Z.
const START: i64 = 1_795_168_800_000;

/// A world with the robot ACME/R-001 on N0 of the line N0 → … → N3 on map M, and
/// a controller on the route.
fn world(delay: u64) -> (World, Connected) {
    let g = line();
    let n0 = g.node_idx("N0").unwrap();
    let mut w = World::new(g, DT, START).unwrap();
    w.set_message_delay(delay);
    w.add_robot(id(1), robot_spec(), ActionTimes::new(1.0), n0, 0.0)
        .unwrap();
    let c = vda_route::controller(&w);
    (w, c)
}

/// The report of the robot as it is now (for a world without a delay).
fn report(w: &mut World) -> State {
    w.report(robot_no(0)).unwrap().state
}

type References = Vec<(String, String)>;

/// The errors of a report: type and references.
fn errors(state: &State) -> Vec<(ErrorType, References)> {
    state
        .errors
        .iter()
        .map(|e| {
            let ErrorKind::Predefined(kind) = e.kind else {
                panic!("a predefined type expected: {e:?}");
            };
            assert_eq!(e.level, kind.level());
            let references = e
                .references
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect();
            (kind, references)
        })
        .collect()
}

fn pairs(pairs: &[(&str, &str)]) -> References {
    pairs
        .iter()
        .map(|&(k, v)| (k.to_string(), v.to_string()))
        .collect()
}

/// A VDA 5050 message to the robot with the fields `rest`.
fn vda(rest: Value) -> Value {
    let mut message = json!({"headerId": 1, "timestamp": "2026-11-20T10:00:00.000Z",
        "version": "3.0.1", "manufacturer": "ACME", "serialNumber": "R-001"});
    for (key, value) in rest.as_object().unwrap() {
        message[key] = value.clone();
    }
    message
}

/// An order message N0 → N1, released.
fn order_message() -> Value {
    vda(json!({"orderId": "o", "orderUpdateId": 0,
        "nodes": [
            {"nodeId": "N0", "sequenceId": 0, "released": true, "actions": []},
            {"nodeId": "N1", "sequenceId": 2, "released": true, "actions": []}],
        "edges": [{"edgeId": "N0-N1", "sequenceId": 1, "released": true, "actions": []}]}))
}

fn raw_order(message: Value) -> vda5050::order::Order {
    serde_json::from_value(message).unwrap()
}

/// The robot gets `message`; it has no order and one error.
fn order_refused(message: vda5050::order::Order, kind: ErrorType) {
    let (mut w, mut c) = world(0);
    let order_id = message.order_id.clone();
    let update = message.order_update_id.to_string();
    let shown = format!("{message:?}");
    assert!(c.receive(&mut w, &RobotMessage::Order(message)));
    let state = report(&mut w);
    assert_eq!(state.order_id, None, "{shown}");
    let mut references = pairs(&[("orderId", &order_id)]);
    if kind.references_order_update_id() {
        references.push(("orderUpdateId".into(), update));
    }
    assert_eq!(errors(&state), [(kind, references)], "{shown}");
}

#[test]
fn every_refusal_of_an_order_message_is_in_the_robots_errors() {
    let with = |change: &dyn Fn(&mut Value)| {
        let mut message = order_message();
        change(&mut message);
        raw_order(message)
    };
    let invalid: [&dyn Fn(&mut Value); 8] = [
        // the counts, the sequence ids and the release rules
        &|m| m["edges"] = json!([]),
        &|m| m["edges"][0]["sequenceId"] = json!(7),
        &|m| m["nodes"][0]["released"] = json!(false),
        &|m| m["nodes"][1]["released"] = json!(false),
        &|m| {
            m["edges"][0]["released"] = json!(false);
        },
        // ids without a text, and the header
        &|m| m["orderId"] = json!(""),
        &|m| {
            m["nodes"][1]["actions"] =
                json!([{"actionId": "", "actionType": "pick", "blockingType": "HARD"}]);
        },
        &|m| m["version"] = json!("2.0.0"),
    ];
    for change in invalid {
        order_refused(with(change), ValidationFailure);
    }
    order_refused(
        with(&|m| m["timestamp"] = json!("yesterday")),
        ValidationFailure,
    );
    // a number that is not finite (JSON text cannot say one)
    let mut message = raw_order(order_message());
    message.edges[0].orientation = Some(f64::NAN);
    order_refused(message, ValidationFailure);
    // names the layout does not have
    order_refused(
        with(&|m| m["nodes"][1]["nodeId"] = json!("N9")),
        NoRouteToTarget,
    );
    order_refused(
        with(&|m| m["edges"][0]["edgeId"] = json!("N9-N1")),
        NoRouteToTarget,
    );
}

fn to_robot(body: ToRobotBody, time: i64) -> Message {
    let header = Header {
        robot: robot_no(0),
        seq: 0,
        time: Millis(time),
    };
    Message::Robot(ToRobot { header, body })
}

#[test]
fn every_refusal_of_an_order_on_its_way_out_is_in_the_robots_errors() {
    let leg = |edge, end, orientation| Leg {
        edge,
        end,
        orientation,
        orientation_type: None,
    };
    let order = |id: &str, start, legs: Vec<Leg>, actions| {
        Order::new(OrderId::new(id), 0, 0, start, legs, 0, actions).unwrap()
    };
    let nameless = vec![NodeAction {
        node: 0,
        action: Action {
            id: ActionId::new(""),
            kind: ActionType::new("pick"),
            blocking: Blocking::Hard,
        },
    }];
    let cases = [
        (
            order("o", 99, Vec::new(), Vec::new()),
            START,
            NoRouteToTarget,
        ),
        (
            order("o", 0, vec![leg(99, 1, None)], Vec::new()),
            START,
            NoRouteToTarget,
        ),
        (
            order("", 0, Vec::new(), Vec::new()),
            START,
            ValidationFailure,
        ),
        (
            order("o", 0, Vec::new(), nameless),
            START,
            ValidationFailure,
        ),
        (
            order("o", 0, vec![leg(0, 1, Some(f64::INFINITY))], Vec::new()),
            START,
            ValidationFailure,
        ),
        // a time no timestamp can say
        (
            order("o", 0, Vec::new(), Vec::new()),
            i64::MAX,
            ValidationFailure,
        ),
    ];
    for (order, time, kind) in cases {
        let (mut w, mut c) = world(0);
        let mut references = pairs(&[("orderId", order.order_id().as_str())]);
        if kind.references_order_update_id() {
            references.push(("orderUpdateId".into(), "0".into()));
        }
        let shown = format!("{order:?}");
        c.send(&mut w, &to_robot(ToRobotBody::Order(order), time));
        let state = report(&mut w);
        assert_eq!(state.order_id, None, "{shown}");
        assert_eq!(errors(&state), [(kind, references)], "{shown}");
    }
}

#[test]
fn an_order_error_stays_until_an_order_is_accepted() {
    let (mut w, mut c) = world(0);
    let mut bad = order_message();
    bad["nodes"][1]["nodeId"] = json!("N9");
    c.receive(&mut w, &RobotMessage::Order(raw_order(bad)));
    for _ in 0..1000 {
        w.step();
    }
    assert_eq!(report(&mut w).errors.len(), 1);
    c.receive(&mut w, &RobotMessage::Order(raw_order(order_message())));
    let state = report(&mut w);
    assert_eq!(state.order_id, Some(OrderId::new("o")));
    assert!(state.errors.is_empty());
}

/// The instant actions the robot lists: id and status.
fn listed(state: &State) -> Vec<(&str, ActionStatus)> {
    state
        .instant_actions
        .iter()
        .map(|a| (a.id.as_str(), a.status))
        .collect()
}

#[test]
fn an_instant_action_that_cannot_be_read_is_listed_as_failed_in_its_place() {
    use ActionStatus::{Failed, Finished};
    let (mut w, mut c) = world(0);
    let action = |id: &str, kind: &str, parameters: Value| {
        json!({"actionId": id, "actionType": kind, "blockingType": "NONE",
               "actionParameters": parameters})
    };
    let parameter = |key: &str, value: &str| json!([{"key": key, "value": value}]);
    let message = vda(json!({"actions": [
        action("first", "stopCharging", json!([])),
        action("no-set", "enableZoneSet", json!([])),
        action("empty-set", "deleteZoneSet", parameter("zoneSetId", "")),
        action("no-order", "cancelOrder", parameter("orderId", "")),
        action("no-type", "", json!([])),
        action("", "stopCharging", json!([])),
        action("last", "stopCharging", json!([])),
    ]}));
    let message = serde_json::from_value(message).unwrap();
    c.receive(&mut w, &RobotMessage::InstantActions(message));
    let state = report(&mut w);
    assert_eq!(
        listed(&state),
        [
            ("first", Finished),
            ("no-set", Failed),
            ("empty-set", Failed),
            ("no-order", Failed),
            ("no-type", Failed),
            ("", Failed),
            ("last", Finished),
        ]
    );
    let error = |kind, id: &str| (kind, pairs(&[("actionId", id)]));
    assert_eq!(
        errors(&state),
        [
            error(InvalidInstantAction, "no-set"),
            error(InvalidInstantAction, "empty-set"),
            error(NoOrderToCancel, "no-order"),
            error(InvalidInstantAction, "no-type"),
            error(InvalidInstantAction, ""),
        ]
    );
    assert!(!c.untranslated().unwrap().is_empty());
}

#[test]
fn instant_actions_left_out_on_their_way_out_keep_their_places() {
    use ActionStatus::{Failed, Finished};
    let (mut w, mut c) = world(0);
    let action = |id: &str, kind| InstantAction {
        id: ActionId::new(id),
        kind,
    };
    let stop = InstantKind::StopCharging;
    let actions = vec![
        action("first", stop.clone()),
        action(
            "empty-set",
            InstantKind::EnableZoneSet {
                zone_set: ZoneSetId::new(""),
            },
        ),
        action(
            "empty-set-2",
            InstantKind::DeleteZoneSet {
                zone_set: ZoneSetId::new(""),
            },
        ),
        action(
            "no-order",
            InstantKind::CancelOrder {
                order_id: Some(OrderId::new("")),
            },
        ),
        action("no-type", InstantKind::Other(ActionType::new(""))),
        action("known", InstantKind::Other(ActionType::new("cancelOrder"))),
        action("last", stop),
    ];
    c.send(
        &mut w,
        &to_robot(ToRobotBody::InstantActions(actions), START),
    );
    let state = report(&mut w);
    assert_eq!(
        listed(&state),
        [
            ("first", Finished),
            ("empty-set", Failed),
            ("empty-set-2", Failed),
            ("no-order", Failed),
            ("no-type", Failed),
            ("known", Failed),
            ("last", Finished),
        ]
    );
    let kinds: Vec<_> = errors(&state).into_iter().map(|(kind, _)| kind).collect();
    assert_eq!(
        kinds,
        [
            InvalidInstantAction,
            InvalidInstantAction,
            NoOrderToCancel,
            InvalidInstantAction,
            InvalidInstantAction,
        ]
    );
    // the types the entries were sent as
    let types: Vec<_> = state
        .instant_actions
        .iter()
        .map(|a| a.kind.as_ref().map_or("", ActionType::as_str))
        .collect();
    assert_eq!(
        types,
        [
            "stopCharging",
            "enableZoneSet",
            "deleteZoneSet",
            "cancelOrder",
            "",
            "cancelOrder",
            "stopCharging",
        ]
    );
}

/// A zone set message with one good zone.
fn zone_set_message() -> Value {
    vda(
        json!({"zoneSet": {"mapId": "M", "zoneSetId": "zs", "zones": [
        {"zoneId": "z", "zoneType": "RELEASE", "vertices": [
            {"x": 40.0, "y": 5.0}, {"x": 42.0, "y": 5.0}, {"x": 42.0, "y": 7.0}]}]}}),
    )
}

/// The zone set error is there for 10 s and then gone.
fn zone_set_refused(w: &mut World, kind: ErrorType, zone_set: &str, shown: &str) {
    let state = report(w);
    assert!(state.zone_sets.is_empty(), "{shown}");
    let expected = [(kind, pairs(&[("zoneSetId", zone_set)]))];
    assert_eq!(errors(&state), expected, "{shown}");
    for _ in 0..(9.9 / DT) as usize {
        w.step();
    }
    assert_eq!(errors(&report(w)), expected, "{shown}");
    for _ in 0..(0.2 / DT) as usize {
        w.step();
    }
    assert!(report(w).errors.is_empty(), "{shown}");
}

#[test]
fn every_refusal_of_a_zone_set_message_is_in_the_robots_errors_for_ten_seconds() {
    let with = |change: &dyn Fn(&mut Value)| {
        let mut message = zone_set_message();
        change(&mut message);
        serde_json::from_value::<vda5050::zone_set::ZoneSetMessage>(message).unwrap()
    };
    let mut not_finite = with(&|_| {});
    not_finite.zone_set.zones[0].vertices[0].x = f64::NAN;
    let cases = [
        (
            with(&|m| m["zoneSet"]["zoneSetId"] = json!("")),
            ValidationFailure,
        ),
        (
            with(&|m| m["zoneSet"]["zones"][0]["zoneId"] = json!("")),
            ValidationFailure,
        ),
        (
            with(&|m| {
                let zone = &mut m["zoneSet"]["zones"][0];
                zone["zoneType"] = json!("ACTION");
                zone["entryActions"] = json!([{"actionType": "", "blockingType": "NONE"}]);
                zone["duringActions"] = json!([]);
                zone["exitActions"] = json!([]);
            }),
            ValidationFailure,
        ),
        (not_finite, ValidationFailure),
        (with(&|m| m["version"] = json!("2.0.0")), ValidationFailure),
        (
            with(&|m| m["zoneSet"]["mapId"] = json!("nowhere")),
            UnknownMapId,
        ),
    ];
    for (message, kind) in cases {
        let (mut w, mut c) = world(0);
        let zone_set = message.zone_set.zone_set_id.clone();
        let shown = format!("{message:?}");
        c.receive(&mut w, &RobotMessage::ZoneSet(message));
        zone_set_refused(&mut w, kind, &zone_set, &shown);
    }
}

#[test]
fn every_refusal_of_a_zone_set_on_its_way_out_is_in_the_robots_errors() {
    let set = |map, vertices: usize, kind| ZoneSet {
        id: ZoneSetId::new("zs"),
        map,
        descriptor: None,
        zones: vec![Zone {
            id: ZoneId::new("z"),
            descriptor: None,
            vertices: [(40.0, 5.0), (42.0, 5.0), (42.0, 7.0)][..vertices].to_vec(),
            kind,
        }],
    };
    let nameless = ZoneAction {
        kind: ActionType::new(""),
        descriptor: None,
        blocking: Blocking::None,
        parameters: None,
        retriable: None,
    };
    let cases = [
        (set(9, 3, ZoneKind::Blocked), START, UnknownMapId),
        (set(0, 2, ZoneKind::Blocked), START, ValidationFailure),
        (
            set(0, 3, ZoneKind::Priority { factor: 2.0 }),
            START,
            ValidationFailure,
        ),
        (
            set(
                0,
                3,
                ZoneKind::SpeedLimit {
                    maximum_speed: f64::NAN,
                },
            ),
            START,
            ValidationFailure,
        ),
        (
            set(
                0,
                3,
                ZoneKind::Action {
                    entry: vec![nameless],
                    during: Vec::new(),
                    exit: Vec::new(),
                },
            ),
            START,
            ValidationFailure,
        ),
        (set(0, 3, ZoneKind::Blocked), i64::MAX, ValidationFailure),
    ];
    for (set, time, kind) in cases {
        let (mut w, mut c) = world(0);
        let shown = format!("{set:?}");
        let body = ToRobotBody::ZoneSet(Arc::new(set));
        c.send(&mut w, &to_robot(body, time));
        zone_set_refused(&mut w, kind, "zs", &shown);
    }
}

#[test]
fn what_vda_5050_has_no_error_for_is_only_counted() {
    let (mut w, mut c) = world(0);
    // a response without a request id, one whose end no timestamp can say, and
    // messages whose time cannot be said
    let response = |id: &str, lease| Response {
        request: RequestId::new(id),
        grant: Grant::Granted,
        lease_expiry: lease,
    };
    let responses = vec![response("", None), response("r", Some(Millis(i64::MAX)))];
    c.send(&mut w, &to_robot(ToRobotBody::Responses(responses), START));
    let stop = InstantAction {
        id: ActionId::new("s"),
        kind: InstantKind::StopCharging,
    };
    for body in [
        ToRobotBody::InstantActions(vec![stop]),
        ToRobotBody::Responses(vec![response("r", None)]),
    ] {
        c.send(&mut w, &to_robot(body, i64::MAX));
    }
    let message = vda(json!({"timestamp": "yesterday", "actions": [
        {"actionId": "s", "actionType": "stopCharging", "blockingType": "NONE"}]}));
    let message = serde_json::from_value(message).unwrap();
    c.receive(&mut w, &RobotMessage::InstantActions(message));
    let state = report(&mut w);
    assert!(state.errors.is_empty() && state.instant_actions.is_empty());
    assert_eq!(c.untranslated().unwrap().total(), 5);
    // and a message for a robot that is not there reaches nobody
    let mut other = order_message();
    other["serialNumber"] = json!("R-009");
    c.receive(&mut w, &RobotMessage::Order(raw_order(other)));
    assert!(report(&mut w).errors.is_empty());
    assert_eq!(c.untranslated().unwrap().total(), 6);
}

#[test]
fn a_refusal_arrives_when_the_message_would_have() {
    let delay = 10;
    let (mut w, mut c) = world(delay);
    let mut bad = order_message();
    bad["nodes"][1]["nodeId"] = json!("N9");
    c.receive(&mut w, &RobotMessage::Order(raw_order(bad)));
    // the first step at which the report, `delay` steps old, shows the error
    let first = (1..100u64).find(|_| {
        w.step();
        !w.report(robot_no(0)).unwrap().state.errors.is_empty()
    });
    // as for the robot's own refusals (`botsim-sim`, `tests/message_delay.rs`)
    assert_eq!(first, Some(2 * delay + 1));
}

#[test]
fn a_direct_connection_translates_and_refuses_nothing_on_its_own() {
    let (mut w, _) = world(0);
    let mut c = botsim::connect::controller(&w);
    // (empty ids are no reason to refuse, as in botsim 0.1)
    let order = Order::new(OrderId::new(""), 0, 0, 0, Vec::new(), 0, Vec::new()).unwrap();
    c.send(&mut w, &to_robot(ToRobotBody::Order(order), i64::MAX));
    let stop = InstantAction {
        id: ActionId::new(""),
        kind: InstantKind::StopCharging,
    };
    let body = ToRobotBody::InstantActions(vec![stop]);
    c.send(&mut w, &to_robot(body, i64::MAX));
    let state = report(&mut w);
    assert!(state.errors.is_empty(), "{:?}", state.errors);
    assert_eq!(listed(&state), [("", ActionStatus::Finished)]);
    assert!(c.untranslated().is_none());
    let raw = RobotMessage::Order(raw_order(order_message()));
    assert!(!c.receive(&mut w, &raw), "only on the route");
}

//! What is refused or left out of instant actions, responses and zone sets.

mod common;

use std::sync::Arc;

use botsim_link::ids::{ActionId, ActionType, OrderId, RequestId, ZoneId, ZoneSetId};
use botsim_link::instant::{InstantAction, InstantKind};
use botsim_link::message::{Millis, ToRobotBody};
use botsim_link::zone_set::{ZoneKind, ZoneSet};
use botsim_model::errors::ErrorType;
use botsim_vda::report::{Issue, Item, Problem, Translated};
use botsim_vda::translator::RobotMessage;
use common::*;
use serde_json::json;
use vda5050::instant_actions::InstantActions;
use vda5050::responses::Responses;
use vda5050::zone_set::ZoneSetMessage;

fn instants(actions: serde_json::Value) -> InstantActions {
    serde_json::from_value(json!({
        "headerId": 1, "timestamp": "2026-11-20T10:00:00.000Z", "version": "3.0.1",
        "manufacturer": "ACME", "serialNumber": "R-001", "actions": actions,
    }))
    .unwrap()
}

fn kinds(message: &InstantActions) -> (Vec<InstantKind>, Vec<Issue>) {
    let out = translator().instant_actions_from_vda(message).unwrap();
    let ToRobotBody::InstantActions(actions) = out.message.body else {
        panic!("not instant actions");
    };
    (actions.into_iter().map(|a| a.kind).collect(), out.issues)
}

#[track_caller]
fn left_out(issue: &Issue, error: ErrorType, action_id: &str, entry: usize) {
    assert_eq!(issue.error, Some(error));
    assert_eq!(
        issue.references,
        [(Arc::from("actionId"), Arc::from(action_id))]
    );
    assert_eq!(issue.entry, Some(entry));
}

#[test]
fn a_bad_instant_action_is_left_out_and_the_others_pass() {
    let message = instants(json!([
        {"actionType": "enableZoneSet", "actionId": "e", "blockingType": "NONE"},
        {"actionType": "startCharging", "actionId": "s", "blockingType": "NONE"},
        {"actionType": "deleteZoneSet", "actionId": "d", "blockingType": "NONE",
         "actionParameters": [{"key": "zoneSetId", "value": 7}]},
        {"actionType": "cancelOrder", "actionId": "c", "blockingType": "NONE",
         "actionParameters": [{"key": "orderId", "value": ""}]},
        {"actionType": "", "actionId": "t", "blockingType": "NONE"},
        {"actionType": "beep", "actionId": "", "blockingType": "NONE"},
    ]));
    let (taken, issues) = kinds(&message);
    assert_eq!(taken, [InstantKind::StartCharging]);
    assert_eq!(issues.len(), 5);
    left_out(&issues[0], ErrorType::InvalidInstantAction, "e", 0);
    left_out(&issues[1], ErrorType::InvalidInstantAction, "d", 2);
    left_out(&issues[2], ErrorType::NoOrderToCancel, "c", 3);
    left_out(&issues[3], ErrorType::InvalidInstantAction, "t", 4);
    left_out(&issues[4], ErrorType::InvalidInstantAction, "", 5);
}

#[test]
fn parameters_are_read_as_a_robot_reads_them() {
    let message = instants(json!([
        // an orderId that is no text counts as none
        {"actionType": "cancelOrder", "actionId": "1", "blockingType": "NONE",
         "actionParameters": [{"key": "orderId", "value": 5}]},
        // other parameters do not matter; of two the first counts
        {"actionType": "enableZoneSet", "actionId": "2", "blockingType": "NONE",
         "actionParameters": [{"key": "x", "value": 1}, {"key": "zoneSetId", "value": "a"},
                              {"key": "zoneSetId", "value": "b"}]},
        {"actionType": "beep", "actionId": "3", "blockingType": "NONE",
         "actionParameters": [{"key": "x", "value": 1}]},
    ]));
    let (taken, issues) = kinds(&message);
    assert_eq!(issues, Vec::new());
    assert_eq!(
        taken,
        [
            InstantKind::CancelOrder { order_id: None },
            InstantKind::EnableZoneSet {
                zone_set: ZoneSetId::new("a")
            },
            InstantKind::Other(ActionType::new("beep")),
        ]
    );
}

#[test]
fn instant_actions_that_cannot_be_written_are_left_out() {
    let action = |id: &str, kind| InstantAction {
        id: ActionId::new(id),
        kind,
    };
    let actions = vec![
        action("", InstantKind::StopCharging),
        action(
            "c",
            InstantKind::CancelOrder {
                order_id: Some(OrderId::new("")),
            },
        ),
        action(
            "e",
            InstantKind::EnableZoneSet {
                zone_set: ZoneSetId::new(""),
            },
        ),
        action("o", InstantKind::Other(ActionType::new(""))),
        action("r", InstantKind::Other(ActionType::new("cancelOrder"))),
        action("ok", InstantKind::StopCharging),
    ];
    let out = translator()
        .to_robot_to_vda(&to_robot(ToRobotBody::InstantActions(actions)))
        .unwrap();
    let RobotMessage::InstantActions(m) = out.message else {
        panic!("not instant actions");
    };
    assert_eq!(m.actions.len(), 1);
    assert_eq!(m.actions[0].action_id, "ok");
    let invalid = ErrorType::InvalidInstantAction;
    left_out(&out.issues[0], invalid, "", 0);
    left_out(&out.issues[1], ErrorType::NoOrderToCancel, "c", 1);
    left_out(&out.issues[2], invalid, "e", 2);
    left_out(&out.issues[3], invalid, "o", 3);
    left_out(&out.issues[4], invalid, "r", 4);
    assert!(matches!(
        out.issues[4].problem,
        Problem::ReadsBackDifferently { .. }
    ));
    assert_eq!(out.issues.len(), 5);
}

#[test]
fn a_bad_response_is_left_out_and_a_bad_lease_counts_as_none() {
    let message: Responses = serde_json::from_value(json!({
        "headerId": 1, "timestamp": "2026-11-20T10:00:00.000Z", "version": "3.0.1",
        "manufacturer": "ACME", "serialNumber": "R-001",
        "responses": [
            {"requestId": "", "grantType": "GRANTED"},
            {"requestId": "r", "grantType": "GRANTED", "leaseExpiry": "soon"},
            {"requestId": "s", "grantType": "GRANTED",
             "leaseExpiry": "99999999999999999-01-01T00:00:00Z"},
        ],
    }))
    .unwrap();
    let out = translator().responses_from_vda(&message).unwrap();
    let ToRobotBody::Responses(taken) = out.message.body else {
        panic!("not responses");
    };
    let taken: Vec<_> = taken
        .iter()
        .map(|r| (r.request.as_str(), r.lease_expiry))
        .collect();
    assert_eq!(taken, [("r", None), ("s", None)]);
    let entries: Vec<_> = out.issues.iter().map(|i| i.entry).collect();
    assert_eq!(entries, [Some(0), Some(1), Some(2)]);
    assert!(out.issues.iter().all(|i| i.error.is_none()));
}

#[test]
fn a_grant_whose_end_cannot_be_written_is_not_sent() {
    let mut list = responses();
    list[0].lease_expiry = Some(Millis(i64::MIN));
    list[1].request = RequestId::new("");
    let out = translator()
        .to_robot_to_vda(&to_robot(ToRobotBody::Responses(list)))
        .unwrap();
    let RobotMessage::Responses(m) = out.message else {
        panic!("not responses");
    };
    let ids: Vec<_> = m.responses.iter().map(|r| r.request_id.as_str()).collect();
    assert_eq!(ids, ["r-2", "r-3"]);
    let entries: Vec<_> = out.issues.iter().map(|i| i.entry).collect();
    assert_eq!(entries, [Some(0), Some(1)]);
}

fn zone_set_out(set: ZoneSet) -> Result<Translated<RobotMessage>, Issue> {
    translator().to_robot_to_vda(&to_robot(ToRobotBody::ZoneSet(Arc::new(set))))
}

#[track_caller]
fn zone_set_refused(issue: Issue, problem: Problem, error: ErrorType, id: &str) {
    assert_eq!(issue.problem, problem);
    assert_eq!(issue.error, Some(error));
    assert_eq!(issue.references, [(Arc::from("zoneSetId"), Arc::from(id))]);
}

#[test]
fn a_zone_set_that_the_schema_forbids_is_not_sent() {
    let invalid = ErrorType::ValidationFailure;
    let refused = |set| zone_set_out(set).unwrap_err();

    let mut set = zone_set(vec![zone("z", ZoneKind::Blocked)]);
    set.zones[0].vertices.pop();
    let problem = Problem::TooFewVertices {
        zone: "z".into(),
        there: 2,
    };
    zone_set_refused(refused(set), problem, invalid, "zs");

    let mut set = zone_set(vec![zone("z", ZoneKind::Blocked)]);
    set.zones[0].vertices[1].1 = f64::NAN;
    let problem = Problem::NotFinite {
        field: "zones[].vertices[]",
    };
    zone_set_refused(refused(set), problem, invalid, "zs");

    for factor in [-0.1, 1.5] {
        let set = zone_set(vec![zone("z", ZoneKind::Priority { factor })]);
        let problem = Problem::OutOfRange {
            field: "zones[].priorityFactor",
        };
        zone_set_refused(refused(set), problem, invalid, "zs");
    }
    for kind in [
        ZoneKind::Penalty { factor: f64::NAN },
        ZoneKind::SpeedLimit {
            maximum_speed: f64::INFINITY,
        },
    ] {
        let issue = refused(zone_set(vec![zone("z", kind)]));
        assert!(
            matches!(issue.problem, Problem::NotFinite { .. }),
            "{issue}"
        );
        assert_eq!(issue.error, Some(invalid));
    }

    let mut set = zone_set(vec![zone("z", ZoneKind::Blocked)]);
    set.zones[0].id = ZoneId::new("");
    let problem = Problem::UnusableId {
        field: "zones[].zoneId",
    };
    zone_set_refused(refused(set), problem, invalid, "zs");

    let mut set = zone_set(Vec::new());
    set.id = ZoneSetId::new("");
    let problem = Problem::UnusableId {
        field: "zoneSet.zoneSetId",
    };
    zone_set_refused(refused(set), problem, invalid, "");

    let mut set = zone_set(Vec::new());
    set.map = 1;
    let problem = Problem::OutsideLayout {
        item: Item::Map,
        number: 1,
    };
    zone_set_refused(refused(set), problem, ErrorType::UnknownMapId, "zs");
}

fn zone_set_message(map: &str, zone: serde_json::Value) -> ZoneSetMessage {
    serde_json::from_value(json!({
        "headerId": 1, "timestamp": "2026-11-20T10:00:00.000Z", "version": "3.0.1",
        "manufacturer": "ACME", "serialNumber": "R-001",
        "zoneSet": {"mapId": map, "zoneSetId": "zs", "zones": [zone]},
    }))
    .unwrap()
}

#[test]
fn an_incoming_zone_set_needs_a_known_map_and_finite_numbers() {
    let t = translator();
    let blocked = json!({"zoneId": "z", "zoneType": "BLOCKED", "vertices": []});
    let issue = t
        .zone_set_from_vda(&zone_set_message("N", blocked.clone()))
        .unwrap_err();
    let problem = Problem::UnknownName {
        item: Item::Map,
        name: "N".into(),
    };
    zone_set_refused(issue, problem, ErrorType::UnknownMapId, "zs");

    let mut message = zone_set_message("M", blocked);
    message.zone_set.zones[0].kind = vda5050::zone_set::ZoneKind::SpeedLimit {
        maximum_speed: f64::NAN,
    };
    let issue = t.zone_set_from_vda(&message).unwrap_err();
    assert_eq!(issue.error, Some(ErrorType::ValidationFailure));
}

#[test]
fn a_zone_set_on_its_own_is_translated_as_in_a_message() {
    let t = translator();
    let blocked = json!({"zoneId": "z", "zoneType": "BLOCKED", "vertices": []});
    let message = zone_set_message("M", blocked.clone());
    let ToRobotBody::ZoneSet(in_message) = t.zone_set_from_vda(&message).unwrap().message.body
    else {
        panic!("not a zone set");
    };
    assert_eq!(
        t.zone_set_content_from_vda(&message.zone_set),
        Ok((*in_message).clone())
    );

    let mut message = zone_set_message("M", blocked);
    message.zone_set.zones[0].zone_id = String::new();
    let issue = t.zone_set_content_from_vda(&message.zone_set).unwrap_err();
    let problem = Problem::UnusableId {
        field: "zones[].zoneId",
    };
    zone_set_refused(issue, problem, ErrorType::ValidationFailure, "zs");
}

#[test]
fn polygons_and_ranges_of_an_incoming_zone_set_are_left_to_the_robot() {
    let zone = json!({"zoneId": "z", "zoneType": "PRIORITY", "priorityFactor": 5.0,
                      "vertices": [{"x": 0.0, "y": 0.0}]});
    let out = translator()
        .zone_set_from_vda(&zone_set_message("M", zone))
        .unwrap();
    assert_eq!(out.issues, Vec::new());
}

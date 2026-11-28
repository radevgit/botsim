//! Every message the translator produces is valid against the official VDA 5050
//! 3.0.1 JSON schema (copies in `data/`, see `data/SOURCE.md`).

mod common;

use botsim_link::message::{Millis, ToRobotBody};
use botsim_link::order::{Leg, Order};
use botsim_link::state::State;
use botsim_vda::translator::RobotMessage;
use common::*;
use serde_json::Value;

fn validator(schema: &str) -> jsonschema::Validator {
    let text = match schema {
        "order" => include_str!("data/order.schema"),
        "instantActions" => include_str!("data/instantActions.schema"),
        "responses" => include_str!("data/responses.schema"),
        "zoneSet" => include_str!("data/zoneSet.schema"),
        "state" => include_str!("data/state.schema"),
        other => panic!("no schema {other}"),
    };
    let schema: Value = serde_json::from_str(text).unwrap();
    jsonschema::options()
        .should_validate_formats(true)
        .build(&schema)
        .unwrap()
}

fn assert_valid_value(schema: &str, value: Value) {
    let errors: Vec<String> = validator(schema)
        .iter_errors(&value)
        .map(|e| format!("{e} at {}", e.instance_path()))
        .collect();
    assert!(errors.is_empty(), "{schema}: {errors:?}\n{value}");
}

macro_rules! assert_valid {
    ($schema:literal, $message:expr) => {
        assert_valid_value($schema, serde_json::to_value($message).unwrap())
    };
}

fn check(message: &RobotMessage) {
    match message {
        RobotMessage::Order(m) => assert_valid!("order", m),
        RobotMessage::InstantActions(m) => assert_valid!("instantActions", m),
        RobotMessage::Responses(m) => assert_valid!("responses", m),
        RobotMessage::ZoneSet(m) => assert_valid!("zoneSet", m),
    }
}

/// The checks of this file can fail.
#[test]
fn the_schemas_are_in_force() {
    let t = translator();
    let valid = serde_json::to_value(vda_order(&t, order(0, 2))).unwrap();
    let schema = validator("order");
    assert!(schema.is_valid(&valid));
    // a copy of the valid message with one part spoiled
    let spoiled = |spoil: fn(&mut Value)| {
        let mut message = valid.clone();
        spoil(&mut message);
        message
    };
    for message in [
        spoiled(|m| m["timestamp"] = "yesterday".into()),
        spoiled(|m| m["edges"][0]["orientation"] = 3.2.into()),
        spoiled(|m| m["nodes"][0]["sequenceId"] = Value::Null),
    ] {
        assert!(!schema.is_valid(&message), "{message}");
    }
}

#[test]
fn messages_to_a_robot_are_valid() {
    let t = translator();
    for message in to_robot_messages() {
        check(&t.to_robot_to_vda(&message).unwrap().message);
    }
}

#[test]
fn state_messages_are_valid() {
    let t = translator();
    for state in states() {
        assert_valid!(
            "state",
            &t.state_to_vda(&from_robot(state)).unwrap().message
        );
    }
}

#[test]
#[allow(clippy::approx_constant)] // (3.1416 is the limit of the schemas)
fn messages_with_changed_angles_are_valid() {
    let t = translator();
    for angle in [3.1416, -3.1416, 3.14161, -4.0, 100.0, 1e300] {
        let leg = Leg {
            orientation: Some(angle),
            ..leg(0, 1)
        };
        let order = Order::new(
            order(0, 0).order_id().clone(),
            0,
            0,
            0,
            vec![leg],
            1,
            vec![],
        )
        .unwrap();
        let out = t
            .to_robot_to_vda(&to_robot(ToRobotBody::Order(order)))
            .unwrap();
        check(&out.message);

        let mut state: State = full_state();
        if let Some(p) = state.position.as_mut() {
            p.theta = angle;
        }
        assert_valid!(
            "state",
            &t.state_to_vda(&from_robot(state)).unwrap().message
        );
    }
}

#[test]
fn messages_of_the_first_and_last_writable_time_are_valid() {
    let t = translator();
    for time in [Millis(-62_167_219_200_000), Millis(253_402_300_799_999)] {
        for mut message in to_robot_messages() {
            message.header.time = time;
            check(&t.to_robot_to_vda(&message).unwrap().message);
        }
        let mut message = from_robot(full_state());
        message.header.time = time;
        assert_valid!("state", &t.state_to_vda(&message).unwrap().message);
    }
}

/// What is left out must leave a valid message behind.
#[test]
fn messages_with_parts_left_out_are_valid() {
    use botsim_link::ids::{ActionId, RequestId};

    let t = translator();
    let mut actions = instant_actions();
    actions[1].id = ActionId::new("");
    let mut answers = responses();
    answers[0].request = RequestId::new("");
    for body in [
        ToRobotBody::InstantActions(actions),
        ToRobotBody::Responses(answers),
    ] {
        let out = t.to_robot_to_vda(&to_robot(body)).unwrap();
        assert_eq!(out.issues.len(), 1);
        check(&out.message);
    }

    let mut state = full_state();
    state.actions[0].id = ActionId::new("");
    state.errors[0].kind = botsim_link::state::ErrorKind::Other("".into());
    if let Some(v) = state.velocity.as_mut() {
        v.vx = f64::NAN;
    }
    if let Some(p) = state.position.as_mut() {
        p.x = f64::INFINITY;
    }
    if let Some(zr) = state.zone_requests.as_mut() {
        zr.clear();
    }
    state.zone_sets[0].map = 9;
    let out = t.state_to_vda(&from_robot(state)).unwrap();
    assert_eq!(out.issues.len(), 5);
    assert_valid!("state", &out.message);
}

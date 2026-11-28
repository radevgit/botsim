//! `link` to VDA 5050 and back gives the same value.

mod common;

use botsim_link::message::Millis;
use botsim_vda::VERSION;
use botsim_vda::translator::{RobotMessage, Translator};
use common::*;

/// Through the JSON value, as the message would travel.
macro_rules! travel {
    ($message:expr) => {{
        let value = serde_json::to_value($message).unwrap();
        let back = serde_json::from_value(value).unwrap();
        assert_eq!(&back, $message);
        back
    }};
}

fn back(t: &Translator, message: &RobotMessage) -> botsim_link::message::ToRobot {
    let out = match message {
        RobotMessage::Order(m) => t.order_from_vda(&travel!(m)),
        RobotMessage::InstantActions(m) => t.instant_actions_from_vda(&travel!(m)),
        RobotMessage::Responses(m) => t.responses_from_vda(&travel!(m)),
        RobotMessage::ZoneSet(m) => t.zone_set_from_vda(&travel!(m)),
    }
    .unwrap();
    assert_eq!(out.issues, Vec::new());
    out.message
}

#[test]
fn messages_to_a_robot_come_back_the_same() {
    let t = translator();
    for message in to_robot_messages() {
        let there = t.to_robot_to_vda(&message).unwrap();
        assert_eq!(there.issues, Vec::new(), "{message:?}");
        assert_eq!(back(&t, &there.message), message);
    }
}

#[test]
fn state_reports_come_back_the_same() {
    let t = translator();
    for state in states() {
        let message = from_robot(state);
        let there = t.state_to_vda(&message).unwrap();
        assert_eq!(there.issues, Vec::new(), "{message:?}");
        let back = t.state_from_vda(&travel!(&there.message)).unwrap();
        assert_eq!(back.issues, Vec::new());
        assert_eq!(back.message, message);
    }
}

#[test]
fn the_header_names_the_robot_the_time_and_the_version() {
    let t = translator();
    let m = vda_order(&t, order(0, 1));
    assert_eq!(m.header_id, 7);
    assert_eq!(m.timestamp, "2026-11-20T10:00:00.250Z");
    assert_eq!(m.version, VERSION);
    assert_eq!(
        (m.manufacturer.as_str(), m.serial_number.as_str()),
        ("ACME", "R-001")
    );
}

#[test]
fn the_first_and_the_last_time_that_can_be_written() {
    let t = translator();
    // 0000-01-01T00:00:00.000Z and 9999-12-31T23:59:59.999Z
    for time in [Millis(-62_167_219_200_000), Millis(253_402_300_799_999)] {
        let mut message = from_robot(idle_state());
        message.header.time = time;
        let there = t.state_to_vda(&message).unwrap();
        assert_eq!(t.state_from_vda(&there.message).unwrap().message, message);
    }
}

#[test]
fn a_state_without_an_order_says_so_with_empty_texts() {
    let t = translator();
    let m = t.state_to_vda(&from_robot(idle_state())).unwrap().message;
    assert_eq!(m.order_id, "");
    assert_eq!((m.last_node_id.as_str(), m.last_node_sequence_id), ("", 0));
    assert_eq!(m.zone_sets, None);
    assert_eq!(m.zone_requests, None);
}

#[test]
fn sequence_ids_follow_from_the_step() {
    let t = translator();
    let m = vda_order(&t, order(4, 1));
    let nodes: Vec<_> = m
        .nodes
        .iter()
        .map(|n| (n.sequence_id, n.released))
        .collect();
    let edges: Vec<_> = m
        .edges
        .iter()
        .map(|e| (e.sequence_id, e.released))
        .collect();
    assert_eq!(nodes, [(8, true), (10, true), (12, false)]);
    assert_eq!(edges, [(9, true), (11, false)]);

    let m = t.state_to_vda(&from_robot(full_state())).unwrap().message;
    assert_eq!(m.last_node_sequence_id, 6);
    let nodes: Vec<_> = m
        .node_states
        .iter()
        .map(|n| (n.sequence_id, n.released))
        .collect();
    let edges: Vec<_> = m
        .edge_states
        .iter()
        .map(|e| (e.sequence_id, e.released))
        .collect();
    assert_eq!(nodes, [(8, true), (10, false)]);
    assert_eq!(edges, [(7, true), (9, false)]);
}

//! What is refused or left out of a state report.

mod common;

use botsim_link::ids::{ActionId, ActionType, OrderId, RequestId, ZoneSetId};
use botsim_link::state::{Ahead, ErrorKind, LastNode, Route, State};
use botsim_model::errors::ErrorType;
use botsim_vda::report::{Item, Problem};
use common::*;
use vda5050::state as vda;

/// A valid message that uses every part.
fn message() -> vda::State {
    translator()
        .state_to_vda(&from_robot(full_state()))
        .unwrap()
        .message
}

/// The report read from the message, and the problems with their entries.
fn read(m: &vda::State) -> (State, Vec<(Problem, Option<usize>)>) {
    let out = translator().state_from_vda(m).unwrap();
    assert!(out.issues.iter().all(|i| i.error.is_none()));
    let issues = out.issues.into_iter().map(|i| (i.problem, i.entry));
    (out.message.state, issues.collect())
}

fn unknown(item: Item, name: &str) -> Problem {
    Problem::UnknownName {
        item,
        name: name.to_string(),
    }
}

#[test]
fn a_bad_header_or_charge_refuses_the_report() {
    let t = translator();
    let refused = |change: &dyn Fn(&mut vda::State)| {
        let mut m = message();
        change(&mut m);
        let issue = t.state_from_vda(&m).unwrap_err();
        assert_eq!(issue.error, None);
        issue.problem
    };
    assert!(matches!(
        refused(&|m| m.manufacturer = "Other".into()),
        Problem::UnknownRobot(_)
    ));
    assert_eq!(
        refused(&|m| m.version = "30.0".into()),
        Problem::Version("30.0".into())
    );
    assert_eq!(
        refused(&|m| m.timestamp = "now".into()),
        Problem::BadTime { field: "timestamp" }
    );
    let field = "powerSupply.stateOfCharge";
    assert_eq!(
        refused(&|m| m.power_supply.state_of_charge = f64::NAN),
        Problem::NotFinite { field }
    );
    for charge in [-1.0, 100.5] {
        assert_eq!(
            refused(&|m| m.power_supply.state_of_charge = charge),
            Problem::OutOfRange { field }
        );
    }
}

#[test]
fn an_unknown_last_node_leaves_nothing_ahead() {
    let mut m = message();
    m.last_node_id = "Z".into();
    let (state, issues) = read(&m);
    assert_eq!(state.route, Route::new(None, vec![], 0).unwrap());
    assert_eq!(issues, [(unknown(Item::Node, "Z"), None)]);
    // the rest of the report is there
    assert_eq!(state.position, full_state().position);
    assert_eq!(state.errors, full_state().errors);

    let mut m = message();
    m.last_node_sequence_id = u32::MAX;
    let (state, issues) = read(&m);
    assert_eq!(state.route.last_node(), None);
    assert!(matches!(
        issues[..],
        [(
            Problem::BadSequenceId {
                found: u32::MAX,
                ..
            },
            None
        )]
    ));

    let mut m = message();
    m.last_node_id = String::new();
    let (state, issues) = read(&m);
    assert_eq!(state.route.last_node(), None);
    assert_eq!(issues, [(Problem::AheadWithoutLastNode, None)]);

    // without a last node and nothing ahead there is nothing to report
    m.node_states.clear();
    m.edge_states.clear();
    assert_eq!(read(&m).1, []);
}

/// The route ahead is the first step only.
#[track_caller]
fn cut_after_one(m: &vda::State) -> Problem {
    let (state, mut issues) = read(m);
    assert_eq!(state.route.ahead(), [Ahead { edge: 0, node: 1 }]);
    assert_eq!(state.route.released(), 1);
    assert_eq!(issues.len(), 1, "{issues:?}");
    let (problem, entry) = issues.remove(0);
    assert_eq!(entry, Some(1));
    problem
}

#[test]
fn the_way_ahead_is_cut_at_the_first_bad_step() {
    let mut m = message();
    m.node_states[1].node_id = "Z".into();
    assert_eq!(cut_after_one(&m), unknown(Item::Node, "Z"));

    let mut m = message();
    m.edge_states[1].edge_id = "X-Y".into();
    assert_eq!(cut_after_one(&m), unknown(Item::Edge, "X-Y"));

    let mut m = message();
    m.node_states.pop();
    assert_eq!(
        cut_after_one(&m),
        Problem::UnpairedRoute { nodes: 1, edges: 2 }
    );

    // a gap
    let mut m = message();
    m.edge_states[1].sequence_id = 11;
    m.node_states[1].sequence_id = 12;
    assert!(matches!(
        cut_after_one(&m),
        Problem::BadSequenceId {
            expected: 9,
            found: 11,
            ..
        }
    ));

    // an edge released without its node
    let mut m = message();
    m.edge_states[1].released = true;
    assert_eq!(
        cut_after_one(&m),
        Problem::ReleaseMismatch { id: "C".into() }
    );
}

#[test]
fn nothing_is_released_after_an_unreleased_step() {
    let mut m = message();
    for (released, node) in [(false, 0), (true, 1)] {
        m.node_states[node].released = released;
        m.edge_states[node].released = released;
    }
    let (state, issues) = read(&m);
    assert_eq!(state.route.ahead().len(), 1);
    assert_eq!(state.route.released(), 0);
    assert_eq!(
        issues,
        [(Problem::ReleaseMismatch { id: "C".into() }, Some(1))]
    );
}

#[test]
fn the_highest_sequence_ids_do_not_overflow() {
    let mut m = message();
    m.last_node_sequence_id = u32::MAX - 3;
    m.edge_states[0].sequence_id = u32::MAX - 2;
    m.node_states[0].sequence_id = u32::MAX - 1;
    m.edge_states[1].sequence_id = u32::MAX;
    m.node_states[1].sequence_id = 0;
    let (state, issues) = read(&m);
    assert_eq!(state.route.ahead().len(), 1);
    assert!(matches!(
        issues[..],
        [(Problem::BadSequenceId { .. }, Some(1))]
    ));
}

#[test]
fn a_bad_position_or_velocity_is_left_out() {
    let mut m = message();
    m.mobile_robot_position.as_mut().unwrap().map_id = "N".into();
    m.velocity.as_mut().unwrap().omega = Some(f64::INFINITY);
    let (state, issues) = read(&m);
    assert_eq!((state.position, state.velocity), (None, None));
    assert_eq!(
        issues,
        [
            (unknown(Item::Map, "N"), None),
            (
                Problem::NotFinite {
                    field: "velocity.omega"
                },
                None
            ),
        ]
    );

    let mut m = message();
    m.mobile_robot_position.as_mut().unwrap().x = f64::NAN;
    assert_eq!(read(&m).0.position, None);
}

#[test]
fn a_missing_part_of_the_velocity_is_zero() {
    let mut m = message();
    m.velocity = Some(vda::Velocity {
        vx: Some(1.0),
        vy: None,
        omega: None,
    });
    let (state, issues) = read(&m);
    let v = state.velocity.unwrap();
    assert_eq!((v.vx, v.vy, v.omega), (1.0, 0.0, 0.0));
    assert_eq!(issues, []);
}

#[test]
fn an_incoming_angle_is_taken_as_it_is() {
    let mut m = message();
    m.mobile_robot_position.as_mut().unwrap().theta = 9.0;
    assert_eq!(read(&m).0.position.unwrap().theta, 9.0);
}

#[test]
fn bad_entries_are_left_out_and_the_others_stay() {
    let mut m = message();
    m.action_states[0].action_id = String::new();
    m.instant_action_states[1].action_type = Some(String::new());
    m.zone_requests.as_mut().unwrap()[2].zone_id = String::new();
    m.zone_sets.as_mut().unwrap()[0].map_id = "N".into();
    m.errors[3].error_type = String::new();
    let (state, issues) = read(&m);
    assert_eq!(state.actions.len(), full_state().actions.len() - 1);
    // an action without a usable type stays
    assert_eq!(
        state.instant_actions.len(),
        full_state().instant_actions.len()
    );
    assert_eq!(state.instant_actions[1].kind, None);
    assert_eq!(state.zone_requests.map(|r| r.len()), Some(4));
    assert_eq!(state.zone_sets.len(), 1);
    assert_eq!(state.errors.len(), full_state().errors.len() - 1);
    let entries: Vec<_> = issues.iter().map(|(_, entry)| *entry).collect();
    assert_eq!(entries, [Some(0), Some(1), Some(2), Some(0), Some(3)]);
}

#[test]
fn error_types_are_read_by_their_names() {
    let mut m = message();
    m.errors.truncate(2);
    m.errors[0].error_type = "NO_ROUTE_TO_TARGET".into();
    m.errors[1].error_type = "SOMETHING_NEW".into();
    let kinds: Vec<_> = read(&m).0.errors.into_iter().map(|e| e.kind).collect();
    assert_eq!(
        kinds,
        [
            ErrorKind::Predefined(ErrorType::NoRouteToTarget),
            ErrorKind::Other("SOMETHING_NEW".into()),
        ]
    );
}

#[test]
fn a_report_with_bad_numbers_is_written_without_those_parts() {
    let mut state = full_state();
    state.route = Route::new(
        Some(LastNode { node: 0, step: 3 }),
        vec![
            Ahead { edge: 0, node: 1 },
            Ahead { edge: 7, node: 2 },
            Ahead { edge: 1, node: 2 },
        ],
        3,
    )
    .unwrap();
    state.position.as_mut().unwrap().map = usize::MAX;
    state.velocity.as_mut().unwrap().vy = f64::NAN;
    let out = translator().state_to_vda(&from_robot(state)).unwrap();
    assert_eq!(out.message.node_states.len(), 1);
    assert_eq!(out.message.edge_states.len(), 1);
    assert_eq!(out.message.mobile_robot_position, None);
    assert_eq!(out.message.velocity, None);
    assert_eq!(out.issues.len(), 3);
    assert_eq!(out.issues[0].entry, Some(1));

    // an unknown last node: nothing ahead
    let mut state = full_state();
    state.route = Route::new(
        Some(LastNode { node: 5, step: 0 }),
        vec![Ahead { edge: 0, node: 1 }],
        1,
    )
    .unwrap();
    let out = translator().state_to_vda(&from_robot(state)).unwrap();
    assert_eq!(out.message.last_node_id, "");
    assert_eq!(out.message.node_states, []);
    assert_eq!(out.issues.len(), 1);
}

#[test]
fn unusable_values_of_a_report_are_not_written() {
    let mut state = full_state();
    state.order_id = Some(OrderId::new(""));
    state.actions[0].id = ActionId::new("");
    state.actions[1].kind = Some(ActionType::new(""));
    state.zone_requests.as_mut().unwrap()[0].id = RequestId::new("");
    state.zone_sets[1].id = ZoneSetId::new("");
    state.errors[0].kind = ErrorKind::Other("VALIDATION_FAILURE".into());
    state.errors[1].kind = ErrorKind::Other("".into());
    let out = translator().state_to_vda(&from_robot(state)).unwrap();
    assert_eq!(out.message.order_id, "");
    assert_eq!(
        out.message.action_states.len(),
        full_state().actions.len() - 1
    );
    assert_eq!(out.message.action_states[0].action_type, None);
    assert_eq!(out.message.zone_requests.map(|r| r.len()), Some(4));
    assert_eq!(out.message.zone_sets.map(|z| z.len()), Some(1));
    assert_eq!(out.message.errors.len(), full_state().errors.len() - 2);
    assert_eq!(out.issues.len(), 7);
}

#[test]
fn an_angle_outside_the_range_is_written_as_the_same_direction() {
    let mut state = full_state();
    state.position.as_mut().unwrap().theta = 3.5;
    let out = translator().state_to_vda(&from_robot(state)).unwrap();
    let theta = out.message.mobile_robot_position.unwrap().theta;
    assert!((theta - (3.5 - std::f64::consts::TAU)).abs() < 1e-12);
    assert_eq!(
        out.issues[0].problem,
        Problem::AngleNormalized {
            field: "mobileRobotPosition.theta",
            from: 3.5
        }
    );
}

#[test]
fn a_charge_a_hair_above_full_is_full_in_both_directions() {
    let t = translator();
    // what a full battery of 0.69 Wh shows as `100 · charge / capacity`
    let capacity: f64 = 0.69;
    let full = 100.0 * capacity / capacity;
    assert!(full > 100.0, "{full}");
    for charge in [full, 100.0 + 1e-9, 100.0] {
        let mut m = message();
        m.power_supply.state_of_charge = charge;
        let read = t.state_from_vda(&m).unwrap();
        assert_eq!(read.message.state.charge, 100.0);
        assert!(read.issues.is_empty());

        let mut state = full_state();
        state.charge = charge;
        let written = t.state_to_vda(&from_robot(state)).unwrap();
        assert_eq!(written.message.power_supply.state_of_charge, 100.0);
        assert!(written.issues.is_empty());
    }
    // further out is refused as before
    let field = "powerSupply.stateOfCharge";
    for charge in [100.0 + 3e-9, 100.000001, -1e-12] {
        let mut m = message();
        m.power_supply.state_of_charge = charge;
        assert_eq!(
            t.state_from_vda(&m).unwrap_err().problem,
            Problem::OutOfRange { field }
        );
        let mut state = full_state();
        state.charge = charge;
        assert_eq!(
            t.state_to_vda(&from_robot(state)).unwrap_err().problem,
            Problem::OutOfRange { field }
        );
    }
}

#[test]
fn a_report_with_an_unusable_charge_is_not_written() {
    let mut state = full_state();
    state.charge = 101.0;
    let issue = translator().state_to_vda(&from_robot(state)).unwrap_err();
    assert_eq!(
        issue.problem,
        Problem::OutOfRange {
            field: "powerSupply.stateOfCharge"
        }
    );
}

//! What a state report says about waiting and progress.

use botsim_link::ids::ActionId;
use botsim_link::state::{
    ActionState, ActionStatus, EmergencyStop, LastNode, OperatingMode, Route, State,
};

fn standing() -> State {
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

fn action(status: ActionStatus) -> ActionState {
    ActionState {
        id: ActionId::new("a"),
        kind: None,
        status,
    }
}

#[test]
fn a_robot_waits_when_it_stands_is_not_held_and_runs_no_action() {
    assert!(standing().is_waiting());
    let mut state = standing();
    state.driving = true;
    assert!(!state.is_waiting());
    let mut state = standing();
    state.field_violation = true;
    assert!(!state.is_waiting());
    let mut state = standing();
    state.actions = vec![action(ActionStatus::Waiting), action(ActionStatus::Running)];
    assert!(!state.is_waiting());
    state.actions.pop();
    assert!(state.is_waiting());
    // only the actions of the order make a robot busy
    state.instant_actions = vec![action(ActionStatus::Running)];
    assert!(state.is_waiting());
}

#[test]
fn the_last_step_is_that_of_the_last_node_or_0() {
    assert_eq!(standing().last_step(), 0);
    let mut state = standing();
    let node = Some(LastNode { node: 3, step: 7 });
    state.route = Route::new(node, Vec::new(), 0).unwrap();
    assert_eq!(state.last_step(), 7);
}

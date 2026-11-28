//! The waiting rule on a robot's state message, for tests: the rule is asked of the
//! report that the translator (`botsim-vda`) reads from it, as the fleet server
//! does.

use std::sync::Arc;

use botsim_model::robot::{RobotId, RobotNo};
use botsim_vda::translator::Translator;
use vda5050::state::State;

/// Is the robot that sent `state` waiting? The layout is the test's `graph()`.
pub fn is_waiting(state: &State) -> bool {
    let mut translator = Translator::new(Arc::new(super::graph()));
    let id = RobotId::new(&state.manufacturer, &state.serial_number);
    translator.add_robot(RobotNo::new(0, 0), id).unwrap();
    let read = translator.state_from_vda(state).unwrap();
    assert!(read.issues.is_empty(), "{:?}", read.issues);
    read.message.state.is_waiting()
}

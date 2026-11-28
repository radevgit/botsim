//! The table of robot numbers and identities.

mod common;

use botsim_model::robot::{RobotId, RobotNo};
use botsim_vda::report::Problem;
use botsim_vda::translator::{RobotTableError, Translator};
use common::*;

fn id(serial: &str) -> RobotId {
    RobotId::new("ACME", serial)
}

#[test]
fn a_number_and_an_identity_stand_for_each_other() {
    let t = translator();
    assert_eq!(t.robot_id(ROBOT), Some(&id("R-001")));
    assert_eq!(t.robot_no(&id("R-001")), Some(ROBOT));
    assert_eq!(t.robot_no(&id("R-002")), None);
    assert_eq!(t.graph().nodes.len(), 3);
}

#[test]
fn a_slot_and_an_identity_hold_one_robot() {
    let mut t = translator();
    for generation in [0, 1, 2] {
        assert_eq!(
            t.add_robot(RobotNo::new(ROBOT.slot(), generation), id("R-002")),
            Err(RobotTableError::SlotTaken(ROBOT))
        );
    }
    assert_eq!(
        t.add_robot(RobotNo::new(0, 0), id("R-001")),
        Err(RobotTableError::IdentityTaken(ROBOT))
    );
    t.add_robot(RobotNo::new(u32::MAX, u32::MAX), id("R-002"))
        .unwrap();
}

#[test]
fn the_number_of_a_robot_that_is_gone_is_old() {
    let mut t = translator();
    assert_eq!(t.remove_robot(ROBOT), Ok(id("R-001")));
    assert_eq!(t.robot_id(ROBOT), None);
    assert_eq!(t.robot_no(&id("R-001")), None);
    assert_eq!(
        t.remove_robot(ROBOT),
        Err(RobotTableError::OldNumber(ROBOT))
    );
    assert_eq!(
        t.add_robot(ROBOT, id("R-002")),
        Err(RobotTableError::OldNumber(ROBOT))
    );
    let message = from_robot(idle_state());
    assert_eq!(
        t.state_to_vda(&message).unwrap_err().problem,
        Problem::OldRobotNo(ROBOT)
    );

    // the slot takes a later generation, and the identity may come back
    let next = ROBOT.next_generation().unwrap();
    t.add_robot(next, id("R-001")).unwrap();
    assert_eq!(t.robot_id(next), Some(&id("R-001")));
    assert_eq!(t.robot_id(ROBOT), None);
    assert_eq!(
        t.remove_robot(ROBOT),
        Err(RobotTableError::OldNumber(ROBOT))
    );
}

#[test]
fn a_number_never_seen_is_unknown() {
    let mut t = translator();
    for no in [RobotNo::new(0, 0), RobotNo::new(ROBOT.slot(), 2)] {
        assert_eq!(t.robot_id(no), None);
        assert_eq!(t.remove_robot(no), Err(RobotTableError::UnknownNumber(no)));
        let mut message = from_robot(idle_state());
        message.header.robot = no;
        assert_eq!(
            t.state_to_vda(&message).unwrap_err().problem,
            Problem::UnknownRobotNo(no)
        );
    }
}

#[test]
fn an_identity_must_fit_a_topic() {
    let mut t = Translator::new(graph());
    for (manufacturer, serial) in [
        ("", "R-1"),
        ("ACME", ""),
        ("AC/ME", "R-1"),
        ("ACME", "R+1"),
        ("ACME", "R#1"),
        ("$ACME", "R-1"),
    ] {
        let id = RobotId::new(manufacturer, serial);
        assert_eq!(
            t.add_robot(ROBOT, id.clone()),
            Err(RobotTableError::UnusableIdentity(id))
        );
    }
    t.add_robot(ROBOT, RobotId::new("A C-ME_.:", "R-1"))
        .unwrap();
}

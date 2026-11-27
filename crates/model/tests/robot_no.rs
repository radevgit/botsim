//! What the small robot number promises.

use std::collections::HashSet;

use botsim_model::robot::RobotNo;

#[test]
fn the_slot_is_the_array_index() {
    let no = RobotNo::new(7, 3);
    assert_eq!(no.slot(), 7);
    assert_eq!(no.index(), 7);
    assert_eq!(no.generation(), 3);
    assert_eq!(RobotNo::new(u32::MAX, 0).index(), u32::MAX as usize);
}

#[test]
fn the_next_generation_keeps_the_slot_and_differs_from_the_one_before() {
    let gone = RobotNo::new(4, 0);
    let next = gone.next_generation().expect("a generation is left");
    assert_eq!((next.slot(), next.generation()), (4, 1));
    assert_ne!(next, gone);
    assert_eq!(HashSet::from([gone, next]).len(), 2);
}

#[test]
fn a_slot_without_a_generation_left_gives_none() {
    assert_eq!(
        RobotNo::new(2, u32::MAX - 1).next_generation(),
        Some(RobotNo::new(2, u32::MAX))
    );
    assert_eq!(RobotNo::new(2, u32::MAX).next_generation(), None);
}

#[test]
fn numbers_are_ordered_by_slot_then_generation() {
    let mut numbers = [RobotNo::new(1, 0), RobotNo::new(0, 5), RobotNo::new(0, 2)];
    numbers.sort_unstable();
    assert_eq!(
        numbers,
        [RobotNo::new(0, 2), RobotNo::new(0, 5), RobotNo::new(1, 0)]
    );
}

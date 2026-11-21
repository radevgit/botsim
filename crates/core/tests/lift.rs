use botsim_core::lift::{Call, DoorState, Lift, LiftError, LiftSpec, LiftStatus};

const DT: f64 = 0.05;

/// The calls for the given floors, all at the first door.
fn at_front(floors: &[usize]) -> Vec<Call> {
    floors
        .iter()
        .map(|&floor| Call { floor, door: 0 })
        .collect()
}

/// Three floors (maps 10, 11, 12): 4 s per floor, doors 2 s to open and 2 s to close.
fn spec(hold: Option<f64>) -> LiftSpec {
    LiftSpec {
        id: "L1".into(),
        floors: vec![10, 11, 12],
        doors: vec!["front".into()],
        stops: vec![],
        start_floor: 0,
        travel_seconds_per_floor: 4.0,
        door_open_seconds: 2.0,
        door_close_seconds: 2.0,
        door_hold_seconds: hold,
    }
}

fn lift(hold: Option<f64>) -> Lift {
    Lift::new(spec(hold)).unwrap()
}

fn run(l: &mut Lift, seconds: f64) {
    for _ in 0..(seconds / DT).round() as u32 {
        l.step(DT);
    }
}

/// Steps until `done` is true; returns the seconds it took.
fn until(l: &mut Lift, mut done: impl FnMut(&LiftStatus) -> bool) -> f64 {
    for i in 0..100_000 {
        if done(&l.status()) {
            return i as f64 * DT;
        }
        l.step(DT);
    }
    panic!("never happened: {:?}", l.status());
}

fn door(l: &Lift) -> DoorState {
    l.status().door
}

#[test]
fn a_lift_needs_two_different_floors_and_sensible_times() {
    let bad = |f: &dyn Fn(&mut LiftSpec)| {
        let mut s = spec(None);
        f(&mut s);
        Lift::new(s).unwrap_err()
    };
    assert_eq!(bad(&|s| s.floors = vec![10]), LiftError::BadFloors);
    assert_eq!(bad(&|s| s.floors = vec![10, 10]), LiftError::BadFloors);
    assert_eq!(bad(&|s| s.start_floor = 3), LiftError::BadStartFloor(3));
    for (what, f) in [
        (
            "travel_seconds_per_floor",
            &(|s: &mut LiftSpec| s.travel_seconds_per_floor = -1.0) as &dyn Fn(&mut LiftSpec),
        ),
        ("door_open_seconds", &|s| s.door_open_seconds = f64::NAN),
        ("door_close_seconds", &|s| {
            s.door_close_seconds = f64::INFINITY
        }),
        ("door_hold_seconds", &|s| s.door_hold_seconds = Some(0.0)),
    ] {
        assert_eq!(bad(f), LiftError::BadTime(what));
    }
}

#[test]
fn a_new_lift_stands_at_its_start_floor_with_the_door_closed() {
    let mut s = spec(None);
    s.start_floor = 2;
    let l = Lift::new(s).unwrap();
    let st = l.status();
    assert_eq!(
        (st.floor, st.position, st.door),
        (2, 2.0, DoorState::Closed)
    );
    assert!(st.moving_to.is_none() && st.queue.is_empty());
}

#[test]
fn a_call_moves_the_cabin_opens_the_door_and_the_door_stays_open() {
    let mut l = lift(None);
    l.call(1).unwrap();

    // moving: door closed, position between the floors
    l.step(DT);
    let mid = l.status();
    assert_eq!((mid.door, mid.moving_to), (DoorState::Closed, Some(1)));
    run(&mut l, 1.9);
    let p = l.status().position;
    assert!(p > 0.3 && p < 0.7, "about half way after 2 of 4 s: {p}");

    // 4 s of travel, then 2 s of opening
    let t = until(&mut l, |s| s.door == DoorState::Opening);
    assert!((t - 2.1).abs() < 0.2, "{t}"); // 2.0 s were left of the travel time
    assert_eq!(l.status().floor, 1);
    let t = until(&mut l, |s| s.door == DoorState::Open);
    assert!((t - 2.0).abs() < 0.1, "opening took {t} s");

    // it stays open and does not move without a release
    run(&mut l, 60.0);
    let st = l.status();
    assert_eq!((st.floor, st.door), (1, DoorState::Open));
}

#[test]
fn release_closes_an_open_door_and_does_nothing_otherwise() {
    let mut l = lift(None);
    l.release(); // idle, closed: nothing
    assert_eq!(door(&l), DoorState::Closed);
    l.call(1).unwrap();
    run(&mut l, 1.0);
    l.release(); // moving: nothing
    assert_eq!(l.status().moving_to, Some(1));
    until(&mut l, |s| s.door == DoorState::Open);
    l.release();
    assert_eq!(door(&l), DoorState::Closing);
    let t = until(&mut l, |s| s.door == DoorState::Closed);
    assert!((t - 2.0).abs() < 0.1, "closing took {t} s");
    // and it stays there
    run(&mut l, 30.0);
    assert_eq!((l.status().floor, door(&l)), (1, DoorState::Closed));
}

#[test]
fn a_call_for_the_floor_it_is_at_opens_the_door_or_changes_nothing() {
    let mut l = lift(None);
    l.call(0).unwrap(); // closed, at floor 0: opens
    let t = until(&mut l, |s| s.door == DoorState::Open);
    assert!((t - 2.05).abs() < 0.2, "{t}");

    l.call(0).unwrap(); // already open there: nothing queued, nothing happens
    assert!(l.status().queue.is_empty());
    run(&mut l, 5.0);
    assert_eq!(door(&l), DoorState::Open);
}

#[test]
fn calls_are_served_in_the_order_they_came() {
    let mut l = lift(None);
    l.call(2).unwrap();
    l.call(1).unwrap();
    l.call(0).unwrap();
    assert_eq!(l.status().queue, at_front(&[2, 1, 0]));

    let mut visited = Vec::new();
    for _ in 0..3 {
        until(&mut l, |s| s.door == DoorState::Open);
        visited.push(l.status().floor);
        l.release();
        until(&mut l, |s| s.door == DoorState::Closed);
        run(&mut l, DT * 2.0); // idle takes the next call
    }
    assert_eq!(visited, vec![2, 1, 0], "no reordering to save travel");
}

#[test]
fn the_same_call_is_not_queued_twice_in_a_row() {
    let mut l = lift(None);
    l.call(2).unwrap();
    l.step(DT); // it is on its way to 2
    l.call(2).unwrap(); // already going there
    assert!(l.status().queue.is_empty());
    l.call(1).unwrap();
    l.call(1).unwrap(); // same as the last queued call
    assert_eq!(l.status().queue, at_front(&[1]));
    l.call(2).unwrap(); // a different one after it: queued
    assert_eq!(l.status().queue, at_front(&[1, 2]));
}

#[test]
fn a_waiting_call_is_not_served_until_the_open_door_is_released() {
    let mut l = lift(None);
    l.call(1).unwrap();
    until(&mut l, |s| s.door == DoorState::Open);
    l.call(2).unwrap(); // somebody else wants the lift
    run(&mut l, 30.0);
    let st = l.status();
    assert_eq!(
        (st.floor, st.door, st.queue.clone()),
        (1, DoorState::Open, at_front(&[2]))
    );

    l.release();
    until(&mut l, |s| s.moving_to == Some(2));
    assert!(l.status().queue.is_empty());
    until(&mut l, |s| s.door == DoorState::Open);
    assert_eq!(l.status().floor, 2);
}

#[test]
fn a_door_hold_time_closes_the_door_by_itself() {
    let mut l = lift(Some(10.0));
    l.call(1).unwrap();
    l.call(2).unwrap();
    until(&mut l, |s| s.door == DoorState::Open);
    // open for 10 s, then it closes without a release and serves the next call
    let t = until(&mut l, |s| s.door == DoorState::Closing);
    assert!((t - 10.0).abs() < 0.2, "open for {t} s");
    until(&mut l, |s| s.moving_to == Some(2));
    until(&mut l, |s| s.door == DoorState::Open);
    assert_eq!(l.status().floor, 2);
    // with nothing queued it closes again after the hold time and stays closed
    until(&mut l, |s| s.door == DoorState::Closed);
    run(&mut l, 60.0);
    assert_eq!((l.status().floor, door(&l)), (2, DoorState::Closed));
}

#[test]
fn travelling_several_floors_takes_the_time_per_floor_each() {
    let mut l = lift(None);
    l.call(2).unwrap();
    let t = until(&mut l, |s| s.door == DoorState::Opening);
    assert!((t - 8.1).abs() < 0.2, "two floors at 4 s: {t}");
    // at 4 s it was half way = floor 1
    let mut l = lift(None);
    l.call(2).unwrap();
    run(&mut l, 4.0);
    assert!((l.status().position - 1.0).abs() < 0.1);
}

#[test]
fn a_call_for_an_unknown_floor_is_an_error_and_zero_times_make_instant_doors() {
    let mut l = lift(None);
    assert_eq!(l.call(3), Err(LiftError::UnknownFloor(3)));

    let mut s = spec(None);
    s.door_open_seconds = 0.0;
    s.door_close_seconds = 0.0;
    s.travel_seconds_per_floor = 0.0;
    let mut l = Lift::new(s).unwrap();
    l.call(2).unwrap();
    l.step(DT); // takes the call: instant travel
    l.step(DT);
    assert_eq!((l.status().floor, door(&l)), (2, DoorState::Open));
    l.release();
    assert_eq!(door(&l), DoorState::Closed);
}

fn through_cabin() -> Lift {
    let mut s = spec(None);
    s.doors = vec!["front".into(), "rear".into()];
    Lift::new(s).unwrap()
}

#[test]
fn a_lift_needs_one_or_more_doors_with_different_names() {
    let bad = |doors: &[&str]| {
        let mut s = spec(None);
        s.doors = doors.iter().map(|d| d.to_string()).collect();
        Lift::new(s).unwrap_err()
    };
    assert_eq!(bad(&[]), LiftError::BadDoors);
    assert_eq!(bad(&["a", "a"]), LiftError::BadDoors);
}

#[test]
fn a_lift_with_one_door_reports_it_and_refuses_another() {
    let mut l = lift(None);
    assert_eq!(
        (l.status().door_index, l.status().door_id),
        (0, "front".into())
    );
    assert_eq!(l.call_door(1, 1), Err(LiftError::UnknownDoor(1)));
}

#[test]
fn a_call_opens_the_door_that_was_asked_for() {
    let mut l = through_cabin();
    l.call_door(1, 1).unwrap();
    until(&mut l, |s| s.door == DoorState::Open);
    let st = l.status();
    assert_eq!((st.floor, st.door_index, st.door_id), (1, 1, "rear".into()));
    // `call` is the first door
    l.release();
    until(&mut l, |s| s.door == DoorState::Closed);
    l.call(1).unwrap();
    until(&mut l, |s| s.door == DoorState::Open);
    assert_eq!(l.status().door_id, "front");
}

#[test]
fn another_door_at_the_same_floor_waits_until_the_open_door_is_released() {
    let mut l = through_cabin();
    l.call_door(1, 0).unwrap();
    until(&mut l, |s| s.door == DoorState::Open);

    l.call_door(1, 0).unwrap(); // the same door: nothing to do
    assert!(l.status().queue.is_empty());
    l.call_door(1, 1).unwrap(); // the other door: a new call
    assert_eq!(l.status().queue, vec![Call { floor: 1, door: 1 }]);
    run(&mut l, 20.0);
    let st = l.status();
    assert_eq!(
        (st.door, st.door_index),
        (DoorState::Open, 0),
        "never two doors open"
    );

    l.release();
    until(&mut l, |s| s.door == DoorState::Closed);
    until(&mut l, |s| s.door == DoorState::Opening);
    let st = l.status();
    assert_eq!((st.floor, st.door_index), (1, 1), "same floor, no travel");
    assert!(st.queue.is_empty());
}

#[test]
fn calls_for_the_same_floor_and_different_doors_are_different_calls() {
    let mut l = through_cabin();
    l.call_door(2, 0).unwrap();
    l.step(DT); // on its way to floor 2, front door
    l.call_door(2, 0).unwrap(); // already going there
    assert!(l.status().queue.is_empty());
    l.call_door(2, 1).unwrap();
    l.call_door(2, 1).unwrap(); // same as the last queued
    assert_eq!(l.status().queue, vec![Call { floor: 2, door: 1 }]);
}

#[test]
fn the_same_commands_give_the_same_timeline() {
    let timeline = || {
        let mut l = lift(Some(6.0));
        let mut out = Vec::new();
        for step in 0..1500 {
            if step == 0 {
                l.call(2).unwrap();
            }
            if step == 40 {
                l.call(1).unwrap();
            }
            l.step(DT);
            out.push(l.status());
        }
        out
    };
    assert_eq!(timeline(), timeline());
}

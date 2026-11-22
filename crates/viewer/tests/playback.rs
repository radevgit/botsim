use botsim_viewer::{Playback, SPEEDS};

#[test]
fn it_starts_paused_at_the_start() {
    let p = Playback::new(2.0, 10.0);
    assert_eq!(
        (p.start(), p.end(), p.time(), p.speed()),
        (2.0, 10.0, 2.0, 1.0)
    );
    assert!(!p.is_playing() && !p.at_end());
}

#[test]
fn time_moves_only_while_playing_and_by_the_speed() {
    let mut p = Playback::new(0.0, 100.0);
    p.advance(1.0);
    assert_eq!(p.time(), 0.0, "paused");
    p.toggle();
    p.advance(0.5);
    assert_eq!(p.time(), 0.5);
    p.set_speed(10.0);
    p.advance(0.5);
    assert_eq!(p.time(), 5.5);
    p.toggle();
    p.advance(1.0);
    assert_eq!(p.time(), 5.5, "paused again");
}

#[test]
fn it_stops_at_the_end_and_plays_again_from_the_start() {
    let mut p = Playback::new(0.0, 10.0);
    p.set_speed(100.0);
    p.toggle();
    p.advance(1.0);
    assert_eq!(p.time(), 10.0);
    assert!(p.at_end() && !p.is_playing());
    p.toggle();
    assert!(p.is_playing());
    assert_eq!(p.time(), 0.0, "from the start");
}

#[test]
fn seeking_is_clamped_and_keeps_playing() {
    let mut p = Playback::new(1.0, 5.0);
    p.toggle();
    p.seek(3.0);
    assert_eq!(p.time(), 3.0);
    assert!(p.is_playing());
    p.seek(-4.0);
    assert_eq!(p.time(), 1.0);
    p.seek(99.0);
    assert_eq!(p.time(), 5.0);
    p.seek(f64::NAN);
    assert_eq!(p.time(), 1.0);
}

#[test]
fn bad_speeds_and_times_are_ignored() {
    let mut p = Playback::new(0.0, 10.0);
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        p.set_speed(bad);
        assert_eq!(p.speed(), 1.0);
    }
    p.toggle();
    for bad in [0.0, -1.0, f64::NAN] {
        p.advance(bad);
        assert_eq!(p.time(), 0.0);
    }
}

#[test]
fn an_empty_run_is_a_single_moment() {
    let mut p = Playback::new(3.0, 3.0);
    assert!(p.at_end());
    p.toggle();
    p.advance(1.0);
    assert_eq!(p.time(), 3.0);
    assert!(!p.is_playing());
    // an end before the start is treated as the start
    assert_eq!(Playback::new(5.0, 1.0).end(), 5.0);
}

#[test]
fn stepping_moves_by_seconds_and_stays_in_the_run() {
    let mut p = Playback::new(0.0, 100.0);
    p.step(1.0);
    assert_eq!(p.time(), 1.0);
    p.step(10.0);
    assert_eq!(p.time(), 11.0);
    p.step(-1.0);
    assert_eq!(p.time(), 10.0);
    p.step(-50.0);
    assert_eq!(p.time(), 0.0);
    p.step(500.0);
    assert_eq!(p.time(), 100.0);
    // stepping does not start or stop playing
    assert!(!p.is_playing());
    p.toggle();
    p.seek(50.0);
    p.step(1.0);
    assert!(p.is_playing());
}

#[test]
fn start_and_end_jump_to_the_ends() {
    let mut p = Playback::new(2.0, 9.0);
    p.toggle();
    p.seek(5.0);
    p.to_end();
    assert_eq!(p.time(), 9.0);
    assert!(!p.is_playing(), "playing stops at the end");
    p.to_start();
    assert_eq!(p.time(), 2.0);
}

#[test]
fn faster_and_slower_walk_the_speed_list_and_stop_at_its_ends() {
    let mut p = Playback::new(0.0, 10.0);
    assert_eq!(p.speed(), 1.0);
    p.faster();
    assert_eq!(p.speed(), 2.0);
    p.faster();
    assert_eq!(p.speed(), 5.0);
    p.slower();
    p.slower();
    assert_eq!(p.speed(), 1.0);
    p.slower();
    assert_eq!(p.speed(), 0.5);
    p.slower();
    assert_eq!(p.speed(), 0.5, "the lowest stays");
    for _ in 0..20 {
        p.faster();
    }
    assert_eq!(p.speed(), 100.0, "the highest stays");
    // a speed that is not in the list: the next one above / below
    p.set_speed(7.0);
    p.faster();
    assert_eq!(p.speed(), 10.0);
    p.set_speed(7.0);
    p.slower();
    assert_eq!(p.speed(), 5.0);
}

#[test]
fn the_speeds_include_real_time_and_go_up() {
    assert!(SPEEDS.contains(&1.0));
    assert!(SPEEDS.windows(2).all(|w| w[0] < w[1]));
}

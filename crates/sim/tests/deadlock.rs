use botsim_sim::deadlock::DeadlockDetector;

const DT: f64 = 0.05;

/// Observe `ticks` times (from tick 1) the same wait-for relation.
fn observe(d: &mut DeadlockDetector, from_tick: u64, ticks: u64, waits_for: &[Option<usize>]) {
    for t in from_tick..from_tick + ticks {
        d.observe(t, waits_for);
    }
}

#[test]
fn two_robots_waiting_for_each_other_are_a_deadlock_after_the_timeout() {
    let mut d = DeadlockDetector::new(30.0, DT); // 600 ticks
    let wf = [Some(1), Some(0)];
    observe(&mut d, 1, 600, &wf); // seen at tick 1, 599 ticks later: not yet
    assert!(d.records().is_empty());
    observe(&mut d, 601, 1, &wf);
    assert_eq!(d.records().len(), 1);
    let r = &d.records()[0];
    assert_eq!(r.robots, vec![0, 1]);
    assert_eq!(
        (r.first_seen_tick, r.detected_tick, r.ended_tick),
        (1, 601, None)
    );
    // it is reported once, however long it lasts
    observe(&mut d, 602, 2000, &wf);
    assert_eq!(d.records().len(), 1);
}

#[test]
fn a_ring_of_three_and_robots_outside_it() {
    let mut d = DeadlockDetector::new(1.0, DT);
    // 0 → 1 → 2 → 0 is a ring; 3 waits for 0 (not in the cycle); 4 waits for nobody
    let wf = [Some(1), Some(2), Some(0), Some(0), None];
    observe(&mut d, 1, 100, &wf);
    assert_eq!(d.records().len(), 1);
    assert_eq!(d.records()[0].robots, vec![0, 1, 2]);
}

#[test]
fn a_chain_without_a_cycle_is_no_deadlock() {
    let mut d = DeadlockDetector::new(1.0, DT);
    // 0 waits for 1, 1 waits for 2, 2 is not waiting
    observe(&mut d, 1, 1000, &[Some(1), Some(2), None]);
    assert!(d.records().is_empty());
    // a robot waiting for itself cannot happen, nothing waits at all
    observe(&mut d, 1001, 1000, &[None, None, None]);
    assert!(d.records().is_empty());
}

#[test]
fn two_separate_cycles_are_two_deadlocks() {
    let mut d = DeadlockDetector::new(1.0, DT);
    observe(&mut d, 1, 100, &[Some(1), Some(0), Some(3), Some(2)]);
    let robots: Vec<_> = d.records().iter().map(|r| r.robots.clone()).collect();
    assert_eq!(robots, vec![vec![0, 1], vec![2, 3]]);
}

#[test]
fn a_cycle_that_breaks_before_the_timeout_is_not_reported() {
    let mut d = DeadlockDetector::new(30.0, DT);
    observe(&mut d, 1, 500, &[Some(1), Some(0)]);
    observe(&mut d, 501, 1, &[None, None]); // broken
    // and forms again: the timeout starts from zero
    observe(&mut d, 502, 500, &[Some(1), Some(0)]);
    assert!(d.records().is_empty());
}

#[test]
fn a_reported_deadlock_records_when_it_ends_and_a_new_one_is_a_new_record() {
    let mut d = DeadlockDetector::new(1.0, DT); // 20 ticks
    observe(&mut d, 1, 50, &[Some(1), Some(0)]);
    assert_eq!(d.records()[0].ended_tick, None);
    d.observe(51, &[None, None]);
    assert_eq!(d.records()[0].ended_tick, Some(51));

    observe(&mut d, 52, 50, &[Some(1), Some(0)]);
    assert_eq!(d.records().len(), 2);
    assert_eq!(d.records()[1].first_seen_tick, 52);
}

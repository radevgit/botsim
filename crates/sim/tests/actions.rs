use botsim_link::order::Blocking::{self, Hard, None as NoBlock, Single, Soft};
use botsim_link::state::ActionStatus::{self, Finished, Running, Waiting};
use botsim_sim::actions::{ActionQueue, ActionTimes, PlannedAction};

const DT: f64 = 0.1;

fn act(id: &str, blocking: Blocking, duration: f64) -> PlannedAction {
    PlannedAction {
        action_id: id.into(),
        action_type: "t".into(),
        blocking,
        duration,
    }
}

fn states(q: &ActionQueue) -> Vec<(String, ActionStatus)> {
    q.states().map(|(id, _, s)| (id.to_string(), s)).collect()
}

fn st(list: &[(&str, ActionStatus)]) -> Vec<(String, ActionStatus)> {
    list.iter().map(|(i, s)| (i.to_string(), *s)).collect()
}

/// Steps until idle; returns the time that passed.
fn run_to_idle(q: &mut ActionQueue) -> f64 {
    let mut t = 0.0;
    while !q.is_idle() {
        q.step(DT);
        t += DT;
        assert!(t < 1000.0);
    }
    t
}

#[test]
fn durations_come_from_type_overrides_or_the_default() {
    let t = ActionTimes::new(2.0).with("pick", 5.0);
    assert_eq!(t.duration("pick"), 5.0);
    assert_eq!(t.duration("drop"), 2.0);
}

#[test]
fn empty_queue_allows_driving_and_is_idle() {
    let q = ActionQueue::new();
    assert!(q.driving_allowed() && q.is_idle());
}

#[test]
fn none_and_soft_actions_run_in_parallel() {
    let mut q = ActionQueue::new();
    q.push([act("a", NoBlock, 1.0), act("b", Soft, 2.0)]);
    assert_eq!(states(&q), st(&[("a", Running), ("b", Running)]));
    // the soft action stops driving
    assert!(!q.driving_allowed());
    let t = run_to_idle(&mut q);
    assert!((t - 2.0).abs() < DT + 1e-9, "t = {t}");
    assert!(q.driving_allowed());
}

#[test]
fn none_action_does_not_stop_driving() {
    let mut q = ActionQueue::new();
    q.push([act("a", NoBlock, 5.0)]);
    assert!(q.driving_allowed());
    assert!(!q.is_idle());
}

#[test]
fn single_allows_driving_but_waits_for_running_actions_and_runs_alone() {
    let mut q = ActionQueue::new();
    q.push([
        act("a", NoBlock, 1.0),
        act("s", Single, 1.0),
        act("b", NoBlock, 1.0),
    ]);
    // a runs; s waits for it; b waits behind s
    assert_eq!(
        states(&q),
        st(&[("a", Running), ("s", Waiting), ("b", Waiting)])
    );
    assert!(q.driving_allowed(), "SINGLE allows driving");

    for _ in 0..10 {
        q.step(DT);
    }
    // a is done (1.0 s), s runs alone, b still waits
    assert_eq!(
        states(&q),
        st(&[("a", Finished), ("s", Running), ("b", Waiting)])
    );
    for _ in 0..10 {
        q.step(DT);
    }
    assert_eq!(
        states(&q),
        st(&[("a", Finished), ("s", Finished), ("b", Running)])
    );
    run_to_idle(&mut q);
}

#[test]
fn hard_runs_alone_and_stops_driving() {
    let mut q = ActionQueue::new();
    q.push([act("h", Hard, 1.0), act("n", NoBlock, 1.0)]);
    assert_eq!(states(&q), st(&[("h", Running), ("n", Waiting)]));
    assert!(!q.driving_allowed());
    let t = run_to_idle(&mut q);
    // h then n, one after the other
    assert!((t - 2.0).abs() < 2.0 * DT + 1e-9, "t = {t}");
    assert!(q.driving_allowed());
}

#[test]
fn order_in_the_list_is_kept() {
    // waiting HARD holds back a NONE behind it even though NONE could run in parallel with a
    let mut q = ActionQueue::new();
    q.push([
        act("a", NoBlock, 1.0),
        act("h", Hard, 1.0),
        act("n", NoBlock, 1.0),
    ]);
    assert_eq!(
        states(&q),
        st(&[("a", Running), ("h", Waiting), ("n", Waiting)])
    );
    // driving is blocked already: an unfinished HARD is in the queue
    assert!(!q.driving_allowed());
}

#[test]
fn zero_duration_actions_finish_at_once() {
    let mut q = ActionQueue::new();
    q.push([
        act("a", Hard, 0.0),
        act("b", Soft, 0.0),
        act("c", Single, 0.0),
    ]);
    assert!(q.is_idle());
    assert_eq!(
        states(&q),
        st(&[("a", Finished), ("b", Finished), ("c", Finished)])
    );
}

#[test]
fn actions_pushed_later_join_the_running_queue() {
    let mut q = ActionQueue::new();
    q.push([act("a", Soft, 1.0)]);
    q.step(0.5);
    q.push([act("b", Hard, 1.0)]); // must wait for a
    assert_eq!(states(&q), st(&[("a", Running), ("b", Waiting)]));
    run_to_idle(&mut q);
}

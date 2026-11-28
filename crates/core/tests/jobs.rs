use botsim_core::jobs::{JobError, JobGenerator, Rng};
use botsim_model::job::Job;

const DT: f64 = 0.05;
const HOUR_TICKS: u64 = 72_000; // one hour at 50 ms

/// All jobs of `ticks` steps.
fn run(g: &mut JobGenerator, ticks: u64) -> Vec<Job> {
    (1..=ticks).filter_map(|t| g.step(t)).collect()
}

#[test]
fn the_generator_is_splitmix64() {
    // reference values of SplitMix64 for seed 0
    let mut r = Rng::new(0);
    assert_eq!(r.next_u64(), 0xE220A8397B1DCDAF);
    assert_eq!(r.next_u64(), 0x6E789E6AA1B965F4);
}

#[test]
fn numbers_are_in_range_and_streams_differ() {
    let mut r = Rng::new(42);
    for _ in 0..10_000 {
        let f = r.next_f64();
        assert!((0.0..1.0).contains(&f));
        assert!(r.below(7) < 7);
    }
    let mut a = Rng::stream(1, "jobs");
    let mut b = Rng::stream(1, "disturbances");
    let mut c = Rng::stream(1, "jobs");
    assert_ne!(a.next_u64(), b.next_u64(), "other purpose, other stream");
    let mut a = Rng::stream(1, "jobs");
    assert_eq!(
        a.next_u64(),
        c.next_u64(),
        "same seed and name, same stream"
    );
}

#[test]
fn the_same_seed_gives_the_same_jobs_and_another_seed_other_jobs() {
    let nodes = [0, 1, 2, 3];
    let mk = |seed| JobGenerator::new(seed, &nodes, 600.0, DT).unwrap();
    let a = run(&mut mk(5), HOUR_TICKS / 4);
    assert_eq!(a, run(&mut mk(5), HOUR_TICKS / 4));
    assert_ne!(a, run(&mut mk(6), HOUR_TICKS / 4));
    assert!(!a.is_empty());
}

#[test]
fn the_number_of_jobs_follows_the_rate() {
    // 360 per hour: mean 360, standard deviation about 19; allow 5 sigma
    let mut g = JobGenerator::new(1, &[0, 1, 2], 360.0, DT).unwrap();
    let n = run(&mut g, HOUR_TICKS).len();
    assert!((265..=455).contains(&n), "{n} jobs in one hour");

    // rate 0 → no jobs at all
    let mut g = JobGenerator::new(1, &[0, 1, 2], 0.0, DT).unwrap();
    assert!(run(&mut g, HOUR_TICKS).is_empty());
}

#[test]
fn jobs_have_two_different_listed_nodes_and_running_ids() {
    let nodes = [10, 20, 30];
    let mut g = JobGenerator::new(9, &nodes, 3600.0, DT).unwrap();
    let jobs = run(&mut g, HOUR_TICKS);
    assert!(jobs.len() > 3000);
    let mut pairs = std::collections::HashSet::new();
    for (i, j) in jobs.iter().enumerate() {
        assert_eq!(j.id, i as u64 + 1);
        assert_ne!(j.pickup, j.drop);
        assert!(nodes.contains(&j.pickup) && nodes.contains(&j.drop));
        pairs.insert((j.pickup, j.drop));
    }
    assert_eq!(pairs.len(), 6, "all ordered pairs of 3 nodes occur");
    // created_tick is the tick of the step, in increasing order
    assert!(
        jobs.windows(2)
            .all(|w| w[0].created_tick < w[1].created_tick)
    );
}

#[test]
fn two_nodes_give_both_directions_and_duplicates_are_ignored() {
    let mut g = JobGenerator::new(3, &[4, 4, 7, 7, 4], 3600.0, DT).unwrap();
    let jobs = run(&mut g, 20_000);
    assert!(jobs.iter().any(|j| (j.pickup, j.drop) == (4, 7)));
    assert!(jobs.iter().any(|j| (j.pickup, j.drop) == (7, 4)));
}

#[test]
fn bad_input_is_refused() {
    assert_eq!(
        JobGenerator::new(1, &[1], 10.0, DT).unwrap_err(),
        JobError::TooFewNodes
    );
    assert_eq!(
        JobGenerator::new(1, &[1, 1], 10.0, DT).unwrap_err(),
        JobError::TooFewNodes
    );
    assert!(matches!(
        JobGenerator::new(1, &[1, 2], -1.0, DT),
        Err(JobError::BadRate(_))
    ));
    assert!(matches!(
        JobGenerator::new(1, &[1, 2], f64::NAN, DT),
        Err(JobError::BadRate(_))
    ));
    // more than one job per step on average is not possible
    assert_eq!(
        JobGenerator::new(1, &[1, 2], 100_000.0, DT).unwrap_err(),
        JobError::RateTooHighForStep
    );
}

/// Pickup places 0 and 1; drop places 2 and the station with nodes 3 and 4.
fn groups(both: bool) -> Result<JobGenerator, JobError> {
    JobGenerator::between_groups(
        9,
        &[vec![0], vec![1]],
        &[vec![2], vec![3, 4]],
        both,
        600.0,
        DT,
    )
}

#[test]
fn jobs_between_groups_go_from_the_pickup_places_to_the_drop_places() {
    let jobs = run(&mut groups(false).unwrap(), HOUR_TICKS);
    assert!(jobs.len() > 300);
    assert!(jobs.iter().all(|j| j.pickup < 2 && j.drop >= 2));
    // every node of both groups is used
    for n in 0..2 {
        assert!(jobs.iter().any(|j| j.pickup == n));
    }
    for n in 2..5 {
        assert!(jobs.iter().any(|j| j.drop == n));
    }
    assert_eq!(jobs, run(&mut groups(false).unwrap(), HOUR_TICKS));
}

#[test]
fn with_both_directions_about_half_of_the_jobs_go_back() {
    let jobs = run(&mut groups(true).unwrap(), HOUR_TICKS);
    // every job crosses from one group to the other
    assert!(jobs.iter().all(|j| (j.pickup < 2) != (j.drop < 2)));
    let back = jobs.iter().filter(|j| j.pickup >= 2).count() as f64;
    let share = back / jobs.len() as f64;
    assert!((0.4..0.6).contains(&share), "{share} go back");
}

#[test]
fn job_groups_must_not_be_empty_or_share_a_node() {
    let new = |p: &[Vec<usize>], d: &[Vec<usize>]| {
        JobGenerator::between_groups(1, p, d, true, 60.0, DT).unwrap_err()
    };
    assert_eq!(new(&[], &[vec![1]]), JobError::EmptyGroup);
    assert_eq!(new(&[vec![1]], &[Vec::new()]), JobError::EmptyGroup);
    assert_eq!(new(&[vec![1, 2]], &[vec![2]]), JobError::SharedNode);
}

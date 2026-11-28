//! Jobs between places (S60).
use botsim_sim::jobs::{JobError, JobGenerator};

const DT: f64 = 0.05;

fn jobs(mut g: JobGenerator, steps: u64) -> Vec<(usize, usize)> {
    (0..steps)
        .filter_map(|t| g.step(t))
        .map(|j| (j.pickup, j.drop))
        .collect()
}

#[test]
fn places_with_one_node_give_the_same_jobs_as_plain_nodes() {
    let nodes = [3, 5, 8, 9];
    let places: Vec<Vec<usize>> = nodes.iter().map(|&n| vec![n]).collect();
    for seed in 1..=4 {
        let a = jobs(JobGenerator::new(seed, &nodes, 600.0, DT).unwrap(), 20000);
        let b = jobs(
            JobGenerator::between_places(seed, &places, 600.0, DT).unwrap(),
            20000,
        );
        assert!(a.len() > 100);
        assert_eq!(a, b, "seed {seed}");
    }
}

#[test]
fn a_job_goes_between_two_places_at_one_of_their_nodes() {
    // place 0 = {1, 2}, place 1 = {3}, place 2 = {4, 5}
    let places = vec![vec![1, 2], vec![3], vec![4, 5]];
    let list = jobs(
        JobGenerator::between_places(7, &places, 1200.0, DT).unwrap(),
        60000,
    );
    assert!(list.len() > 500);
    let place = |n: usize| places.iter().position(|p| p.contains(&n)).unwrap();
    for &(pickup, drop) in &list {
        assert_ne!(
            place(pickup),
            place(drop),
            "a job needs two different places"
        );
    }
    // both nodes of a station are used, and nothing else
    let used: std::collections::BTreeSet<usize> = list.iter().flat_map(|&(p, d)| [p, d]).collect();
    assert_eq!(used.into_iter().collect::<Vec<_>>(), vec![1, 2, 3, 4, 5]);
    // and the same seed gives the same jobs
    let again = jobs(
        JobGenerator::between_places(7, &places, 1200.0, DT).unwrap(),
        60000,
    );
    assert_eq!(list, again);
}

#[test]
fn at_least_two_different_places_are_needed() {
    let one = JobGenerator::between_places(1, &[vec![1, 2]], 60.0, DT).err();
    assert_eq!(one, Some(JobError::TooFewNodes));
    // the same list twice is one place; an empty list is no place
    let same = JobGenerator::between_places(1, &[vec![1, 2], vec![1, 2], vec![]], 60.0, DT).err();
    assert_eq!(same, Some(JobError::TooFewNodes));
    assert!(JobGenerator::between_places(1, &[vec![1], vec![1, 2]], 60.0, DT).is_ok());
}

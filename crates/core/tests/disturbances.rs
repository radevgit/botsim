use botsim_core::disturbances::{DisturbanceError, Disturbances};

#[test]
fn factors_stay_within_the_variation_and_vary() {
    let mut d = Disturbances::new(1, 0.05).unwrap();
    let f: Vec<f64> = (0..1000).map(|_| d.next_speed_factor()).collect();
    assert!(f.iter().all(|x| *x > 0.95 && *x <= 1.0), "{f:?}");
    let (lo, hi) = (
        f.iter().copied().fold(f64::MAX, f64::min),
        f.iter().copied().fold(f64::MIN, f64::max),
    );
    assert!(lo < 0.955 && hi > 0.995, "spread {lo} .. {hi}");
}

#[test]
fn no_variation_means_factor_one_and_the_same_seed_gives_the_same_factors() {
    let mut d = Disturbances::new(1, 0.0).unwrap();
    assert!((0..10).all(|_| d.next_speed_factor() == 1.0));

    let run = |seed| {
        let mut d = Disturbances::new(seed, 0.1).unwrap();
        (0..5).map(|_| d.next_speed_factor()).collect::<Vec<_>>()
    };
    assert_eq!(run(3), run(3));
    assert_ne!(run(3), run(4));
}

#[test]
fn a_robots_factor_does_not_depend_on_the_robots_after_it() {
    let mut a = Disturbances::new(7, 0.05).unwrap();
    let mut b = Disturbances::new(7, 0.05).unwrap();
    let first_of_two = (a.next_speed_factor(), a.next_speed_factor());
    let first_of_five = b.next_speed_factor();
    assert_eq!(first_of_two.0, first_of_five);
}

#[test]
fn bad_variations_are_refused() {
    for v in [-0.1, 1.0, 2.0, f64::NAN, f64::INFINITY] {
        assert!(
            matches!(
                Disturbances::new(1, v),
                Err(DisturbanceError::BadSpeedVariation(_))
            ),
            "{v}"
        );
    }
}

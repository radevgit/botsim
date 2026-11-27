//! Angles. Turning a robot step by step is tested in `botsim-core`.

mod rotation {
    use botsim_model::angle::normalize_angle;
    use std::f64::consts::{FRAC_PI_2, PI};

    #[test]
    fn normalises_angles() {
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-12);
        assert!((normalize_angle(-PI) - PI).abs() < 1e-12);
        assert!((normalize_angle(2.5 * PI) - FRAC_PI_2).abs() < 1e-12);
        assert!((normalize_angle(-2.5 * PI) + FRAC_PI_2).abs() < 1e-12);
        assert_eq!(normalize_angle(0.3), 0.3);
    }
}

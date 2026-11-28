//! The figures of the jobs done.

use botsim_baseline::metrics::summarize;

#[test]
fn summarize_gives_throughput_average_and_maximum() {
    // 3 jobs in half an hour = 6 per hour
    let m = summarize(&[0.0, 10.0, 5.0], 1800.0);
    assert_eq!(m.jobs_done, 3);
    assert!((m.throughput_per_hour - 6.0).abs() < 1e-12);
    assert_eq!(m.waiting_avg_seconds, Some(5.0));
    assert_eq!(m.waiting_max_seconds, Some(10.0));
}

#[test]
fn summarize_without_jobs_or_time_has_no_waiting_and_no_division_by_zero() {
    let m = summarize(&[], 3600.0);
    assert_eq!((m.jobs_done, m.throughput_per_hour), (0, 0.0));
    assert_eq!((m.waiting_avg_seconds, m.waiting_max_seconds), (None, None));

    let m = summarize(&[1.0], 0.0);
    assert_eq!(
        m.throughput_per_hour, 0.0,
        "no elapsed time → 0, not infinity"
    );
}

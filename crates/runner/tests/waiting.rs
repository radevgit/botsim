//! The summary's "waiting caused by others" figures (brief U12).

use std::path::{Path, PathBuf};

use botsim::scenario::Scenario;
use botsim::summary::{self, Summary};
use botsim::waiting_summary::TOP;

fn scenarios() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

/// Run the shipped scenario `name` from its file (with the conflict map).
fn run(name: &str) -> Summary {
    let s = Scenario::load(&scenarios().join(name)).unwrap();
    let world = s.build_world(&scenarios()).unwrap();
    summary::run(&s, world).unwrap()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-3
}

#[test]
fn the_figures_add_up_and_two_identical_runs_give_the_same() {
    // RELEASE zones and disturbances
    let first = run("ring_jobs.toml");
    let w = first.waiting_for_others.clone().unwrap();
    assert!(w.total_seconds > 0.0);
    let r = &w.by_reason;
    let by_reason = r.release + r.zone + r.lift + r.door + r.safety_stop;
    assert!(close(by_reason, w.total_seconds), "{w:?}");
    let suffered: f64 = w.robots.iter().map(|r| r.suffered_seconds).sum();
    let caused: f64 = w.robots.iter().map(|r| r.caused_seconds).sum();
    assert!(close(suffered, w.total_seconds), "{w:?}");
    assert!(close(caused + w.unknown_seconds, w.total_seconds), "{w:?}");
    assert_eq!(w.robots.len(), first.robots.len());
    // the longest first, at most TOP, never more than the total
    assert!(!w.top_pairs.is_empty() && w.top_pairs.len() <= TOP);
    assert!(!w.top_places.is_empty() && w.top_places.len() <= TOP);
    assert!(w.top_pairs.windows(2).all(|p| p[0].seconds >= p[1].seconds));
    assert!(
        w.top_places
            .windows(2)
            .all(|p| p[0].seconds >= p[1].seconds)
    );
    assert!(w.top_pairs.iter().all(|p| p.blocker != p.waiter));
    assert!(
        w.top_places
            .iter()
            .all(|p| p.node_id.is_some() != p.edge_id.is_some())
    );
    assert_eq!(run("ring_jobs.toml").waiting_for_others, Some(w));
}

//! The server with traffic rules of its caller, neither the baseline's nor the
//! traffic module's, on hand-written data.

use botsim_model::graph::Graph;
use botsim_model::waiting::{HeldBack, WaitReason};
use botsim_server::controller::Controller;
use botsim_server::order_builder::Plan;
use botsim_server::policy::{Ask, Fleet, Release, Setup, TrafficPolicy, ZoneAnswer};

mod common;
use common::graph;
mod hand;
use hand::{config, cycle, job, orders, report};

/// One robot drives at a time: the first that asks, until its route ends.
#[derive(Debug, Default)]
struct OneDrives {
    driver: Option<usize>,
}

impl OneDrives {
    fn leaves(&mut self, robot: usize) {
        if self.driver == Some(robot) {
            self.driver = None;
        }
    }
}

impl TrafficPolicy for OneDrives {
    fn start(&mut self, _setup: &Setup) {}

    fn place(&mut self, ask: &Ask) -> Release {
        self.release(ask)
    }

    fn release(&mut self, ask: &Ask) -> Release {
        let driver = *self.driver.get_or_insert(ask.robot);
        Release {
            through: if driver == ask.robot {
                ask.limit
            } else {
                ask.released_through
            },
            reroute: None,
        }
    }

    fn route_finished(&mut self, robot: usize, _route: &Plan) {
        self.leaves(robot);
    }

    fn route_dropped(&mut self, robot: usize, _route: &Plan, _released_through: usize) {
        self.leaves(robot);
    }

    fn zone_answers(&mut self, _fleet: &Fleet) -> Vec<ZoneAnswer> {
        Vec::new()
    }

    fn blocker(&self, robot: usize, _route: &Plan, _released_through: usize) -> Option<usize> {
        self.driver.filter(|&d| d != robot)
    }
}

/// Two lines far apart: A → B → C and D → E.
fn lines() -> Graph {
    graph(
        &[
            ("A", 0.0, 0.0),
            ("B", 10.0, 0.0),
            ("C", 20.0, 0.0),
            ("D", 0.0, 50.0),
            ("E", 10.0, 50.0),
        ],
        &[("A", "B"), ("B", "C"), ("D", "E")],
    )
}

#[test]
fn the_rules_of_the_caller_say_who_may_go() {
    let g = lines();
    let [a, b, c, d, e] = ["A", "B", "C", "D", "E"].map(|id| g.node_idx(id).unwrap());
    let mut ctl = Controller::with_policy(&config(g.clone()), OneDrives::default());
    let (first, second) = ([a, b, c], [d, e]);

    // both get their job; only the first is let go
    let idle = [
        report(&g, 0, "", &[a], 0, false),
        report(&g, 1, "", &[d], 0, false),
    ];
    let out = cycle(&mut ctl, 0, &idle, [job(1, a, c), job(2, d, e)]);
    assert_eq!(orders(&out), [(0, 0, vec![a, b, c]), (1, 0, vec![d])]);

    // the second stands at the end of what it has, held back by the first
    let states = [
        report(&g, 0, "job-1", &first, 0, true),
        report(&g, 1, "job-2", &second, 0, false),
    ];
    let out = cycle(&mut ctl, 1, &states, []);
    assert!(out.messages.is_empty());
    let held = HeldBack {
        reason: WaitReason::Release,
        blockers: vec![0],
    };
    assert_eq!(out.held_back, vec![None, Some(held)]);

    // the first arrives: the second is released in the same cycle
    let states = [
        report(&g, 0, "job-1", &first, 2, false),
        report(&g, 1, "job-2", &second, 0, false),
    ];
    let out = cycle(&mut ctl, 2, &states, []);
    assert_eq!(orders(&out), [(1, 1, vec![d, e])]);
    assert_eq!(ctl.completed().len(), 1);

    let states = [
        report(&g, 0, "job-1", &first, 2, false),
        report(&g, 1, "job-2", &second, 1, false),
    ];
    cycle(&mut ctl, 3, &states, []);
    assert_eq!(ctl.completed().len(), 2);
    assert_eq!(ctl.busy(), 0);
}

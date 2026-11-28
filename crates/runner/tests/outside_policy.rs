//! A traffic policy written outside the controller, with the public interface only
//! (brief `traffic_policy_slot.md`, "Done when"): it releases one node at a time, if
//! no other robot holds it, and grants RELEASE zones in the order of the requests,
//! one robot per zone. It is not a good controller; it shows that another policy can
//! drive a run.

use std::cell::RefCell;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::rc::Rc;

use botsim::scenario::Scenario;
use botsim::summary::{self, Summary};
use botsim_link::ids::{RequestId, ZoneId};
use botsim_link::response::Grant;
use botsim_link::state::ZoneRequestType;
use botsim_model::graph::NodeIdx;
use botsim_server::order_builder::Plan;
use botsim_server::policy::{Ask, Fleet, Release, Setup, TrafficPolicy, ZoneAnswer};

/// What the policy was asked, seen by the test after the run.
#[derive(Debug, Default)]
struct Seen {
    started_robots: usize,
    cycles: usize,
    releases: usize,
    places: usize,
    finished: usize,
    grants: usize,
}

#[derive(Debug, Default)]
struct OneAtATime {
    seen: Rc<RefCell<Seen>>,
    /// Node → the robot that holds it.
    held: HashMap<NodeIdx, usize>,
    /// Zone requests waiting, first seen first: (robot, request id, zone id).
    queue: Vec<(usize, RequestId, ZoneId)>,
    /// Requests granted whose robot still states them: (robot, request id, zone id).
    granted: Vec<(usize, RequestId, ZoneId)>,
}

impl OneAtATime {
    /// Release the next node if nobody else holds it.
    fn next(&mut self, ask: &Ask) -> Release {
        let mut through = ask.released_through;
        if through < ask.limit
            && let Some(&node) = ask.route.nodes.get(through + 1)
            && *self.held.entry(node).or_insert(ask.robot) == ask.robot
        {
            through += 1;
        }
        Release {
            through,
            reroute: None,
        }
    }

    /// The robot holds only the nodes of `keep`.
    fn keep_only(&mut self, robot: usize, keep: &[NodeIdx]) {
        self.held
            .retain(|node, r| *r != robot || keep.contains(node));
    }
}

impl TrafficPolicy for OneAtATime {
    fn start(&mut self, setup: &Setup) {
        self.seen.borrow_mut().started_robots = setup.robots.len();
    }

    fn window(&self) -> usize {
        1
    }

    fn cycle_start(&mut self, _fleet: &Fleet) {
        self.seen.borrow_mut().cycles += 1;
    }

    fn place(&mut self, ask: &Ask) -> Release {
        self.seen.borrow_mut().places += 1;
        if let Some(&first) = ask.route.nodes.first() {
            self.held.insert(first, ask.robot);
        }
        self.next(ask)
    }

    fn release(&mut self, ask: &Ask) -> Release {
        self.seen.borrow_mut().releases += 1;
        // give back what is behind the robot
        let ahead = ask
            .route
            .nodes
            .get(ask.at..=ask.released_through)
            .unwrap_or(&[]);
        self.keep_only(ask.robot, ahead);
        self.next(ask)
    }

    fn route_finished(&mut self, robot: usize, route: &Plan) {
        self.seen.borrow_mut().finished += 1;
        let last: Vec<NodeIdx> = route.nodes.last().copied().into_iter().collect();
        self.keep_only(robot, &last);
    }

    fn route_dropped(&mut self, robot: usize, _route: &Plan, _released_through: usize) {
        self.keep_only(robot, &[]);
    }

    fn idle(&mut self, robot: usize, node: NodeIdx) {
        self.held.entry(node).or_insert(robot);
    }

    fn zone_answers(&mut self, fleet: &Fleet) -> Vec<ZoneAnswer> {
        let stated: Vec<(usize, RequestId, ZoneId)> = fleet
            .robots
            .iter()
            .enumerate()
            .flat_map(|(robot, s)| {
                s.report
                    .state
                    .zone_requests
                    .iter()
                    .flatten()
                    .filter(|r| r.kind == ZoneRequestType::Access)
                    .map(move |r| (robot, r.id.clone(), r.zone.clone()))
            })
            .collect();
        // a request that is gone frees its zone
        self.granted.retain(|g| stated.contains(g));
        self.queue.retain(|q| stated.contains(q));
        for r in stated {
            if !self.queue.contains(&r) && !self.granted.contains(&r) {
                self.queue.push(r);
            }
        }
        // the oldest request for a free zone is granted
        let mut answers = Vec::new();
        let mut k = 0;
        while k < self.queue.len() {
            let zone = &self.queue[k].2;
            if self.granted.iter().any(|g| &g.2 == zone) {
                k += 1;
                continue;
            }
            let r = self.queue.remove(k);
            self.seen.borrow_mut().grants += 1;
            answers.push(ZoneAnswer {
                robot: r.0,
                request_id: r.1.clone(),
                grant: Grant::Granted,
            });
            self.granted.push(r);
        }
        answers
    }
}

fn scenarios() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

fn run(name: &str) -> (Summary, Rc<RefCell<Seen>>) {
    let s = Scenario::load(&scenarios().join(name)).unwrap();
    let world = s.build_world(&scenarios()).unwrap();
    let policy = OneAtATime::default();
    let seen = policy.seen.clone();
    let summary = summary::run_with_policy(&s, world, Box::new(policy)).unwrap();
    (summary, seen)
}

#[test]
fn an_outside_policy_drives_a_run_with_release_zones() {
    let (r, seen) = run("ring_jobs.toml");
    let seen = seen.borrow();
    assert!(r.jobs.done > 0, "{:?}", r.jobs);
    assert_eq!(seen.started_robots, 2);
    assert!(seen.cycles > 0 && seen.releases > 0);
    assert!(seen.places >= r.jobs.done && seen.finished >= r.jobs.done);
    assert!(seen.grants > 0, "the zone of the ring was asked for");
}

#[test]
fn an_outside_policy_drives_a_mixed_fleet() {
    let (r, seen) = run("mixed_fleet.toml");
    assert!(r.jobs.done > 0, "{:?}", r.jobs);
    assert!(seen.borrow().releases > 0);
}

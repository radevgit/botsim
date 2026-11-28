//! The controller takes another dispatcher, and does nothing a dispatcher asks that
//! cannot be done.

use std::cell::RefCell;
use std::rc::Rc;

use botsim::connect;
use botsim_baseline::baseline_policy::BaselinePolicy;
use botsim_dispatcher::{
    Answer, ChargePolicy, Control, Counts, Cycle, Dispatcher, Purpose, Robot, Site, Task, Work,
};
use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_model::time::Millis;
use botsim_sim::actions::ActionTimes;
use botsim_sim::world::World;
use vda5050::factsheet::Factsheet;

#[path = "../../baseline/tests/common/mod.rs"]
mod common;
use common::graph;
#[path = "../../sim/tests/vda_port/mod.rs"]
mod vda_port;
use vda_port::VdaWorld;

const DT: f64 = 0.05;

// the nodes of the line A → B → C → D
const A: NodeIdx = 0;
const B: NodeIdx = 1;
const C: NodeIdx = 2;
const D: NodeIdx = 3;

/// What the scripted dispatcher was told and answered.
#[derive(Debug, Default)]
struct Log {
    answers: Vec<(&'static str, Answer)>,
    /// The trip as the controller showed it when it could take a job.
    trip: Option<Work>,
    jobs: Vec<Job>,
}

/// Asks the controller the impossible and some of the possible.
#[derive(Debug)]
struct Script {
    log: Rc<RefCell<Log>>,
    cycle: usize,
}

/// The edge from node `a` to node `b`.
fn edge(g: &Graph, a: NodeIdx, b: NodeIdx) -> EdgeIdx {
    g.edges
        .iter()
        .position(|e| e.start == a && e.end == b)
        .unwrap()
}

fn task(purpose: Purpose, from: NodeIdx, edges: Vec<EdgeIdx>, pickup_at: usize) -> Task {
    Task {
        // the job that fits a route along the line from `from`: each edge leads to
        // the next node
        job: Job {
            id: 7,
            pickup: from + pickup_at,
            drop: from + edges.len(),
            created: Millis(0),
        },
        purpose,
        from,
        edges,
        pickup_at,
    }
}

impl Dispatcher for Script {
    fn start(&mut self, _graph: &Graph, _robots: usize) {}
    fn set_charging(&mut self, _policy: ChargePolicy) -> usize {
        0
    }
    fn add_job(&mut self, job: Job) {
        self.log.borrow_mut().jobs.push(job);
    }
    fn pending(&self) -> Box<dyn Iterator<Item = &Job> + '_> {
        Box::new(std::iter::empty())
    }
    fn stop_charging(&mut self, _robot: usize, _charge: f64) -> bool {
        false
    }
    fn arrived(&mut self, _robot: usize, _purpose: Purpose) {}
    fn given_up(&mut self, _robot: usize, _purpose: Purpose) {}
    fn counts(&self) -> Counts {
        Counts {
            park_trips: 42,
            ..Counts::default()
        }
    }

    fn dispatch(&mut self, site: &Site, robots: &[Robot], _cycle: Cycle, c: &mut dyn Control) {
        self.cycle += 1;
        let g = site.graph;
        let (ab, bc, cd) = (edge(g, A, B), edge(g, B, C), edge(g, C, D));
        let job = |from, edges, at| task(Purpose::Job, from, edges, at);
        let mut log = self.log.borrow_mut();
        let mut said = Vec::new();
        if self.cycle == 1 {
            assert_eq!(robots.len(), 1);
            assert_eq!((robots[0].node, &robots[0].work), (Some(A), &Work::None));
            // numbers the controller does not have
            assert_eq!(c.node_holder(g.nodes.len() + 3), None);
            assert_eq!(c.route_cost(9), None);
            c.charge_here(9, 0);
            said.extend([
                ("unknown robot", c.give(9, job(A, vec![ab], 0))),
                ("not where it stands", c.give(0, job(B, vec![bc], 0))),
                ("starts elsewhere", c.give(0, job(A, vec![bc], 0))),
                ("does not connect", c.give(0, job(A, vec![ab, cd], 0))),
                ("unknown edge", c.give(0, job(A, vec![g.edges.len()], 0))),
                ("pickup beyond", c.give(0, job(A, vec![ab], 2))),
                ("extend a free robot", c.extend(0, job(A, vec![ab], 0))),
                ("extend nobody", c.extend(9, job(A, vec![ab], 0))),
                // a trip (its pickup_at is not used): released up to C
                (
                    "park",
                    c.give(0, task(Purpose::Park, A, vec![ab, bc, cd], 99)),
                ),
                ("give twice", c.give(0, job(A, vec![ab], 0))),
            ]);
            c.charge_here(0, 0); // it has a task: nothing happens
        } else if self.cycle == 2 {
            log.trip = Some(robots[0].work.clone());
            said.extend([
                ("give on a trip", c.give(0, job(C, vec![cd], 1))),
                ("not the released end", c.extend(0, job(B, vec![bc, cd], 1))),
                ("route elsewhere", c.extend(0, job(C, vec![bc], 1))),
                (
                    "extend with a trip",
                    c.extend(0, task(Purpose::Charge, C, vec![cd], 0)),
                ),
                ("extend beyond", c.extend(0, job(C, vec![cd], 2))),
                ("extend", c.extend(0, job(C, vec![cd], 1))),
                ("extend a job", c.extend(0, job(C, vec![cd], 1))),
            ]);
        }
        log.answers.extend(said);
    }
}

#[test]
fn another_dispatcher_can_ask_the_impossible_without_effect() {
    let g = graph(
        &[
            ("A", 0.0, 0.0),
            ("B", 10.0, 0.0),
            ("C", 20.0, 0.0),
            ("D", 30.0, 0.0),
        ],
        &[("A", "B"), ("B", "C"), ("C", "D")],
    );
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();
    let robot = RobotId::new("ACME", "R1");
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(robot.clone(), spec, ActionTimes::new(1.0), A, 0.0)
        .unwrap();
    let log = Rc::new(RefCell::new(Log::default()));
    let script = Script {
        log: log.clone(),
        cycle: 0,
    };
    let policy = BaselinePolicy::new(w.graph());
    let mut ctl = connect::with_dispatcher(&w, policy, Box::new(script));

    let new_job = Job {
        id: 1,
        pickup: A,
        drop: B,
        created: Millis(0),
    };
    connect::step(&mut ctl, &mut w, [new_job]);
    for _ in 0..(60.0 / DT) as usize {
        w.step();
        connect::step(&mut ctl, &mut w, []);
    }

    let log = log.borrow();
    assert_eq!(log.jobs, [new_job]);
    let taken: Vec<&str> = log
        .answers
        .iter()
        .filter(|a| a.1 != Answer::Invalid)
        .map(|a| a.0)
        .collect();
    assert_eq!(log.answers.len(), 17);
    assert_eq!(taken, ["park", "extend"]);
    assert!(log.answers.iter().all(|a| a.1 != Answer::Refused));
    let g = w.graph();
    assert_eq!(
        log.trip,
        Some(Work::Parking {
            from: C,
            ahead: vec![edge(g, A, B), edge(g, B, C)],
            arrival: Some(edge(g, B, C)),
        })
    );
    // one order, extended to the job, which is done
    let state = w.state(&robot).unwrap();
    assert_eq!(state.order_id, "park-7");
    assert_eq!(state.last_node_id, "D");
    assert!(state.errors.is_empty());
    assert!(!state.power_supply.charging);
    assert_eq!(ctl.completed().len(), 1);
    assert!(ctl.rejected().is_empty());
    // the controller's figures are the dispatcher's
    assert_eq!(ctl.park_trips(), 42);
}

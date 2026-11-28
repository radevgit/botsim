//! A small layout and a controller that only writes down what it is asked.

// (each test file uses a part of this)
#![allow(dead_code)]

use std::collections::{HashMap, HashSet};

use botsim_dispatcher::baseline::BaselineDispatcher;
use botsim_dispatcher::{Answer, Control, Cycle, Dispatcher, Purpose, Robot, Site, Task, Work};
use botsim_model::graph::{EdgeIdx, Graph, GraphStation, NodeIdx};
use botsim_model::job::Job;
use botsim_model::spec::Kinematics;
use botsim_model::time::Millis;
use botsim_router::types::TypeRouting;
use vdma_lif::Lif;

/// A layout from nodes (id, x, y) and directed edges (from, to); edge ids are "from-to".
pub fn graph(nodes: &[(&str, f64, f64)], edges: &[(&str, &str)]) -> Graph {
    let ns: Vec<String> = nodes
        .iter()
        .map(|(id, x, y)| {
            format!(
                r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
                "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]}}]}}"#
            )
        })
        .collect();
    let es: Vec<String> = edges
        .iter()
        .map(|(a, b)| {
            format!(
                r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}",
                "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}],
                "orientationType":"TANGENTIAL","mobileRobotOrientations":[0.0]}}]}}"#
            )
        })
        .collect();
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-20T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{}],"edges":[{}]}}]}}]}}"#,
        ns.join(","),
        es.join(",")
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif).unwrap()
}

/// The names of the nodes of [`line`], 10 m apart.
pub const A: NodeIdx = 0;
pub const B: NodeIdx = 1;
pub const C: NodeIdx = 2;
pub const D: NodeIdx = 3;
pub const E: NodeIdx = 4;
/// A node with no edge into it: nothing can drive there.
pub const X: NodeIdx = 5;

/// A two-way line A – B – C – D – E, and X with a one-way edge to E. `stations`:
/// (type, node).
pub fn line(stations: &[(&str, NodeIdx)]) -> Graph {
    let nodes = [
        ("A", 0.0, 0.0),
        ("B", 10.0, 0.0),
        ("C", 20.0, 0.0),
        ("D", 30.0, 0.0),
        ("E", 40.0, 0.0),
        ("X", 50.0, 0.0),
    ];
    let mut edges = Vec::new();
    for (a, b) in [("A", "B"), ("B", "C"), ("C", "D"), ("D", "E")] {
        edges.push((a, b));
        edges.push((b, a));
    }
    edges.push(("X", "E"));
    let mut g = graph(&nodes, &edges);
    for (k, &(kind, node)) in stations.iter().enumerate() {
        g.stations.push(GraphStation {
            id: format!("S{k}"),
            name: None,
            station_type: Some(kind.into()),
            interaction_nodes: vec![node],
        });
    }
    g
}

/// The edge from `a` to `b`.
pub fn edge(g: &Graph, a: NodeIdx, b: NodeIdx) -> EdgeIdx {
    g.edges
        .iter()
        .position(|e| e.start == a && e.end == b)
        .unwrap()
}

/// A free, ready robot of type 0 standing on `node` with a full battery.
pub fn at(node: NodeIdx) -> Robot {
    Robot {
        type_idx: 0,
        node: Some(node),
        heading: Some(0.0),
        charge: 100.0,
        ready: true,
        work: Work::None,
        // (a second a dispatch, see `Rig::dispatch`)
        elapsed: 1.0,
    }
}

pub fn low(node: NodeIdx, charge: f64) -> Robot {
    Robot { charge, ..at(node) }
}

/// A robot with a task that cannot be extended.
pub fn busy(node: NodeIdx) -> Robot {
    Robot {
        work: Work::Busy,
        ..at(node)
    }
}

pub fn job(id: u64, pickup: NodeIdx, drop: NodeIdx) -> Job {
    Job {
        id,
        pickup,
        drop,
        created: Millis(0),
    }
}

/// What a controller was asked, and how it answers.
#[derive(Debug, Default)]
pub struct Script {
    pub held: HashMap<NodeIdx, usize>,
    /// A route cost per robot; none: the plain lengths.
    pub costs: HashMap<usize, Vec<f64>>,
    /// Robots that refuse their tasks.
    pub refuse: HashSet<usize>,
    /// Robots for which every task is answered with `Invalid`.
    pub invalid: HashSet<usize>,
    /// A taken task holds its last node from then on.
    pub claim_goals: bool,
    pub given: Vec<(usize, Task)>,
    pub extended: Vec<(usize, Task)>,
    /// (robot, trip number)
    pub charged_here: Vec<(usize, usize)>,
    /// The end node of each edge of the layout.
    ends: Vec<NodeIdx>,
}

impl Script {
    fn answer(&mut self, robot: usize, task: &Task) -> Answer {
        if self.invalid.contains(&robot) {
            Answer::Invalid
        } else if self.refuse.contains(&robot) {
            Answer::Refused
        } else {
            if self.claim_goals {
                let goal = task.edges.last().map_or(task.from, |&e| self.ends[e]);
                self.held.insert(goal, robot);
            }
            Answer::Taken
        }
    }

    /// (robot, purpose, node the route ends at) of every task given.
    pub fn summary(&self, g: &Graph) -> Vec<(usize, Purpose, NodeIdx)> {
        self.given
            .iter()
            .map(|(r, t)| {
                let goal = t.edges.last().map_or(t.from, |&e| g.edges[e].end);
                (*r, t.purpose, goal)
            })
            .collect()
    }

    pub fn clear(&mut self) {
        self.given.clear();
        self.extended.clear();
        self.charged_here.clear();
    }
}

impl Control for Script {
    fn node_holder(&self, node: NodeIdx) -> Option<usize> {
        self.held.get(&node).copied()
    }

    fn route_cost(&self, robot: usize) -> Option<Vec<f64>> {
        self.costs.get(&robot).cloned()
    }

    fn give(&mut self, robot: usize, task: Task) -> Answer {
        let answer = self.answer(robot, &task);
        self.given.push((robot, task));
        answer
    }

    fn extend(&mut self, robot: usize, task: Task) -> Answer {
        let answer = self.answer(robot, &task);
        self.extended.push((robot, task));
        answer
    }

    fn charge_here(&mut self, robot: usize, trip: usize) {
        self.charged_here.push((robot, trip));
    }
}

/// A started baseline dispatcher on a layout, with one robot type.
pub struct Rig {
    pub graph: Graph,
    types: Vec<TypeRouting>,
    rides: HashMap<EdgeIdx, Option<NodeIdx>>,
    pub dispatcher: BaselineDispatcher,
    pub script: Script,
    tick: u64,
}

impl Rig {
    pub fn new(graph: Graph, robots: usize) -> Rig {
        let types = vec![TypeRouting::new(
            &graph,
            Kinematics::Differential,
            "ACME",
            "X1",
        )];
        let mut dispatcher = BaselineDispatcher::new();
        dispatcher.start(&graph, robots);
        let script = Script {
            ends: graph.edges.iter().map(|e| e.end).collect(),
            ..Script::default()
        };
        Rig {
            graph,
            types,
            rides: HashMap::new(),
            dispatcher,
            script,
            tick: 0,
        }
    }

    /// One dispatch, a second after the one before.
    pub fn dispatch(&mut self, robots: &[Robot]) {
        self.tick += 1;
        let site = Site {
            graph: &self.graph,
            types: &self.types,
            rides: &self.rides,
        };
        let cycle = Cycle {
            now: Millis(1000 * self.tick as i64),
            elapsed: 1.0,
        };
        self.dispatcher
            .dispatch(&site, robots, cycle, &mut self.script);
    }

    pub fn jobs(&mut self, jobs: &[Job]) {
        for &j in jobs {
            self.dispatcher.add_job(j);
        }
    }

    /// The ids of the jobs still waiting.
    pub fn pending(&self) -> Vec<u64> {
        self.dispatcher.pending().map(|j| j.id).collect()
    }
}

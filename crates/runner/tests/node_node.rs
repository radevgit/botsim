//! The runner gives a traffic policy the conflict map's node-node pairs (brief U16).

use std::cell::RefCell;
use std::path::Path;
use std::rc::Rc;

use botsim::scenario::Scenario;
use botsim::summary;
use botsim_server::conflicts::NodeNodeConflictInput;
use botsim_server::policy::{Ask, Fleet, Release, Setup, TrafficPolicy, ZoneAnswer};

/// A policy that keeps what it is given and lets no robot go.
#[derive(Debug, Default)]
struct Listen {
    heard: Rc<RefCell<Vec<NodeNodeConflictInput>>>,
}

impl TrafficPolicy for Listen {
    fn start(&mut self, _setup: &Setup) {}

    fn node_node_conflicts(&mut self, nodes: &[NodeNodeConflictInput]) {
        self.heard.borrow_mut().extend_from_slice(nodes);
    }

    fn place(&mut self, ask: &Ask) -> Release {
        self.release(ask)
    }

    fn release(&mut self, ask: &Ask) -> Release {
        Release {
            through: ask.released_through,
            reroute: None,
        }
    }

    fn zone_answers(&mut self, _fleet: &Fleet) -> Vec<ZoneAnswer> {
        Vec::new()
    }
}

/// Two lanes 1.0 m apart that share no node: A → P → B and C → Q → D.
fn layout() -> String {
    let types = r#""mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]"#;
    let nodes: Vec<String> = [
        ("A", -5.0, 0.0),
        ("P", 0.0, 0.0),
        ("B", 0.0, -5.0),
        ("C", 5.0, 1.0),
        ("Q", 0.0, 1.0),
        ("D", 0.0, 6.0),
    ]
    .iter()
    .map(|(id, x, y)| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":{y}}},
            "mobileRobotTypeNodeProperties":[{{{types}}}]}}"#
        )
    })
    .collect();
    let edges: Vec<String> = [("A", "P"), ("P", "B"), ("C", "Q"), ("Q", "D")]
        .iter()
        .map(|(a, b)| {
            format!(
                r#"{{"edgeId":"{a}-{b}","startNodeId":"{a}","endNodeId":"{b}",
                "mobileRobotTypeEdgeProperties":[{{{types},
                "orientationType":"TANGENTIAL","mobileRobotOrientations":[0.0]}}]}}"#
            )
        })
        .collect();
    format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t",
        "exportTimestamp":"2026-11-20T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{}],"edges":[{}]}}]}}]}}"#,
        nodes.join(","),
        edges.join(",")
    )
}

/// The node-node lists a policy is given for a run on the layout in `dir`.
fn heard(dir: &Path, conflict_reservation: bool) -> Vec<NodeNodeConflictInput> {
    let text = format!(
        r#"name = "node-node"
seed = 1
duration = 1.0
time_step = 0.05
start_time = 2026-11-20T10:00:00Z
lif = "layout.lif.json"
conflict_reservation = {conflict_reservation}

[[robot]]
manufacturer = "ACME"
serial_prefix = "R"
factsheet = "diff_robot.factsheet.json"
start_nodes = ["A", "C"]
action_time = 1.0

[jobs]
rate_per_hour = 60.0
nodes = ["A", "B"]
"#
    );
    let file = dir.join(format!("scenario-{conflict_reservation}.toml"));
    std::fs::write(&file, text).unwrap();
    let s = Scenario::load(&file).unwrap();
    let policy = Listen::default();
    let heard = Rc::clone(&policy.heard);
    summary::run_with_policy(&s, s.build_world(dir).unwrap(), Box::new(policy)).unwrap();
    heard.take()
}

#[test]
fn a_policy_is_given_the_pairs_of_nodes_too_close_for_two_robots() {
    let dir = std::env::temp_dir().join(format!("botsim-node-node-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("layout.lif.json"), layout()).unwrap();
    let data = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
    let factsheet = "diff_robot.factsheet.json";
    std::fs::copy(data.join(factsheet), dir.join(factsheet)).unwrap();

    let given = heard(&dir, true);
    assert_eq!(given.len(), 1, "{given:?}");
    assert_eq!(
        (given[0].type_a.as_str(), given[0].type_b.as_str()),
        ("ACME/X1", "ACME/X1")
    );
    assert!(
        given[0].pairs.contains(&("P".into(), "Q".into())),
        "{given:?}"
    );
    // without the swept-area reservation: none
    assert!(heard(&dir, false).is_empty());
    std::fs::remove_dir_all(&dir).unwrap();
}

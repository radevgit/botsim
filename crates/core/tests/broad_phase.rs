//! The collision check uses a broad phase (S52). It must find exactly the pairs
//! that checking every pair finds, in the same order.

use botsim_core::actions::ActionTimes;
use botsim_core::collision::{overlap, place};
use botsim_core::jobs::Rng;
use botsim_core::world::{RobotId, World};
use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vdma_lif::Lif;

const DT: f64 = 0.05;

/// Pairs of robot indices.
type Pairs = Vec<(usize, usize)>;

/// A `side` × `side` block of nodes `spacing` metres apart on one map, plus one
/// edge so that the layout is valid.
fn grid(side: usize, spacing: f64) -> Graph {
    let prop = r#""mobileRobotTypeNodeProperties":[{"mobileRobotTypes":[{"manufacturer":"ACME","seriesName":"X1"}]}]"#;
    let mut nodes = Vec::new();
    for r in 0..side {
        for c in 0..side {
            nodes.push(format!(
                r#"{{"nodeId":"N{r}_{c}","mapId":"M","nodePosition":{{"x":{},"y":{}}},{prop}}}"#,
                c as f64 * spacing,
                r as f64 * spacing
            ));
        }
    }
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{}],"edges":[{{"edgeId":"E","startNodeId":"N0_0","endNodeId":"N0_1",
        "mobileRobotTypeEdgeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}],
        "orientationType":"TANGENTIAL","mobileRobotOrientations":[0.0]}}]}}]}}]}}]}}"#,
        nodes.join(",")
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

/// `robots` robots on random nodes of the block (several may share a node);
/// returns the collisions the world found, as index pairs in the order found, and
/// the pairs that checking every pair finds.
fn run(seed: u64, side: usize, spacing: f64, robots: usize) -> (Pairs, Pairs) {
    let g = grid(side, spacing);
    let mut rng = Rng::new(seed);
    let starts: Vec<usize> = (0..robots).map(|_| rng.below(g.nodes.len())).collect();
    let spots: Vec<(f64, f64)> = starts
        .iter()
        .map(|&n| (g.nodes[n].x, g.nodes[n].y))
        .collect();
    let mut w = World::new(g, DT, 0).unwrap();
    for (i, &n) in starts.iter().enumerate() {
        let id = RobotId::new("ACME", &format!("R-{i}"));
        w.add_robot(id, spec(), ActionTimes::new(1.0), n, 0.0)
            .unwrap();
    }
    w.step();

    let found: Vec<(usize, usize)> = w
        .collisions()
        .iter()
        .map(|c| {
            let n = |id: &RobotId| id.serial_number[2..].parse::<usize>().unwrap();
            (n(&c.a), n(&c.b))
        })
        .collect();

    let s = spec();
    let bodies: Vec<_> = spots
        .iter()
        .map(|&(x, y)| place(&s.shape, x, y, 0.0))
        .collect();
    let mut expected = Vec::new();
    for i in 0..robots {
        for j in i + 1..robots {
            if overlap(&bodies[i], &bodies[j]) {
                expected.push((i, j));
            }
        }
    }
    (found, expected)
}

#[test]
fn the_broad_phase_finds_the_same_collisions_as_checking_every_pair() {
    // dense blocks: robots 1.2 m long and 0.8 m wide on nodes 0.9 m apart, so
    // there are many overlaps, near misses and robots on the same node
    for (seed, side, spacing, robots) in [
        (1, 6, 0.9, 30),
        (2, 8, 1.0, 40),
        (3, 10, 1.3, 60),
        (4, 5, 0.4, 20),
    ] {
        let (found, expected) = run(seed, side, spacing, robots);
        assert!(
            !expected.is_empty(),
            "the test should have overlaps (seed {seed})"
        );
        assert_eq!(found, expected, "seed {seed}");
    }
}

#[test]
fn touching_robots_are_found_and_robots_a_hair_apart_are_not() {
    // 1.2 m long robots facing east on one row: 1.2 m apart touch, 1.2 + 1e-4 do not
    let touching = run_two(1.2);
    assert_eq!(touching, 1);
    let apart = run_two(1.2 + 1e-4);
    assert_eq!(apart, 0);
}

/// Two robots on a row of nodes `spacing` apart; the number of collisions.
fn run_two(spacing: f64) -> usize {
    let g = grid(2, spacing);
    let (a, b) = (g.node_idx("N0_0").unwrap(), g.node_idx("N0_1").unwrap());
    let mut w = World::new(g, DT, 0).unwrap();
    w.add_robot(
        RobotId::new("ACME", "R-0"),
        spec(),
        ActionTimes::new(1.0),
        a,
        0.0,
    )
    .unwrap();
    w.add_robot(
        RobotId::new("ACME", "R-1"),
        spec(),
        ActionTimes::new(1.0),
        b,
        0.0,
    )
    .unwrap();
    w.step();
    w.collisions().len()
}

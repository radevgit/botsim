//! The brief's "Done when": small layouts on which the robots drive their routes
//! to the end, without a collision or a deadlock.

use std::collections::BTreeMap;

use botsim_sweep::ConflictMap;

use crate::graph::{Pass, Refused};
use crate::groups::{Route, groups};
use crate::spots::Spots;
use crate::testing::{LARGE, SMALL, edge, edge_pair, empty_map, layout, node, route};
use crate::traffic::{NotAdded, Report, Reroute, Traffic};

/// How far ahead the robots ask, and whether the last one added asks first.
const WAYS: [(usize, bool); 6] = [
    (1, false),
    (1, true),
    (2, false),
    (2, true),
    (99, false),
    (99, true),
];

/// The module, what it was set up with, and the routes it has taken.
struct World {
    traffic: Traffic,
    spots: Spots,
    routes: Vec<Route>,
}

impl World {
    /// A world over a layout that has what these paths use, without robots.
    fn new(paths: &[&[&str]], map: &ConflictMap) -> World {
        World::with(Spots::new(&layout(paths), map).expect("setup"))
    }

    fn with(spots: Spots) -> World {
        World {
            traffic: Traffic::new(spots.clone()),
            spots,
            routes: Vec::new(),
        }
    }

    fn add(&mut self, route: Route) -> Result<usize, NotAdded> {
        let added = self.traffic.add(route.clone());
        match &added {
            Ok(_) => self.routes.push(route),
            // it stays on its first node
            Err(NotAdded::Standing { .. }) => {
                let mut stands = route;
                stands.cut(0);
                self.routes.push(stands);
            }
            Err(_) => {}
        }
        added
    }

    /// Drive until every robot stands on the end of its route, step by step: each
    /// robot asks for the node `ahead` of the one it stands on, then each one that
    /// has a further node goes to the next, then all report. Returns for each
    /// robot the nodes it stood still on, once per step. Panics on a collision, a
    /// deadlock and a call for another way.
    fn drive(&mut self, (ahead, last_first): (usize, bool)) -> Vec<Vec<usize>> {
        let ends: Vec<usize> = self.routes.iter().map(|r| r.pieces().len() / 2).collect();
        let mut at = vec![0; ends.len()];
        let mut waited = vec![Vec::new(); ends.len()];
        while at != ends {
            let mut turns: Vec<usize> = (0..ends.len()).collect();
            if last_first {
                turns.reverse();
            }
            let mut given = vec![0; ends.len()];
            for robot in turns {
                let release = self.traffic.release(robot, at[robot] + ahead);
                let release = release.expect("a known robot");
                assert_eq!(release.reroute, None, "robot {robot} at {at:?}");
                given[robot] = release.through;
            }
            self.assert_apart(&at, &given);

            let mut moved = false;
            for robot in 0..ends.len() {
                if given[robot] > at[robot] {
                    at[robot] += 1;
                    moved = true;
                } else if at[robot] < ends[robot] {
                    waited[robot].push(at[robot]);
                }
            }
            assert!(moved, "deadlock: the robots stand on {at:?}");
            let reports: Vec<Report> = (0..ends.len())
                .map(|robot| Report {
                    robot,
                    reached: at[robot],
                    released: given[robot],
                })
                .collect();
            assert_eq!(self.traffic.cycle_start(&reports), []);
        }
        waited
    }

    /// No two robots hold pieces that conflict. A robot holds everything from the
    /// node it stands on, or has left last, to the last node released to it, so it
    /// is nowhere else while it drives.
    fn assert_apart(&self, at: &[usize], given: &[usize]) {
        let held = |robot: usize| &self.routes[robot].pieces()[2 * at[robot]..=2 * given[robot]];
        for a in 0..self.routes.len() {
            for b in a + 1..self.routes.len() {
                for p in held(a) {
                    for q in held(b) {
                        let types = (self.routes[a].type_name(), self.routes[b].type_name());
                        assert_eq!(
                            self.spots.conflict(types.0, p, types.1, q),
                            Some(false),
                            "collision: robot {a} on {p}, robot {b} on {q}"
                        );
                    }
                }
            }
        }
    }
}

/// Of two robots, one drives through and the other only waits before it starts.
fn one_waits_at_its_start(waited: &[Vec<usize>]) {
    let [a, b] = waited else {
        panic!("two robots");
    };
    assert_ne!(a.is_empty(), b.is_empty(), "{waited:?}");
    assert!(a.iter().chain(b).all(|&node| node == 0), "{waited:?}");
}

// head on through the corridor X - M - Y
const EAST: [&str; 5] = ["W", "X", "M", "Y", "Z"];
const WEST: [&str; 5] = ["U", "Y", "M", "X", "V"];

fn head_on() -> World {
    let mut world = World::new(&[&EAST, &WEST], &empty_map());
    assert_eq!(world.add(route(&EAST)), Ok(0));
    assert_eq!(world.add(route(&WEST)), Ok(1));
    world
}

#[test]
fn head_on_in_a_corridor() {
    let world = head_on();
    let corridor = groups(&world.spots, &world.routes[0], &world.routes[1]).expect("names");
    assert_eq!(corridor.len(), 1);
    for way in WAYS {
        one_waits_at_its_start(&head_on().drive(way));
    }
}

#[test]
fn two_robots_at_a_crossing() {
    let (a, b) = (["S", "X", "T"], ["U", "X", "V"]);
    for way in WAYS {
        let mut world = World::new(&[&a, &b], &empty_map());
        assert_eq!(world.add(route(&a)), Ok(0));
        assert_eq!(world.add(route(&b)), Ok(1));
        one_waits_at_its_start(&world.drive(way));
    }
}

#[test]
fn three_robots_around_a_ring() {
    // Each drives over two of P, Q, R. If each is given its first one, every
    // robot waits for the next: the deadlock of reserving node by node.
    let routes = [
        ["S0", "P", "Q", "T0"],
        ["S1", "Q", "R", "T1"],
        ["S2", "R", "P", "T2"],
    ];
    for way in WAYS {
        let mut world = World::new(&[&routes[0], &routes[1], &routes[2]], &empty_map());
        for (i, r) in routes.iter().enumerate() {
            assert_eq!(world.add(route(r)), Ok(i));
        }
        let waited = world.drive(way);
        assert!(waited.iter().any(|w| !w.is_empty()), "{waited:?}");
    }
}

#[test]
fn no_reorder_once_the_held_back_robot_is_released() {
    for way in WAYS {
        let mut world = head_on();
        // east is given X; west would go first only if east still could wait
        assert_eq!(world.traffic.release(0, 1).map(|r| r.through), Some(1));
        let at_x = |robot, piece| Pass { robot, piece };
        assert_eq!(
            world.traffic.reverse(at_x(0, 2), at_x(1, 6)),
            Err(Refused::Released(at_x(0, 2)))
        );
        let waited = world.drive(way);
        assert_eq!(waited[0], [0; 0]);
        one_waits_at_its_start(&waited);
    }
}

#[test]
fn another_way_around_a_standing_robot() {
    let stands = ["X"];
    let through = ["S", "X", "T"];
    let around = ["S", "R", "T"];
    for way in WAYS {
        let mut world = World::new(&[&stands, &through, &around], &empty_map());
        assert_eq!(world.add(route(&stands)), Ok(0));
        let why = Box::new(NotAdded::Reroute(Reroute {
            avoid: vec![node("X")],
        }));
        let refused = NotAdded::Standing {
            robot: 1,
            why,
            released_there: Vec::new(),
        };
        assert_eq!(world.add(route(&through)), Err(refused));
        assert_eq!(world.traffic.change(1, route(&around)), Ok(()));
        world.routes[1] = route(&around);
        assert_eq!(world.drive(way), [[0; 0], [0; 0]]);
    }
}

#[test]
fn one_robot_at_a_time_in_a_zone() {
    // two lanes that meet nowhere, both through one zone from node 1 to node 2
    let a = ["A0", "A1", "A2", "A3"];
    let b = ["B0", "B1", "B2", "B3"];
    let lanes = vec![
        node("A1"),
        edge("A1-A2"),
        node("A2"),
        node("B1"),
        edge("B1-B2"),
        node("B2"),
    ];
    let zone = BTreeMap::from([(SMALL.to_string(), lanes)]);
    for way in WAYS {
        let mut spots = Spots::new(&layout(&[&a, &b]), &empty_map()).expect("setup");
        assert_eq!(spots.add_zone("S", "Z", &zone), Ok(()));
        let mut world = World::with(spots);
        assert_eq!(world.add(route(&a)), Ok(0));
        assert_eq!(world.add(route(&b)), Ok(1));
        // one waits before the zone until the other is on node 3, out of it
        let mut waited = world.drive(way);
        waited.sort();
        assert_eq!(waited, [vec![], vec![0, 0, 0]]);

        // without the zone nobody waits
        let mut world = World::new(&[&a, &b], &empty_map());
        assert_eq!(world.add(route(&a)), Ok(0));
        assert_eq!(world.add(route(&b)), Ok(1));
        assert_eq!(world.drive(way), [[0; 0], [0; 0]]);
    }
}

#[test]
fn mixed_sizes_on_neighbouring_edges() {
    let a = ["A0", "A1", "A2"];
    let b = ["B0", "B1", "B2"];
    // the two first edges are too close for a small robot beside a large one
    let mut map = empty_map();
    map.pairs.push(edge_pair(SMALL, "A0-A1", LARGE, "B0-B1"));
    let large = || Route::new(LARGE, &b, &["B0-B1", "B1-B2"]).expect("route");
    for way in WAYS {
        let mut world = World::new(&[&a, &b], &map);
        assert_eq!(world.add(route(&a)), Ok(0));
        assert_eq!(world.add(large()), Ok(1));
        one_waits_at_its_start(&world.drive(way));

        // two small robots pass each other there
        let mut world = World::new(&[&a, &b], &map);
        assert_eq!(world.add(route(&a)), Ok(0));
        assert_eq!(world.add(route(&b)), Ok(1));
        assert_eq!(world.drive(way), [[0; 0], [0; 0]]);
    }
}

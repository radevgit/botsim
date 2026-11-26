//! A check of how [`Traffic::add`], [`Traffic::change`] and [`Traffic::new_order`]
//! place a route: on many small random cases, against trying every "this robot
//! before or after" for every group. Only the conflicts without an order are tried;
//! the orders of what the robot holds stay.
//!
//! The brute force has its own rules for which order may be taken and its own cycle
//! check, so it shares only the groups with the code under test.

use std::collections::BTreeSet;

use crate::graph::{Before, Pass};
use crate::groups::{Route, groups_where};
use crate::spots::{Piece, Spots};
use crate::testing::{SMALL, edge_pair, empty_map, lif, route};
use crate::traffic::{Cancel, NotAdded, Report, Traffic};

/// Cases of the test that always runs, and of the long one.
const CASES: u64 = 5_000;
const MANY_CASES: u64 = 1_000_000;
/// Cases with more groups for the new robot are left out (2^groups choices).
const MOST_GROUPS: usize = 12;

/// xorshift64*: the same cases on every run.
struct Random(u64);

impl Random {
    fn below(&mut self, n: usize) -> usize {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 33) as usize % n
    }
}

/// Every node joined to every other. Opposite edges conflict, and some random
/// pairs of other edges.
fn random_spots(random: &mut Random, nodes: &[String]) -> Spots {
    let mut edges = Vec::new();
    for a in nodes {
        for b in nodes {
            if a != b {
                edges.push((format!("{a}-{b}"), a.as_str(), b.as_str()));
            }
        }
    }
    let ids: Vec<&str> = nodes.iter().map(String::as_str).collect();
    let listed: Vec<(&str, &str, &str)> = edges
        .iter()
        .map(|(id, a, b)| (id.as_str(), *a, *b))
        .collect();
    let mut map = empty_map();
    for (id, a, b) in &edges {
        map.pairs
            .push(edge_pair(SMALL, id, SMALL, &format!("{b}-{a}")));
    }
    for _ in 0..random.below(8) {
        let (p, q) = (random.below(edges.len()), random.below(edges.len()));
        map.pairs
            .push(edge_pair(SMALL, &edges[p].0, SMALL, &edges[q].0));
    }
    Spots::new(&lif(&ids, &listed), &map).expect("setup")
}

/// A random walk. It mostly keeps its first and last node off those where another
/// robot stands or ends: such routes are nearly always refused at once.
fn random_route(random: &mut Random, nodes: &[String], fleet: &Fleet) -> Route {
    let taken: BTreeSet<&Piece> = fleet
        .routes
        .iter()
        .zip(&fleet.reached)
        .flat_map(|(r, &at)| [r.pieces().get(2 * at), r.pieces().last()])
        .flatten()
        .collect();
    let mut tries = 0;
    loop {
        let first = random.below(nodes.len());
        let route = walk(random, nodes, vec![first]);
        let ends = [route.pieces().first(), route.pieces().last()];
        tries += 1;
        if tries > 5 || ends.into_iter().flatten().all(|p| !taken.contains(p)) {
            return route;
        }
    }
}

/// A changed route of the robot: what is released of it, then a random walk on.
fn random_change(random: &mut Random, nodes: &[String], fleet: &Fleet, robot: usize) -> Route {
    let released = fleet.routes[robot]
        .pieces()
        .iter()
        .step_by(2)
        .take(fleet.released[robot] + 1)
        .map(|piece| index_of(nodes, piece))
        .collect();
    walk_on(random, nodes, fleet, robot, released)
}

/// A new order of a robot that stands at the end of its route: a random walk from
/// there.
fn random_next(random: &mut Random, nodes: &[String], fleet: &Fleet, robot: usize) -> Route {
    let end = fleet.routes[robot].pieces().last().expect("a node");
    walk_on(random, nodes, fleet, robot, vec![index_of(nodes, end)])
}

fn index_of(nodes: &[String], piece: &Piece) -> usize {
    let found = nodes.iter().position(|n| *piece == Piece::Node(n.clone()));
    found.expect("a node of the layout")
}

/// A random walk on from `through` for a robot of the fleet. It mostly ends off the
/// nodes where another robot stands or ends.
fn walk_on(
    random: &mut Random,
    nodes: &[String],
    fleet: &Fleet,
    robot: usize,
    through: Vec<usize>,
) -> Route {
    let taken: BTreeSet<&Piece> = fleet
        .routes
        .iter()
        .zip(&fleet.reached)
        .enumerate()
        .filter(|&(other, _)| other != robot)
        .flat_map(|(_, (r, &at))| [r.pieces().get(2 * at), r.pieces().last()])
        .flatten()
        .collect();
    let mut tries = 0;
    loop {
        let route = walk(random, nodes, through.clone());
        tries += 1;
        if tries > 5 || route.pieces().last().is_some_and(|p| !taken.contains(p)) {
            return route;
        }
    }
}

/// The route through the nodes `through` and up to 5 more, each another one than
/// the one before.
fn walk(random: &mut Random, nodes: &[String], mut through: Vec<usize>) -> Route {
    for _ in 0..random.below(6) {
        let at = through[through.len() - 1];
        through.push((at + 1 + random.below(nodes.len() - 1)) % nodes.len());
    }
    let names: Vec<&str> = through.iter().map(|&i| nodes[i].as_str()).collect();
    route(&names)
}

/// What the test knows of the robots: route lengths and how far they are.
#[derive(Default)]
struct Fleet {
    routes: Vec<Route>,
    reached: Vec<usize>,
    released: Vec<usize>,
    /// New orders taken so far.
    new_orders: usize,
}

impl Fleet {
    fn last(&self, robot: usize) -> usize {
        self.routes[robot].pieces().len() / 2
    }

    fn has_left(&self, pass: Pass) -> bool {
        pass.piece / 2 < self.reached[pass.robot]
    }

    /// May this order be taken, apart from cycles?
    fn may(&self, order: &Before) -> bool {
        let ends_there = order.first.piece / 2 == self.last(order.first.robot);
        let given = order.then.piece.div_ceil(2) <= self.released[order.then.robot];
        !ends_there && (!given || self.has_left(order.first))
    }

    /// Let one robot go one step, if the orders allow: release or arrive.
    fn step(&mut self, random: &mut Random, traffic: &mut Traffic) {
        if self.routes.is_empty() {
            return;
        }
        let r = random.below(self.routes.len());
        let next = self.released[r] + 1;
        if random.below(2) == 0 && next <= self.last(r) {
            let free = traffic.graph().orders().iter().all(|o| {
                o.then.robot != r || o.then.piece.div_ceil(2) > next || self.has_left(o.first)
            });
            if free {
                self.released[r] = next;
            }
        } else if self.reached[r] < self.released[r] {
            self.reached[r] += 1;
        }
        assert_eq!(
            traffic.progress(r, self.reached[r], self.released[r]),
            Ok(())
        );
    }

    /// The robots that stand at the end of their route.
    fn at_end(&self) -> Vec<usize> {
        (0..self.routes.len())
            .filter(|&r| self.reached[r] == self.last(r))
            .collect()
    }

    /// The robot stops on a node it holds and the cancelling is confirmed.
    fn cancel(&mut self, random: &mut Random, traffic: &mut Traffic, r: usize) {
        let node = self.reached[r] + random.below(self.released[r] - self.reached[r] + 1);
        let confirmed = Cancel {
            finished: true,
            nothing_left: true,
            standing_on: Some(node),
        };
        assert_eq!(traffic.cancelled(r, confirmed), Ok(true));
        (self.reached[r], self.released[r]) = (node, node);
        self.routes[r].cut(node);
    }

    /// The fleet's side of a new order.
    fn restart(&mut self, r: usize, route: Route) {
        self.routes[r] = route;
        (self.reached[r], self.released[r]) = (0, 0);
    }

    /// One thing happens to the robots: mostly one goes a step, else its route is
    /// changed, cut short, or cancelled and confirmed, or a robot at the end of its
    /// route gets a new order.
    fn happen(&mut self, random: &mut Random, nodes: &[String], traffic: &mut Traffic) {
        if self.routes.is_empty() {
            return;
        }
        let what = random.below(10);
        let at_end = self.at_end();
        if what > 3 || (what == 3 && at_end.is_empty()) {
            return self.step(random, traffic);
        }
        if what == 3 {
            let r = at_end[random.below(at_end.len())];
            let route = random_next(random, nodes, self, r);
            match traffic.new_order(r, route.clone()) {
                // where nothing is taken, it is on the new order all the same
                Ok(()) | Err(NotAdded::Reroute(_)) => {
                    self.restart(r, taken(traffic, r, route));
                    self.new_orders += 1;
                }
                Err(other) => panic!("{other}"),
            }
            assert_eq!(traffic.graph().released(r), Some(self.released[r]));
            return;
        }
        let r = random.below(self.routes.len());
        match what {
            0 => {
                let route = random_change(random, nodes, self, r);
                match traffic.change(r, route.clone()) {
                    Ok(()) | Err(NotAdded::Reroute(_)) => {
                        self.routes[r] = taken(traffic, r, route);
                    }
                    Err(other) => panic!("{other}"),
                }
            }
            1 => {
                assert!(traffic.cut_short(r));
                self.routes[r].cut(self.released[r]);
            }
            _ => self.cancel(random, traffic, r),
        }
        assert_eq!(traffic.graph().released(r), Some(self.released[r]));
    }
}

/// The part of `route` the module has taken for the robot.
fn taken(traffic: &Traffic, robot: usize, mut route: Route) -> Route {
    route.cut(traffic.route_end(robot).expect("a robot of the module"));
    route
}

/// Can the events of all robots be put in a line that keeps their own sequence and
/// all orders? (Take away, again and again, an event nothing waits before.)
fn free_of_cycles(lasts: &[usize], orders: &[Before]) -> bool {
    // per robot the releases of nodes 0..=last, then the arrivals
    let mut start = vec![0];
    for last in lasts {
        start.push(start[start.len() - 1] + 2 * (last + 1));
    }
    let released = |r: usize, k: usize| start[r] + k;
    let reached = |r: usize, k: usize| start[r] + lasts[r] + 1 + k;
    let events = start[lasts.len()];
    let mut later = vec![Vec::new(); events];
    for (r, &last) in lasts.iter().enumerate() {
        for k in 0..=last {
            later[released(r, k)].push(reached(r, k));
            if k < last {
                later[released(r, k)].push(released(r, k + 1));
                later[reached(r, k)].push(reached(r, k + 1));
            }
        }
    }
    for o in orders {
        let leaves_at = o.first.piece / 2 + 1;
        // a pass at the end of a route is never over: nothing waits before it
        if leaves_at <= lasts[o.first.robot] {
            later[reached(o.first.robot, leaves_at)]
                .push(released(o.then.robot, o.then.piece.div_ceil(2)));
        }
    }
    let mut before = vec![0usize; events];
    for &to in later.iter().flatten() {
        before[to] += 1;
    }
    let mut open: Vec<usize> = (0..events).filter(|&e| before[e] == 0).collect();
    let mut taken = 0;
    while let Some(e) = open.pop() {
        taken += 1;
        for &to in &later[e] {
            before[to] -= 1;
            if before[to] == 0 {
                open.push(to);
            }
        }
    }
    taken == events
}

/// How often each kind of case came up.
#[derive(Debug, Default)]
struct Seen {
    /// Placed, and only where it cannot wait did the new robot go first.
    placed: usize,
    /// Placed, and it also had to go first somewhere to keep clear of a cycle.
    overtook: usize,
    /// As `overtook`, in two groups or more.
    overtook_twice: usize,
    /// Refused although each group on its own could go one way or the other.
    refused_for_cycles: usize,
    /// Refused because some group can go neither way.
    refused_at_once: usize,
    left_out: usize,
    /// The robot had orders already, from what is released to it.
    kept_orders: usize,
    /// Some group had conflicts with an order and conflicts without.
    partly_ordered: usize,
    /// Some robot was waiting for a route that was cut short.
    waiting_for_ever: usize,
    /// The robot's route was one cut short that another robot was waiting for.
    awaited_end: usize,
    /// Some robot had been given a new order before.
    after_a_new_order: usize,
    /// Only a first part of the route was taken.
    taken_partly: usize,
}

/// Random robots that have been added, have moved on and may have had their routes
/// changed or cancelled and new orders.
struct State {
    random: Random,
    nodes: Vec<String>,
    spots: Spots,
    traffic: Traffic,
    fleet: Fleet,
}

fn random_state(seed: u64) -> State {
    let mut random = Random(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    let nodes: Vec<String> = (0..6 + random.below(7)).map(|i| format!("N{i}")).collect();
    let spots = random_spots(&mut random, &nodes);
    let mut traffic = Traffic::new(spots.clone());
    let mut fleet = Fleet::default();
    for _ in 0..4 + random.below(20) {
        if fleet.routes.len() < 5 && random.below(3) == 0 {
            let route = random_route(&mut random, &nodes, &fleet);
            let new = fleet.routes.len();
            match traffic.add(route.clone()) {
                // where nothing is taken, it stands on its first node
                Ok(_) | Err(NotAdded::Standing { .. }) => {}
                Err(other) => panic!("{other}"),
            }
            fleet.routes.push(taken(&traffic, new, route));
            fleet.reached.push(0);
            fleet.released.push(0);
        } else {
            fleet.happen(&mut random, &nodes, &mut traffic);
        }
    }
    State {
        random,
        nodes,
        spots,
        traffic,
        fleet,
    }
}

/// A new robot is added.
fn add_case(seed: u64, seen: &mut Seen) {
    let mut s = random_state(seed);
    let route = random_route(&mut s.random, &s.nodes, &s.fleet);
    let new = s.fleet.routes.len();
    s.fleet.routes.push(route.clone());
    s.fleet.reached.push(0);
    s.fleet.released.push(0);
    let place = |t: &mut Traffic| match t.add(route) {
        Ok(robot) => {
            assert_eq!(robot, new);
            Ok(())
        }
        Err(NotAdded::Standing { robot, why, .. }) => {
            assert_eq!(robot, new);
            Err(*why)
        }
        Err(other) => Err(other),
    };
    let kept = what_stays(&s, new);
    judge(seed, s, new, kept, place, seen);
}

/// A random state with at least one robot.
fn random_fleet(seed: u64) -> State {
    let mut s = random_state(seed);
    if s.fleet.routes.is_empty() {
        let route = random_route(&mut s.random, &s.nodes, &s.fleet);
        assert_eq!(s.traffic.add(route.clone()), Ok(0), "seed {seed}");
        assert_eq!(s.traffic.route_end(0), Some(route.pieces().len() / 2));
        s.fleet.routes.push(route);
        s.fleet.reached.push(0);
        s.fleet.released.push(0);
    }
    s
}

/// The orders that stay when the unreleased part of a robot's route is replaced:
/// all but those about what is not released to it.
fn what_stays(s: &State, robot: usize) -> BTreeSet<Before> {
    let released = s.fleet.released[robot];
    let open = |p: Pass| p.robot == robot && p.piece > 2 * released;
    let orders = s.traffic.graph().orders().into_iter();
    orders.filter(|o| !open(o.first) && !open(o.then)).collect()
}

/// Is the robot's route one cut short that another robot is waiting for? Only such
/// a route has orders that wait for its end.
fn is_awaited(s: &State, robot: usize) -> bool {
    let end = 2 * s.fleet.last(robot);
    let orders = s.traffic.graph().orders();
    orders
        .iter()
        .any(|o| o.first.robot == robot && o.first.piece == end)
}

/// A robot's route is changed beyond what is released.
fn change_case(seed: u64, seen: &mut Seen) {
    let mut s = random_fleet(seed);
    let robot = s.random.below(s.fleet.routes.len());
    seen.awaited_end += usize::from(is_awaited(&s, robot));
    let route = random_change(&mut s.random, &s.nodes, &s.fleet, robot);
    let kept = what_stays(&s, robot);
    s.fleet.routes[robot] = route.clone();
    let place = |t: &mut Traffic| t.change(robot, route);
    judge(seed, s, robot, kept, place, seen);
}

/// A robot that stands at the end of its route gets a new order. If none does, one
/// is made to by a confirmed cancel.
fn new_order_case(seed: u64, seen: &mut Seen) {
    let mut s = random_fleet(seed);
    let mut at_end = s.fleet.at_end();
    if at_end.is_empty() {
        let r = s.random.below(s.fleet.routes.len());
        s.fleet.cancel(&mut s.random, &mut s.traffic, r);
        at_end.push(r);
    }
    let robot = at_end[s.random.below(at_end.len())];
    seen.awaited_end += usize::from(is_awaited(&s, robot));
    let route = random_next(&mut s.random, &s.nodes, &s.fleet, robot);
    // the orders about the node it stands on go over to piece 0, those about the
    // pieces it has left go
    let end = 2 * s.fleet.last(robot);
    let renamed = |p: Pass| {
        if p.robot != robot {
            Some(p)
        } else {
            (p.piece == end).then_some(Pass { robot, piece: 0 })
        }
    };
    let kept = s
        .traffic
        .graph()
        .orders()
        .into_iter()
        .filter_map(|o| {
            Some(Before {
                first: renamed(o.first)?,
                then: renamed(o.then)?,
            })
        })
        .collect();
    s.fleet.restart(robot, route.clone());
    let place = |t: &mut Traffic| t.new_order(robot, route);
    judge(seed, s, robot, kept, place, seen);
}

/// The choices there are for the route of one robot.
struct Choices {
    /// Per group the orders still to be chosen: with the robot waiting, and first.
    groups: Vec<[Vec<Before>; 2]>,
    /// Some group has conflicts with an order and conflicts without.
    partly_ordered: bool,
}

fn choices(spots: &Spots, fleet: &Fleet, robot: usize, kept: &BTreeSet<Before>) -> Choices {
    let pass = |robot, piece| Pass { robot, piece };
    let mut groups = Vec::new();
    let mut partly_ordered = false;
    for (other, theirs) in fleet.routes.iter().enumerate() {
        if other == robot {
            continue;
        }
        let found = groups_where(spots, theirs, &fleet.routes[robot], |q, p| {
            !fleet.has_left(pass(other, q)) && !fleet.has_left(pass(robot, p))
        })
        .expect("known names");
        for group in found {
            let (waits, first): (Vec<Before>, Vec<Before>) = group
                .iter()
                .map(|&(q, p)| {
                    let (first, then) = (pass(other, q), pass(robot, p));
                    (
                        Before { first, then },
                        Before {
                            first: then,
                            then: first,
                        },
                    )
                })
                .filter(|(a, b)| !kept.contains(a) && !kept.contains(b))
                .unzip();
            if !waits.is_empty() {
                partly_ordered |= waits.len() < group.len();
                groups.push([waits, first]);
            }
        }
    }
    Choices {
        groups,
        partly_ordered,
    }
}

/// Brute force: every valid choice, bit i set = the robot goes first in group i.
fn valid(fleet: &Fleet, old: &[Before], groups: &[[Vec<Before>; 2]]) -> Vec<usize> {
    let lasts: Vec<usize> = (0..fleet.routes.len()).map(|r| fleet.last(r)).collect();
    (0..1usize << groups.len())
        .filter(|mask| {
            let mut orders = old.to_vec();
            for (i, choice) in groups.iter().enumerate() {
                let taken = &choice[(mask >> i) & 1];
                if !taken.iter().all(|o| fleet.may(o)) {
                    return false;
                }
                orders.extend(taken);
            }
            free_of_cycles(&lasts, &orders)
        })
        .collect()
}

/// `place` puts the route of `robot`, which the fleet has already, into the module:
/// compare what it does with every choice there is. The orders in `kept` stay.
///
/// Where the module takes only a first part, the whole route must have no valid
/// choice, and the part is checked like a whole route. That no longer part could
/// have been taken is **not** checked.
fn judge(
    seed: u64,
    s: State,
    robot: usize,
    kept: BTreeSet<Before>,
    place: impl FnOnce(&mut Traffic) -> Result<(), NotAdded>,
    seen: &mut Seen,
) {
    let State {
        spots,
        mut traffic,
        mut fleet,
        ..
    } = s;
    let old: Vec<Before> = kept.iter().copied().collect();
    let whole = choices(&spots, &fleet, robot, &kept);
    if whole.groups.len() > MOST_GROUPS {
        seen.left_out += 1;
        return;
    }
    seen.kept_orders += usize::from(
        old.iter()
            .any(|o| o.first.robot == robot || o.then.robot == robot),
    );
    seen.partly_ordered += usize::from(whole.partly_ordered);
    seen.after_a_new_order += usize::from(fleet.new_orders > 0);
    // not `reroute_needed`: that is also said for a route that was refused
    seen.waiting_for_ever +=
        usize::from((0..fleet.routes.len()).any(|r| !traffic.graph().waits_for_ever(r).is_empty()));
    let lasts: Vec<usize> = (0..fleet.routes.len()).map(|r| fleet.last(r)).collect();
    assert!(free_of_cycles(&lasts, &old), "seed {seed}: cycle before");
    let valid_whole = valid(&fleet, &old, &whole.groups);

    match place(&mut traffic) {
        Ok(()) => {
            let part = taken(&traffic, robot, fleet.routes[robot].clone());
            let cut = part.pieces().len() < fleet.routes[robot].pieces().len();
            let (choices, valid) = if cut {
                assert!(valid_whole.is_empty(), "seed {seed}: cut {valid_whole:?}");
                fleet.routes[robot] = part;
                // a part goes beyond what is held
                assert!(fleet.last(robot) > fleet.released[robot], "seed {seed}");
                let part = choices(&spots, &fleet, robot, &kept);
                if part.groups.len() > MOST_GROUPS {
                    seen.left_out += 1;
                    return;
                }
                seen.taken_partly += 1;
                let valid = valid(&fleet, &old, &part.groups);
                (part.groups, valid)
            } else {
                (whole.groups, valid_whole)
            };
            // the groups where it cannot wait
            let must = (0..choices.len())
                .filter(|&i| !choices[i][0].iter().all(|o| fleet.may(o)))
                .fold(0usize, |mask, i| mask | 1 << i);
            let orders: BTreeSet<Before> = traffic.graph().orders();
            let mut expected = kept.clone();
            let mut mask = 0;
            for (i, [waits, first]) in choices.iter().enumerate() {
                let has = |group: &[Before]| group.iter().all(|o| orders.contains(o));
                assert!(
                    has(waits) != has(first),
                    "seed {seed}: group {i} not ordered"
                );
                if has(first) {
                    mask |= 1 << i;
                    expected.extend(first);
                } else {
                    expected.extend(waits);
                }
            }
            assert_eq!(orders, expected, "seed {seed}: other orders than chosen");
            assert!(valid.contains(&mask), "seed {seed}: invalid placement");
            // it goes first nowhere it could have waited
            assert!(
                valid.iter().all(|v| v & mask == mask),
                "seed {seed}: went first without need, {mask:b} of {valid:?}"
            );
            match (mask & !must).count_ones() {
                0 => seen.placed += 1,
                n => {
                    seen.overtook += 1;
                    seen.overtook_twice += usize::from(n > 1);
                }
            }
        }
        Err(NotAdded::Reroute(_)) => {
            assert!(
                valid_whole.is_empty(),
                "seed {seed}: missed {valid_whole:?}"
            );
            // it holds no more than it did
            assert_eq!(traffic.route_end(robot), Some(fleet.released[robot]));
            let neither = whole
                .groups
                .iter()
                .any(|c| c.iter().all(|way| !way.iter().all(|o| fleet.may(o))));
            if neither {
                seen.refused_at_once += 1;
            } else {
                seen.refused_for_cycles += 1;
            }
        }
        Err(other) => panic!("seed {seed}: {other}"),
    }
}

#[test]
fn placing_a_route_agrees_with_brute_force() {
    compare(CASES, add_case);
}

#[test]
fn changing_a_route_agrees_with_brute_force() {
    compare_known(CASES, change_case);
}

#[test]
fn a_new_order_agrees_with_brute_force() {
    compare_known(CASES, new_order_case);
}

/// Robots that ask for releases again and again, between whatever else happens to
/// them; returns how many attempts the module would have skipped. In tests it makes
/// those all the same and panics if one succeeds or changes something.
fn releases_between_changes(seed: u64) -> usize {
    let mut random = Random(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    let nodes: Vec<String> = (0..6 + random.below(7)).map(|i| format!("N{i}")).collect();
    let spots = random_spots(&mut random, &nodes);
    let mut traffic = Traffic::new(spots);
    let mut fleet = Fleet::default();
    // the route each robot was offered last; the module has a first part of it
    let mut offered: Vec<Route> = Vec::new();
    for _ in 0..200 {
        let robots = fleet.routes.len();
        if robots == 0 || (robots < 5 && random.below(20) == 0) {
            let route = random_route(&mut random, &nodes, &fleet);
            match traffic.add(route.clone()) {
                Ok(_) | Err(NotAdded::Standing { .. }) => {}
                Err(other) => panic!("seed {seed}: {other}"),
            }
            fleet.routes.push(taken(&traffic, robots, route.clone()));
            fleet.reached.push(0);
            fleet.released.push(0);
            offered.push(route);
        } else {
            let r = random.below(robots);
            match random.below(20) {
                0 => {
                    let route = random_change(&mut random, &nodes, &fleet, r);
                    match traffic.change(r, route.clone()) {
                        Ok(()) | Err(NotAdded::Reroute(_)) => offered[r] = route,
                        Err(other) => panic!("seed {seed}: {other}"),
                    }
                }
                1 => {
                    assert!(traffic.cut_short(r));
                    offered[r] = taken(&traffic, r, offered[r].clone());
                }
                2 => {
                    fleet.cancel(&mut random, &mut traffic, r);
                    offered[r] = fleet.routes[r].clone();
                }
                3 | 4 if fleet.reached[r] == fleet.last(r) => {
                    let route = random_next(&mut random, &nodes, &fleet, r);
                    match traffic.new_order(r, route.clone()) {
                        Ok(()) | Err(NotAdded::Reroute(_)) => {
                            offered[r] = route;
                            fleet.reached[r] = 0;
                        }
                        Err(other) => panic!("seed {seed}: {other}"),
                    }
                }
                // one that goes on, or only reports what it reported before
                5..=8 => {
                    if fleet.reached[r] < fleet.released[r] && random.below(2) == 0 {
                        fleet.reached[r] += 1;
                    }
                    let report = Report {
                        robot: r,
                        reached: fleet.reached[r],
                        released: fleet.released[r],
                    };
                    assert_eq!(traffic.cycle_start(&[report]), [], "seed {seed}");
                }
                9 => {
                    traffic.give_next(r);
                }
                _ => {
                    let limit = fleet.released[r] + random.below(3);
                    assert!(traffic.release(r, limit).is_some());
                }
            }
        }
        // as the module has it now
        for (r, route) in offered.iter().enumerate() {
            fleet.released[r] = traffic.graph().released(r).expect("a robot");
            fleet.routes[r] = taken(&traffic, r, route.clone());
        }
    }
    traffic.skipped()
}

#[test]
fn a_skipped_attempt_would_fail_again() {
    let skipped: usize = (0..CASES / 5).map(releases_between_changes).sum();
    assert!(skipped > 20_000, "only {skipped} to check");
}

/// `cargo test -p botsim-traffic --release many_cases -- --ignored --nocapture`
#[test]
#[ignore = "takes minutes"]
fn placing_a_route_agrees_with_brute_force_on_many_cases() {
    compare(MANY_CASES, add_case);
}

#[test]
#[ignore = "takes minutes"]
fn changing_a_route_agrees_with_brute_force_on_many_cases() {
    compare_known(MANY_CASES, change_case);
}

#[test]
#[ignore = "takes minutes"]
fn a_new_order_agrees_with_brute_force_on_many_cases() {
    compare_known(MANY_CASES, new_order_case);
}

/// [`compare`] for a robot the module knows already.
fn compare_known(cases: u64, case: fn(u64, &mut Seen)) {
    let seen = compare(cases, case);
    for (kind, times) in [
        ("with kept orders", seen.kept_orders),
        ("with a group partly ordered", seen.partly_ordered),
        ("with an awaited end", seen.awaited_end),
    ] {
        assert!(times as u64 >= at_least(cases), "{kind}: only {times}");
    }
}

/// How often each kind of case has to come up: 0.01 % of the cases, and once.
fn at_least(cases: u64) -> u64 {
    (cases / 10_000).max(1)
}

fn compare(cases: u64, case: fn(u64, &mut Seen)) -> Seen {
    let mut seen = Seen::default();
    for seed in 0..cases {
        case(seed, &mut seen);
    }
    println!("{cases} cases: {seen:?}");
    // every kind of case has to come up, or the check says little
    for (kind, times) in [
        ("placed", seen.placed),
        ("overtook", seen.overtook),
        ("overtook twice", seen.overtook_twice),
        ("refused for cycles", seen.refused_for_cycles),
        ("refused at once", seen.refused_at_once),
        ("with a robot waiting for ever", seen.waiting_for_ever),
        ("after a new order", seen.after_a_new_order),
        ("taken partly", seen.taken_partly),
    ] {
        assert!(times as u64 >= at_least(cases), "{kind}: only {times}");
    }
    assert!(seen.left_out < cases as usize / 100, "{seen:?}");
    seen
}

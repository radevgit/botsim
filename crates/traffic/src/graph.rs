//! The order graph (D1, D4): who passes first where, kept free of cycles.
//!
//! Each robot has two events per node `k` of its route: node `k` is **released**
//! to it (with the edge before it), and it **reaches** node `k`. Its own events
//! come in order: released `k` before released `k + 1` and before reached `k`;
//! reached `k` before reached `k + 1`.
//!
//! "A passes piece `p` before B passes piece `q`" says: B gets `q` released only
//! after A has left `p`, that is after A has reached the node that follows `p`.
//! While these edges form no cycle, some robot can always go on, so there is no
//! deadlock; and every conflict that has an order is free of collisions.
//!
//! What is released stays: no new order may make a robot wait for a piece it has
//! already been given (D4). At the end of its route a robot stays, so that pass is
//! never over and is always the last one at its spot.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;
use std::ops::RangeInclusive;

use crate::groups::Route;

/// One piece of one robot's route.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Pass {
    /// The number [`OrderGraph::add_robot`] gave.
    pub robot: usize,
    /// Index of the piece in the route (the `sequenceId`).
    pub piece: usize,
}

/// `first` is over before `then` begins.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Before {
    pub first: Pass,
    pub then: Pass,
}

/// Why an order is not taken.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refused {
    /// There is no such robot or its route has no such piece.
    UnknownPass(Pass),
    /// Both passes are of the same robot.
    SameRobot(Before),
    /// The pass is at the end of its robot's route: the robot never leaves it, so
    /// nothing can come after it.
    NeverLeft(Pass),
    /// The robot that would have to wait has already been given the piece.
    Released(Pass),
    /// With this order the robots would wait for each other.
    Cycle(Before),
    /// There is no order between these two passes that could be turned round: they
    /// do not conflict, one is over, or none was given.
    NoOrder(Pass, Pass),
}

/// Progress that cannot be: past the end of the route, further than released, or
/// less than before.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BadProgress {
    /// The number of the robot.
    pub robot: usize,
    /// The node index reported as reached.
    pub reached: usize,
    /// The node index reported as released.
    pub released: usize,
}

impl fmt::Display for BadProgress {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "robot {} cannot be at node {} with node {} released",
            self.robot, self.reached, self.released
        )
    }
}

impl Error for BadProgress {}

impl fmt::Display for Refused {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let pass = |p: &Pass| format!("piece {} of robot {}", p.piece, p.robot);
        match self {
            Refused::UnknownPass(p) => write!(f, "there is no {}", pass(p)),
            Refused::SameRobot(b) => {
                write!(f, "robot {} cannot be ordered with itself", b.first.robot)
            }
            Refused::NeverLeft(p) => {
                write!(f, "{} is never left, nothing can follow it", pass(p))
            }
            Refused::Released(p) => write!(f, "{} is already released", pass(p)),
            Refused::Cycle(b) => write!(
                f,
                "{} before {} would be a deadlock",
                pass(&b.first),
                pass(&b.then)
            ),
            Refused::NoOrder(a, b) => {
                write!(f, "there is no order between {} and {}", pass(a), pass(b))
            }
        }
    }
}

impl Error for Refused {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Moment {
    Released(usize),
    Reached(usize),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Event {
    robot: usize,
    moment: Moment,
}

/// An event that waits, with the order that says so.
type Waiter = (Event, Before);

/// How long a robot's route is and how far it is, as node indices.
#[derive(Debug, Clone, Copy)]
struct Progress {
    last: usize,
    reached: usize,
    released: usize,
}

impl Progress {
    /// The index of the piece, if the route has it.
    fn has(&self, piece: usize) -> bool {
        piece <= 2 * self.last
    }

    /// The node the robot must reach to have left the piece. Beyond `last` for the
    /// node the route ends on.
    fn leaves_at(piece: usize) -> usize {
        piece / 2 + 1
    }

    /// The node whose release gives the robot the piece: an edge is released with
    /// the node it leads to.
    fn given_with(piece: usize) -> usize {
        piece.div_ceil(2)
    }
}

/// The event an order waits for, and the one that waits.
fn events(order: Before) -> (Event, Event) {
    let from = Event {
        robot: order.first.robot,
        moment: Moment::Reached(Progress::leaves_at(order.first.piece)),
    };
    let to = Event {
        robot: order.then.robot,
        moment: Moment::Released(Progress::given_with(order.then.piece)),
    };
    (from, to)
}

/// An order as it is filed by who waits: the robot, the node whose release it
/// holds back, the order.
type ByWaiter = (usize, usize, Before);

fn by_waiter(order: Before) -> ByWaiter {
    let node = Progress::given_with(order.then.piece);
    (order.then.robot, node, order)
}

/// No order sorts before this one.
const LEAST: Before = Before {
    first: Pass { robot: 0, piece: 0 },
    then: Pass { robot: 0, piece: 0 },
};

/// The robots' events and the orders between different robots.
#[derive(Debug, Clone, Default)]
pub struct OrderGraph {
    robots: Vec<Progress>,
    /// Event → the events of other robots that wait for it.
    waiting: BTreeMap<Event, BTreeSet<Waiter>>,
    /// The same orders once more, to find those of one robot that waits.
    waiters: BTreeSet<ByWaiter>,
}

impl OrderGraph {
    pub fn new() -> OrderGraph {
        OrderGraph::default()
    }

    /// Add a robot with its route, without any order yet; it stands on the first
    /// node and nothing further is released. Returns its number.
    pub fn add_robot(&mut self, route: &Route) -> usize {
        self.robots.push(Progress {
            // a route alternates nodes and edges and has a node at both ends
            last: route.pieces().len() / 2,
            reached: 0,
            released: 0,
        });
        self.robots.len() - 1
    }

    /// A robot that was cut short gets a longer route again: it now ends at node
    /// `last`. The new passes have no order yet.
    pub(crate) fn lengthen(&mut self, robot: usize, last: usize) {
        if let Some(p) = self.robots.get_mut(robot) {
            p.last = last.max(p.released);
        }
    }

    /// A robot that stands at the end of its route starts a new one, which ends at
    /// node `last`: it stands on node 0 of it and nothing further is released. The
    /// orders about the node it stands on go over to piece 0; those about the pieces
    /// it has left are gone. `false`, and no change, for an unknown robot or one
    /// that is not at the end.
    pub(crate) fn restart(&mut self, robot: usize, last: usize) -> bool {
        let Some(p) = self.robots.get_mut(robot) else {
            return false;
        };
        // what is reached is released, so nothing is held beyond the end either
        if p.reached != p.last {
            return false;
        }
        let end = 2 * p.last;
        *p = Progress {
            last,
            reached: 0,
            released: 0,
        };
        // the pass as it is called from now on, if it stays
        let renamed = |pass: Pass| {
            if pass.robot != robot {
                Some(pass)
            } else {
                (pass.piece == end).then_some(Pass { robot, piece: 0 })
            }
        };
        let waiting = std::mem::take(&mut self.waiting);
        self.waiters.clear();
        for (_, order) in waiting.into_values().flatten() {
            if let (Some(first), Some(then)) = (renamed(order.first), renamed(order.then)) {
                self.file(Before { first, then });
            }
        }
        true
    }

    /// The robot has reached node `reached` and holds its route up to node
    /// `released`. Neither goes back.
    pub fn progress(
        &mut self,
        robot: usize,
        reached: usize,
        released: usize,
    ) -> Result<(), BadProgress> {
        let bad = BadProgress {
            robot,
            reached,
            released,
        };
        let p = self.robots.get_mut(robot).ok_or(bad)?;
        if reached < p.reached || released < p.released || reached > released || released > p.last {
            return Err(bad);
        }
        p.reached = reached;
        p.released = released;
        Ok(())
    }

    /// The last node released to the robot; `None` for an unknown robot.
    pub fn released(&self, robot: usize) -> Option<usize> {
        self.robots.get(robot).map(|p| p.released)
    }

    /// How far the robot is: the node it has reached and the last one released to
    /// it; `None` for an unknown robot.
    pub(crate) fn position(&self, robot: usize) -> Option<(usize, usize)> {
        self.robots.get(robot).map(|p| (p.reached, p.released))
    }

    /// Does the robot stand on the last node of its route? `false` for an unknown
    /// robot.
    pub(crate) fn at_end(&self, robot: usize) -> bool {
        self.robots.get(robot).is_some_and(|p| p.reached == p.last)
    }

    /// The orders that keep node `node`, with the edge before it, from the robot:
    /// those about these pieces whose first pass is not over.
    pub(crate) fn holding_back(&self, robot: usize, node: usize) -> Vec<Before> {
        self.of_waiter(robot, node..=node)
            .filter(|order| !self.has_left(order.first))
            .collect()
    }

    /// The orders that hold back the release to the robot of one of these nodes.
    fn of_waiter(
        &self,
        robot: usize,
        nodes: RangeInclusive<usize>,
    ) -> impl Iterator<Item = Before> {
        let last = *nodes.end();
        self.waiters
            .range((robot, *nodes.start(), LEAST)..)
            .take_while(move |&&(r, node, _)| r == robot && node <= last)
            .map(|&(_, _, order)| order)
    }

    /// File an order; `true` if it was not there.
    fn file(&mut self, order: Before) -> bool {
        let (from, to) = events(order);
        self.waiters.insert(by_waiter(order));
        self.waiting.entry(from).or_default().insert((to, order))
    }

    /// Take an order out, if it is there.
    fn unfile(&mut self, order: Before) {
        let (from, to) = events(order);
        if let Some(set) = self.waiting.get_mut(&from) {
            set.remove(&(to, order));
            if set.is_empty() {
                self.waiting.remove(&from);
            }
        }
        self.waiters.remove(&by_waiter(order));
    }

    /// Release the next node to the robot. `false` if its route has no further one,
    /// or for an unknown robot.
    pub(crate) fn release_next(&mut self, robot: usize) -> bool {
        match self.robots.get_mut(robot) {
            Some(p) if p.released < p.last => {
                p.released += 1;
                true
            }
            _ => false,
        }
    }

    /// Has the robot been given this piece? `false` for an unknown pass.
    pub fn is_released(&self, pass: Pass) -> bool {
        self.robots
            .get(pass.robot)
            .is_some_and(|p| p.has(pass.piece) && Progress::given_with(pass.piece) <= p.released)
    }

    /// Is the robot done with this piece? `false` for an unknown pass.
    pub fn has_left(&self, pass: Pass) -> bool {
        self.robots
            .get(pass.robot)
            .is_some_and(|p| p.has(pass.piece) && Progress::leaves_at(pass.piece) <= p.reached)
    }

    /// Would this order be taken, as far as the two passes go? Whether it closes a
    /// cycle only [`order`](OrderGraph::order) tells.
    pub fn allows(&self, order: Before) -> Result<(), Refused> {
        let first = self.robot_with(order.first)?;
        self.robot_with(order.then)?;
        if order.first.robot == order.then.robot {
            return Err(Refused::SameRobot(order));
        }
        if Progress::leaves_at(order.first.piece) > first.last {
            return Err(Refused::NeverLeft(order.first));
        }
        // an order that is already met makes nobody wait
        if self.is_released(order.then) && !self.has_left(order.first) {
            return Err(Refused::Released(order.then));
        }
        Ok(())
    }

    /// The robot's order is given up: its route now ends at the last node released
    /// to it. The passes beyond that are gone, with every order about them. Orders
    /// that wait for the pass that is now the end stay, and are met by no progress
    /// on this route. `false` for an unknown robot.
    pub fn cut(&mut self, robot: usize) -> bool {
        let Some(p) = self.robots.get_mut(robot) else {
            return false;
        };
        p.last = p.released;
        let end = 2 * p.last;
        let gone = |pass: Pass| pass.robot == robot && pass.piece > end;
        let waiters = &mut self.waiters;
        self.waiting.retain(|_, set| {
            set.retain(|&(_, o)| {
                let stays = !gone(o.first) && !gone(o.then);
                if !stays {
                    waiters.remove(&by_waiter(o));
                }
                stays
            });
            !set.is_empty()
        });
        true
    }

    /// The robot has confirmed that it stands on node `node` and will drive no
    /// further on this route, so what was released beyond is given back: its route
    /// ends there, as after [`cut`](OrderGraph::cut). This removes passes that will
    /// never be driven; the orders of what stays are as before. The node must lie
    /// between the one last reached and the one last released.
    pub fn stop(&mut self, robot: usize, node: usize) -> Result<(), BadProgress> {
        let bad = BadProgress {
            robot,
            reached: node,
            released: node,
        };
        let p = self.robots.get_mut(robot).ok_or(bad)?;
        if node < p.reached || node > p.released {
            return Err(bad);
        }
        p.reached = node;
        p.released = node;
        self.cut(robot);
        Ok(())
    }

    /// Take these orders out of the graph.
    pub(crate) fn remove(&mut self, orders: &[Before]) {
        for &order in orders {
            self.unfile(order);
        }
    }

    /// Remove the orders whose first pass is over: the robot has left the piece, so
    /// they make nobody wait any more. `true` if there was one to remove.
    pub(crate) fn remove_passed(&mut self) -> bool {
        let before = self.waiters.len();
        let robots = &self.robots;
        let waiters = &mut self.waiters;
        // an order is filed under the arrival that ends its first pass
        self.waiting.retain(|from, set| {
            let stays = match from.moment {
                Moment::Reached(node) => robots.get(from.robot).is_some_and(|p| p.reached < node),
                Moment::Released(_) => true,
            };
            if !stays {
                for &(_, order) in set.iter() {
                    waiters.remove(&by_waiter(order));
                }
            }
            stays
        });
        self.waiters.len() != before
    }

    /// The pieces of the robot that wait for a pass that is never over, because the
    /// other robot's route ends there; sorted. Only [`cut`](OrderGraph::cut) and
    /// [`stop`](OrderGraph::stop) leave such orders behind.
    pub fn waits_for_ever(&self, robot: usize) -> Vec<usize> {
        let never = |pass: Pass| {
            self.robots
                .get(pass.robot)
                .is_some_and(|p| Progress::leaves_at(pass.piece) > p.last)
        };
        let pieces: BTreeSet<usize> = self
            .of_waiter(robot, 0..=usize::MAX)
            .filter(|o| never(o.first))
            .map(|o| o.then.piece)
            .collect();
        pieces.into_iter().collect()
    }

    /// All orders in the graph, for tests that check it by other means. Each must
    /// be filed in its place, both ways.
    #[cfg(test)]
    pub(crate) fn orders(&self) -> BTreeSet<Before> {
        for (from, set) in &self.waiting {
            assert!(!set.is_empty());
            for &(to, order) in set {
                assert_eq!((*from, to), events(order));
            }
        }
        let all: BTreeSet<Before> = self.waiting.values().flatten().map(|w| w.1).collect();
        let again: BTreeSet<ByWaiter> = all.iter().map(|&o| by_waiter(o)).collect();
        assert_eq!(self.waiters, again);
        all
    }

    /// Is this order in the graph?
    pub fn is_ordered(&self, order: &Before) -> bool {
        let (from, to) = events(*order);
        self.waiting
            .get(&from)
            .is_some_and(|set| set.contains(&(to, *order)))
    }

    /// Take all these orders, or none of them if one is refused.
    pub fn order(&mut self, orders: &[Before]) -> Result<(), Refused> {
        let mut added = Vec::new();
        for order in orders {
            match self.insert(*order) {
                Ok(true) => added.push(*order),
                Ok(false) => {} // it was there already
                Err(refused) => {
                    self.remove(&added);
                    return Err(refused);
                }
            }
        }
        Ok(())
    }

    /// Take orders that make robots wait for a robot which stays where it stands:
    /// `first` of each is the node its route ends on, so they are never met by
    /// progress on that route. [`order`](OrderGraph::order) refuses those. The
    /// caller makes sure that no `then` is released and that nothing waits before
    /// the standing robot's events, so no cycle can come of it.
    pub(crate) fn wait_for_standing(&mut self, orders: &[Before]) {
        for &order in orders {
            self.file(order);
        }
    }

    /// Add one order; `true` if it was not there.
    fn insert(&mut self, order: Before) -> Result<bool, Refused> {
        self.allows(order)?;
        let (from, to) = events(order);
        if self.reaches(to, from) {
            return Err(Refused::Cycle(order));
        }
        Ok(self.file(order))
    }

    /// The robot of the pass, if the pass exists.
    fn robot_with(&self, pass: Pass) -> Result<Progress, Refused> {
        match self.robots.get(pass.robot) {
            Some(&p) if p.has(pass.piece) => Ok(p),
            _ => Err(Refused::UnknownPass(pass)),
        }
    }

    /// Add to `next` the events that come right after this one.
    fn add_next(&self, event: Event, next: &mut Vec<Event>) {
        let Event { robot, moment } = event;
        let last = self.robots.get(robot).map_or(0, |p| p.last);
        let own = |moment| Event { robot, moment };
        match moment {
            Moment::Released(k) => {
                next.push(own(Moment::Reached(k)));
                if k < last {
                    next.push(own(Moment::Released(k + 1)));
                }
            }
            Moment::Reached(k) => {
                if k < last {
                    next.push(own(Moment::Reached(k + 1)));
                }
            }
        }
        next.extend(self.waiting.get(&event).into_iter().flatten().map(|w| w.0));
    }

    /// Must `to` come after `from` (or are they the same)?
    fn reaches(&self, from: Event, to: Event) -> bool {
        let mut seen = BTreeSet::new();
        let mut open = vec![from];
        while let Some(event) = open.pop() {
            if event == to {
                return true;
            }
            if !seen.insert(event) {
                continue;
            }
            self.add_next(event, &mut open);
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::groups::groups;
    use crate::spots::Spots;
    use crate::testing::{SMALL, edge_pair, empty_map, layout, route};
    use botsim_sweep::ConflictMap;

    /// For the two robots' only group: how many ways to order its conflicts one by
    /// one are accepted, and whether "first robot first everywhere" and "second
    /// first everywhere" are among them.
    fn try_all_orders(a: &[&str], b: &[&str], map: &ConflictMap) -> (usize, bool, bool) {
        let spots = Spots::new(&layout(&[a, b]), map).expect("setup");
        let (a, b) = (route(a), route(b));
        let found = groups(&spots, &a, &b).expect("known names");
        assert_eq!(found.len(), 1, "{found:?}");
        let conflicts = &found[0];
        let all_b = (1usize << conflicts.len()) - 1;
        let (mut accepted, mut a_first, mut b_first) = (0, false, false);
        for choice in 0..=all_b {
            let mut graph = OrderGraph::new();
            let (ra, rb) = (graph.add_robot(&a), graph.add_robot(&b));
            let orders: Vec<Before> = conflicts
                .iter()
                .enumerate()
                .map(|(i, &(p, q))| {
                    let pa = Pass {
                        robot: ra,
                        piece: p,
                    };
                    let pb = Pass {
                        robot: rb,
                        piece: q,
                    };
                    if choice & (1 << i) == 0 {
                        Before {
                            first: pa,
                            then: pb,
                        }
                    } else {
                        Before {
                            first: pb,
                            then: pa,
                        }
                    }
                })
                .collect();
            if graph.order(&orders).is_ok() {
                accepted += 1;
                a_first |= choice == 0;
                b_first |= choice == all_b;
            }
        }
        (accepted, a_first, b_first)
    }

    #[test]
    fn head_on_group_takes_one_order_only() {
        // opposite ways along X - M - Y, with sidings at both ends
        let a = ["W", "X", "M", "Y", "Z"];
        let b = ["U", "Y", "M", "X", "V"];
        let mut map = empty_map();
        map.pairs.push(edge_pair(SMALL, "X-M", SMALL, "M-X"));
        map.pairs.push(edge_pair(SMALL, "M-Y", SMALL, "Y-M"));
        assert_eq!(try_all_orders(&a, &b, &map), (2, true, true));
        // the same with the node conflicts alone
        assert_eq!(try_all_orders(&a, &b, &empty_map()), (2, true, true));
    }

    #[test]
    fn following_group_takes_one_order_only() {
        let a = ["W", "X", "M", "Y", "Z"];
        let b = ["U", "X", "M", "Y", "V"];
        assert_eq!(try_all_orders(&a, &b, &empty_map()), (2, true, true));
    }

    #[test]
    fn separate_groups_may_be_ordered_differently() {
        // two places where only edges conflict, one after the other
        let a: &[&str] = &["A0", "A1", "A2"];
        let b: &[&str] = &["B0", "B1", "B2"];
        let mut map = empty_map();
        map.pairs.push(edge_pair(SMALL, "A0-A1", SMALL, "B0-B1"));
        map.pairs.push(edge_pair(SMALL, "A1-A2", SMALL, "B1-B2"));
        let spots = Spots::new(&layout(&[a, b]), &map).expect("setup");
        let mut graph = OrderGraph::new();
        let (ra, rb) = (graph.add_robot(&route(a)), graph.add_robot(&route(b)));
        assert_eq!(groups(&spots, &route(a), &route(b)).map(|g| g.len()), Ok(2));
        let pass = |robot, piece| Pass { robot, piece };
        let mixed = [
            Before {
                first: pass(ra, 1),
                then: pass(rb, 1),
            },
            Before {
                first: pass(rb, 3),
                then: pass(ra, 3),
            },
        ];
        assert_eq!(graph.order(&mixed), Ok(()));
    }

    #[test]
    fn circular_wait_of_three_is_refused() {
        // each robot drives over two of the crossings P, Q, R of a ring
        let routes = [
            route(&["S0", "P", "Q", "T0"]),
            route(&["S1", "Q", "R", "T1"]),
            route(&["S2", "R", "P", "T2"]),
        ];
        let mut graph = OrderGraph::new();
        for r in &routes {
            graph.add_robot(r);
        }
        // at each crossing, the robot that gets there first on its way goes first
        let at = |first: usize, then: usize| Before {
            first: Pass {
                robot: first,
                piece: 2,
            },
            then: Pass {
                robot: then,
                piece: 4,
            },
        };
        let (q, r, p) = (at(1, 0), at(2, 1), at(0, 2));
        assert_eq!(graph.order(&[q, r]), Ok(()));
        assert_eq!(graph.order(&[p]), Err(Refused::Cycle(p)));
        // the other way round at P is fine
        let p_other = Before {
            first: p.then,
            then: p.first,
        };
        assert_eq!(graph.order(&[p_other]), Ok(()));
    }

    #[test]
    fn what_is_released_does_not_wait() {
        let mut graph = OrderGraph::new();
        let a = graph.add_robot(&route(&["A", "X", "B"]));
        let b = graph.add_robot(&route(&["C", "X", "D"]));
        let pass = |robot, piece| Pass { robot, piece };
        let a_first = Before {
            first: pass(a, 2),
            then: pass(b, 2),
        };
        assert_eq!(graph.progress(b, 0, 1), Ok(()));
        assert!(graph.is_released(pass(b, 2)));
        assert_eq!(graph.order(&[a_first]), Err(Refused::Released(pass(b, 2))));
        // once a has left X the order is met and makes nobody wait
        assert_eq!(graph.progress(a, 2, 2), Ok(()));
        assert!(graph.has_left(pass(a, 2)));
        assert_eq!(graph.order(&[a_first]), Ok(()));
    }

    #[test]
    fn impossible_progress_is_reported() {
        let mut graph = OrderGraph::new();
        let a = graph.add_robot(&route(&["A", "X", "B"]));
        assert_eq!(graph.progress(a, 1, 2), Ok(()));
        // further than released, past the end, backwards, unknown robot
        for (robot, reached, released) in [(a, 2, 1), (a, 1, 3), (a, 0, 2), (7, 0, 0)] {
            let bad = BadProgress {
                robot,
                reached,
                released,
            };
            assert_eq!(graph.progress(robot, reached, released), Err(bad));
        }
    }

    #[test]
    fn refused_orders_leave_the_graph_as_it_was() {
        let mut graph = OrderGraph::new();
        let a = graph.add_robot(&route(&["A", "X", "B"]));
        let b = graph.add_robot(&route(&["C", "X"]));
        let pass = |robot, piece| Pass { robot, piece };
        let good = Before {
            first: pass(a, 2),
            then: pass(b, 2),
        };
        // b's route ends at X: b never leaves it
        let stays = Before {
            first: pass(b, 2),
            then: pass(a, 2),
        };
        assert_eq!(
            graph.order(&[good, stays]),
            Err(Refused::NeverLeft(pass(b, 2)))
        );
        assert!(!graph.is_ordered(&good));
        assert_eq!(graph.order(&[good]), Ok(()));
        // an order taken before stays when a later call is refused
        assert!(graph.order(&[good, stays]).is_err());
        assert!(graph.is_ordered(&good));

        let missing = Before {
            first: pass(a, 5),
            then: pass(b, 0),
        };
        assert_eq!(
            graph.order(&[missing]),
            Err(Refused::UnknownPass(pass(a, 5)))
        );
        let own = Before {
            first: pass(a, 0),
            then: pass(a, 2),
        };
        assert_eq!(graph.order(&[own]), Err(Refused::SameRobot(own)));
    }
}

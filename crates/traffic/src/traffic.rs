//! The robots' routes and the order between them.
//!
//! A new route's passes go **after** those already ordered (D4). A robot goes
//! **ahead** instead, for a whole group, only if it cannot wait there:
//!
//! - the other robot's route ends there, so it would never be over; or
//! - the new robot stands there; or
//! - waiting there, together with the groups already ahead, would make robots
//!   wait for each other.
//!
//! Going ahead needs the other robot not yet released into those pieces and the
//! new robot not ending on them. If a group can go neither way, the route is not
//! taken: the dispatcher shall find another one (D9). The orders between the other
//! robots stay as they are. A new robot whose route is not taken stays all the
//! same, standing on its first node, and the others wait for it there.
//!
//! A **changed route** ([`Traffic::change`]) keeps what is released. The rest of
//! the old route is removed and the rest of the new one is added in the same way
//! (D4). If it is not taken, the robot's route ends at its last released node
//! until one is.
//!
//! A **new order** ([`Traffic::new_order`]), numbered from 0 again, is taken only
//! from a robot that holds nothing beyond the node it starts on: it stands at the
//! end of its route, which it has finished or whose cancelling is confirmed.
//!
//! While the robots drive, the **first to come** goes first where both orders are
//! safe ([`Traffic::give_next`]). A **release query** ([`Traffic::release`]) asks
//! node by node and says whether the robot must take another way. A **zone
//! request** ([`Traffic::zone_request`]) is granted by what is released, and
//! [`Traffic::holder`] names who has a piece, [`Traffic::blockers`] who holds a
//! robot back.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

use crate::graph::{BadProgress, Before, OrderGraph, Pass, Refused};
use crate::groups::{Route, UnknownName, check, groups_where};
use crate::spots::{Piece, Spots};

/// "Cannot place this route": what another route should keep clear of.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Reroute {
    /// Pieces of the refused route, without what the robot already holds, in order
    /// and each once.
    pub avoid: Vec<Piece>,
}

/// What a robot's state says about the cancelling of its order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cancel {
    /// The `cancelOrder` action is `FINISHED`.
    pub finished: bool,
    /// `nodeStates` and `edgeStates` are empty.
    pub nothing_left: bool,
    /// The index in the route of the node it stands on; `None` if it is not
    /// standing on a node.
    pub standing_on: Option<usize>,
}

/// How far one robot is, as node indices in its route.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Report {
    /// The number of the robot.
    pub robot: usize,
    /// The node it has reached last.
    pub reached: usize,
    /// The last node released to it.
    pub released: usize,
}

/// The answer to a release query.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Release {
    /// The index in the route of the last node now released to the robot.
    pub through: usize,
    /// The robot must take another way (see
    /// [`reroute_needed`](Traffic::reroute_needed)).
    pub reroute: Option<Reroute>,
}

/// The answer to a request to enter a RELEASE zone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZoneAnswer {
    /// The robot may enter.
    Granted,
    /// Not yet: it has to wait.
    Queued,
}

/// Why a route is not taken.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NotAdded {
    Unknown(UnknownName),
    Reroute(Reroute),
    /// A change or a new order for a robot there is none of.
    UnknownRobot(usize),
    /// A route of another robot type; or a changed route that does not begin with
    /// what the robot has been released; or a new order that does not start on the
    /// node the robot stands on.
    OtherRoute,
    /// A new order for a robot that does not stand at the end of its route.
    NotAtEnd,
    /// The route of a new robot is not taken, for the reason `why`. The robot is
    /// in the module all the same, with this number, standing on the first node:
    /// that is its route until a change or a new order is taken.
    Standing {
        robot: usize,
        why: Box<NotAdded>,
        /// The robots that are released into a piece conflicting with that node
        /// and have not left it, lowest number first. The module cannot hold
        /// them back: the dispatcher shall cancel them.
        released_there: Vec<usize>,
    },
}

impl fmt::Display for NotAdded {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NotAdded::Unknown(name) => name.fmt(f),
            NotAdded::Reroute(r) => {
                write!(f, "the route cannot be placed, avoid")?;
                for (i, piece) in r.avoid.iter().enumerate() {
                    write!(f, "{} {piece}", if i == 0 { "" } else { "," })?;
                }
                Ok(())
            }
            NotAdded::UnknownRobot(robot) => write!(f, "there is no robot {robot}"),
            NotAdded::OtherRoute => {
                write!(f, "the route does not go on from where the robot is")
            }
            NotAdded::NotAtEnd => {
                write!(f, "the robot does not stand at the end of its route")
            }
            NotAdded::Standing {
                robot,
                why,
                released_there,
            } => {
                write!(f, "robot {robot} stays on its first node: {why}")?;
                for (i, other) in released_there.iter().enumerate() {
                    let lead = if i == 0 {
                        "; released there already: robot"
                    } else {
                        ", robot"
                    };
                    write!(f, "{lead} {other}")?;
                }
                Ok(())
            }
        }
    }
}

impl Error for NotAdded {}

impl From<UnknownName> for NotAdded {
    fn from(name: UnknownName) -> NotAdded {
        NotAdded::Unknown(name)
    }
}

/// The routes of all robots and the order graph over them.
#[derive(Debug, Clone)]
pub struct Traffic {
    spots: Spots,
    graph: OrderGraph,
    /// By robot number.
    routes: Vec<Route>,
    /// The robots whose first or last changed route could not be placed, with what
    /// another one should keep clear of.
    refused: BTreeMap<usize, Reroute>,
    /// The robots that kept a robot's last route from being taken whole.
    refused_by: BTreeMap<usize, Vec<usize>>,
    /// Experiment (T38): are routes taken as far as they can be?
    partly: bool,
}

/// Why a route cannot be placed.
struct Blocked {
    why: NotAdded,
    /// The first piece of the route that is to blame, if one can be named.
    first: Option<usize>,
    /// The robots that keep the route from being placed.
    by: BTreeSet<usize>,
}

impl From<UnknownName> for Blocked {
    fn from(name: UnknownName) -> Blocked {
        Blocked {
            why: name.into(),
            first: None,
            by: BTreeSet::new(),
        }
    }
}

impl Traffic {
    pub fn new(spots: Spots) -> Traffic {
        Traffic {
            spots,
            graph: OrderGraph::new(),
            routes: Vec::new(),
            refused: BTreeMap::new(),
            refused_by: BTreeMap::new(),
            partly: false,
        }
    }

    pub fn graph(&self) -> &OrderGraph {
        &self.graph
    }

    /// **Experiment (T36), off unless called; not part of the design.** A route may
    /// pass where a robot stands at the end of its route: it waits behind that
    /// robot instead of being refused. Until the standing robot gets a route on,
    /// the waiting one waits for ever and is told to re-route. A standing robot
    /// whose own route would have to wait in turn for the waiting one is refused.
    pub fn wait_behind_standing(&mut self) {
        self.graph.allow_waiting_behind_standing();
    }

    /// **Experiment (T38), off unless called; not part of the design.** A route
    /// that cannot be placed whole is taken as far as it can be: [`add`],
    /// [`change`] and [`new_order`] then succeed, and the robot's route ends where
    /// the part taken ends ([`route_end`](Traffic::route_end)) until the rest is
    /// offered again ([`extend`](Traffic::extend)). The part is found by cutting
    /// before the first piece that is to blame, again and again. A route of which
    /// nothing beyond what is held can be taken is refused as without this.
    ///
    /// [`add`]: Traffic::add
    /// [`change`]: Traffic::change
    /// [`new_order`]: Traffic::new_order
    pub fn take_routes_partly(&mut self) {
        self.partly = true;
    }

    /// The last node of the robot's route as the module has it.
    pub fn route_end(&self, robot: usize) -> Option<usize> {
        self.routes.get(robot).map(|r| r.pieces().len() / 2)
    }

    /// The robots that kept the robot's last route from being taken, or from being
    /// taken further, lowest number first; empty if it was taken whole.
    pub fn refused_by(&self, robot: usize) -> Vec<usize> {
        self.refused_by.get(&robot).cloned().unwrap_or_default()
    }

    /// **Experiment (T38).** Offer a route that was taken partly again: `route` is
    /// the whole route. It is taken as by [`change`](Traffic::change) if that gets
    /// further than before; otherwise nothing changes but what
    /// [`refused_by`](Traffic::refused_by) says. `true`: it goes further now.
    pub fn extend(&mut self, robot: usize, route: Route) -> bool {
        let Some(old) = self.routes.get(robot).cloned() else {
            return false;
        };
        let end = old.pieces().len() / 2;
        let graph = self.graph.clone();
        let refused = self.refused.get(&robot).cloned();
        let further = self.change(robot, route).is_ok() && self.route_end(robot) > Some(end);
        if !further {
            self.graph = graph;
            if let Some(held) = self.routes.get_mut(robot) {
                *held = old;
            }
            match refused {
                Some(refused) => self.refused.insert(robot, refused),
                None => self.refused.remove(&robot),
            };
        }
        further
    }

    /// See [`OrderGraph::progress`].
    pub fn progress(
        &mut self,
        robot: usize,
        reached: usize,
        released: usize,
    ) -> Result<(), BadProgress> {
        self.graph.progress(robot, reached, released)
    }

    /// The start of a cycle: take the progress of the robots, then remove what has
    /// been passed. Returns the reports that cannot be (see
    /// [`OrderGraph::progress`]); the others are taken all the same.
    pub fn cycle_start(&mut self, reports: &[Report]) -> Vec<BadProgress> {
        let bad = reports
            .iter()
            .filter_map(|r| self.graph.progress(r.robot, r.reached, r.released).err())
            .collect();
        self.graph.remove_passed();
        bad
    }

    /// The robot's order is refused or cancelled: its route now ends at the last
    /// node released to it, and it still holds everything up to there, until a
    /// confirmed cancel ([`cancelled`](Traffic::cancelled)) says where it stopped.
    /// Robots ordered to pass the end after it are told to re-route
    /// ([`reroute_needed`](Traffic::reroute_needed)). `false` for an unknown robot.
    pub fn cut_short(&mut self, robot: usize) -> bool {
        let Some(released) = self.graph.released(robot) else {
            return false;
        };
        if let Some(route) = self.routes.get_mut(robot) {
            route.cut(released);
        }
        self.graph.cut(robot)
    }

    /// The robot reports on the cancelling of its order. Once all three signs are
    /// there, its route ends at the node it stands on and what was released beyond
    /// is free; robots ordered to pass that node after it are told to re-route.
    /// `Ok(false)`: not confirmed, nothing changes. An error for an unknown robot,
    /// or a confirmed node it cannot be on.
    pub fn cancelled(&mut self, robot: usize, report: Cancel) -> Result<bool, BadProgress> {
        let confirmed = report.finished && report.nothing_left;
        let Some(node) = report.standing_on.filter(|_| confirmed) else {
            return match self.graph.released(robot) {
                Some(_) => Ok(false),
                // nothing is known about it
                None => Err(BadProgress {
                    robot,
                    reached: 0,
                    released: 0,
                }),
            };
        };
        self.graph.stop(robot, node)?;
        if let Some(route) = self.routes.get_mut(robot) {
            route.cut(node);
        }
        Ok(true)
    }

    /// Turn round the order of two robots for the whole group of the conflict
    /// between pass `a` and pass `b` (the validity rule, D4): every order of the
    /// group, so that who waited now goes first. All or nothing: each turned order
    /// must be allowed (the robot that would now wait is not released into its
    /// piece, the one that would go first does not end its route there) and together
    /// they must close no cycle.
    pub fn reverse(&mut self, a: Pass, b: Pass) -> Result<(), Refused> {
        let known = |p: Pass| {
            self.routes
                .get(p.robot)
                .filter(|r| p.piece < r.pieces().len())
                .ok_or(Refused::UnknownPass(p))
        };
        let (route_a, route_b) = (known(a)?, known(b)?);
        if a.robot == b.robot {
            return Err(Refused::SameRobot(Before { first: a, then: b }));
        }
        let old = self.group_orders(route_a, route_b, a, b);
        if old.is_empty() {
            return Err(Refused::NoOrder(a, b));
        }
        self.graph = turned(&self.graph, &old)?;
        Ok(())
    }

    /// The robot asks for its next node (D6, "safe first come, first served"). It
    /// gets it if nothing holds it back. Where others are ordered before it there
    /// and have not left, all the groups that hold it back are turned round
    /// together, under the rules of [`reverse`](Traffic::reverse), and stay turned
    /// only if the robot then gets the node. `true`: the node is released to it.
    /// `false`: it has to wait, its route has no further node, or there is no such
    /// robot; nothing has changed.
    pub fn give_next(&mut self, robot: usize) -> bool {
        let Some(released) = self.graph.released(robot) else {
            return false;
        };
        let node = released + 1;
        let holding = self.graph.holding_back(robot, node);
        if holding.is_empty() {
            return self.graph.release_next(robot);
        }
        // each order of each group once, also if a group holds back with several
        let mut old = BTreeSet::new();
        for order in holding {
            let (mine, theirs) = (order.then, order.first);
            let (Some(a), Some(b)) = (self.routes.get(mine.robot), self.routes.get(theirs.robot))
            else {
                return false;
            };
            old.extend(self.group_orders(a, b, mine, theirs));
        }
        let old: Vec<Before> = old.into_iter().collect();
        let Ok(mut trial) = turned(&self.graph, &old) else {
            return false;
        };
        if !trial.holding_back(robot, node).is_empty() || !trial.release_next(robot) {
            return false;
        }
        self.graph = trial;
        true
    }

    /// The orders of the group that holds the conflict between pass `a`, on
    /// `route_a`, and pass `b`, on `route_b`; none if that is not a conflict still
    /// to come.
    fn group_orders(&self, route_a: &Route, route_b: &Route, a: Pass, b: Pass) -> Vec<Before> {
        let pass = |robot, piece| Pass { robot, piece };
        // a pass that is over takes no part in a group
        let found = groups_where(&self.spots, route_a, route_b, |p, q| {
            !self.graph.has_left(pass(a.robot, p)) && !self.graph.has_left(pass(b.robot, q))
        });
        // the routes are in the module, so their names are known
        let group = found
            .ok()
            .into_iter()
            .flatten()
            .find(|g| g.contains(&(a.piece, b.piece)));
        group
            .iter()
            .flatten()
            .flat_map(|&(p, q)| {
                let (first, then) = (pass(a.robot, p), pass(b.robot, q));
                let one_way = Before { first, then };
                let other_way = Before {
                    first: then,
                    then: first,
                };
                [one_way, other_way]
            })
            .filter(|o| self.graph.is_ordered(o))
            .collect()
    }

    /// Must this robot take another way? Yes if its first route
    /// ([`add`](Traffic::add)) or its last changed one could not be placed, until a
    /// route of it is taken; and while a piece of its route waits for a robot whose
    /// route ends there. If it has no other way, it goes on waiting.
    pub fn reroute_needed(&self, robot: usize) -> Option<Reroute> {
        let route = self.routes.get(robot)?;
        let waits = avoiding(route, self.graph.waits_for_ever(robot));
        let Some(refused) = self.refused.get(&robot) else {
            return (!waits.avoid.is_empty()).then_some(waits);
        };
        let mut both = refused.clone();
        for piece in waits.avoid {
            if !both.avoid.contains(&piece) {
                both.avoid.push(piece);
            }
        }
        Some(both)
    }

    /// The release query: give the robot node by node, as
    /// [`give_next`](Traffic::give_next) does, up to node `limit` of its route or
    /// until it has to wait. `None` for an unknown robot.
    pub fn release(&mut self, robot: usize, limit: usize) -> Option<Release> {
        let mut through = self.graph.released(robot)?;
        while through < limit && self.give_next(robot) {
            through += 1;
        }
        Some(Release {
            through,
            reroute: self.reroute_needed(robot),
        })
    }

    /// The robot asks to enter a RELEASE zone. [`ZoneAnswer::Granted`] while a
    /// piece of its route inside the zone, not yet left, is released to it; the
    /// zone's conflicts keep the others out until it has left. Else
    /// [`ZoneAnswer::Queued`]. Asking releases nothing: who is released there
    /// first goes first, not who asks first. `None` for an unknown robot or zone.
    pub fn zone_request(
        &self,
        robot: usize,
        zone_set_id: &str,
        zone_id: &str,
    ) -> Option<ZoneAnswer> {
        let route = self.routes.get(robot)?;
        let inside = self.spots.zone(zone_set_id, zone_id)?;
        let granted = inside
            .get(route.type_name())
            .is_some_and(|inside| self.held(robot).any(|piece| inside.contains(piece)));
        Some(if granted {
            ZoneAnswer::Granted
        } else {
            ZoneAnswer::Queued
        })
    }

    /// The holder query: the robot that holds this node or edge, which is released
    /// to it and which it has not left. A piece that only conflicts with a held one
    /// has no holder. `None` if no robot holds it.
    pub fn holder(&self, piece: &Piece) -> Option<usize> {
        // releases of this module give a piece to one robot; should reports say
        // otherwise, the lowest number is named
        (0..self.routes.len()).find(|&robot| self.held(robot).any(|held| held == piece))
    }

    /// Who holds the robot back: the robots ordered to pass before it where its
    /// next node would take it, that have not left there; lowest number first.
    /// Empty if the node is free to be released, the route has no further node, or
    /// there is no such robot. Asking changes no order:
    /// [`give_next`](Traffic::give_next) may still turn them round.
    pub fn blockers(&self, robot: usize) -> Vec<usize> {
        let Some(released) = self.graph.released(robot) else {
            return Vec::new();
        };
        let holding = self.graph.holding_back(robot, released + 1);
        let firsts: BTreeSet<usize> = holding.into_iter().map(|o| o.first.robot).collect();
        firsts.into_iter().collect()
    }

    /// The pieces released to the robot that it has not left, in route order.
    fn held(&self, robot: usize) -> impl Iterator<Item = &Piece> {
        let route = self.routes.get(robot);
        let released = self.graph.released(robot).unwrap_or(0);
        (0..=2 * released)
            .filter(move |&piece| !self.graph.has_left(Pass { robot, piece }))
            .filter_map(move |piece| route?.pieces().get(piece))
    }

    /// Add one more robot, standing on the first node of its route. Returns its
    /// number. If the route is not taken, the robot still stands there: it is kept
    /// with that node as its route ([`NotAdded::Standing`] gives its number), and
    /// robots ordered to pass there are told to re-route
    /// ([`reroute_needed`](Traffic::reroute_needed)). Those released there
    /// already cannot be held back; the answer names them. Only a robot whose type
    /// or first node is unknown is not kept.
    pub fn add(&mut self, route: Route) -> Result<usize, NotAdded> {
        let mut graph = self.graph.clone();
        let new = graph.add_robot(&route);
        match self.place_far(graph, new, route.clone()) {
            Ok((graph, taken, by)) => {
                self.graph = graph;
                self.routes.push(taken);
                self.note_refused_by(new, by);
                Ok(new)
            }
            Err(blocked) => {
                let answer = self.keep_standing(route, blocked.why);
                if let NotAdded::Standing { robot, .. } = answer {
                    self.note_refused_by(robot, blocked.by.into_iter().collect());
                }
                Err(answer)
            }
        }
    }

    fn note_refused_by(&mut self, robot: usize, by: Vec<usize>) {
        if by.is_empty() {
            self.refused_by.remove(&robot);
        } else {
            self.refused_by.insert(robot, by);
        }
    }

    /// A new robot's route is not taken, for the reason `why`: keep the robot on
    /// its first node and make the others wait for it, as far as they are not
    /// released there. Returns what to answer.
    fn keep_standing(&mut self, mut route: Route, why: NotAdded) -> NotAdded {
        route.cut(0);
        if check(&self.spots, &route).is_err() {
            return why;
        }
        // the number it gets
        if let NotAdded::Reroute(reroute) = &why {
            self.refused.insert(self.routes.len(), reroute.clone());
        }
        let mut graph = self.graph.clone();
        let robot = graph.add_robot(&route);
        let mut waiting = Vec::new();
        let mut released_there = Vec::new();
        for (other, theirs) in self.routes.iter().enumerate() {
            let theirs_pass = |piece| Pass {
                robot: other,
                piece,
            };
            let found = groups_where(&self.spots, theirs, &route, |q, _| {
                !graph.has_left(theirs_pass(q))
            });
            // both routes have only known names
            for (q, piece) in found.into_iter().flatten().flatten() {
                if !graph.is_released(theirs_pass(q)) {
                    waiting.push(Before {
                        first: Pass { robot, piece },
                        then: theirs_pass(q),
                    });
                } else if released_there.last() != Some(&other) {
                    released_there.push(other);
                }
            }
        }
        graph.wait_for_standing(&waiting);
        self.graph = graph;
        self.routes.push(route);
        NotAdded::Standing {
            robot,
            why: Box::new(why),
            released_there,
        }
    }

    /// The robot's route is changed beyond what is released: `route` is the whole
    /// route of the same order, beginning with what is released. If it is not
    /// taken, the robot keeps only what is released: its route ends there, as after
    /// [`cut_short`](Traffic::cut_short), until a changed route is taken.
    pub fn change(&mut self, robot: usize, route: Route) -> Result<(), NotAdded> {
        // remove what is not released; this stays if the new rest is not taken
        if !self.cut_short(robot) {
            return Err(NotAdded::UnknownRobot(robot));
        }
        let Some(held) = self.routes.get(robot) else {
            return Err(NotAdded::UnknownRobot(robot));
        };
        if held.type_name() != route.type_name() || !route.pieces().starts_with(held.pieces()) {
            return Err(NotAdded::OtherRoute);
        }
        let mut graph = self.graph.clone();
        graph.lengthen(robot, route.pieces().len() / 2);
        match self.place_far(graph, robot, route) {
            Ok((graph, taken, by)) => {
                self.graph = graph;
                if let Some(old) = self.routes.get_mut(robot) {
                    *old = taken;
                }
                self.refused.remove(&robot);
                self.note_refused_by(robot, by);
                Ok(())
            }
            Err(blocked) => {
                if let NotAdded::Reroute(reroute) = &blocked.why {
                    self.refused.insert(robot, reroute.clone());
                }
                self.note_refused_by(robot, blocked.by.into_iter().collect());
                Err(blocked.why)
            }
        }
    }

    /// A new order for a robot that stands at the end of its route: `route` starts
    /// on that node and is numbered from 0 again, and so is progress from now on.
    /// Robots that wait for it to leave that node go on waiting for it. A route
    /// that is not taken changes nothing: the robot stays where it is.
    pub fn new_order(&mut self, robot: usize, route: Route) -> Result<(), NotAdded> {
        let Some(old) = self.routes.get(robot) else {
            return Err(NotAdded::UnknownRobot(robot));
        };
        let mut graph = self.graph.clone();
        if !graph.restart(robot, route.pieces().len() / 2) {
            return Err(NotAdded::NotAtEnd);
        }
        if old.type_name() != route.type_name() || old.pieces().last() != route.pieces().first() {
            return Err(NotAdded::OtherRoute);
        }
        match self.place_far(graph, robot, route) {
            Ok((graph, taken, by)) => {
                self.graph = graph;
                if let Some(old) = self.routes.get_mut(robot) {
                    *old = taken;
                }
                self.refused.remove(&robot);
                self.note_refused_by(robot, by);
                Ok(())
            }
            Err(blocked) => {
                self.note_refused_by(robot, blocked.by.into_iter().collect());
                Err(blocked.why)
            }
        }
    }

    /// [`place`](Traffic::place) the route and, where routes are taken partly
    /// (T38) and the whole one cannot be placed, as far as it can. `graph` knows
    /// the whole length. Returns the graph, the route as taken, and the robots that
    /// keep the next piece from being taken; none if it is taken whole.
    fn place_far(
        &self,
        graph: OrderGraph,
        robot: usize,
        route: Route,
    ) -> Result<(OrderGraph, Route, Vec<usize>), Blocked> {
        let mut blocked = match self.place(graph.clone(), robot, &route) {
            Ok(graph) => return Ok((graph, route, Vec::new())),
            Err(blocked) => blocked,
        };
        let held = graph.released(robot).unwrap_or(0);
        let mut first = blocked.first.filter(|_| self.partly);
        while let Some(piece) = first {
            // the node before the first piece that is to blame; each time an
            // earlier one, as that piece is on the part tried
            let node = piece.saturating_sub(1) / 2;
            if node <= held {
                break;
            }
            let mut part = route.clone();
            part.cut(node);
            let mut shorter = graph.clone();
            shorter.lengthen(robot, node);
            match self.place(shorter, robot, &part) {
                Ok(graph) => return Ok((graph, part, blocked.by.into_iter().collect())),
                Err(again) => {
                    first = again.first;
                    blocked.by = again.by;
                }
            }
        }
        Err(blocked)
    }

    /// Order the passes of robot `new`, with this route, that have no order yet
    /// against those of the other robots. `graph` knows the route's length and how
    /// far the robot is; the result is that graph with the orders.
    fn place(&self, graph: OrderGraph, new: usize, route: &Route) -> Result<OrderGraph, Blocked> {
        // also when there is no other robot
        check(&self.spots, route)?;
        // the last piece the robot holds already
        let held = 2 * graph.released(new).unwrap_or(0);

        // the orders that put the new robot first, and the groups where it waits
        let mut ahead = Vec::new();
        let mut after: Vec<Vec<Before>> = Vec::new();
        // is there a group that can go neither way, and which pieces of the new
        // route are to blame
        let mut refused = false;
        let mut blocked = BTreeSet::new();
        let mut by = BTreeSet::new();
        for (other, theirs) in self.routes.iter().enumerate() {
            if other == new {
                continue;
            }
            let theirs_pass = |piece| Pass {
                robot: other,
                piece,
            };
            let new_pass = |piece| Pass { robot: new, piece };
            let found = groups_where(&self.spots, theirs, route, |q, p| {
                !graph.has_left(theirs_pass(q)) && !graph.has_left(new_pass(p))
            })?;
            for group in found {
                let they_first: Vec<Before> = group
                    .iter()
                    .map(|&(q, p)| Before {
                        first: theirs_pass(q),
                        then: new_pass(p),
                    })
                    .collect();
                let new_first = reversed(&they_first);
                // all its conflicts may have their order from before
                if they_first
                    .iter()
                    .zip(&new_first)
                    .all(|(a, b)| graph.is_ordered(a) || graph.is_ordered(b))
                {
                    continue;
                }
                if allowed(&graph, &they_first) {
                    after.push(they_first);
                } else if allowed(&graph, &new_first) {
                    ahead.extend(new_first);
                } else {
                    refused = true;
                    by.insert(other);
                    // The pieces that rule out "after": those the others end on.
                    // If there is none, the new robot stands in the way: all of
                    // the group. What it already holds cannot be avoided.
                    let ends: Vec<usize> = they_first
                        .iter()
                        .filter(|&&o| graph.allows(o).is_err() && o.then.piece > held)
                        .map(|o| o.then.piece)
                        .collect();
                    if ends.is_empty() {
                        blocked.extend(group.iter().map(|&(_, p)| p).filter(|&p| p > held));
                    } else {
                        blocked.extend(ends);
                    }
                }
            }
        }
        if refused {
            return Err(Blocked {
                first: blocked.first().copied(),
                why: reroute(route, blocked),
                by,
            });
        }
        let cannot_happen = || Blocked {
            why: reroute(route, []),
            first: None,
            by: BTreeSet::new(),
        };
        // Where waiting closes a cycle, the new robot has to go first, if it may.
        // That adds orders ahead of it, so all groups are tried again.
        loop {
            let mut trial = graph.clone();
            // cannot close a cycle: no order makes another robot's event still to
            // come lead to the new robot yet
            if trial.order(&ahead).is_err() {
                return Err(cannot_happen()); // all were allowed
            }
            let closing = after
                .iter()
                .enumerate()
                .find_map(|(i, group)| trial.order(group).err().map(|why| (i, why)));
            let Some((i, why)) = closing else {
                return Ok(trial);
            };
            let new_first = reversed(&after.remove(i));
            if !allowed(&graph, &new_first) {
                let Refused::Cycle(order) = why else {
                    return Err(cannot_happen()); // all were allowed
                };
                return Err(Blocked {
                    why: reroute(route, [order.then.piece]),
                    first: Some(order.then.piece),
                    by: BTreeSet::from([order.first.robot]),
                });
            }
            ahead.extend(new_first);
        }
    }
}

/// The graph with these orders turned round, if each turned one is allowed and
/// together they close no cycle.
fn turned(graph: &OrderGraph, orders: &[Before]) -> Result<OrderGraph, Refused> {
    let mut trial = graph.clone();
    trial.remove(orders);
    trial.order(&reversed(orders))?;
    Ok(trial)
}

/// Each order the other way round.
fn reversed(orders: &[Before]) -> Vec<Before> {
    orders
        .iter()
        .map(|o| Before {
            first: o.then,
            then: o.first,
        })
        .collect()
}

/// May each of these orders be taken, as far as its two passes go?
fn allowed(graph: &OrderGraph, orders: &[Before]) -> bool {
    orders.iter().all(|&o| graph.allows(o).is_ok())
}

/// The answer "re-route" with these pieces of the route, each once.
fn avoiding(route: &Route, pieces: impl IntoIterator<Item = usize>) -> Reroute {
    let mut seen = BTreeSet::new();
    let avoid = pieces
        .into_iter()
        .filter_map(|p| route.pieces().get(p))
        .filter(|&piece| seen.insert(piece))
        .cloned()
        .collect();
    Reroute { avoid }
}

/// A new route is not taken: keep clear of these pieces of it.
fn reroute(route: &Route, pieces: impl IntoIterator<Item = usize>) -> NotAdded {
    NotAdded::Reroute(avoiding(route, pieces))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{LARGE, SMALL, edge, empty_map, layout, node, route};
    use botsim_sweep::{ConflictMap, NodeConflict, NodeTypePair};

    /// The module over a layout that has what these routes use.
    fn traffic(routes: &[&[&str]], map: &ConflictMap) -> Traffic {
        Traffic::new(Spots::new(&layout(routes), map).expect("setup"))
    }

    /// Does robot `first` pass piece `p` before robot `then` passes piece `q`?
    fn before(t: &Traffic, first: usize, p: usize, then: usize, q: usize) -> bool {
        t.graph().is_ordered(&Before {
            first: Pass {
                robot: first,
                piece: p,
            },
            then: Pass {
                robot: then,
                piece: q,
            },
        })
    }

    /// The answer of [`Traffic::add`] to a route of what is then robot `robot`,
    /// where no other robot is released onto its first node.
    fn avoid(robot: usize, pieces: Vec<Piece>) -> Result<usize, NotAdded> {
        avoid_beside(robot, pieces, Vec::new())
    }

    /// [`avoid`] where these robots are.
    fn avoid_beside(
        robot: usize,
        pieces: Vec<Piece>,
        released_there: Vec<usize>,
    ) -> Result<usize, NotAdded> {
        let why = Box::new(NotAdded::Reroute(Reroute { avoid: pieces }));
        Err(NotAdded::Standing {
            robot,
            why,
            released_there,
        })
    }

    /// [`avoid`] for a change or a new order.
    fn avoid_new(pieces: Vec<Piece>) -> Result<(), NotAdded> {
        Err(NotAdded::Reroute(Reroute { avoid: pieces }))
    }

    #[test]
    fn a_new_route_goes_after() {
        let (a, b) = (["S", "X", "T"], ["U", "X", "V"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert!(before(&t, 0, 2, 1, 2));
    }

    #[test]
    fn a_route_through_another_robots_end_goes_first() {
        // the first robot's route ends at N, the second one drives through
        let (a, b) = (["S", "N"], ["T", "N", "U"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert!(before(&t, 1, 2, 0, 2));
        assert!(!before(&t, 0, 2, 1, 2));
    }

    #[test]
    fn following_into_the_end_goes_first_in_the_whole_group() {
        let (a, b) = (["S", "M", "N"], ["T", "M", "N", "U"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        // M, the edge M-N and N
        for p in [2, 3, 4] {
            assert!(before(&t, 1, p, 0, p), "piece {p}");
        }
    }

    #[test]
    fn reroute_when_the_ending_robot_is_released_there() {
        let (a, b, c) = (["S", "N"], ["T", "N", "U"], ["V", "W"]);
        let mut t = traffic(&[&a, &b, &c], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.progress(0, 0, 1), Ok(()));
        assert_eq!(t.add(route(&b)), avoid(1, vec![node("N")]));
        // b keeps its number
        assert_eq!(t.add(route(&c)), Ok(2));
    }

    #[test]
    fn reroute_when_released_into_the_group_before_the_end() {
        // released up to M, not into N: still no way for the other to go first
        let (a, b) = (["S", "M", "N"], ["T", "M", "N", "U"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.progress(0, 0, 1), Ok(()));
        assert_eq!(t.add(route(&b)), avoid(1, vec![node("N")]));
    }

    // c waits for a at X. b has to go first at N, the end of a. If it also waited
    // for c at Y, then a would wait for b, b for c and c for a.
    const A: [&str; 3] = ["S", "X", "N"];
    const C: [&str; 4] = ["V", "Y", "X", "W"];
    const B: [&str; 4] = ["T", "Y", "N", "U"];

    #[test]
    fn goes_first_where_waiting_would_close_a_cycle() {
        let mut t = traffic(&[&A, &B, &C], &empty_map());
        assert_eq!(t.add(route(&A)), Ok(0));
        // without c, b only goes first at N
        let mut alone = t.clone();
        assert_eq!(alone.add(route(&B)), Ok(1));
        assert_eq!(t.add(route(&C)), Ok(1));
        assert!(before(&t, 0, 2, 1, 4)); // a before c at X
        assert_eq!(t.add(route(&B)), Ok(2));
        assert!(before(&t, 2, 4, 0, 4)); // b before a at N
        assert!(before(&t, 2, 2, 1, 2)); // b before c at Y
    }

    #[test]
    fn reroute_when_it_may_not_go_first_there() {
        let mut t = traffic(&[&A, &B, &C], &empty_map());
        assert_eq!(t.add(route(&A)), Ok(0));
        assert_eq!(t.add(route(&C)), Ok(1));
        // c has Y: b cannot go before it
        assert_eq!(t.progress(1, 0, 1), Ok(()));
        assert_eq!(t.add(route(&B)), avoid(2, vec![node("Y")]));
    }

    #[test]
    fn a_route_cut_short_sends_the_waiting_robot_another_way() {
        // a drives S - N - Z - Y; b passes N after it, d passes Z after it
        let a = ["S", "N", "Z", "Y"];
        let b = ["T", "N", "U"];
        let d = ["V", "Z", "W"];
        let c = ["R", "N", "Q"];
        let mut t = traffic(&[&a, &b, &d, &c], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert_eq!(t.add(route(&d)), Ok(2));
        assert_eq!(t.progress(0, 0, 1), Ok(()));
        assert!(before(&t, 0, 2, 1, 2));
        assert!(before(&t, 0, 4, 2, 2));
        // b only waits: a will leave N
        assert_eq!(t.reroute_needed(1), None);

        // a's order is refused or cancelled with N released
        assert!(t.cut_short(0));
        assert_eq!(
            t.reroute_needed(1),
            Some(Reroute {
                avoid: vec![node("N")]
            })
        );
        assert!(before(&t, 0, 2, 1, 2), "b still waits");
        // a no longer goes to Z: d is free of it
        assert!(!before(&t, 0, 4, 2, 2));
        assert_eq!(t.reroute_needed(2), None);
        assert_eq!(t.reroute_needed(0), None);
        assert!(t.graph().orders().iter().all(|o| {
            (o.first.robot != 0 || o.first.piece <= 2) && (o.then.robot != 0 || o.then.piece <= 2)
        }));
        // and a route that comes later cannot pass N either
        assert_eq!(t.add(route(&c)), avoid(3, vec![node("N")]));
    }

    #[test]
    fn a_confirmed_cancel_frees_what_lies_beyond_the_stop() {
        // a drives S - M - N with N released; b passes M after it, d passes N
        let a = ["S", "M", "N", "Z"];
        let b = ["T", "M", "U"];
        let d = ["V", "N", "W"];
        let through_n = ["R", "N", "Q"];
        let through_m = ["P", "M", "O"];
        let mut t = traffic(&[&a, &b, &d, &through_n, &through_m], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert_eq!(t.add(route(&d)), Ok(2));
        assert_eq!(t.progress(0, 0, 2), Ok(()));
        assert!(t.cut_short(0));
        assert_eq!(t.progress(0, 1, 2), Ok(()));
        let told = |name| {
            Some(Reroute {
                avoid: vec![node(name)],
            })
        };

        // as long as one sign is missing, everything up to N stays held
        let confirmed = Cancel {
            finished: true,
            nothing_left: true,
            standing_on: Some(1),
        };
        for report in [
            Cancel {
                finished: false,
                ..confirmed
            },
            Cancel {
                nothing_left: false,
                ..confirmed
            },
            Cancel {
                standing_on: None,
                ..confirmed
            },
        ] {
            assert_eq!(t.cancelled(0, report), Ok(false), "{report:?}");
            assert_eq!(t.graph().released(0), Some(2));
            assert_eq!(t.reroute_needed(1), None);
            assert_eq!(t.reroute_needed(2), told("N"));
            assert_eq!(t.clone().add(route(&through_n)), avoid(3, vec![node("N")]));
        }

        // confirmed: a stands on M and will not go to N
        assert_eq!(t.cancelled(0, confirmed), Ok(true));
        assert_eq!(t.graph().released(0), Some(1));
        assert_eq!(t.reroute_needed(1), told("M"));
        assert_eq!(t.reroute_needed(2), None);
        assert!(!before(&t, 0, 4, 2, 2), "d no longer waits at N");
        assert_eq!(t.add(route(&through_n)), Ok(3));
        assert_eq!(t.add(route(&through_m)), avoid(4, vec![node("M")]));
    }

    #[test]
    fn a_cancel_cannot_stop_where_the_robot_is_not() {
        let a = ["S", "M", "N", "Z"];
        let mut t = traffic(&[&a], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.progress(0, 1, 2), Ok(()));
        let on = |node| Cancel {
            finished: true,
            nothing_left: true,
            standing_on: Some(node),
        };
        // behind the node last reached, beyond the one last released
        for node in [0, 3] {
            assert_eq!(t.cancelled(0, on(node)).map_err(|e| e.reached), Err(node));
        }
        assert_eq!(t.graph().released(0), Some(2));
        assert!(t.cancelled(7, on(0)).is_err());
        let unconfirmed = Cancel {
            finished: false,
            ..on(0)
        };
        assert!(t.cancelled(7, unconfirmed).is_err());
        // without a cut_short before: the confirmation alone ends the route
        assert_eq!(t.cancelled(0, on(2)), Ok(true));
        assert_eq!(t.graph().released(0), Some(2));
    }

    #[test]
    fn a_change_that_cannot_be_placed_holds_only_what_is_released() {
        // a drives S - M - N - Z with M released; b passes M after it, d passes N.
        // a is then sent to X, where e stands and stays.
        let a = ["S", "M", "N", "Z"];
        let b = ["T", "M", "U"];
        let d = ["V", "N", "W"];
        let e = ["Q", "X"];
        let to_x = ["S", "M", "X"];
        let to_r = ["S", "M", "R"];
        let through_m = ["P", "M", "O"];
        let mut t = traffic(&[&a, &b, &d, &e, &to_x, &to_r, &through_m], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert_eq!(t.add(route(&d)), Ok(2));
        assert_eq!(t.add(route(&e)), Ok(3));
        assert_eq!(t.progress(0, 0, 1), Ok(()));
        assert_eq!(t.progress(3, 0, 1), Ok(()));
        assert!(before(&t, 0, 4, 2, 2));

        assert_eq!(
            t.change(0, route(&to_x)),
            Err(NotAdded::Reroute(Reroute {
                avoid: vec![node("X")]
            }))
        );
        let told_m = Some(Reroute {
            avoid: vec![node("M")],
        });
        assert_eq!(t.graph().released(0), Some(1));
        // b, ordered after a at M, is told to re-route; d is free of a
        assert_eq!(t.reroute_needed(1), told_m);
        assert!(!before(&t, 0, 4, 2, 2));
        assert_eq!(t.reroute_needed(2), None);
        // a still holds M, and no new route may pass it
        assert_eq!(t.clone().add(route(&through_m)), avoid(4, vec![node("M")]));

        // a route that does not go on from what is released is not taken either
        assert_eq!(t.change(0, route(&["S", "N"])), Err(NotAdded::OtherRoute));
        assert_eq!(t.reroute_needed(1), told_m);

        // one that can be placed: b waits for a as before, and no longer for ever
        assert_eq!(t.change(0, route(&to_r)), Ok(()));
        assert_eq!(t.reroute_needed(1), None);
        assert!(before(&t, 0, 2, 1, 2));
        assert_eq!(t.progress(0, 2, 2), Ok(()));
        assert_eq!(t.add(route(&through_m)), Ok(4));
    }

    #[test]
    fn a_change_that_is_not_a_route_of_the_robot() {
        let a = ["S", "M", "N"];
        let mut t = traffic(&[&a], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.progress(0, 0, 1), Ok(()));
        assert_eq!(t.change(7, route(&a)), Err(NotAdded::UnknownRobot(7)));
        // an unknown node after what is released: only the released part is left
        assert_eq!(
            t.change(0, route(&["S", "M", "Z"])),
            Err(NotAdded::Unknown(UnknownName::Piece(edge("M-Z"))))
        );
        assert_eq!(t.progress(0, 0, 2).map_err(|e| e.released), Err(2));
        assert_eq!(t.change(0, route(&a)), Ok(()));
        assert_eq!(t.progress(0, 0, 2), Ok(()));
    }

    #[test]
    fn a_new_order_after_a_finished_route() {
        let a = ["S", "N"];
        let next = ["N", "W"];
        let b = ["T", "N", "U"];
        let through_s = ["R", "S", "Q"];
        let mut t = traffic(&[&a, &next, &b, &through_s], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        // not before it stands on the end
        assert_eq!(t.new_order(0, route(&next)), Err(NotAdded::NotAtEnd));
        assert_eq!(t.progress(0, 0, 1), Ok(()));
        assert_eq!(t.new_order(0, route(&next)), Err(NotAdded::NotAtEnd));
        assert_eq!(t.progress(0, 1, 1), Ok(()));

        assert_eq!(t.new_order(0, route(&next)), Ok(()));
        // numbered from 0 again: it holds N, nothing more
        assert_eq!(t.graph().released(0), Some(0));
        assert_eq!(t.progress(0, 0, 1), Ok(()));
        // a route through N waits for it; S, which it has left, is free
        assert_eq!(t.add(route(&b)), Ok(1));
        assert!(before(&t, 0, 0, 1, 2));
        assert_eq!(t.add(route(&through_s)), Ok(2));
        assert!(t.graph().orders().iter().all(|o| o.then.robot != 2));
    }

    #[test]
    fn a_new_order_after_a_cancel_waits_for_the_confirmation() {
        // a is cancelled on S with M released; b passes M after it
        let a = ["S", "M", "N"];
        let next = ["S", "W"];
        let b = ["T", "M", "U"];
        let mut t = traffic(&[&a, &next, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.progress(0, 0, 1), Ok(()));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert!(t.cut_short(0));
        assert_eq!(t.new_order(0, route(&next)), Err(NotAdded::NotAtEnd));
        assert!(t.reroute_needed(1).is_some());

        let confirmed = Cancel {
            finished: true,
            nothing_left: true,
            standing_on: Some(0),
        };
        assert_eq!(t.cancelled(0, confirmed), Ok(true));
        assert_eq!(t.new_order(0, route(&next)), Ok(()));
        // M was given back with the cancel: b is free of a
        assert_eq!(t.reroute_needed(1), None);
        assert!(t.graph().orders().is_empty());
    }

    #[test]
    fn a_new_order_takes_over_the_robots_waiting_for_its_node() {
        // a is cut short on N; b waits to pass N and holds T
        let a = ["S", "N", "Z"];
        let b = ["T", "N", "U"];
        let to_t = ["N", "T"];
        let away = ["N", "W"];
        let mut t = traffic(&[&a, &b, &to_t, &away], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert_eq!(t.progress(0, 0, 1), Ok(()));
        assert!(t.cut_short(0));
        assert_eq!(t.progress(0, 1, 1), Ok(()));
        let told = Some(Reroute {
            avoid: vec![node("N")],
        });
        assert_eq!(t.reroute_needed(1), told);

        // towards b, which cannot pass first: not taken, and nothing changes
        assert_eq!(t.new_order(0, route(&to_t)), avoid_new(vec![node("T")]));
        assert_eq!(t.graph().released(0), Some(1));
        assert_eq!(t.reroute_needed(1), told);

        assert_eq!(t.new_order(0, route(&away)), Ok(()));
        assert_eq!(t.reroute_needed(1), None);
        assert!(before(&t, 0, 0, 1, 2), "b waits for a to leave N");
    }

    #[test]
    fn a_new_order_that_is_not_one_for_the_robot() {
        let a = ["S", "N"];
        let mut t = traffic(&[&a, &["N", "W"]], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.progress(0, 1, 1), Ok(()));
        assert_eq!(t.new_order(7, route(&a)), Err(NotAdded::UnknownRobot(7)));
        // does not start where it stands
        assert_eq!(t.new_order(0, route(&a)), Err(NotAdded::OtherRoute));
        // another robot type
        let large = Route::new(LARGE, &["N", "W"], &["N-W"]).expect("route");
        assert_eq!(t.new_order(0, large), Err(NotAdded::OtherRoute));
        assert_eq!(
            t.new_order(0, route(&["N", "Z"])),
            Err(NotAdded::Unknown(UnknownName::Piece(edge("N-Z"))))
        );
        assert_eq!(t.graph().released(0), Some(1));
        assert_eq!(t.new_order(0, route(&["N", "W"])), Ok(()));
    }

    fn report(robot: usize, reached: usize, released: usize) -> Report {
        Report {
            robot,
            reached,
            released,
        }
    }

    #[test]
    fn a_cycle_start_removes_what_has_been_passed() {
        // b passes X after a
        let (a, b) = (["S", "X", "T"], ["U", "X", "V"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        // on X: not left yet
        assert_eq!(t.cycle_start(&[report(0, 1, 1), report(1, 0, 0)]), []);
        assert!(before(&t, 0, 2, 1, 2));
        assert_eq!(t.cycle_start(&[report(0, 2, 2)]), []);
        assert!(t.graph().orders().is_empty());
        // what is passed takes no part in a later order either
        assert_eq!(t.change(1, route(&b)), Ok(()));
        assert!(t.graph().orders().is_empty());
    }

    #[test]
    fn a_bad_report_does_not_stop_the_others() {
        let (a, b) = (["S", "X", "T"], ["U", "X", "V"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        // b further than released, and a robot there is none of, around a's report
        let bad = |robot, reached, released| BadProgress {
            robot,
            reached,
            released,
        };
        assert_eq!(
            t.cycle_start(&[report(1, 2, 1), report(0, 2, 2), report(7, 0, 0)]),
            [bad(1, 2, 1), bad(7, 0, 0)]
        );
        assert_eq!(t.graph().released(0), Some(2));
        assert_eq!(t.graph().released(1), Some(0));
        assert!(t.graph().orders().is_empty());
    }

    #[test]
    fn a_cycle_start_keeps_what_waits_for_ever() {
        // a is cut short on N, b waits to pass N
        let (a, b) = (["S", "N", "Z"], ["T", "N", "U"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert_eq!(t.progress(0, 0, 1), Ok(()));
        assert!(t.cut_short(0));
        assert_eq!(t.cycle_start(&[report(0, 1, 1)]), []);
        assert!(before(&t, 0, 2, 1, 2));
        assert!(t.reroute_needed(1).is_some());
    }

    fn pass(robot: usize, piece: usize) -> Pass {
        Pass { robot, piece }
    }

    /// Two robots head on through X - M - Y; the first one added goes first.
    fn head_on() -> Traffic {
        let a = ["W", "X", "M", "Y", "Z"];
        let b = ["U", "Y", "M", "X", "V"];
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        t
    }

    // the conflicts of `head_on`: X, M and Y, as (piece of a, piece of b)
    const AT_X: (usize, usize) = (2, 6);
    const AT_M: (usize, usize) = (4, 4);
    const AT_Y: (usize, usize) = (6, 2);

    #[test]
    fn reverse_turns_the_whole_group_round() {
        let mut t = head_on();
        let first = t.graph().orders();
        for (p, q) in [AT_X, AT_M, AT_Y] {
            assert!(before(&t, 0, p, 1, q));
        }
        // named by the conflict in the middle, in either order of the two passes
        assert_eq!(t.reverse(pass(0, 4), pass(1, 4)), Ok(()));
        for (p, q) in [AT_X, AT_M, AT_Y] {
            assert!(before(&t, 1, q, 0, p), "piece {p}");
            assert!(!before(&t, 0, p, 1, q), "piece {p}");
        }
        assert_eq!(t.reverse(pass(1, AT_Y.1), pass(0, AT_Y.0)), Ok(()));
        assert_eq!(t.graph().orders(), first);
    }

    #[test]
    fn reverse_is_refused_once_the_robot_that_would_wait_is_released() {
        let mut t = head_on();
        // a is given X, the first conflict of the group
        assert_eq!(t.progress(0, 0, 1), Ok(()));
        let orders = t.graph().orders();
        // refused for the whole group, also when named by a conflict further on
        for (p, q) in [AT_X, AT_Y] {
            assert_eq!(
                t.reverse(pass(0, p), pass(1, q)),
                Err(Refused::Released(pass(0, AT_X.0)))
            );
            assert_eq!(t.graph().orders(), orders);
        }
    }

    #[test]
    fn reverse_is_refused_where_the_robot_going_first_would_stay() {
        // b ends on N, so it passes after a
        let (a, b) = (["S", "N", "Z"], ["T", "N"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        let orders = t.graph().orders();
        assert_eq!(
            t.reverse(pass(0, 2), pass(1, 2)),
            Err(Refused::NeverLeft(pass(1, 2)))
        );
        assert_eq!(t.graph().orders(), orders);
    }

    #[test]
    fn reverse_is_refused_when_it_closes_a_cycle() {
        // three robots, each over two of the crossings P, Q, R of a ring
        let routes = [
            ["S0", "P", "Q", "T0"],
            ["S1", "Q", "R", "T1"],
            ["S2", "R", "P", "T2"],
        ];
        let mut t = traffic(&[&routes[0], &routes[1], &routes[2]], &empty_map());
        for (i, r) in routes.iter().enumerate() {
            assert_eq!(t.add(route(r)), Ok(i));
        }
        // at Q 0 before 1, at R 1 before 2, at P 0 before 2
        let orders = t.graph().orders();
        let turned = t.reverse(pass(0, 2), pass(2, 4));
        assert!(matches!(turned, Err(Refused::Cycle(_))), "{turned:?}");
        assert_eq!(t.graph().orders(), orders);
        // Q can be turned round
        assert_eq!(t.reverse(pass(0, 4), pass(1, 2)), Ok(()));
        assert!(before(&t, 1, 2, 0, 4));
    }

    #[test]
    fn reverse_leaves_other_groups_alone() {
        // a and b cross at P and again at Q
        let a = ["A0", "P", "A1", "Q", "A2"];
        let b = ["B0", "P", "B1", "Q", "B2"];
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert_eq!(t.reverse(pass(0, 2), pass(1, 2)), Ok(()));
        assert!(before(&t, 1, 2, 0, 2));
        assert!(before(&t, 0, 6, 1, 6));
    }

    #[test]
    fn reverse_needs_an_order_between_the_two_passes() {
        let mut t = head_on();
        let no_order = |a, b| Err(Refused::NoOrder(a, b));
        // no conflict between these pieces
        let (a, b) = (pass(0, 0), pass(1, 0));
        assert_eq!(t.reverse(a, b), no_order(a, b));
        assert_eq!(
            t.reverse(pass(0, 99), b),
            Err(Refused::UnknownPass(pass(0, 99)))
        );
        assert_eq!(
            t.reverse(a, pass(7, 0)),
            Err(Refused::UnknownPass(pass(7, 0)))
        );
        let own = Before {
            first: a,
            then: pass(0, 2),
        };
        assert_eq!(t.reverse(own.first, own.then), Err(Refused::SameRobot(own)));
        // a conflict that is over: a has driven past all three
        assert_eq!(t.progress(0, 4, 4), Ok(()));
        let (a, b) = (pass(0, AT_M.0), pass(1, AT_M.1));
        assert_eq!(t.reverse(a, b), no_order(a, b));
    }

    #[test]
    fn the_first_to_come_goes_first() {
        let mut t = head_on();
        let b_node = |t: &Traffic| t.graph().released(1);
        // b asks for Y first, though a was added first
        assert!(t.give_next(1));
        assert_eq!(b_node(&t), Some(1));
        for (p, q) in [AT_X, AT_M, AT_Y] {
            assert!(before(&t, 1, q, 0, p), "piece {p}");
        }
        // a can no longer have it the other way round: it waits at W
        let orders = t.graph().orders();
        assert!(!t.give_next(0));
        assert_eq!(t.graph().released(0), Some(0));
        assert_eq!(t.graph().orders(), orders);
        // b drives through; a goes on once b is past X
        for node in 2..=4 {
            assert!(t.give_next(1));
            assert_eq!(t.progress(1, node - 1, node), Ok(()));
            assert!(!t.give_next(0), "b at node {}", node - 1);
        }
        assert_eq!(t.progress(1, 4, 4), Ok(()));
        assert!(t.give_next(0));
        assert_eq!(t.graph().released(0), Some(1));
    }

    #[test]
    fn a_robot_that_is_not_held_back_changes_no_order() {
        let mut t = head_on();
        let orders = t.graph().orders();
        assert!(t.give_next(0));
        assert_eq!(t.graph().orders(), orders);
        // and b, which asks later, waits
        assert!(!t.give_next(1));
        assert_eq!(t.graph().orders(), orders);
    }

    #[test]
    fn nothing_is_given_past_the_end_of_a_route() {
        let a = ["S", "N"];
        let mut t = traffic(&[&a], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert!(t.give_next(0));
        assert!(!t.give_next(0));
        assert_eq!(t.graph().released(0), Some(1));
        assert!(!t.give_next(7));
    }

    /// a, b and c cross at X and are ordered as added.
    fn three_at_one_node() -> Traffic {
        let routes = [["A0", "X", "A1"], ["B0", "X", "B1"], ["C0", "X", "C1"]];
        let mut t = traffic(&[&routes[0], &routes[1], &routes[2]], &empty_map());
        for (i, r) in routes.iter().enumerate() {
            assert_eq!(t.add(route(r)), Ok(i));
        }
        t
    }

    #[test]
    fn all_groups_that_hold_a_robot_back_are_turned_together() {
        let mut t = three_at_one_node();
        assert!(t.give_next(2));
        assert!(before(&t, 2, 2, 0, 2));
        assert!(before(&t, 2, 2, 1, 2));
        assert!(before(&t, 0, 2, 1, 2), "a and b keep their order");
    }

    #[test]
    fn no_group_is_turned_unless_the_robot_gets_the_node() {
        let mut t = three_at_one_node();
        // b gets X before a
        assert!(t.give_next(1));
        let orders = t.graph().orders();
        // c could pass before a, but not before b: nothing changes
        assert!(!t.give_next(2));
        assert_eq!(t.graph().orders(), orders);
        assert_eq!(t.graph().released(2), Some(0));
    }

    #[test]
    fn a_robot_waits_where_turning_would_close_a_cycle() {
        let routes = [
            ["S0", "P", "Q", "T0"],
            ["S1", "Q", "R", "T1"],
            ["S2", "R", "P", "T2"],
        ];
        let mut t = traffic(&[&routes[0], &routes[1], &routes[2]], &empty_map());
        for (i, r) in routes.iter().enumerate() {
            assert_eq!(t.add(route(r)), Ok(i));
        }
        // 1 before 0 at Q; still 1 before 2 at R and 0 before 2 at P
        assert_eq!(t.reverse(pass(0, 4), pass(1, 2)), Ok(()));
        let orders = t.graph().orders();
        // 2 before 1 at R would have each wait for the next
        assert!(!t.give_next(2));
        assert_eq!(t.graph().orders(), orders);
    }

    #[test]
    fn a_robot_cannot_come_first_where_it_ends_its_route() {
        // b ends on N, so it passes after a
        let (a, b) = (["S", "N", "Z"], ["T", "N"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert!(!t.give_next(1));
        // a drives over N
        assert!(t.give_next(0));
        assert!(t.give_next(0));
        assert_eq!(t.progress(0, 2, 2), Ok(()));
        assert!(t.give_next(1));
    }

    fn released(through: usize, avoid: Option<Vec<Piece>>) -> Option<Release> {
        Some(Release {
            through,
            reroute: avoid.map(|avoid| Reroute { avoid }),
        })
    }

    #[test]
    fn a_release_turns_orders_node_by_node_up_to_the_limit() {
        // a and b cross at P and again at Q; a goes first at both
        let a = ["A0", "P", "A1", "Q", "A2"];
        let b = ["B0", "P", "B1", "Q", "B2"];
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        // b asks as far as Q: it comes first at both crossings
        assert_eq!(t.release(1, 3), released(3, None));
        assert!(before(&t, 1, 2, 0, 2));
        assert!(before(&t, 1, 6, 0, 6));
        // a limit already met gives nothing
        assert_eq!(t.release(1, 1), released(3, None));
        // not past the end of the route
        assert_eq!(t.release(1, 99), released(4, None));
        assert_eq!(t.release(7, 1), None);
    }

    #[test]
    fn a_release_stops_where_the_robot_has_to_wait() {
        let b = ["B0", "P", "B1", "Q", "B2"];
        let c = ["C0", "Q", "C1"];
        let mut t = traffic(&[&b, &c], &empty_map());
        assert_eq!(t.add(route(&b)), Ok(0));
        assert_eq!(t.add(route(&c)), Ok(1));
        // c is given Q first
        assert_eq!(t.release(1, 1), released(1, None));
        assert_eq!(t.release(0, 4), released(2, None));
    }

    #[test]
    fn a_release_says_when_the_robot_waits_for_ever() {
        let (a, b) = (["S", "N", "Z"], ["T", "M", "N", "U"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert_eq!(t.release(0, 1), released(1, None));
        assert!(t.cut_short(0));
        // b gets the way up to N
        assert_eq!(t.release(1, 3), released(1, Some(vec![node("N")])));
    }

    #[test]
    fn a_release_repeats_a_refused_change_until_a_route_is_taken() {
        // a is sent to X, where e stands and stays
        let a = ["S", "M", "N"];
        let e = ["Q", "X"];
        let to_x = ["S", "M", "X"];
        let to_r = ["S", "M", "R"];
        let mut t = traffic(&[&a, &e, &to_x, &to_r], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&e)), Ok(1));
        assert_eq!(t.release(0, 1), released(1, None));
        assert_eq!(t.release(1, 1), released(1, None));
        assert!(t.change(0, route(&to_x)).is_err());
        let told = released(1, Some(vec![node("X")]));
        assert_eq!(t.release(0, 2), told);
        // a change that is no route of the robot leaves the answer as it was
        assert_eq!(t.change(0, route(&["S", "N"])), Err(NotAdded::OtherRoute));
        assert_eq!(t.release(0, 2), told);

        assert_eq!(t.change(0, route(&to_r)), Ok(()));
        assert_eq!(t.release(0, 2), released(2, None));
    }

    #[test]
    fn a_zone_is_granted_to_the_robot_released_into_it() {
        // two lanes through one zone, from node 1 to node 2
        let a = ["A0", "A1", "A2", "A3"];
        let b = ["B0", "B1", "B2", "B3"];
        let inside = [
            node("A1"),
            edge("A1-A2"),
            node("A2"),
            node("B1"),
            edge("B1-B2"),
            node("B2"),
        ];
        let mut spots = Spots::new(&layout(&[&a, &b]), &empty_map()).expect("setup");
        let inside = BTreeMap::from([(SMALL.to_string(), inside.to_vec())]);
        assert_eq!(spots.add_zone("S", "Z", &inside), Ok(()));
        let mut t = Traffic::new(spots);
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        let ask = |t: &Traffic, robot| t.zone_request(robot, "S", "Z");
        let (granted, queued) = (Some(ZoneAnswer::Granted), Some(ZoneAnswer::Queued));

        // asking first gives nothing
        let orders = t.graph().orders();
        assert_eq!(ask(&t, 1), queued);
        assert_eq!(t.graph().orders(), orders);
        // b is released into the zone first, so a has to wait
        assert_eq!(t.release(1, 1), released(1, None));
        assert_eq!(t.release(0, 1), released(0, None));
        assert_eq!((ask(&t, 0), ask(&t, 1)), (queued, granted));
        // granted as long as b is inside
        assert_eq!(t.release(1, 3), released(3, None));
        assert_eq!(t.progress(1, 2, 3), Ok(()));
        assert_eq!((ask(&t, 0), ask(&t, 1)), (queued, granted));
        // b is out: nothing of the zone is ahead of it, and a gets in
        assert_eq!(t.progress(1, 3, 3), Ok(()));
        assert_eq!(ask(&t, 1), queued);
        assert_eq!(t.release(0, 1), released(1, None));
        assert_eq!(ask(&t, 0), granted);

        assert_eq!(ask(&t, 7), None);
        assert_eq!(t.zone_request(0, "S", "Y"), None);
        assert_eq!(t.zone_request(0, "R", "Z"), None);
    }

    #[test]
    fn a_piece_is_held_from_its_release_until_it_is_left() {
        let (a, b) = (["S", "M", "N"], ["T", "U"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        let holders = |t: &Traffic| {
            [
                node("S"),
                edge("S-M"),
                node("M"),
                edge("M-N"),
                node("N"),
                node("T"),
            ]
            .map(|piece| t.holder(&piece))
        };
        let (a, b) = (Some(0), Some(1));
        // each stands on its first node
        assert_eq!(holders(&t), [a, None, None, None, None, b]);
        assert_eq!(t.release(0, 2), released(2, None));
        assert_eq!(holders(&t), [a, a, a, a, a, b]);
        // on M: what is behind is free
        assert_eq!(t.progress(0, 1, 2), Ok(()));
        assert_eq!(holders(&t), [None, None, a, a, a, b]);
        // a confirmed cancel frees what lies beyond the stop
        let stopped = Cancel {
            finished: true,
            nothing_left: true,
            standing_on: Some(1),
        };
        assert_eq!(t.cancelled(0, stopped), Ok(true));
        assert_eq!(holders(&t), [None, None, a, None, None, b]);
        assert_eq!(t.holder(&node("Z")), None);
    }

    const NOBODY: [usize; 0] = [];

    #[test]
    fn the_robots_that_hold_one_back_are_named() {
        let mut t = three_at_one_node();
        // X is next for each: b waits for a, c for both
        assert_eq!(t.blockers(0), NOBODY);
        assert_eq!(t.blockers(1), [0]);
        assert_eq!(t.blockers(2), [0, 1]);
        let orders = t.graph().orders();
        assert_eq!(t.blockers(2), [0, 1]);
        assert_eq!(t.graph().orders(), orders);
        // a drives through and leaves X
        assert_eq!(t.release(0, 2), released(2, None));
        assert_eq!(t.progress(0, 2, 2), Ok(()));
        assert_eq!(t.blockers(1), NOBODY);
        assert_eq!(t.blockers(2), [1]);
        // nothing further, and no such robot
        assert_eq!(t.blockers(0), NOBODY);
        assert_eq!(t.blockers(7), NOBODY);
    }

    #[test]
    fn a_robot_standing_for_good_is_named_as_a_blocker() {
        let (a, b) = (["S", "X", "T"], ["X"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), avoid(1, Vec::new()));
        assert_eq!(t.blockers(0), [1]);
        assert_eq!(t.blockers(1), NOBODY);
    }

    #[test]
    fn waiting_behind_a_standing_robot_lets_two_pass_each_others_places() {
        // a stands on P and passes where b stands, b stands on R and passes P
        let (a, b) = (["P", "Q", "R"], ["R", "S", "P"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&["P"])), Ok(0));
        assert_eq!(t.add(route(&["R"])), Ok(1));
        // as designed: refused
        assert_eq!(
            t.clone().new_order(0, route(&a)),
            avoid_new(vec![node("R")])
        );

        t.wait_behind_standing();
        assert_eq!(t.new_order(0, route(&a)), Ok(()));
        // a gets as far as Q and, until b has a route, is told to re-route
        assert_eq!(t.release(0, 2), released(1, Some(vec![node("R")])));
        assert_eq!(t.new_order(1, route(&b)), Ok(()));
        assert_eq!(t.release(0, 2), released(1, None));
        // b leaves R, a leaves P, and both arrive
        assert_eq!(t.release(1, 1), released(1, None));
        assert_eq!(t.progress(1, 1, 1), Ok(()));
        assert_eq!(t.progress(0, 1, 1), Ok(()));
        assert_eq!(t.release(0, 2), released(2, None));
        assert_eq!(t.release(1, 2), released(2, None));
    }

    #[test]
    fn waiting_behind_a_standing_robot_still_refuses_a_head_on_swap() {
        let (a, b) = (["P", "Q", "R"], ["R", "Q", "P"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        t.wait_behind_standing();
        assert_eq!(t.add(route(&["P"])), Ok(0));
        assert_eq!(t.add(route(&["R"])), Ok(1));
        assert_eq!(t.new_order(0, route(&a)), Ok(()));
        assert!(matches!(
            t.new_order(1, route(&b)),
            Err(NotAdded::Reroute(_))
        ));
        // a robot that only drives towards the end of its route is not "standing"
        let (c, d) = (["V", "W"], ["U", "W", "X"]);
        let mut t = traffic(&[&c, &d], &empty_map());
        t.wait_behind_standing();
        assert_eq!(t.add(route(&c)), Ok(0));
        assert_eq!(t.progress(0, 0, 1), Ok(()));
        assert_eq!(t.add(route(&d)), avoid(1, vec![node("W")]));
    }

    #[test]
    fn a_route_taken_partly_goes_further_once_it_can() {
        // both end on N; c shows that the way to M is free
        let (a, b) = (["S", "N"], ["T", "M", "N"]);
        let mut t = traffic(&[&a, &b, &["N", "W"]], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        // as designed: refused, and a is to blame
        let mut whole = t.clone();
        assert_eq!(whole.add(route(&b)), avoid(1, vec![node("N")]));
        assert_eq!(whole.refused_by(1), [0]);

        t.take_routes_partly();
        assert_eq!(t.add(route(&b)), Ok(1));
        assert_eq!(t.route_end(1), Some(1));
        assert_eq!(t.refused_by(1), [0]);
        assert_eq!(t.release(1, 2), released(1, None));
        // no further while a ends on N, and nothing changes
        let orders = t.graph().orders();
        assert!(!t.extend(1, route(&b)));
        assert_eq!(t.graph().orders(), orders);
        assert_eq!((t.route_end(1), t.graph().released(1)), (Some(1), Some(1)));
        assert_eq!(t.reroute_needed(1), None);
        assert_eq!(t.refused_by(1), [0]);

        // a gets a route that leaves N
        assert_eq!(t.release(0, 1), released(1, None));
        assert_eq!(t.progress(0, 1, 1), Ok(()));
        assert_eq!(t.new_order(0, route(&["N", "W"])), Ok(()));
        assert!(t.extend(1, route(&b)));
        assert_eq!(t.route_end(1), Some(2));
        assert_eq!(t.refused_by(1), NOBODY);
        assert!(before(&t, 0, 0, 1, 4));
    }

    #[test]
    fn nothing_is_taken_partly_where_the_first_step_is_barred() {
        // b stands on N, which is a's second node
        let (a, b) = (["S", "N", "Z"], ["N"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        t.take_routes_partly();
        assert_eq!(t.add(route(&b)), Ok(0));
        assert_eq!(t.add(route(&a)), avoid(1, vec![node("N")]));
        assert_eq!(t.refused_by(1), [0]);
        assert_eq!(
            t.new_order(1, route(&["S", "N"])),
            avoid_new(vec![node("N")])
        );
        assert_eq!(t.refused_by(1), [0]);
    }

    #[test]
    fn cutting_short_where_nobody_waits() {
        let a = ["S", "N", "Z"];
        let mut t = traffic(&[&a], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        // nothing released: the route is the node the robot stands on
        assert!(t.cut_short(0));
        assert_eq!(t.progress(0, 0, 1).map_err(|e| e.released), Err(1));
        assert_eq!(t.reroute_needed(0), None);
        assert!(!t.cut_short(7));
        assert_eq!(t.reroute_needed(7), None);
    }

    #[test]
    fn two_routes_ending_on_the_same_node() {
        let (a, b) = (["S", "N"], ["T", "N"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), avoid(1, vec![node("N")]));
    }

    #[test]
    fn two_routes_ending_on_conflicting_nodes() {
        // N and P are too close: standing on one is in the way of driving to the other
        let (a, b) = (["V", "N"], ["W", "P"]);
        let near = |node: &str, edge: &str| NodeConflict {
            node: node.into(),
            edge: edge.into(),
            junction_nodes: None,
            depth: 0.2,
            distance: 0.0,
        };
        let mut map = empty_map();
        map.node_pairs.push(NodeTypePair {
            type_node: SMALL.into(),
            type_edge: SMALL.into(),
            conflicts: vec![near("N", "W-P"), near("P", "V-N")],
        });
        let mut t = traffic(&[&a, &b], &map);
        assert_eq!(t.add(route(&a)), Ok(0));
        // the way into P is what meets the robot standing on N
        assert_eq!(t.add(route(&b)), avoid(1, vec![edge("W-P")]));
    }

    #[test]
    fn a_robot_standing_on_the_way_goes_first_if_it_gets_away() {
        let (a, b) = (["S", "M", "Y"], ["M", "Y", "Q"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert!(before(&t, 1, 0, 0, 2));
    }

    #[test]
    fn a_robot_standing_on_the_way_cannot_come_towards_the_other() {
        let (a, b) = (["S", "M", "Y"], ["M", "S", "Q"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), avoid(1, vec![node("S")]));
    }

    #[test]
    fn two_robots_standing_on_one_node_are_refused() {
        let a = ["N"];
        let mut t = traffic(&[&a], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        // nothing a route could avoid
        assert_eq!(t.add(route(&a)), avoid_beside(1, Vec::new(), vec![0]));
        // the first one has the node
        assert_eq!(t.holder(&node("N")), Some(0));
    }

    #[test]
    fn a_refused_robot_stays_and_blocks_its_node() {
        // b appears on X, which a is to pass
        let (a, b, away) = (["S", "X", "T"], ["X"], ["X", "U"]);
        let mut t = traffic(&[&a, &b, &away], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&b)), avoid(1, Vec::new()));
        assert_eq!(t.holder(&node("X")), Some(1));
        // a is not released into X, and is told to take another way
        assert_eq!(t.release(0, 2), released(0, Some(vec![node("X")])));
        assert_eq!(t.holder(&node("X")), Some(1));

        // b is told to re-route until a route of it is taken
        let told = released(0, Some(Vec::new()));
        assert_eq!(t.release(1, 1), told);
        assert_eq!(t.change(1, route(&["X", "Z"])).map_err(|_| ()), Err(()));
        assert_eq!(t.release(1, 1), told);

        // with a new route b gets away, and a follows once it has left X
        assert_eq!(t.change(1, route(&away)), Ok(()));
        assert_eq!(t.release(0, 2), released(0, None));
        assert_eq!(t.release(1, 1), released(1, None));
        assert_eq!(t.progress(1, 1, 1), Ok(()));
        assert_eq!(t.release(0, 2), released(2, None));
    }

    #[test]
    fn a_robot_released_there_is_named_and_cannot_be_held_back() {
        // a has X already when b appears on it; c has not
        let (a, b, c) = (["S", "X", "T"], ["X"], ["U", "X", "V"]);
        let mut t = traffic(&[&a, &c, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.add(route(&c)), Ok(1));
        assert_eq!(t.release(0, 1), released(1, None));
        let named = avoid_beside(2, Vec::new(), vec![0]);
        assert_eq!(t.clone().add(route(&b)), named);
        assert_eq!(
            named.map_err(|e| e.to_string()),
            Err(
                "robot 2 stays on its first node: the route cannot be placed, avoid; \
                 released there already: robot 0"
                    .into()
            )
        );
        assert!(t.add(route(&b)).is_err());
        assert!(!before(&t, 2, 0, 0, 2));
        assert!(before(&t, 2, 0, 1, 2));
        assert_eq!(t.reroute_needed(1).map(|r| r.avoid), Some(vec![node("X")]));

        // the dispatcher cancels a, which stops before X
        let stopped = Cancel {
            finished: true,
            nothing_left: true,
            standing_on: Some(0),
        };
        assert_eq!(t.cancelled(0, stopped), Ok(true));
        assert_eq!(t.holder(&node("X")), Some(2));
    }

    #[test]
    fn a_robot_that_has_left_the_node_is_not_named() {
        let (a, b) = (["S", "X", "T"], ["X"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.progress(0, 2, 2), Ok(()));
        assert_eq!(t.add(route(&b)), Ok(1));
    }

    #[test]
    fn a_pass_that_is_over_takes_no_part() {
        let (a, b) = (["S", "M", "Y"], ["M", "Q"]);
        let mut t = traffic(&[&a, &b], &empty_map());
        assert_eq!(t.add(route(&a)), Ok(0));
        assert_eq!(t.progress(0, 2, 2), Ok(()));
        assert_eq!(t.add(route(&b)), Ok(1));
        assert!(!before(&t, 0, 2, 1, 0));
    }

    #[test]
    fn unknown_names_are_reported_without_other_robots() {
        let mut t = traffic(&[&["A"]], &empty_map());
        let unknown = NotAdded::Unknown(UnknownName::Piece(node("Z")));
        assert_eq!(t.add(route(&["Z"])), Err(unknown));
        assert_eq!(t.holder(&node("A")), None);
        // with a first node that is known, the robot is kept on it
        let unknown = NotAdded::Unknown(UnknownName::Piece(edge("A-Z")));
        let kept = NotAdded::Standing {
            robot: 0,
            why: Box::new(unknown),
            released_there: Vec::new(),
        };
        assert_eq!(t.add(route(&["A", "Z"])), Err(kept));
        assert_eq!(t.holder(&node("A")), Some(0));
    }
}

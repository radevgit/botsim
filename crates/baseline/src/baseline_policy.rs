//! The baseline's traffic rules, the first [`TrafficPolicy`]: node-by-node
//! reservations with a release window, swept-area conflicts, go-first, the wait
//! timeout that asks for another route, and the RELEASE zone answers.
//!
//! - **Reservations** (S34): a node, and the edge leading to it, are reserved
//!   for a robot when they are released to it, and given back when the robot
//!   has driven on to the next node. A robot is released only onto nodes and
//!   edges nobody else holds; otherwise it waits at the end of its base and the
//!   controller asks again every cycle. A robot without a route holds the node
//!   it stands on.
//! - **Re-routing** (S53): a robot that has stood at the end of its released
//!   route for [`REROUTE_AFTER_SECONDS`] because another robot holds the way asks
//!   the dispatcher for a new route around the nodes others hold.
//! - **Go first** (S77), see `go_first`.
//!
//! These rules are frozen (brief U10): only bug fixes; they are the reference other
//! policies are measured against.

use std::collections::{HashMap, HashSet};

use botsim_core::collision::{overlap, place};
use botsim_core::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_core::metrics::is_waiting;
use botsim_core::observer::Reservation;
use botsim_core::spec::Shape;
use vda5050::state::State;

use crate::conflicts::{ConflictInput, ConflictTable, Held, NodeConflictInput};
use crate::controller::NODE_TOLERANCE;
use crate::node_grid::NodeGrid;
use crate::order_builder::Plan;
use crate::policy::{Ask, Fleet, PolicyRobot, Release, Reroute, RouteView, Setup, TrafficPolicy};
use crate::reservations::Reservations;
use crate::zone_grants::{Answer, RobotView, ZoneGrants};

/// A robot that has been held at the end of its released route this long by
/// other robots gets a new route around them (S53, my choice: well before the
/// 30 s a deadlock needs to be reported).
pub const REROUTE_AFTER_SECONDS: f64 = 10.0;

/// A robot that has been held this long at the end of its released route by a
/// robot that is stuck behind it goes first (S77, my choice: before the re-route).
pub const GIVE_WAY_AFTER_SECONDS: f64 = 5.0;

/// Go-first takes the node or edge of a robot H only if H has had its route released
/// and has not moved for this long (it is held by its protective field, not just
/// released: warehouse 4, 1102 s, two robots were released onto the same node in the
/// same second and drove into each other).
pub const GIVE_WAY_HOLD_SECONDS: f64 = 2.0;

/// Extra distance (m) kept between the body swept by a turn on the spot and a stopped
/// robot.
const TURN_CLEARANCE: f64 = 0.05;

/// The baseline's traffic rules.
#[derive(Debug)]
pub struct BaselinePolicy {
    graph: Graph,
    /// Which node a position is on (S52).
    node_grid: NodeGrid,
    /// The robot types as `manufacturer/seriesName`.
    type_names: Vec<String>,
    robots: Vec<PolicyRobot>,
    /// The type of each robot, by robot index.
    robot_types: Vec<usize>,
    reservations: Reservations,
    /// Edges whose swept outlines overlap; `None` reserves nodes and edges only.
    conflicts: Option<ConflictTable>,
    /// Robot → the edge it may be released onto although it conflicts with an edge
    /// another robot holds (go-first, S77); cleared when it has been released.
    waived: HashMap<usize, EdgeIdx>,
    /// How long each robot has stood at the end of its released route because the
    /// next node or edge is held by another robot (S53); 0 on a new route.
    blocked_seconds: Vec<f64>,
    /// How long each robot has had a route released ahead of it and has not moved
    /// (stopped by its protective field, or just released). Go-first takes a node
    /// or edge from a robot only after [`GIVE_WAY_HOLD_SECONDS`] of it.
    still_seconds: Vec<f64>,
    /// Answers to RELEASE zone requests; `None` if the run has no zones.
    zone_grants: Option<ZoneGrants>,
    /// How many times a robot was let go first, ahead of the robot stuck behind it
    /// (S77).
    give_ways: usize,
}

impl BaselinePolicy {
    /// The baseline rules on a layout; the controller then calls
    /// [`TrafficPolicy::start`].
    pub fn new(graph: &Graph) -> BaselinePolicy {
        BaselinePolicy {
            graph: graph.clone(),
            node_grid: NodeGrid::new(graph),
            type_names: Vec::new(),
            robots: Vec::new(),
            robot_types: Vec::new(),
            reservations: Reservations::new(graph),
            conflicts: None,
            waived: HashMap::new(),
            blocked_seconds: Vec::new(),
            still_seconds: Vec::new(),
            zone_grants: None,
            give_ways: 0,
        }
    }

    /// Swept-area reservation (see [`crate::conflicts`]): from now on a robot is
    /// released onto an edge only if no other robot holds an edge that conflicts with
    /// it. Returns the number of conflicting edge pairs read.
    pub fn set_conflicts(&mut self, input: &[ConflictInput]) -> usize {
        let table = ConflictTable::new(&self.graph, &self.type_names, input);
        let n = table.pairs();
        self.conflicts = Some(table);
        n
    }

    /// Node-edge conflicts for the swept-area reservation: a robot is not released
    /// onto an edge that passes too close to a node another robot holds, nor to a node
    /// where it would stand next to an edge another robot holds. Call after
    /// [`BaselinePolicy::set_conflicts`]; returns the number read (0 if that was not
    /// called).
    pub fn set_node_conflicts(&mut self, input: &[NodeConflictInput]) -> usize {
        let Some(table) = self.conflicts.as_mut() else {
            return 0;
        };
        table.add_node_conflicts(&self.graph, &self.type_names, input)
    }

    /// Who holds which node and edge.
    pub fn table(&self) -> &Reservations {
        &self.reservations
    }

    /// How many times a robot went first, ahead of the robot stuck behind it
    /// (S77, [`GIVE_WAY_AFTER_SECONDS`]).
    pub fn give_ways(&self) -> usize {
        self.give_ways
    }

    /// Deadlocks with a safety link (S77): robot X stands at the end of its released
    /// route and waits for the next node, which robot H has released; H is stopped
    /// (its protective field sees X) and X is right in front of it, heading the same
    /// way. H cannot move until X does, so X goes first: it takes the node over (H
    /// gets it back when X lets go of it) and its release goes on as usual. Not
    /// done head-on (X heading against H): there both would run into each other.
    fn go_first(&mut self, fleet: &Fleet) {
        let states = fleet.states;
        for i in 0..self.robots.len() {
            let Some(work) = route_of(fleet, i) else {
                continue;
            };
            if self.blocked_seconds[i] < GIVE_WAY_AFTER_SECONDS || work.cancelling {
                continue;
            }
            let from = work.released_through;
            let (Some(&node), Some(&edge)) =
                (work.route.nodes.get(from + 1), work.route.edges.get(from))
            else {
                continue;
            };
            // X waits for the node, or for an edge whose swept area overlaps the one
            // of its next edge (swept-area reservation): the holder is H
            let all_node_holder = self.reservations.node_holder(node).filter(|&h| h != i);
            let all_swept = self
                .conflicts
                .as_ref()
                .and_then(|c| c.blocker(&self.reservations, &self.robot_types, i, edge));
            // or for a node (or an edge) that the next edge passes too close to
            // (node-edge conflict)
            let all_nblock = self
                .conflicts
                .as_ref()
                .and_then(|c| c.node_blocker(&self.reservations, &self.robot_types, i, edge));
            // X may be kept by several robots (the node holder and the holder of a
            // conflicting edge are often different ones: a robot passing through the
            // node, and a stopped one on a curve beside it). Each is tried in turn as
            // H; the first one that lets X go first is taken. A holder that is not
            // taken is simply waited for.
            for pass in 0..3 {
                let (node_holder, swept, nblock) = match pass {
                    // the node holder; if it also holds a conflicting edge, X is
                    // released onto its edge although it does
                    0 => (
                        all_node_holder,
                        all_swept.filter(|&(sh, _)| Some(sh) == all_node_holder),
                        None,
                    ),
                    1 => (None, all_swept, None),
                    _ => (None, None, all_nblock),
                };
                let Some(h) = node_holder
                    .or(swept.map(|(h, _)| h))
                    .or(nblock.map(|(h, _)| h))
                else {
                    continue;
                };
                // H must be on its way through the node (it has released it as part of
                // its route and does not stand on it); a robot that is parked there
                // holds it because it stands on it, and cannot be gone around. For a
                // conflicting edge: H has released it as part of its route.
                let on_its_way = match (node_holder, swept) {
                    (None, None) => nblock.is_some_and(|(_, held)| {
                        route_of(fleet, h).is_some_and(|w| match held {
                            Held::Node(n) => w
                                .route
                                .nodes
                                .iter()
                                .take(w.released_through + 1)
                                .any(|&m| m == n),
                            Held::Edge(f) => w
                                .route
                                .edges
                                .iter()
                                .take(w.released_through)
                                .any(|&e| e == f),
                        })
                    }),
                    (Some(_), _) => {
                        route_of(fleet, h).is_some_and(|w| {
                            w.route
                                .nodes
                                .iter()
                                .take(w.released_through + 1)
                                .any(|&n| n == node)
                        }) && self.node_of(&states[h]) != Some(node)
                    }
                    (None, Some((_, f))) => route_of(fleet, h).is_some_and(|w| {
                        w.route
                            .edges
                            .iter()
                            .take(w.released_through)
                            .any(|&e| e == f)
                    }),
                };
                if !on_its_way {
                    continue;
                }
                // H has not moved for a while with its route released
                if !(route_of(fleet, h).is_some() && self.still_seconds[h] >= GIVE_WAY_HOLD_SECONDS)
                {
                    continue;
                }
                // the edge must be free, or held by the same robot
                if !self
                    .reservations
                    .edge_holder(edge)
                    .is_none_or(|e| e == i || e == h)
                {
                    continue;
                }
                let (Some(px), Some(ph)) = (
                    states[i].mobile_robot_position.as_ref(),
                    states[h].mobile_robot_position.as_ref(),
                ) else {
                    continue;
                };
                if !stands_still(&states[h]) || px.map_id != ph.map_id {
                    continue;
                }
                // X is ahead of H, within 4 m and 60° of H's heading, and heads the same
                // way (within 45°)
                let (dx, dy) = (px.x - ph.x, px.y - ph.y);
                let off_heading = angle_between(dy.atan2(dx), ph.theta);
                // or its route after the node is the one H takes after it (a lane change
                // or merge: X stands on H's curve and both go on to the same node); then
                // X may also be beside H's curve (up to 100° off its heading, 5 m away).
                // The same holds if X stands on a node that H has released (H's way
                // goes through X).
                // (for a conflicting edge: or both routes meet at the end of H's edge and
                // go on to the same node, N908/T-009, warehouse 4)
                let same_route = same_route_after(fleet, i, h, node)
                    || swept.is_some_and(|(_, f)| self.meet_and_go_on(fleet, i, h, f));
                // (not by who holds the node: X may have taken it over earlier)
                let on_h_way = self.node_of(&states[i]).is_some_and(|n| {
                    route_of(fleet, h).is_some_and(|w| {
                        w.route
                            .nodes
                            .iter()
                            .take(w.released_through + 1)
                            .any(|&m| m == n)
                    })
                });
                let wide = same_route || on_h_way;
                let near = dx.hypot(dy) <= if wide { 5.0 } else { 4.0 };
                // the way X will head once it is released: along its next edge (a robot
                // standing on a node has not turned to the edge yet; warehouse 4, O-008
                // stood facing west, 117° from T-008, while its edge ran 45°)
                let start = self
                    .graph
                    .edge_directions(
                        edge,
                        &self.robots[i].id.manufacturer,
                        &self.robots[i].series,
                    )
                    .0;
                // X standing on H's way may head any way as long as it does not face H
                let faces_away = angle_between(start, (-dy).atan2(-dx)) >= 90f64.to_radians();
                let same_way = angle_between(start, ph.theta) <= 45f64.to_radians()
                    || same_route
                    || (on_h_way && faces_away)
                    // a wait for a conflicting edge or a close node (the map): X's edge
                    // leads away from H, which is stopped, so X may drive off first
                    || ((swept.is_some() || nblock.is_some()) && faces_away);
                let limit = if wide { 100.0 } else { 60.0 };
                let ahead = off_heading <= f64::to_radians(limit);
                // X may have to turn on the spot to drive off; its body then sweeps
                // through the angles to the start of the edge and must not touch H (a
                // forklift turning 1.7 m from a stopped robot hit it, warehouse 4, 309 s)
                // (an omnidirectional robot does not turn: it drives off the way it is
                // facing)
                let turns = !self.robots[i].omnidirectional
                    && angle_between(px.theta, start) > 15f64.to_radians();
                let clear = !turns
                    || turn_is_clear(
                        &self.robots[i].shape,
                        (px.x, px.y, px.theta),
                        start,
                        &self.robots[h].shape,
                        (ph.x, ph.y, ph.theta),
                    );
                if !(near && ahead && same_way && clear) {
                    continue;
                }
                if node_holder.is_some() {
                    if self.reservations.take_over_node(node, i).is_none() {
                        continue;
                    }
                    self.reservations.take_over_edge(edge, i);
                }
                // the edge conflicts with one H holds: X may be released onto its edge
                // although H still holds the other (until X has been released)
                if swept.is_some_and(|(sh, _)| sh == h) || nblock.is_some_and(|(nh, _)| nh == h) {
                    self.waived.insert(i, edge);
                }
                self.give_ways += 1;
                break;
            }
        }
    }

    /// Does the route of `a`, within its next three nodes, reach the end node of edge
    /// `f` of `b`, and do both go on from there to the same node?
    fn meet_and_go_on(&self, fleet: &Fleet, a: usize, b: usize, f: EdgeIdx) -> bool {
        let (Some(wa), Some(wb)) = (route_of(fleet, a), route_of(fleet, b)) else {
            return false;
        };
        let m = self.graph.edges[f].end;
        let first = wa.released_through + 1;
        let Some(p) = wa
            .route
            .nodes
            .iter()
            .enumerate()
            .skip(first)
            .take(3)
            .find_map(|(k, &n)| (n == m).then_some(k))
        else {
            return false;
        };
        let after_a = wa.route.nodes.get(p + 1);
        let after_b = wb
            .route
            .nodes
            .iter()
            .skip(wb.released_through.saturating_sub(1))
            .position(|&n| n == m)
            .and_then(|k| {
                wb.route
                    .nodes
                    .get(wb.released_through.saturating_sub(1) + k + 1)
            });
        after_a.is_some() && after_a == after_b
    }

    /// The layout node a robot stands on, from its reported position.
    fn node_of(&self, state: &State) -> Option<NodeIdx> {
        let p = state.mobile_robot_position.as_ref()?;
        let map = self.graph.map_idx(&p.map_id)?;
        self.node_grid.node_near(map, p.x, p.y, NODE_TOLERANCE)
    }
}

impl TrafficPolicy for BaselinePolicy {
    fn start(&mut self, setup: &Setup) {
        self.type_names = setup.types.to_vec();
        self.robots = setup.robots.to_vec();
        self.robot_types = self.robots.iter().map(|r| r.type_idx).collect();
        self.blocked_seconds = vec![0.0; self.robots.len()];
        self.still_seconds = vec![0.0; self.robots.len()];
        self.zone_grants = setup.zones.cloned().map(ZoneGrants::new);
    }

    fn conflicts(&mut self, edges: &[ConflictInput]) {
        self.set_conflicts(edges);
    }

    fn node_conflicts(&mut self, nodes: &[NodeConflictInput]) {
        self.set_node_conflicts(nodes);
    }

    fn place(&mut self, ask: &Ask) -> Release {
        let i = ask.robot;
        self.blocked_seconds[i] = 0.0;
        self.still_seconds[i] = 0.0;
        // node 0 is where the robot stands (it holds it already)
        self.reservations.reserve_node(ask.route.nodes[0], i);
        let swept = self
            .conflicts
            .as_ref()
            .map(|c| (c, self.robot_types.as_slice()));
        let through = reserve_up_to(
            &mut self.reservations,
            ask.route,
            i,
            ask.released_through,
            ask.limit,
            swept,
            None,
        );
        Release {
            through,
            reroute: None,
        }
    }

    fn release(&mut self, ask: &Ask) -> Release {
        let (i, at, from, wanted) = (ask.robot, ask.at, ask.released_through, ask.limit);
        let plan = ask.route;
        // the robot has driven on: give back what it has left behind (S34)
        // (a route can pass the same node or edge again, e.g. a lap of a ring:
        // what is still ahead stays reserved)
        let ahead_nodes = &plan.nodes[at..=from];
        let ahead_edges = &plan.edges[at..from];
        // A node taken over from a robot in the way (S77) is held before it is
        // released: it is the next node after the released ones. It is not given
        // back while the route has it there, even if the route passed it earlier;
        // but it is once the robot has driven on, also if the route has it again
        // much later (a lap of a ring: it then held the node for the whole lap,
        // warehouse 4, Y-007 held two nodes 158 m away).
        let last = plan.last();
        let next_nodes = &plan.nodes[at..=(from + 1).min(last)];
        let next_edges = &plan.edges[at..(from + 1).min(last)];
        for j in 0..at {
            let (n, e) = (plan.nodes[j], plan.edges[j]);
            if !ahead_nodes.contains(&n)
                && !(self.reservations.is_taken_over_node(n, i) && next_nodes.contains(&n))
            {
                self.reservations.release_node(n, i);
            }
            if !ahead_edges.contains(&e)
                && !(self.reservations.is_taken_over_edge(e, i) && next_edges.contains(&e))
            {
                self.reservations.release_edge(e, i);
            }
        }

        // A node or edge on the released way ahead that another robot has taken over
        // (go-first): nothing beyond it until it is this robot's again. Otherwise it
        // is released past the robot in its way and drives after it, and its
        // protective field does not see that robot on a curve (warehouse 4, 770 s).
        let other = |holder: Option<usize>| holder.is_some_and(|h| h != i);
        let overtaken = plan.nodes[at + 1..=from]
            .iter()
            .any(|&n| other(self.reservations.node_holder(n)))
            || plan.edges[at..from]
                .iter()
                .any(|&e| other(self.reservations.edge_holder(e)));
        let limit = if overtaken { from } else { wanted };

        // release as far as the window allows and nobody else holds the way
        let swept = self
            .conflicts
            .as_ref()
            .map(|c| (c, self.robot_types.as_slice()));
        let waived = self.waived.get(&i).copied();
        let rt = reserve_up_to(&mut self.reservations, plan, i, from, limit, swept, waived);
        if rt > from {
            self.waived.remove(&i);
        }

        // Timeout re-routing (S53): the robot stands at the end of its released
        // route and wants to go on, but another robot holds the way (not a lift
        // or a door, which have their own rules).
        let stuck = at == from && wanted > from && rt == from && is_waiting(ask.state);
        self.blocked_seconds[i] = if stuck {
            self.blocked_seconds[i] + ask.elapsed
        } else {
            0.0
        };
        self.still_seconds[i] = if rt > at && stands_still(ask.state) {
            self.still_seconds[i] + ask.elapsed
        } else {
            0.0
        };
        let reroute = (stuck && self.blocked_seconds[i] >= REROUTE_AFTER_SECONDS).then(|| {
            self.blocked_seconds[i] = 0.0; // try again after another timeout
            // around the nodes others hold (they hold them or have had them released)
            let avoid: HashSet<NodeIdx> = (0..self.graph.nodes.len())
                .filter(|&n| self.reservations.node_holder(n).is_some_and(|h| h != i))
                .collect();
            Reroute { avoid }
        });
        Release {
            through: rt,
            reroute,
        }
    }

    /// The robot is at the end of its route: it has driven on from everything before
    /// it, so give that back (S34). A route without an action at its end (a trip to
    /// a charger or a parking place) ends in the very cycle the robot arrives, before
    /// [`TrafficPolicy::release`] would have given back the last edge and the node
    /// before it.
    fn route_finished(&mut self, robot: usize, plan: &Plan) {
        let here = plan.nodes[plan.last()];
        for j in 0..plan.last() {
            if plan.nodes[j] != here {
                self.reservations.release_node(plan.nodes[j], robot);
            }
            self.reservations.release_edge(plan.edges[j], robot);
        }
    }

    /// The robot refused the order: give back what was reserved for it (it still
    /// holds the node it stands on).
    fn route_dropped(&mut self, robot: usize, plan: &Plan, released_through: usize) {
        for j in 1..=released_through {
            self.reservations.release_node(plan.nodes[j], robot);
            self.reservations.release_edge(plan.edges[j - 1], robot);
        }
    }

    fn after_releases(&mut self, fleet: &Fleet) {
        self.go_first(fleet);
    }

    // a robot without a job holds the node it stands on (S34)
    fn idle(&mut self, robot: usize, node: NodeIdx) {
        self.reservations.reserve_node(node, robot);
    }

    /// Answer the RELEASE zone requests the robots state (see [`crate::zone_grants`]).
    fn zone_answers(&mut self, fleet: &Fleet) -> Vec<Answer> {
        let Some(grants) = self.zone_grants.as_mut() else {
            return Vec::new();
        };
        let views: Vec<RobotView> = fleet
            .states
            .iter()
            .map(|s| RobotView {
                requests: s.zone_requests.clone().unwrap_or_default(),
                position: s
                    .mobile_robot_position
                    .as_ref()
                    .and_then(|p| self.graph.map_idx(&p.map_id).map(|m| (m, p.x, p.y))),
            })
            .collect();
        grants.decide(&views)
    }

    fn node_holder(&self, node: NodeIdx) -> Option<usize> {
        self.reservations.node_holder(node)
    }

    fn edge_holder(&self, edge: EdgeIdx) -> Option<usize> {
        self.reservations.edge_holder(edge)
    }

    fn reservations(&self) -> Vec<Reservation> {
        self.reservations.to_observer()
    }

    fn zone_blockers(&self, robot: usize) -> Vec<usize> {
        self.zone_grants
            .as_ref()
            .map_or_else(Vec::new, |g| g.blockers(robot))
    }

    // the first of: who holds the next node, who holds the edge to it, who holds an
    // edge whose swept area meets it, who is on the other side of a node-edge
    // conflict (the checks of `reserve_up_to`)
    fn blocker(&self, robot: usize, route: &Plan, released_through: usize) -> Option<usize> {
        let &edge = route.edges.get(released_through)?;
        let &node = route.nodes.get(released_through + 1)?;
        let other = |holder: Option<usize>| holder.filter(|&h| h != robot);
        other(self.reservations.node_holder(node))
            .or_else(|| other(self.reservations.edge_holder(edge)))
            .or_else(|| {
                // (a robot that went first is let onto this edge despite a conflict)
                let table = self
                    .conflicts
                    .as_ref()
                    .filter(|_| self.waived.get(&robot) != Some(&edge))?;
                let (held, types) = (&self.reservations, &self.robot_types);
                table
                    .blocker(held, types, robot, edge)
                    .map(|(h, _)| h)
                    .or_else(|| table.node_blocker(held, types, robot, edge).map(|(h, _)| h))
            })
    }
}

/// Does the robot neither drive (forwards or sideways) nor turn?
fn stands_still(state: &State) -> bool {
    state.velocity.as_ref().is_none_or(|v| {
        [v.vx, v.vy, v.omega]
            .iter()
            .all(|c| c.unwrap_or(0.0).abs() < 0.05)
    })
}

/// The route of robot `i`, if it has one.
fn route_of<'a>(fleet: &Fleet<'a>, i: usize) -> Option<RouteView<'a>> {
    fleet.routes.get(i).copied().flatten()
}

/// Do robots `a` and `b` both go on from `node` to the same next node? `a`
/// has `node` next in its plan; `b` has it somewhere ahead in its own.
fn same_route_after(fleet: &Fleet, a: usize, b: usize, node: NodeIdx) -> bool {
    let (Some(wa), Some(wb)) = (route_of(fleet, a), route_of(fleet, b)) else {
        return false;
    };
    let after_a = wa.route.nodes.get(wa.released_through + 2);
    let after_b = wb
        .route
        .nodes
        .iter()
        .skip(wb.released_through.saturating_sub(1))
        .position(|&n| n == node)
        .and_then(|k| {
            wb.route
                .nodes
                .get(wb.released_through.saturating_sub(1) + k + 1)
        });
    after_a.is_some() && after_a == after_b
}

/// Can a robot at `x` (x, y, heading) turn on the spot to the heading `to` without
/// touching the robot `h` (standing still)? The bodies are placed like the world
/// does; `h` is grown by `TURN_CLEARANCE`, and the turn is checked every 5°.
fn turn_is_clear(
    shape: &Shape,
    x: (f64, f64, f64),
    to: f64,
    h_shape: &Shape,
    h: (f64, f64, f64),
) -> bool {
    let grown = match *h_shape {
        Shape::Rect {
            cx,
            cy,
            half_length,
            half_width,
        } => Shape::Rect {
            cx,
            cy,
            half_length: half_length + TURN_CLEARANCE,
            half_width: half_width + TURN_CLEARANCE,
        },
        Shape::Circle { cx, cy, radius } => Shape::Circle {
            cx,
            cy,
            radius: radius + TURN_CLEARANCE,
        },
    };
    let body_h = place(&grown, h.0, h.1, h.2);
    // the shortest way round, from -π to π
    let turn =
        (to - x.2 + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
    let steps = ((turn.abs() / 5f64.to_radians()).ceil() as usize).max(1);
    (0..=steps).all(|k| {
        let heading = x.2 + turn * k as f64 / steps as f64;
        !overlap(&place(shape, x.0, x.1, heading), &body_h)
    })
}

/// The angle between two directions, 0..π.
fn angle_between(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(std::f64::consts::TAU);
    if d > std::f64::consts::PI {
        std::f64::consts::TAU - d
    } else {
        d
    }
}

/// Reserve plan nodes `from + 1 ..= wanted` for `robot`, in order, each together
/// with the edge leading to it; stop at the first one somebody else holds, or whose
/// edge conflicts with an edge another robot holds (`swept`: the conflict table and
/// the type of each robot). Returns the index of the last node now released (`from`
/// if none).
fn reserve_up_to(
    reservations: &mut Reservations,
    plan: &Plan,
    robot: usize,
    from: usize,
    wanted: usize,
    swept: Option<(&ConflictTable, &[usize])>,
    waived: Option<EdgeIdx>,
) -> usize {
    let mut rt = from;
    while rt < wanted {
        let e = plan.edges[rt];
        if waived != Some(e)
            && swept.is_some_and(|(table, types)| table.blocks(reservations, types, robot, e))
        {
            break;
        }
        if !reservations.reserve_step(e, plan.nodes[rt + 1], robot) {
            break;
        }
        rt += 1;
    }
    rt
}

#[cfg(test)]
#[path = "../tests/common/mod.rs"]
mod test_graph;

#[cfg(test)]
mod tests {
    use super::*;
    use botsim_core::world::RobotId;

    /// A state of a standing robot; the release rule does not look into it here.
    fn standing() -> State {
        serde_json::from_str(
            r#"{"headerId":0,"timestamp":"2026-11-22T10:00:00.000Z","version":"3.0.0",
            "manufacturer":"ACME","serialNumber":"H","orderId":"job-1","orderUpdateId":0,
            "lastNodeId":"A","lastNodeSequenceId":0,"nodeStates":[],"edgeStates":[],
            "driving":false,"actionStates":[],"instantActionStates":[],
            "powerSupply":{"stateOfCharge":100.0,"charging":false},
            "operatingMode":"AUTOMATIC","errors":[],
            "safetyState":{"activeEmergencyStop":"NONE","fieldViolation":false}}"#,
        )
        .unwrap()
    }

    /// A robot that turns on the spot, or drives sideways, with its way released is
    /// not standing still: go-first must not take a node from it (M1, 354 s: Y-001
    /// turned, X-001 was given the node at the corner and they collided).
    #[test]
    fn a_robot_that_turns_or_drives_sideways_is_not_standing_still() {
        let g = test_graph::graph(&[("A", 0.0, 0.0), ("B", 3.0, 0.0)], &[("A", "B")]);
        let route = Plan::new(&g, g.node_idx("A").unwrap(), vec![0]);
        let mut policy = BaselinePolicy::new(&g);
        policy.start(&Setup {
            graph: &g,
            types: &["ACME/X1".to_string()],
            robots: &[PolicyRobot {
                id: RobotId::new("ACME", "H"),
                series: "X1".into(),
                type_idx: 0,
                shape: Shape::Circle {
                    cx: 0.0,
                    cy: 0.0,
                    radius: 0.5,
                },
                omnidirectional: false,
            }],
            zones: None,
        });
        // three seconds with B released ahead, at the given velocity
        let mut still_after = |velocity: &str| {
            let mut state = standing();
            state.velocity = serde_json::from_str(velocity).unwrap();
            policy.release(&Ask {
                robot: 0,
                state: &state,
                route: &route,
                at: 0,
                released_through: 1,
                limit: 1,
                elapsed: 3.0,
            });
            policy.still_seconds[0]
        };
        assert_eq!(still_after(r#"{"vx":0.0,"vy":0.0,"omega":0.0}"#), 3.0);
        assert_eq!(still_after(r#"{"vx":0.0,"vy":0.0,"omega":-0.6}"#), 0.0);
        assert_eq!(still_after(r#"{"vx":0.0,"vy":0.8,"omega":0.0}"#), 0.0);
    }

    /// After go-first has given a node on the released way of H to another robot,
    /// H gets nothing beyond it until it has the node back (warehouse 4, 770 s:
    /// T-002 was released past T-008 and drove into it on the curve behind).
    #[test]
    fn no_release_past_a_node_another_robot_took_over() {
        let g = test_graph::graph(
            &[
                ("A", 0.0, 0.0),
                ("B", 3.0, 0.0),
                ("C", 6.0, 0.0),
                ("D", 9.0, 0.0),
            ],
            &[("A", "B"), ("B", "C"), ("C", "D")],
        );
        let n = |id: &str| g.node_idx(id).unwrap();
        let (a, b) = (n("A"), n("B"));
        let route = Plan::new(&g, a, vec![0, 1, 2]);
        let robot = |serial: &str| PolicyRobot {
            id: RobotId::new("ACME", serial),
            series: "X1".into(),
            type_idx: 0,
            shape: Shape::Circle {
                cx: 0.0,
                cy: 0.0,
                radius: 0.5,
            },
            omnidirectional: false,
        };
        let mut policy = BaselinePolicy::new(&g);
        policy.start(&Setup {
            graph: &g,
            types: &["ACME/X1".to_string()],
            robots: &[robot("H"), robot("X")],
            zones: None,
        });
        let state = standing();
        let ask = |from, limit| Ask {
            robot: 0,
            state: &state,
            route: &route,
            at: 0,
            released_through: from,
            limit,
            elapsed: 0.05,
        };
        assert_eq!(policy.place(&ask(0, 1)).through, 1);
        // robot 1 takes B over, as go-first does
        assert_eq!(policy.reservations.take_over_node(b, 1), Some(0));
        assert_eq!(policy.release(&ask(1, 2)).through, 1, "past robot 1");
        assert_eq!(policy.node_holder(n("C")), None, "C is not held for it");
        // robot 1 has driven on: B goes back, and the release goes on
        policy.reservations.release_node(b, 1);
        assert_eq!(policy.release(&ask(1, 2)).through, 2);
    }
}

//! The baseline controller's use of lifts (S47).
//!
//! A job whose route has a **ride** (a layout edge between two stops of one
//! lift, see `docs/rules.md` §7g) needs the lift. The controller handles it
//! with the lift's own command port ([`World::call_lift_door`],
//! [`World::release_lift`]); the robot only drives and waits.
//!
//! **One robot uses a lift at a time** (it "owns" it), first come first served:
//! - **Where a robot waits** (design rule §7d: outside the approach to the
//!   entrance): at the stop's **waiting node** (lay-by) if the layout has one
//!   (`waiting` of the stop, S49) — then the route to the lift is planned
//!   through that node ([`LiftDesk::route`]) — else at the node **before** the
//!   entrance. That node is the robot's **gate**. A robot that starts on the
//!   entrance cannot wait outside.
//! - A robot is never released past its gate unless it owns the lift. At the
//!   gate it asks for the lift: if the lift is free the robot owns it from then
//!   on and is released through the ride; if another robot owns it, the robot
//!   waits at the gate and asks again every cycle. Without a waiting node in a
//!   two-way layout the robot waits on the approach and can block the robot that
//!   comes out (deadlock): the layout must provide the lay-by.
//! - **Calling:** the controller calls the cabin to the entrance (floor and
//!   door of the stop) and keeps calling until the robot is in the cabin.
//! - **Riding:** when the robot is in the cabin (the lift reports its rider)
//!   the controller releases the door and sends the cabin to the floor and door
//!   of the stop at the end of the ride.
//! - **Exiting:** when the robot has got out (its last node is the end of the
//!   ride) the door stays open until the robot has driven away from the node, or
//!   its job is done; then the controller releases the door and the lift is free
//!   for the next robot.
//!
//! Not yet: several robots in a cabin, choosing the lift (a ride is the one
//! layout edge), giving the lift back if a robot never comes, a lift going
//! somewhere else while idle (the cabin stays where the last ride ended).

use std::collections::HashMap;

use botsim_core::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_core::world::{RobotId, World};

use crate::order_builder::Plan;
use crate::route::shortest_path;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    Calling,
    Riding,
    Exiting,
}

/// The robot that owns a lift, and where it is in the ride.
#[derive(Debug, Clone)]
struct Owner {
    robot: usize,
    /// Plan index of the ride edge.
    ride: usize,
    edge: EdgeIdx,
    stage: Stage,
}

#[derive(Debug)]
struct Stop {
    node: NodeIdx,
    map_id: String,
    door_id: String,
    /// The lay-by for robots that wait for the lift.
    waiting: Option<NodeIdx>,
}

#[derive(Debug)]
struct LiftInfo {
    id: String,
    stops: Vec<Stop>,
}

impl LiftInfo {
    fn stop(&self, node: NodeIdx) -> &Stop {
        self.stops
            .iter()
            .find(|s| s.node == node)
            .expect("a ride starts and ends at stops of the lift")
    }
}

/// What the lift handling needs to know about a robot each cycle.
#[derive(Debug, Clone, Copy)]
pub struct LiftRobot<'a> {
    pub id: &'a RobotId,
    /// Plan index of the robot's last node; `None` if it has no job.
    pub at: Option<usize>,
    /// The layout node the robot stands on, if any.
    pub node: Option<NodeIdx>,
}

#[derive(Debug)]
pub struct LiftDesk {
    graph: Graph,
    lifts: Vec<LiftInfo>,
    /// Layout edge → (lift index, start node, end node) for every ride.
    rides: HashMap<EdgeIdx, (usize, NodeIdx, NodeIdx)>,
    owners: Vec<Option<Owner>>,
}

impl LiftDesk {
    /// The lifts of `world` and the rides between their stops.
    pub fn new(world: &World) -> LiftDesk {
        let graph = world.graph();
        let lifts: Vec<LiftInfo> = world
            .lift_specs()
            .into_iter()
            .map(|s| LiftInfo {
                id: s.id.clone(),
                stops: s
                    .stops
                    .iter()
                    .map(|st| Stop {
                        node: st.node,
                        map_id: graph.maps[graph.nodes[st.node].map].clone(),
                        door_id: s.doors[st.door].clone(),
                        waiting: st.waiting,
                    })
                    .collect(),
            })
            .collect();
        let mut rides = HashMap::new();
        for (i, e) in graph.edges.iter().enumerate() {
            if !e.map_transition {
                continue;
            }
            let on = |l: &LiftInfo, n| l.stops.iter().any(|s| s.node == n);
            if let Some(l) = lifts.iter().position(|l| on(l, e.start) && on(l, e.end)) {
                rides.insert(i, (l, e.start, e.end));
            }
        }
        LiftDesk {
            graph: graph.clone(),
            owners: vec![None; lifts.len()],
            lifts,
            rides,
        }
    }

    /// The shortest route from `from` to `to`, planned **through the waiting
    /// node** of every lift entrance it uses that has one: to the lay-by, from
    /// there to the entrance, then the ride. No detour if the robot starts on
    /// the entrance, already passes the lay-by before the ride, or the lay-by
    /// cannot be reached.
    pub fn route(&self, from: NodeIdx, to: NodeIdx) -> Option<Vec<EdgeIdx>> {
        let path = shortest_path(&self.graph, from, to)?;
        let mut nodes = vec![from];
        nodes.extend(path.iter().map(|&e| self.graph.edges[e].end));
        for (i, &e) in path.iter().enumerate() {
            let Some(&(l, a, b)) = self.rides.get(&e) else {
                continue;
            };
            let waiting = self.lifts[l].stop(a).waiting;
            let before = match waiting {
                Some(w) if i > 0 && !nodes[..i].contains(&w) => {
                    match (
                        shortest_path(&self.graph, from, w),
                        shortest_path(&self.graph, w, a),
                    ) {
                        (Some(mut to_w), Some(w_to_a)) => {
                            to_w.extend(w_to_a);
                            to_w
                        }
                        _ => path[..i].to_vec(),
                    }
                }
                _ => path[..i].to_vec(),
            };
            let mut edges = before;
            edges.push(e);
            edges.extend(self.route(b, to)?);
            return Some(edges);
        }
        Some(path)
    }

    /// The plan index of the node where a robot must wait for the ride `k` of
    /// `plan` (its gate): the last time the plan passes the waiting node of the
    /// entrance before the ride, else the node before the entrance.
    fn gate(&self, plan: &Plan, k: usize, lift: usize, entrance: NodeIdx) -> usize {
        self.lifts[lift]
            .stop(entrance)
            .waiting
            .and_then(|w| plan.nodes[..k].iter().rposition(|&n| n == w))
            .unwrap_or(k.saturating_sub(1))
    }

    /// How far the order of `robot` may be released: `wanted` is the last node
    /// the release window reaches when the robot is at plan node `at`, `from` the
    /// last node already released. The release never passes the robot's gate (see
    /// the module text) unless the robot owns the lift; at the gate the robot
    /// gets the lift if it is free.
    pub fn window(
        &mut self,
        robot: usize,
        plan: &Plan,
        at: usize,
        from: usize,
        wanted: usize,
    ) -> usize {
        for (k, &edge) in plan.edges.iter().enumerate().skip(at) {
            let Some(&(l, a, _)) = self.rides.get(&edge) else {
                continue;
            };
            let gate = self.gate(plan, k, l, a);
            if wanted <= gate {
                return wanted; // the window does not reach the gate yet
            }
            match &self.owners[l] {
                Some(o) if o.robot == robot && o.ride == k => {}
                None if at >= gate => {
                    self.owners[l] = Some(Owner {
                        robot,
                        ride: k,
                        edge,
                        stage: Stage::Calling,
                    });
                }
                _ => return gate.max(from),
            }
        }
        wanted
    }

    /// The robot's order was refused: it gives back any lift it was given.
    pub fn forget(&mut self, robot: usize) {
        for o in &mut self.owners {
            if o.as_ref().is_some_and(|o| o.robot == robot) {
                *o = None;
            }
        }
    }

    /// One control cycle: call, release and send the cabins of the lifts that
    /// have an owner.
    pub fn update(&mut self, world: &mut World, robots: &[LiftRobot]) {
        for l in 0..self.owners.len() {
            let Some(mut o) = self.owners[l].clone() else {
                continue;
            };
            let info = &self.lifts[l];
            let (_, a, b) = self.rides[&o.edge];
            let (stop_a, stop_b) = (info.stop(a), info.stop(b));
            let r = robots[o.robot];
            let call = |world: &mut World, s: &Stop| {
                world
                    .call_lift_door(&info.id, &s.map_id, &s.door_id)
                    .expect("the lift, map and door exist");
            };

            let Some(at) = r.at else {
                // the job is over: the door is let go and the lift is free
                world.release_lift(&info.id).expect("the lift exists");
                self.owners[l] = None;
                continue;
            };
            if o.stage == Stage::Calling {
                if world.lift_rider(&info.id).expect("the lift exists") == Some(r.id) {
                    world.release_lift(&info.id).expect("the lift exists");
                    call(world, stop_b);
                    o.stage = Stage::Riding;
                } else {
                    call(world, stop_a);
                }
            }
            if o.stage == Stage::Riding && at > o.ride {
                o.stage = Stage::Exiting;
            }
            if o.stage == Stage::Exiting && r.node != Some(b) {
                world.release_lift(&info.id).expect("the lift exists");
                self.owners[l] = None;
                continue;
            }
            self.owners[l] = Some(o);
        }
    }
}

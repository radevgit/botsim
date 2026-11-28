//! The baseline controller's use of lifts (S47).
//!
//! A job whose route has a **ride** (a layout edge between two stops of one
//! lift, see `docs/rules.md` §7g) needs the lift. The controller handles it
//! with messages to the lift ([`Message::CallLift`], [`Message::ReleaseLift`]);
//! the robot only drives and waits.
//!
//! **One robot uses a lift at a time** (it "owns" it), first come first served:
//! - **Where a robot waits** (design rule §7d: outside the approach to the
//!   entrance): at the stop's **waiting node** (lay-by) if the layout has one
//!   (`waiting` of the stop, S49) — then the route to the lift is planned
//!   through that node
//!   ([`route_through_waiting`](botsim_router::lift::route_through_waiting) with
//!   [`LiftDesk::waiting_nodes`]) — else at the node **before** the
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

use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_model::robot::RobotId;

use crate::cycle::Message;
use crate::order_builder::Plan;

/// A lift, as the controller is told of it.
#[derive(Debug, Clone, PartialEq)]
pub struct Lift {
    pub id: String,
    /// The names of the cabin's doors.
    pub doors: Vec<String>,
    /// Where robots get in and out. A layout edge between two stops on different
    /// maps is a ride.
    pub stops: Vec<LiftStop>,
}

/// A place where robots get into or out of a lift.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LiftStop {
    /// The layout node at the cabin entrance.
    pub node: NodeIdx,
    /// Index into [`Lift::doors`]: the door that opens here.
    pub door: usize,
    /// A waiting node (lay-by) for robots that wait for the lift (S49).
    pub waiting: Option<NodeIdx>,
}

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
    /// The stop of the lift at `node` (a ride starts and ends at stops of its lift).
    fn stop(&self, node: NodeIdx) -> Option<&Stop> {
        self.stops.iter().find(|s| s.node == node)
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
    lifts: Vec<LiftInfo>,
    /// Layout edge → (lift index, start node, end node) for every ride.
    rides: HashMap<EdgeIdx, (usize, NodeIdx, NodeIdx)>,
    /// Every ride → the waiting node of its entrance, for the route search.
    waiting: HashMap<EdgeIdx, Option<NodeIdx>>,
    owners: Vec<Option<Owner>>,
}

impl LiftDesk {
    /// The lifts of the site and the rides between their stops on the layout
    /// `graph`. A stop at a node the layout does not have, or with a door the lift
    /// does not have, is left out.
    pub fn new(graph: &Graph, lifts: &[Lift]) -> LiftDesk {
        let stop = |lift: &Lift, st: &LiftStop| {
            Some(Stop {
                node: st.node,
                map_id: graph.maps.get(graph.nodes.get(st.node)?.map)?.clone(),
                door_id: lift.doors.get(st.door)?.clone(),
                waiting: st.waiting,
            })
        };
        let lifts: Vec<LiftInfo> = lifts
            .iter()
            .map(|l| LiftInfo {
                id: l.id.clone(),
                stops: l.stops.iter().filter_map(|st| stop(l, st)).collect(),
            })
            .collect();
        let mut rides = HashMap::new();
        let mut waiting = HashMap::new();
        for (i, e) in graph.edges.iter().enumerate() {
            if !e.map_transition {
                continue;
            }
            let on = |l: &LiftInfo, n| l.stops.iter().any(|s| s.node == n);
            if let Some(l) = lifts.iter().position(|l| on(l, e.start) && on(l, e.end)) {
                rides.insert(i, (l, e.start, e.end));
                waiting.insert(i, lifts[l].stop(e.start).and_then(|s| s.waiting));
            }
        }
        LiftDesk {
            owners: vec![None; lifts.len()],
            lifts,
            rides,
            waiting,
        }
    }

    /// Every ride with the waiting node of its entrance (`None` if the stop has
    /// none), as the router takes it to plan routes through those nodes.
    pub fn waiting_nodes(&self) -> &HashMap<EdgeIdx, Option<NodeIdx>> {
        &self.waiting
    }

    /// The plan index of the node where a robot must wait for the ride `k` of
    /// `plan` (its gate): the last time the plan passes the waiting node of the
    /// entrance before the ride, else the node before the entrance.
    fn gate(&self, plan: &Plan, k: usize, lift: usize, entrance: NodeIdx) -> usize {
        self.lifts[lift]
            .stop(entrance)
            .and_then(|s| s.waiting)
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

    /// The other robot that owns the lift [`LiftDesk::window`] keeps `robot` waiting
    /// for at plan node `from`, the end of its released route; `None` if no lift does.
    pub fn held_by(&self, robot: usize, plan: &Plan, from: usize) -> Option<usize> {
        for (k, &edge) in plan.edges.iter().enumerate().skip(from) {
            let Some(&(l, a, _)) = self.rides.get(&edge) else {
                continue;
            };
            if self.gate(plan, k, l, a) > from {
                return None; // the route may go on up to the gate
            }
            let owner = self.owners[l].as_ref().map(|o| o.robot);
            if let Some(other) = owner.filter(|&r| r != robot) {
                return Some(other);
            }
        }
        None
    }

    /// Is this layout edge a lift ride?
    pub fn is_ride(&self, edge: EdgeIdx) -> bool {
        self.rides.contains_key(&edge)
    }

    /// Does the robot own a lift now?
    pub fn owns(&self, robot: usize) -> bool {
        self.owners
            .iter()
            .any(|o| o.as_ref().is_some_and(|o| o.robot == robot))
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
    /// have an owner. `riders` names, lift by lift, the robot in each cabin as
    /// far as the lift says; the commands to the lifts go to `out`.
    pub fn update(
        &mut self,
        riders: &[Option<RobotId>],
        robots: &[LiftRobot],
        out: &mut Vec<Message>,
    ) {
        for l in 0..self.owners.len() {
            let Some(mut o) = self.owners[l].clone() else {
                continue;
            };
            let info = &self.lifts[l];
            // (a ride is between two stops of its lift, which the desk found in the
            // layout; if not, the owner lets go)
            let ride = self
                .rides
                .get(&o.edge)
                .and_then(|&(_, a, b)| Some((b, info.stop(a)?, info.stop(b)?)));
            let Some((b, stop_a, stop_b)) = ride else {
                self.owners[l] = None;
                continue;
            };
            let r = robots[o.robot];
            let call = |s: &Stop| Message::CallLift {
                lift: info.id.clone(),
                map: s.map_id.clone(),
                door: s.door_id.clone(),
            };
            let release = || Message::ReleaseLift(info.id.clone());

            let Some(at) = r.at else {
                // the job is over: the door is let go and the lift is free
                out.push(release());
                self.owners[l] = None;
                continue;
            };
            if o.stage == Stage::Calling {
                if riders.get(l).and_then(Option::as_ref) == Some(r.id) {
                    out.push(release());
                    out.push(call(stop_b));
                    o.stage = Stage::Riding;
                } else {
                    out.push(call(stop_a));
                }
            }
            if o.stage == Stage::Riding && at > o.ride {
                o.stage = Stage::Exiting;
            }
            if o.stage == Stage::Exiting && r.node != Some(b) {
                out.push(release());
                self.owners[l] = None;
                continue;
            }
            self.owners[l] = Some(o);
        }
    }
}

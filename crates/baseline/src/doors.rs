//! The baseline controller's use of doors (S51).
//!
//! A door guards layout edges (see `docs/rules.md` §7h). A robot does not start
//! such an edge unless the door is fully open, and the door does not close on a
//! robot that is on the edge. The controller opens and releases the door with
//! the door's own command port ([`World::open_door`], [`World::release_door`]).
//!
//! **One robot uses a door at a time** (it "owns" it), first come first served:
//! - A robot asks for the door when its release window would include a guarded
//!   edge. If the door is free the robot owns it and may be released through the
//!   edge; if another robot owns it, the robot is released only up to the start
//!   node of the edge and waits there.
//! - The claim is **tentative**: if the robot cannot be released onto the end
//!   node of the edge (somebody else holds it), the claim is dropped again
//!   ([`DoorDesk::settle`]), so a robot that cannot use the door does not keep
//!   it shut for others.
//! - **Opening:** the controller asks for the door open every cycle while the
//!   robot owns it.
//! - **Passing:** when the robot has reached the end node of the edge, or its job
//!   is over, the door is released and is free for the next robot. The door
//!   itself does not close before the robot has left the edge.
//!
//! Not yet: robots sharing an open door, a route around a door that is too
//! narrow (such a job is refused), two robots going opposite ways through a
//! door in a one-lane corridor (each waits for the other's node: stuck, as at a
//! lift without a lay-by).

use std::collections::HashMap;

use botsim_core::world::World;
use botsim_model::graph::EdgeIdx;

use crate::order_builder::Plan;

/// The robot that owns a door.
#[derive(Debug, Clone)]
struct Owner {
    robot: usize,
    /// Plan index of the guarded edge.
    edge_index: usize,
}

#[derive(Debug)]
pub struct DoorDesk {
    ids: Vec<String>,
    /// Layout edge → door index.
    door_of: HashMap<EdgeIdx, usize>,
    owners: Vec<Option<Owner>>,
}

impl DoorDesk {
    /// The doors of `world`.
    pub fn new(world: &World) -> DoorDesk {
        let specs = world.door_specs();
        let mut door_of = HashMap::new();
        for (d, s) in specs.iter().enumerate() {
            for &e in &s.edges {
                door_of.insert(e, d);
            }
        }
        DoorDesk {
            owners: vec![None; specs.len()],
            ids: specs.into_iter().map(|s| s.id).collect(),
            door_of,
        }
    }

    /// How far the order of `robot` may be released: `wanted` is the last node
    /// the release window reaches when the robot is at plan node `at`, `from` the
    /// last node already released. A guarded edge inside the window needs its
    /// door: the robot gets it if it is free (or is the robot's already);
    /// otherwise the release stops at the start node of the edge.
    pub fn window(
        &mut self,
        robot: usize,
        plan: &Plan,
        at: usize,
        from: usize,
        wanted: usize,
    ) -> usize {
        for (k, &edge) in plan.edges.iter().enumerate().skip(at) {
            let Some(&d) = self.door_of.get(&edge) else {
                continue;
            };
            if wanted <= k {
                return wanted; // the window does not reach the edge yet
            }
            match &self.owners[d] {
                Some(o) if o.robot == robot && o.edge_index == k => {}
                None => {
                    self.owners[d] = Some(Owner {
                        robot,
                        edge_index: k,
                    });
                }
                Some(_) => return k.max(from),
            }
        }
        wanted
    }

    /// The robot has been released through plan node `released_through`: a door
    /// it asked for but could not be released through is given back.
    pub fn settle(&mut self, robot: usize, released_through: usize) {
        for o in &mut self.owners {
            if o.as_ref()
                .is_some_and(|o| o.robot == robot && o.edge_index >= released_through)
            {
                *o = None;
            }
        }
    }

    /// The other robot that owns the door [`DoorDesk::window`] keeps `robot` waiting
    /// for at plan node `from`, the end of its released route (the door of the edge
    /// that starts there); `None` if no door does.
    pub fn held_by(&self, robot: usize, plan: &Plan, from: usize) -> Option<usize> {
        let door = *self.door_of.get(plan.edges.get(from)?)?;
        let owner = self.owners[door].as_ref()?.robot;
        (owner != robot).then_some(owner)
    }

    /// Does a door guard this layout edge?
    pub fn is_guarded(&self, edge: EdgeIdx) -> bool {
        self.door_of.contains_key(&edge)
    }

    /// Does the robot own a door now?
    pub fn owns(&self, robot: usize) -> bool {
        self.owners
            .iter()
            .any(|o| o.as_ref().is_some_and(|o| o.robot == robot))
    }

    /// The robot's order was refused: it gives back any door it was given.
    pub fn forget(&mut self, robot: usize) {
        for o in &mut self.owners {
            if o.as_ref().is_some_and(|o| o.robot == robot) {
                *o = None;
            }
        }
    }

    /// One control cycle. `at[i]` is the plan index of the last node of robot
    /// `i`, `None` if it has no job.
    pub fn update(&mut self, world: &mut World, at: &[Option<usize>]) {
        for d in 0..self.owners.len() {
            let Some(o) = self.owners[d].clone() else {
                continue;
            };
            // passed (at the end node of the edge) or the job is over: let go
            if at[o.robot].is_none_or(|a| a > o.edge_index) {
                // (the doors are the ones the controller found in the world)
                let _ = world.release_door(&self.ids[d]);
                self.owners[d] = None;
            } else {
                let _ = world.open_door(&self.ids[d]);
            }
        }
    }
}

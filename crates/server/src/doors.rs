//! The controller's use of doors (S51).
//!
//! A door guards layout edges (see `docs/rules.md` §7h). A robot does not start
//! such an edge unless the door is fully open, and the door does not close on a
//! robot that is on the edge. The controller opens and releases the door with
//! messages to it ([`DoorCommand::Open`], [`DoorCommand::Release`]).
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

use botsim_link::device::{DeviceHeader, DoorCommand, DoorNo, ToDoor};
use botsim_link::message::Millis;
use botsim_model::graph::EdgeIdx;

use crate::cycle::Message;
use crate::order_builder::Plan;

/// A door, as the controller is told of it.
#[derive(Debug, Clone, PartialEq)]
pub struct Door {
    pub id: String,
    /// The layout edges that lead through the door.
    pub edges: Vec<EdgeIdx>,
}

/// The robot that owns a door.
#[derive(Debug, Clone)]
struct Owner {
    robot: usize,
    /// Plan index of the guarded edge.
    edge_index: usize,
}

#[derive(Debug)]
pub struct DoorDesk {
    /// Layout edge → door index.
    door_of: HashMap<EdgeIdx, usize>,
    owners: Vec<Option<Owner>>,
    /// For each door, the sequence number of the next command to it.
    seqs: Vec<u32>,
}

impl DoorDesk {
    /// The doors of the site.
    pub fn new(doors: &[Door]) -> DoorDesk {
        let mut door_of = HashMap::new();
        for (d, s) in doors.iter().enumerate() {
            for &e in &s.edges {
                door_of.insert(e, d);
            }
        }
        DoorDesk {
            owners: vec![None; doors.len()],
            seqs: vec![0; doors.len()],
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

    /// One control cycle at `time`. `at[i]` is the plan index of the last node of
    /// robot `i`, `None` if it has no job. The commands to the doors go to `out`.
    pub fn update(&mut self, at: &[Option<usize>], time: Millis, out: &mut Vec<Message>) {
        for d in 0..self.owners.len() {
            let Some(o) = self.owners[d].clone() else {
                continue;
            };
            // passed (at the end node of the edge) or the job is over: let go
            let passed = at[o.robot].is_none_or(|a| a > o.edge_index);
            if passed {
                self.owners[d] = None;
            }
            // (a site with more doors than 32 bits can number does not fit in
            // memory)
            let header = DeviceHeader {
                device: DoorNo(u32::try_from(d).unwrap_or(u32::MAX)),
                seq: self.seqs[d],
                time,
            };
            self.seqs[d] = self.seqs[d].wrapping_add(1);
            out.push(Message::Door(ToDoor {
                header,
                command: if passed {
                    DoorCommand::Release
                } else {
                    DoorCommand::Open
                },
            }));
        }
    }
}

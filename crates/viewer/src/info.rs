//! What the side panel says about one robot.

use botsim_record::{LayoutLine, SnapshotLine};

/// A robot at one moment, in words.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotInfo {
    /// `manufacturer/serialNumber`.
    pub id: String,
    /// The status, as its name (`WaitingForRelease`).
    pub status: String,
    /// m/s, negative when driving backwards.
    pub speed: f64,
    /// Empty if the robot never had an order.
    pub order: String,
    /// The node ids; `None` if there is none yet.
    pub last_node: Option<String>,
    /// The nodes ahead that the robot may drive (released), in order.
    pub released: Vec<String>,
    /// The nodes ahead that are not released yet (the horizon), in order.
    pub planned: Vec<String>,
    /// The robots this one waits for (it stands still and they are in its way).
    pub waits_for: Vec<String>,
    /// The robots that wait for this one.
    pub waited_for_by: Vec<String>,
}

fn node_name(layout: &LayoutLine, node: usize) -> String {
    layout
        .nodes
        .get(node)
        .map_or_else(|| format!("#{node}"), |n| n.id.clone())
}

/// The robot's information, or `None` if the snapshot has no robot at `index`.
pub fn describe(layout: &LayoutLine, snapshot: &SnapshotLine, index: usize) -> Option<RobotInfo> {
    let robot = snapshot.robots.get(index)?;
    let id_of = |i: usize| {
        snapshot
            .robots
            .get(i)
            .map_or_else(|| format!("#{i}"), |r| r.id.clone())
    };
    let names = |nodes: &[usize]| nodes.iter().map(|&n| node_name(layout, n)).collect();
    Some(RobotInfo {
        id: robot.id.clone(),
        status: format!("{:?}", robot.status),
        speed: robot.speed,
        order: robot.order.clone(),
        last_node: robot.last_node.map(|n| node_name(layout, n)),
        released: names(&robot.released.nodes),
        planned: names(&robot.planned.nodes),
        waits_for: snapshot
            .blocking
            .iter()
            .filter(|[waiter, _]| *waiter == index)
            .map(|&[_, blocker]| id_of(blocker))
            .collect(),
        waited_for_by: snapshot
            .blocking
            .iter()
            .filter(|[_, blocker]| *blocker == index)
            .map(|&[waiter, _]| id_of(waiter))
            .collect(),
    })
}

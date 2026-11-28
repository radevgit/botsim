//! A site, state reports and cycles written by hand, for a controller with any
//! traffic rules: two robots of one type, no simulator and no VDA 5050 message.

use botsim_link::ids::OrderId;
use botsim_link::message::{FromRobot, Header, Millis, ToRobotBody};
use botsim_link::state::{Ahead, EmergencyStop, LastNode, OperatingMode, Position, Route, State};
use botsim_model::graph::{Graph, NodeIdx};
use botsim_model::job::Job;
use botsim_model::robot::RobotId;
use botsim_model::spec::{Kinematics, Shape};
use botsim_server::controller::Controller;
use botsim_server::cycle::{Config, FleetRobot, Inbox, Message, Outbox, robot_no};
use botsim_server::policy::TrafficPolicy;

/// The site `graph` with two robots of the type ACME/X1.
pub fn config(graph: Graph) -> Config {
    let robot = |serial: &str| FleetRobot {
        id: RobotId::new("ACME", serial),
        series: "X1".into(),
        kinematics: Kinematics::Differential,
        shape: Shape::Rect {
            cx: 0.0,
            cy: 0.0,
            half_length: 0.6,
            half_width: 0.4,
        },
    };
    Config {
        graph,
        robots: vec![robot("R-1"), robot("R-2")],
        zones: None,
        zone_set: None,
        lifts: Vec::new(),
        doors: Vec::new(),
        start: Millis(0),
    }
}

/// The report of robot `robot` on the order `order` (`""`: it never had one): its
/// last node is `route[step]`, the rest of `route` is ahead of it.
pub fn report(
    graph: &Graph,
    robot: usize,
    order: &str,
    route: &[NodeIdx],
    step: usize,
    driving: bool,
) -> FromRobot {
    let node = route[step];
    let ahead = route[step..]
        .windows(2)
        .map(|w| Ahead {
            edge: graph
                .edges
                .iter()
                .position(|e| e.start == w[0] && e.end == w[1])
                .expect("an edge of the layout"),
            node: w[1],
        })
        .collect();
    let last = LastNode {
        node,
        step: u32::try_from(step).unwrap(),
    };
    let at = &graph.nodes[node];
    FromRobot {
        header: Header {
            robot: robot_no(robot),
            seq: 0,
            time: Millis(0),
        },
        state: State {
            order_id: (!order.is_empty()).then(|| OrderId::new(order)),
            update_id: 0,
            route: Route::new(Some(last), ahead, 0).unwrap(),
            position: Some(Position {
                map: at.map,
                x: at.x,
                y: at.y,
                theta: 0.0,
                localized: true,
            }),
            velocity: None,
            driving,
            field_violation: false,
            emergency_stop: EmergencyStop::None,
            operating_mode: OperatingMode::Automatic,
            charge: 100.0,
            charging: false,
            actions: Vec::new(),
            instant_actions: Vec::new(),
            zone_requests: None,
            zone_sets: Vec::new(),
            errors: Vec::new(),
        },
    }
}

/// One cycle at `tick`.
pub fn cycle<P: TrafficPolicy>(
    c: &mut Controller<P>,
    tick: u64,
    states: &[FromRobot],
    jobs: impl IntoIterator<Item = Job>,
) -> Outbox {
    let inbox = Inbox {
        robots: states.to_vec(),
        jobs: jobs.into_iter().collect(),
        ..Inbox::default()
    };
    c.step(Millis(50 * tick as i64), inbox)
}

/// The orders sent: the robot, the update id and the released nodes.
pub fn orders(out: &Outbox) -> Vec<(u32, u32, Vec<NodeIdx>)> {
    out.messages
        .iter()
        .filter_map(|m| {
            let Message::Robot(m) = m else {
                return None;
            };
            let ToRobotBody::Order(order) = &m.body else {
                return None;
            };
            let released = order.legs().iter().take(order.released() as usize);
            let nodes = std::iter::once(order.start())
                .chain(released.map(|leg| leg.end))
                .collect();
            Some((m.header.robot.slot(), order.update_id(), nodes))
        })
        .collect()
}

pub fn job(id: u64, pickup: NodeIdx, drop: NodeIdx) -> Job {
    Job {
        id,
        pickup,
        drop,
        created: Millis(0),
    }
}

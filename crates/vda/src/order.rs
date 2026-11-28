//! `order`.
//!
//! The refusals of an incoming order are those a VDA 5050 robot makes before it
//! looks at the layout and at itself: structure, `sequenceId`s and release rules
//! (`VALIDATION_FAILURE`), and names the layout does not have
//! (`NO_ROUTE_TO_TARGET`). Whether the edges connect the nodes, whether the robot
//! may use them and whether the order fits what it is doing is left to the robot.

use botsim_link::ids::{ActionId, ActionType, OrderId};
use botsim_link::message::{Header, ToRobot, ToRobotBody};
use botsim_link::order::{Action, Blocking, Leg, NodeAction, Order, OrientationType};
use botsim_model::errors::ErrorType;
use vda5050::order as vda;

use crate::header::{Incoming, angle_to_vda, finite, usable};
use crate::report::{Issue, Problem, Translated};
use crate::translator::Translator;

/// Why an order is refused: the problem and the error type that goes with it.
type Refusal = (Problem, ErrorType);

/// The problem makes the order invalid.
fn invalid(problem: Problem) -> Refusal {
    (problem, ErrorType::ValidationFailure)
}

/// The problem is a part of the layout that does not exist.
fn unreachable(problem: Problem) -> Refusal {
    (problem, ErrorType::NoRouteToTarget)
}

/// The refusal of the order `order_id`, with the references the standard names.
fn refuse((problem, error): Refusal, order_id: &str, update_id: u32) -> Issue {
    let mut issue = Issue::new(problem).with_error(error, "orderId", order_id);
    if error.references_order_update_id() {
        issue
            .references
            .push(("orderUpdateId".into(), update_id.to_string().into()));
    }
    issue
}

impl Translator {
    pub(crate) fn order_to_vda(
        &self,
        header: &Header,
        order: &Order,
    ) -> Result<Translated<vda::Order>, Issue> {
        let order_id = order.order_id().as_str();
        let update_id = order.update_id();
        // (a robot that is not there refuses nothing)
        let head = self.outgoing(header).map_err(|problem| match problem {
            Problem::BadTime { .. } => refuse(invalid(problem), order_id, update_id),
            _ => Issue::new(problem),
        })?;
        let mut issues = Vec::new();
        let (nodes, edges) = self
            .nodes_and_edges(order, &mut issues)
            .map_err(|r| refuse(r, order_id, update_id))?;
        Ok(Translated {
            message: vda::Order {
                header_id: head.header_id,
                timestamp: head.timestamp,
                version: head.version,
                manufacturer: head.manufacturer,
                serial_number: head.serial_number,
                order_id: order_id.to_string(),
                order_update_id: update_id,
                order_description: None,
                nodes,
                edges,
            },
            issues,
        })
    }

    /// The nodes and edges of the message; changed angles are added to `issues`.
    fn nodes_and_edges(
        &self,
        order: &Order,
        issues: &mut Vec<Issue>,
    ) -> Result<(Vec<vda::Node>, Vec<vda::Edge>), Refusal> {
        usable(order.order_id().as_str(), "orderId").map_err(invalid)?;
        let first = 2 * order.first_step();
        let released = order.released() as usize;

        let node = |i: usize, number| -> Result<vda::Node, Refusal> {
            // (`Order::new` has checked that the numbers of the order's nodes fit)
            let at = i as u32;
            let actions = order
                .actions_at(at)
                .iter()
                .map(|a| action_to_vda(&a.action))
                .collect::<Result<Vec<_>, _>>()
                .map_err(invalid)?;
            Ok(vda::Node {
                node_id: self.node_name(number).map_err(unreachable)?.to_string(),
                sequence_id: first + 2 * at,
                node_descriptor: None,
                released: i <= released,
                node_position: None,
                actions,
            })
        };

        let mut nodes = vec![node(0, order.start())?];
        let mut edges = Vec::with_capacity(order.legs().len());
        for (i, leg) in order.legs().iter().enumerate() {
            let edge_id = self.edge_name(leg.edge).map_err(unreachable)?.to_string();
            let orientation = match leg.orientation {
                None => None,
                Some(angle) => {
                    let (angle, changed) =
                        angle_to_vda(angle, "edges[].orientation").map_err(invalid)?;
                    issues.extend(changed.map(|p| Issue::new(p).at(i)));
                    Some(angle)
                }
            };
            edges.push(vda::Edge {
                edge_id,
                sequence_id: first + 2 * i as u32 + 1,
                edge_descriptor: None,
                released: i < released,
                maximum_speed: None,
                maximum_mobile_robot_height: None,
                minimum_load_handling_device_height: None,
                orientation,
                orientation_type: leg.orientation_type.map(orientation_type_to_vda),
                direction: None,
                reach_orientation_before_entering: None,
                maximum_rotation_speed: None,
                trajectory: None,
                length: None,
                corridor: None,
                actions: Vec::new(),
            });
            nodes.push(node(i + 1, leg.end)?);
        }
        Ok((nodes, edges))
    }

    /// What the order `message` says, as a robot receives it. The `Err` is why a
    /// robot would refuse it (or why it is for no robot here).
    pub fn order_from_vda(&self, message: &vda::Order) -> Result<Translated<ToRobot>, Issue> {
        let order_id = message.order_id.as_str();
        let update_id = message.order_update_id;
        let header = self
            .incoming(Incoming {
                header_id: message.header_id,
                timestamp: &message.timestamp,
                version: &message.version,
                manufacturer: &message.manufacturer,
                serial_number: &message.serial_number,
            })
            .map_err(|problem| match problem {
                Problem::UnknownRobot(_) => Issue::new(problem),
                _ => refuse(invalid(problem), order_id, update_id),
            })?;
        let order = self
            .order_of(message)
            .map_err(|r| refuse(r, order_id, update_id))?;
        Ok(Translated {
            message: ToRobot {
                header,
                body: ToRobotBody::Order(order),
            },
            issues: Vec::new(),
        })
    }

    /// The order that the message says.
    fn order_of(&self, message: &vda::Order) -> Result<Order, Refusal> {
        let (nodes, edges) = (&message.nodes, &message.edges);
        usable(&message.order_id, "orderId").map_err(invalid)?;
        let Some(first_node) = nodes.first() else {
            return Err(invalid(Problem::WrongCounts {
                nodes: 0,
                edges: edges.len(),
            }));
        };
        if edges.len() != nodes.len() - 1 {
            return Err(invalid(Problem::WrongCounts {
                nodes: nodes.len(),
                edges: edges.len(),
            }));
        }
        check_sequence_ids(nodes, edges).map_err(invalid)?;
        check_release(nodes, edges).map_err(invalid)?;

        let start = self.node_number(&first_node.node_id).map_err(unreachable)?;
        let mut legs = Vec::with_capacity(edges.len());
        for (e, end) in edges.iter().zip(&nodes[1..]) {
            let end = self.node_number(&end.node_id).map_err(unreachable)?;
            let edge = self.edge_number(&e.edge_id).map_err(unreachable)?;
            let orientation = e
                .orientation
                .map(|a| finite(a, "edges[].orientation"))
                .transpose()
                .map_err(invalid)?;
            legs.push(Leg {
                edge,
                end,
                orientation,
                orientation_type: e.orientation_type.map(orientation_type_from_vda),
            });
        }
        let mut actions = Vec::new();
        for (i, n) in nodes.iter().enumerate() {
            for a in &n.actions {
                actions.push(NodeAction {
                    // (more nodes than that cannot have passed the sequence ids)
                    node: i as u32,
                    action: action_from_vda(a).map_err(invalid)?,
                });
            }
        }
        // (the release rules give: an edge is released if and only if the node it
        // ends at is, and those are the first ones)
        let released = edges.iter().take_while(|e| e.released).count() as u32;
        Order::new(
            OrderId::new(message.order_id.as_str()),
            message.order_update_id,
            first_node.sequence_id / 2,
            start,
            legs,
            released,
            actions,
        )
        .map_err(|e| invalid(Problem::Shape(e)))
    }
}

/// The nodes have the numbers `first`, `first + 2`, …, where `first` is even, and
/// the edges the numbers between.
fn check_sequence_ids(nodes: &[vda::Node], edges: &[vda::Edge]) -> Result<(), Problem> {
    let first = nodes.first().map_or(0, |n| u64::from(n.sequence_id));
    // a node's number is even: an odd first one is taken as one too low
    let first = first - first % 2;
    for (i, n) in nodes.iter().enumerate() {
        let expected = first + 2 * i as u64;
        if u64::from(n.sequence_id) != expected {
            return Err(Problem::BadSequenceId {
                field: "nodes[].sequenceId",
                id: n.node_id.clone(),
                expected,
                found: n.sequence_id,
            });
        }
    }
    for (i, e) in edges.iter().enumerate() {
        let expected = first + 2 * i as u64 + 1;
        if u64::from(e.sequence_id) != expected {
            return Err(Problem::BadSequenceId {
                field: "edges[].sequenceId",
                id: e.edge_id.clone(),
                expected,
                found: e.sequence_id,
            });
        }
    }
    Ok(())
}

/// The release rules of VDA 5050 §6.1. `edges` is one shorter than `nodes`.
fn check_release(nodes: &[vda::Node], edges: &[vda::Edge]) -> Result<(), Problem> {
    if !nodes.first().is_some_and(|n| n.released) {
        return Err(Problem::FirstNodeNotReleased);
    }
    let mut horizon_started = false;
    for (e, ends) in edges.iter().zip(nodes.windows(2)) {
        let (a, b) = (&ends[0], &ends[1]);
        if e.released && !(a.released && b.released) {
            return Err(Problem::EdgeReleasedWithoutNode {
                edge: e.edge_id.clone(),
            });
        }
        horizon_started |= !e.released;
        if horizon_started && b.released {
            return Err(Problem::ReleasedAfterHorizon {
                id: b.node_id.clone(),
            });
        }
    }
    Ok(())
}

fn action_to_vda(a: &Action) -> Result<vda::Action, Problem> {
    Ok(vda::Action {
        action_type: usable(a.kind.as_str(), "actions[].actionType")?.to_string(),
        action_id: usable(a.id.as_str(), "actions[].actionId")?.to_string(),
        action_descriptor: None,
        blocking_type: blocking_to_vda(a.blocking),
        action_parameters: None,
        retriable: None,
    })
}

fn action_from_vda(a: &vda::Action) -> Result<Action, Problem> {
    Ok(Action {
        id: ActionId::new(usable(&a.action_id, "actions[].actionId")?),
        kind: ActionType::new(usable(&a.action_type, "actions[].actionType")?),
        blocking: blocking_from_vda(a.blocking_type),
    })
}

pub(crate) fn blocking_to_vda(b: Blocking) -> vda::BlockingType {
    match b {
        Blocking::None => vda::BlockingType::None,
        Blocking::Soft => vda::BlockingType::Soft,
        Blocking::Single => vda::BlockingType::Single,
        Blocking::Hard => vda::BlockingType::Hard,
    }
}

pub(crate) fn blocking_from_vda(b: vda::BlockingType) -> Blocking {
    match b {
        vda::BlockingType::None => Blocking::None,
        vda::BlockingType::Soft => Blocking::Soft,
        vda::BlockingType::Single => Blocking::Single,
        vda::BlockingType::Hard => Blocking::Hard,
    }
}

fn orientation_type_to_vda(t: OrientationType) -> vda::OrientationType {
    match t {
        OrientationType::Global => vda::OrientationType::Global,
        OrientationType::Tangential => vda::OrientationType::Tangential,
    }
}

fn orientation_type_from_vda(t: vda::OrientationType) -> OrientationType {
    match t {
        vda::OrientationType::Global => OrientationType::Global,
        vda::OrientationType::Tangential => OrientationType::Tangential,
    }
}

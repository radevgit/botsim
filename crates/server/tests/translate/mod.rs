//! Translation between VDA 5050 and the controller's messages, for the tests that
//! look at VDA 5050 messages.

use std::sync::Arc;

use botsim_link::message::ToRobot;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_server::cycle::robot_no;
use botsim_vda::translator::{RobotMessage, Translator};
use vda5050::order::Order;

/// A translator for the layout `graph` and a fleet of the one robot `robot`.
pub fn translator(graph: &Graph, robot: &RobotId) -> Translator {
    let mut translator = Translator::new(Arc::new(graph.clone()));
    translator.add_robot(robot_no(0), robot.clone()).unwrap();
    translator
}

/// The VDA 5050 `order` that says all that `message` to `robot` says.
pub fn vda_order(graph: &Graph, robot: &RobotId, message: &ToRobot) -> Order {
    let out = translator(graph, robot).to_robot_to_vda(message).unwrap();
    assert!(out.issues.is_empty(), "{:?}", out.issues);
    let RobotMessage::Order(order) = out.message else {
        panic!("not an order: {:?}", out.message);
    };
    order
}

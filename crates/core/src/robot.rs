//! A simulated robot as the controller sees it: it receives VDA 5050 orders,
//! accepts or rejects them, executes them, and reports its state.
//!
//! Order acceptance follows VDA 5050 §6.1.2 (Figure 8), in this order:
//! 1. the order must be valid (structure, release rules, ids in the layout);
//! 2. a **different `orderId`** is a new order:
//!    - the robot must be idle (order done, and no horizon to wait for), else
//!      `OTHER_ORDER_ACTIVE`;
//!    - `orderUpdateId` must be 0 (the standard names no error: `VALIDATION_FAILURE`);
//!    - the first node must be where the robot stands, else `START_NODE_OUT_OF_RANGE`;
//! 3. the **same `orderId`** is an update: a lower `orderUpdateId` →
//!    `OUTDATED_ORDER_UPDATE`, after a `cancelOrder` → `ORDER_UPDATE_FOLLOWING_CANCEL`,
//!    the same → `SAME_ORDER_UPDATE_ID`, higher → it must start at the decision
//!    point (the end of the base so far).
//!
//! A rejected message changes nothing except that its error is reported in the
//! state ("until new order is accepted"); an accepted order or update clears the
//! errors.
//!
//! Instant actions: only `cancelOrder` (see [`Robot::receive_instant_actions`]).
//!
//! Not yet: operating modes (`MOBILE_ROBOT_NOT_AVAILABLE`), other instant actions,
//! a deviation range around the start node (a fixed tolerance is used).

use vda5050::instant_actions::InstantActions;
use vda5050::order::Order;
use vda5050::state::{ActionStatus, State};

use crate::actions::ActionTimes;
use crate::drive::{DriveError, Driver, Phase, Pose};
use crate::errors::{ErrorType, RobotError};
use crate::graph::{Graph, NodeIdx};
use crate::order_route::route_from_order;
use crate::spec::RobotSpec;
use crate::state_report::{Header, InstantActionState, build_state};

/// How close (metres) the robot must be to the first node of a new order.
pub const START_NODE_TOLERANCE: f64 = 0.1;

/// What happened to a received order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reception {
    Accepted,
    /// Rejected; the error is in the robot's error list.
    Rejected(ErrorType),
}

#[derive(Debug, Clone)]
pub struct Robot {
    spec: RobotSpec,
    times: ActionTimes,
    driver: Driver,
    errors: Vec<RobotError>,
    /// All instant actions received, with their status (kept, as the standard says).
    instant: Vec<InstantActionState>,
    /// Index in `instant` of a `cancelOrder` that is still stopping the robot.
    cancel_action: Option<usize>,
}

impl Robot {
    /// An idle robot standing on `start` with `heading`.
    pub fn new(
        graph: &Graph,
        spec: RobotSpec,
        times: ActionTimes,
        start: NodeIdx,
        heading: f64,
    ) -> Result<Robot, DriveError> {
        let driver = Driver::new(graph, &spec, start, heading, &[])?;
        Ok(Robot {
            spec,
            times,
            driver,
            errors: Vec::new(),
            instant: Vec::new(),
            cancel_action: None,
        })
    }

    pub fn pose(&self) -> Pose {
        self.driver.pose()
    }

    /// The body outline, relative to the kinematic center.
    pub fn shape(&self) -> &crate::spec::Shape {
        &self.spec.shape
    }

    pub fn errors(&self) -> &[RobotError] {
        &self.errors
    }

    /// Advance by `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        self.driver.step(dt);
        self.update_cancel();
    }

    /// The state message.
    pub fn state(&self, graph: &Graph, header: &Header) -> State {
        build_state(
            graph,
            header,
            &self.driver.progress(),
            &self.errors,
            &self.instant,
        )
    }

    fn reject(&mut self, error: RobotError) -> Reception {
        let t = error.error_type;
        self.errors.push(error);
        Reception::Rejected(t)
    }

    fn reject_order(&mut self, t: ErrorType, order: &Order, text: &str) -> Reception {
        let e = RobotError::for_order(t, &order.order_id, order.order_update_id, text.to_string());
        self.reject(e)
    }

    /// Receive an `order` message.
    pub fn receive_order(&mut self, graph: &Graph, order: &Order) -> Reception {
        // 1. valid?
        let route = match route_from_order(graph, order) {
            Ok(r) => r,
            Err(e) => {
                let err = RobotError::from_order_error(&e, &order.order_id, order.order_update_id);
                return self.reject(err);
            }
        };

        let current = self.driver.order().map(|(id, u)| (id.to_string(), u));
        let is_update = current
            .as_ref()
            .is_some_and(|(id, _)| *id == order.order_id);

        if !is_update {
            // 2. a new order
            if self.driver.phase() != Phase::Finished {
                return self.reject_order(
                    ErrorType::OtherOrderActive,
                    order,
                    "another order is still active",
                );
            }
            if order.order_update_id != 0 {
                return self.reject_order(
                    ErrorType::ValidationFailure,
                    order,
                    "a new order must have orderUpdateId 0",
                );
            }
            let p = self.driver.pose();
            let start = &graph.nodes[route.start];
            let near =
                p.map == start.map && (p.x - start.x).hypot(p.y - start.y) <= START_NODE_TOLERANCE;
            if !near {
                return self.reject_order(
                    ErrorType::StartNodeOutOfRange,
                    order,
                    "the first node of the order is not where the robot stands",
                );
            }
            return match Driver::from_order_route(
                graph,
                &self.spec,
                p.heading,
                &route,
                self.times.clone(),
            ) {
                Ok(d) => {
                    self.driver = d;
                    self.errors.clear();
                    Reception::Accepted
                }
                Err(e) => {
                    let err =
                        RobotError::from_drive_error(&e, &order.order_id, order.order_update_id);
                    self.reject(err)
                }
            };
        }

        // 3. an update of the current order
        let (_, current_update) = current.expect("is_update implies a current order");
        if order.order_update_id < current_update {
            return self.reject_order(
                ErrorType::OutdatedOrderUpdate,
                order,
                "orderUpdateId is lower than the current one",
            );
        }
        if self.driver.is_canceled() {
            return self.reject_order(
                ErrorType::OrderUpdateFollowingCancel,
                order,
                "the order was cancelled; no updates are accepted",
            );
        }
        if order.order_update_id == current_update {
            return self.reject_order(
                ErrorType::SameOrderUpdateId,
                order,
                "orderUpdateId is the same as the current one",
            );
        }
        match self.driver.extend(graph, &route) {
            Ok(()) => {
                self.errors.clear();
                Reception::Accepted
            }
            Err(e) => {
                let err = RobotError::from_drive_error(&e, &order.order_id, order.order_update_id);
                self.reject(err)
            }
        }
    }

    /// Receive an `instantActions` message. Every action is listed in
    /// `instantActionStates`. Supported: `cancelOrder` (optional parameter
    /// `orderId`). It fails (`FAILED` + `NO_ORDER_TO_CANCEL`, reference `actionId`)
    /// if the robot is idle or the `orderId` is not the active order. Any other
    /// type fails with `INVALID_INSTANT_ACTION`.
    pub fn receive_instant_actions(&mut self, msg: &InstantActions) {
        for a in &msg.actions {
            self.instant.push(InstantActionState {
                action_id: a.action_id.clone(),
                action_type: a.action_type.clone(),
                status: ActionStatus::Failed,
            });
            let idx = self.instant.len() - 1;

            if a.action_type != "cancelOrder" {
                self.errors.push(action_error(
                    ErrorType::InvalidInstantAction,
                    &a.action_id,
                    "unsupported instant action",
                ));
                continue;
            }

            let wanted = a
                .action_parameters
                .iter()
                .flatten()
                .find(|p| p.key == "orderId")
                .and_then(|p| p.value.as_str());
            let active = self.driver.phase() != Phase::Finished;
            let matches = match (wanted, self.driver.order()) {
                (Some(w), Some((id, _))) => w == id,
                (Some(_), None) => false,
                (None, _) => true,
            };
            if !active || !matches {
                self.errors.push(action_error(
                    ErrorType::NoOrderToCancel,
                    &a.action_id,
                    "no active order to cancel",
                ));
                continue;
            }

            self.driver.cancel();
            self.instant[idx].status = ActionStatus::Running;
            self.cancel_action = Some(idx);
            self.update_cancel();
        }
    }

    /// The `cancelOrder` action is FINISHED once all movement and actions have stopped.
    fn update_cancel(&mut self) {
        if let Some(idx) = self.cancel_action
            && self.driver.phase() == Phase::Finished
        {
            self.instant[idx].status = ActionStatus::Finished;
            self.cancel_action = None;
        }
    }
}

fn action_error(t: ErrorType, action_id: &str, text: &str) -> RobotError {
    RobotError {
        error_type: t,
        references: vec![("actionId".to_string(), action_id.to_string())],
        description: Some(text.to_string()),
    }
}

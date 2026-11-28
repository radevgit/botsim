//! The traffic module (`botsim-traffic`) behind the server's slot for the traffic
//! rules ([`crate::policy`]); `botsim --policy traffic` uses it. This adapter only
//! translates: who may go now is decided by the module.
//!
//! **Robots.** The controller numbers them in the order of its fleet; the module in
//! the order it gets them. A robot goes into the module standing on its node, the
//! first time the controller says it has no job.
//!
//! **Conflicts.** The module orders every conflict of the map, so the adapter
//! computes the whole map itself, from the layout and the robot types it is given,
//! and leaves aside the lists given to the policy
//! ([`TrafficPolicy::conflicts`]). A robot whose safety stop is on also waits for
//! one its protective field reaches; with [`MapOptions::protective_field`] the map
//! counts what the field of a standing robot reaches along an edge, so that the
//! order covers that wait.
//!
//! **Progress.** The controller's calls carry no order id. When a robot gets a
//! route, its state still names the order before; such states are not about the new
//! route and are skipped.
//!
//! **Routes.** The module takes as much of a route as it can order and, in every
//! release, more when it can. While it can take nothing, it asks for another way,
//! as it does for a robot on a ring of waiting robots. The controller can only be
//! told nodes to keep clear of, so an edge to avoid is given as the node it ends
//! at.
//!
//! **Who holds a robot back** is what the module says. At the end of what is taken
//! of its route, it is who holds its next node, where somebody does. Else it is
//! who is ordered before it at its next node, or who keeps the rest from being
//! taken; of several, the one with the lowest module number is named.
//!
//! Built so far: plain job runs. Not yet: routes that are given up (cancel), RELEASE
//! zones (no request is answered), the holder queries (`node_holder`,
//! `edge_holder`) and the reservations for a watcher.

use std::collections::{BTreeMap, HashSet};

use botsim_link::ids::OrderId;
use botsim_model::graph::NodeIdx;
use botsim_space::{MapOptions, TypeRef};
use botsim_traffic::{NotAdded, Piece, Report, Route, Spots, Traffic};
use vdma_lif::Lif;

use crate::order_builder::Plan;
use crate::policy::{Ask, Fleet, Release, Reroute, Setup, TrafficPolicy, ZoneAnswer};

/// The traffic module as a traffic policy.
#[derive(Debug)]
pub struct TrafficAdapter {
    traffic: Traffic,
    /// The layout's node ids, by node index.
    nodes: Vec<String>,
    /// The layout's edges by edge index: id and the node it ends at.
    edges: Vec<(String, NodeIdx)>,
    node_by_id: BTreeMap<String, NodeIdx>,
    edge_by_id: BTreeMap<String, NodeIdx>,
    /// By controller number: the type, `manufacturer/seriesName`.
    types: Vec<String>,
    /// By controller number; `None` for a robot the module does not have.
    robots: Vec<Option<Robot>>,
}

/// What the adapter knows of a robot of the module.
#[derive(Debug)]
struct Robot {
    /// Its number in the module.
    number: usize,
    /// The order id its state showed when it got its route (`Some(None)`: it
    /// showed none); `None` before it has one. States that show it are not about
    /// the route.
    order_before: Option<Option<OrderId>>,
    /// A new order the module has not started: it does not have the robot at the
    /// end of the order before.
    pending: Option<Route>,
}

impl TrafficAdapter {
    /// For the layout `lif` and the robot types of the fleet: computes their
    /// conflict map as `options` says. Of several types of one name the first
    /// counts. The error says why no map can be made.
    pub fn new(
        lif: &Lif,
        types: &[TypeRef],
        options: &MapOptions,
    ) -> Result<TrafficAdapter, String> {
        let mut named: Vec<TypeRef> = Vec::new();
        for t in types {
            if !named.iter().any(|x| x.name == t.name) {
                named.push(t.clone());
            }
        }
        let map = botsim_space::compute(lif, &named, options)
            .ok_or("no conflict map: the layout cannot be turned into a graph")?;
        let spots = Spots::new(lif, &map).map_err(|e| e.to_string())?;
        Ok(TrafficAdapter {
            traffic: Traffic::new(spots),
            nodes: Vec::new(),
            edges: Vec::new(),
            node_by_id: BTreeMap::new(),
            edge_by_id: BTreeMap::new(),
            types: Vec::new(),
            robots: Vec::new(),
        })
    }

    /// The route of a robot in the module's terms; `None` if the plan names what
    /// the layout does not have.
    fn route(&self, robot: usize, plan: &Plan) -> Option<Route> {
        let nodes: Option<Vec<&str>> = plan
            .nodes
            .iter()
            .map(|&n| self.nodes.get(n).map(String::as_str))
            .collect();
        let edges: Option<Vec<&str>> = plan
            .edges
            .iter()
            .map(|&e| self.edges.get(e).map(|(id, _)| id.as_str()))
            .collect();
        Route::new(self.types.get(robot)?, &nodes?, &edges?)
    }

    /// The nodes the controller shall keep clear of; `None` if there is none to
    /// name.
    fn avoid(&self, wish: &botsim_traffic::Reroute) -> Option<Reroute> {
        let avoid: HashSet<NodeIdx> = wish
            .avoid
            .iter()
            .filter_map(|piece| match piece {
                Piece::Node(id) => self.node_by_id.get(id),
                Piece::Edge(id) => self.edge_by_id.get(id),
            })
            .copied()
            .collect();
        (!avoid.is_empty()).then_some(Reroute { avoid })
    }

    /// Put a robot into the module with this route.
    fn enter(&mut self, robot: usize, route: Route, order_before: Option<Option<OrderId>>) {
        let number = match self.traffic.add(route) {
            Ok(number) => number,
            // nothing beyond its first node can be taken yet
            Err(NotAdded::Standing { robot, .. }) => robot,
            // its type or first node is unknown: it gets nothing released
            Err(_) => return,
        };
        if let Some(slot) = self.robots.get_mut(robot) {
            *slot = Some(Robot {
                number,
                order_before,
                pending: None,
            });
        }
    }

    /// Give the module the new order it has not started, if there is one.
    fn offer(&mut self, robot: usize) {
        let Some(Some(known)) = self.robots.get_mut(robot) else {
            return;
        };
        let Some(route) = known.pending.clone() else {
            return;
        };
        // with a "re-route" it is on the new order too, holding its node
        if let Ok(()) | Err(NotAdded::Reroute(_)) = self.traffic.new_order(known.number, route) {
            known.pending = None;
        }
    }

    /// Offer what is pending, then ask the module how far the robot may go.
    fn answer(&mut self, ask: &Ask) -> Release {
        self.offer(ask.robot);
        let wait = Release {
            through: ask.released_through,
            reroute: None,
        };
        let Some(Some(robot)) = self.robots.get(ask.robot) else {
            return wait;
        };
        if robot.pending.is_some() {
            return wait;
        }
        match self.traffic.release(robot.number, ask.limit) {
            Some(release) => Release {
                through: release.through,
                reroute: release.reroute.and_then(|wish| self.avoid(&wish)),
            },
            None => wait,
        }
    }
}

impl TrafficPolicy for TrafficAdapter {
    fn start(&mut self, setup: &Setup) {
        let graph = setup.graph;
        self.nodes = graph.nodes.iter().map(|n| n.id.clone()).collect();
        self.edges = graph.edges.iter().map(|e| (e.id.clone(), e.end)).collect();
        self.node_by_id = self.nodes.iter().cloned().zip(0..).collect();
        self.edge_by_id = self.edges.iter().cloned().collect();
        self.types = setup
            .robots
            .iter()
            .map(|r| setup.types.get(r.type_idx).cloned().unwrap_or_default())
            .collect();
        self.robots = setup.robots.iter().map(|_| None).collect();
    }

    fn cycle_start(&mut self, fleet: &Fleet) {
        let reports: Vec<Report> = self
            .robots
            .iter()
            .zip(fleet.states)
            .filter_map(|(robot, report)| {
                let state = &report.state;
                let robot = robot.as_ref()?;
                let before = robot.order_before.as_ref()?;
                // the module does not have the new order yet, or the state is not
                // about it
                if robot.pending.is_some() || state.order_id == *before {
                    return None;
                }
                Some(Report {
                    robot: robot.number,
                    reached: state.last_step() as usize,
                    released: self.traffic.graph().released(robot.number)?,
                })
            })
            .collect();
        // a report that cannot be is left out by the module
        self.traffic.cycle_start(&reports);
    }

    fn place(&mut self, ask: &Ask) -> Release {
        let Some(route) = self.route(ask.robot, ask.route) else {
            return Release {
                through: ask.released_through,
                reroute: None,
            };
        };
        let before = ask.state.order_id.clone();
        match self.robots.get_mut(ask.robot) {
            Some(Some(robot)) => {
                robot.order_before = Some(before);
                robot.pending = Some(route);
            }
            _ => self.enter(ask.robot, route, Some(before)),
        }
        self.answer(ask)
    }

    fn release(&mut self, ask: &Ask) -> Release {
        self.answer(ask)
    }

    fn route_changed(&mut self, robot: usize, route: &Plan) {
        let Some(route) = self.route(robot, route) else {
            return;
        };
        let Some(Some(known)) = self.robots.get_mut(robot) else {
            return;
        };
        if known.pending.is_some() {
            known.pending = Some(route);
            self.offer(robot);
        } else {
            // what the module cannot take now it keeps, and says in the releases
            let _ = self.traffic.change(known.number, route);
        }
    }

    fn idle(&mut self, robot: usize, node: NodeIdx) {
        if matches!(self.robots.get(robot), Some(Some(_))) {
            return;
        }
        let (Some(type_name), Some(node)) = (self.types.get(robot), self.nodes.get(node)) else {
            return;
        };
        if let Some(stands) = Route::new(type_name, &[node.as_str()], &[]) {
            self.enter(robot, stands, None);
        }
    }

    fn zone_answers(&mut self, _fleet: &Fleet) -> Vec<ZoneAnswer> {
        Vec::new()
    }

    fn blocker(&self, robot: usize, _route: &Plan, _released_through: usize) -> Option<usize> {
        let known = self.robots.get(robot)?.as_ref()?;
        let held_back = self.traffic.blockers(known.number);
        // at the end of the part taken: who keeps the rest from being taken
        let others = if held_back.is_empty() {
            self.traffic.refused_by(known.number)
        } else {
            held_back
        };
        let holder = self.traffic.next_holder(known.number);
        let other = holder.or_else(|| others.into_iter().find(|&o| o != known.number))?;
        self.robots
            .iter()
            .position(|r| r.as_ref().is_some_and(|r| r.number == other))
    }
}

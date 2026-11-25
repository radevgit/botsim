//! `--policy traffic`: the traffic module (`botsim-traffic`) behind the policy slot
//! of the controller ([`botsim_baseline::policy`]). This adapter only translates:
//! who may go now is decided by the module.
//!
//! **Robots.** The controller numbers them as in the world; the module in the order
//! it gets them. A robot goes into the module standing on its node, the first time
//! the controller says it has no job.
//!
//! **Conflicts.** The module orders every conflict of the map, so the adapter
//! computes the whole map itself and leaves aside the shorter lists the runner makes
//! for the baseline.
//!
//! **Progress.** The controller's calls carry no order id. When a robot gets a
//! route, its state still names the order before; such states are not about the new
//! route and are skipped.
//!
//! **A route the module does not take** is offered again when it changes and in
//! every release; meanwhile nothing more is released and the dispatcher is asked
//! for another way. The dispatcher can only be told nodes to keep clear of, so an
//! edge to avoid is given as the node it ends at.
//!
//! **Who holds a robot back** is what the module says: who is ordered before it at
//! its next node, or, where its route or the rest of it is refused, who keeps it
//! from being taken. Of several, the one with the lowest module number is named.
//!
//! **Experiments**, not part of the design, each on only with its environment
//! variable set:
//! - `BOTSIM_TRAFFIC_WAIT_BEHIND` (T36): a route may wait behind a standing robot
//!   instead of being refused ([`Traffic::wait_behind_standing`]);
//! - `BOTSIM_TRAFFIC_PARTLY` (T38): a route is taken as far as it can be
//!   ([`Traffic::take_routes_partly`]) and the rest offered again in every release.
//!
//! Built so far: plain job runs. Not yet: routes that are given up (cancel), RELEASE
//! zones (no request is answered), the dispatcher's holder queries and the
//! reservations for a watcher.

use std::collections::{BTreeMap, HashSet};
use std::path::Path;

use botsim_baseline::order_builder::Plan;
use botsim_baseline::policy::{Ask, Fleet, Release, Reroute, Setup, TrafficPolicy};
use botsim_baseline::zone_grants::Answer;
use botsim_core::graph::NodeIdx;
use botsim_sweep::{MapOptions, TypeRef};
use botsim_traffic::{NotAdded, Piece, Report, Route, Spots, Traffic};

use crate::scenario::Scenario;

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
    /// The order id its state showed when it got its route; `None` before it has
    /// one. States that show it are not about the route.
    order_before: Option<String>,
    /// The route the module has not taken yet, or not all of.
    pending: Option<Pending>,
}

/// A route to offer to the module.
#[derive(Debug, Clone)]
enum Pending {
    /// A new order.
    Order(Route),
    /// A change of the order the module has.
    Change(Route),
    /// The whole route of which the module has taken the first part (T38).
    Rest(Route),
}

/// What is still to be offered of a route the module has just taken: nothing,
/// unless it took only a part.
fn rest(traffic: &Traffic, number: usize, route: Route) -> Option<Pending> {
    let end = route.pieces().len() / 2;
    (traffic.route_end(number) < Some(end)).then_some(Pending::Rest(route))
}

impl TrafficAdapter {
    /// For a scenario read from `scenario_file`: reads its layout and factsheets
    /// again and computes the conflict map of the fleet.
    pub fn new(scenario: &Scenario, scenario_file: &Path) -> Result<TrafficAdapter, String> {
        let base = scenario_file.parent().unwrap_or(Path::new("."));
        let read = |path: &Path| {
            let path = base.join(path);
            std::fs::read_to_string(&path)
                .map_err(|e| format!("cannot read {}: {e}", path.display()))
        };
        let lif: vdma_lif::Lif = serde_json::from_str(&read(&scenario.lif)?)
            .map_err(|e| format!("{}: {e}", scenario.lif.display()))?;
        let mut types: Vec<TypeRef> = Vec::new();
        for group in &scenario.robots {
            let file = group.factsheet.display();
            let factsheet: vda5050::factsheet::Factsheet =
                serde_json::from_str(&read(&group.factsheet)?)
                    .map_err(|e| format!("{file}: {e}"))?;
            let t = TypeRef::from_factsheet(&factsheet).map_err(|e| format!("{file}: {e}"))?;
            if !types.iter().any(|x| x.name == t.name) {
                types.push(t);
            }
        }
        let options = MapOptions {
            clearance: scenario.conflict_clearance,
            ..MapOptions::default()
        };
        let map = botsim_sweep::compute(&lif, &types, &options)
            .ok_or("no conflict map: the layout cannot be turned into a graph")?;
        let spots = Spots::new(&lif, &map).map_err(|e| e.to_string())?;
        let mut traffic = Traffic::new(spots);
        if std::env::var_os("BOTSIM_TRAFFIC_WAIT_BEHIND").is_some() {
            traffic.wait_behind_standing();
        }
        if std::env::var_os("BOTSIM_TRAFFIC_PARTLY").is_some() {
            traffic.take_routes_partly();
        }
        Ok(TrafficAdapter {
            traffic,
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

    /// The nodes the dispatcher shall keep clear of; `None` if there is none to
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

    /// Put a robot into the module with this route. If the module does not take the
    /// route, it keeps the robot on its first node.
    fn enter(&mut self, robot: usize, route: Route, order_before: Option<String>) {
        let (number, pending) = match self.traffic.add(route.clone()) {
            Ok(number) => (number, rest(&self.traffic, number, route)),
            Err(NotAdded::Standing { robot, .. }) => (robot, Some(Pending::Change(route))),
            // its type or first node is unknown: it gets nothing released
            Err(_) => return,
        };
        // a robot without a job has nothing to offer again
        let pending = pending.filter(|_| order_before.is_some());
        if let Some(slot) = self.robots.get_mut(robot) {
            *slot = Some(Robot {
                number,
                order_before,
                pending,
            });
        }
    }

    /// Offer the robot's pending route to the module, if there is one. Returns why
    /// it is still not taken.
    fn offer(&mut self, robot: usize) -> Option<NotAdded> {
        let known = self.robots.get_mut(robot)?.as_mut()?;
        let number = known.number;
        let (route, taken) = match known.pending.clone()? {
            Pending::Order(route) => (route.clone(), self.traffic.new_order(number, route)),
            Pending::Change(route) => (route.clone(), self.traffic.change(number, route)),
            Pending::Rest(route) => {
                self.traffic.extend(number, route.clone());
                (route, Ok(()))
            }
        };
        if taken.is_ok() {
            known.pending = rest(&self.traffic, number, route);
        }
        taken.err()
    }

    /// Offer what is pending, then ask the module how far the robot may go.
    fn answer(&mut self, ask: &Ask) -> Release {
        let refused = self.offer(ask.robot);
        let wait = |reroute| Release {
            through: ask.released_through,
            reroute,
        };
        let Some(Some(robot)) = self.robots.get(ask.robot) else {
            return wait(None);
        };
        if matches!(robot.pending, Some(Pending::Order(_) | Pending::Change(_))) {
            let wish = match refused {
                Some(NotAdded::Reroute(wish)) => self.avoid(&wish),
                _ => None,
            };
            return wait(wish);
        }
        match self.traffic.release(robot.number, ask.limit) {
            Some(release) => Release {
                through: release.through,
                reroute: release.reroute.and_then(|wish| self.avoid(&wish)),
            },
            None => wait(None),
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
            .filter_map(|(robot, state)| {
                let robot = robot.as_ref()?;
                let before = robot.order_before.as_ref()?;
                // the module does not have the new order yet, or the state is not
                // about it
                if matches!(robot.pending, Some(Pending::Order(_))) || state.order_id == *before {
                    return None;
                }
                Some(Report {
                    robot: robot.number,
                    reached: (state.last_node_sequence_id / 2) as usize,
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
                robot.pending = Some(Pending::Order(route));
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
        known.pending = Some(match known.pending {
            Some(Pending::Order(_)) => Pending::Order(route),
            _ => Pending::Change(route),
        });
        self.offer(robot);
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

    fn zone_answers(&mut self, _fleet: &Fleet) -> Vec<Answer> {
        Vec::new()
    }

    fn blocker(&self, robot: usize, _route: &Plan, _released_through: usize) -> Option<usize> {
        let known = self.robots.get(robot)?.as_ref()?;
        let others = match known.pending {
            // the route is refused: who keeps it from being taken
            Some(Pending::Order(_) | Pending::Change(_)) => self.traffic.refused_by(known.number),
            // at the end of the part taken, the same
            Some(Pending::Rest(_)) => {
                let held_back = self.traffic.blockers(known.number);
                if held_back.is_empty() {
                    self.traffic.refused_by(known.number)
                } else {
                    held_back
                }
            }
            None => self.traffic.blockers(known.number),
        };
        let other = others.into_iter().find(|&o| o != known.number)?;
        self.robots
            .iter()
            .position(|r| r.as_ref().is_some_and(|r| r.number == other))
    }
}

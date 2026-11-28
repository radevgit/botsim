//! The bundled dispatcher, kept as the reference that others are measured
//! against.
//!
//! Each [`dispatch`](Dispatcher::dispatch) goes in this order:
//! 1. **Chargers** (only with [`set_charging`](Dispatcher::set_charging)): the free
//!    robots below the low limit go to free chargers, the pair of a robot and a
//!    charger with the shortest route first. A charger is free if no robot holds it
//!    and none is on its way there. Robots left below the critical limit take no
//!    jobs.
//! 2. **Jobs**, the oldest first, each to the nearest robot (by route length) that
//!    is free or on its way to a parking place (S144); ties go to the lower robot
//!    number. A job no such robot can do stays in the queue and does not block the
//!    jobs behind it.
//! 3. **Parking** (U13): the free robots that got nothing and are not on a parking
//!    place go to free places, the pair with the shortest route first. A place is
//!    free if no robot stands on it or holds it and none is on its way there. A
//!    robot that gets none stays where it is and is counted.
//!
//! A job whose pickup or drop is not a node of the graph stays in the queue for
//! ever, and a robot whose type or node numbers do not fit the [`Site`] is given
//! nothing.

use std::collections::{HashMap, HashSet, VecDeque};

use botsim_model::graph::{EdgeIdx, Graph, NodeIdx};
use botsim_model::job::Job;
use botsim_router::lift::route_through_waiting;
use botsim_router::route::{Arrival, distances_to};

use crate::{
    Answer, ChargePolicy, Control, Counts, Cycle, Dispatcher, Purpose, Robot, Site, Task, Work,
};

/// What the dispatcher remembers of a robot.
#[derive(Debug, Clone, Copy, Default)]
struct Memory {
    /// It stands on a charger and charging was started: it gets no jobs.
    charging: bool,
    /// It was counted as finding no free parking place; reset when it gets a task.
    unparked: bool,
    /// It refused an order to a parking place: not sent again until it has had a job.
    park_refused: bool,
}

/// A robot that a job can be given to.
#[derive(Debug, Clone, Copy)]
struct Candidate {
    robot: usize,
    /// Where its route to the pickup starts: the node it stands on, or the last
    /// released node of its trip to a parking place.
    from: NodeIdx,
    /// Metres it still drives to get there.
    ahead: f64,
    /// It is on its way to a parking place: a job extends that order (S144).
    parking: bool,
    /// On such a trip: the edge by which it arrives at `from`.
    arrival: Option<EdgeIdx>,
}

/// Jobs first come first served to the nearest robot; see the module text.
#[derive(Debug, Default)]
pub struct BaselineDispatcher {
    pending: VecDeque<Job>,
    robots: Vec<Memory>,
    /// (job, robot type) pairs whose drop cannot be reached from their pickup
    /// by that type (the job waits for ever for such a robot).
    unserviceable: HashSet<(u64, usize)>,
    /// Sending robots to chargers; `None` (default) is off.
    charge_policy: Option<ChargePolicy>,
    /// The nodes of the stations of type `charger`.
    charger_nodes: Vec<NodeIdx>,
    /// Charger node → the robot that is on its way there.
    charger_targets: HashMap<NodeIdx, usize>,
    /// Trips to a charger started so far (also the id of the stand-in jobs).
    charge_trips: usize,
    /// Seconds that free robots below the low limit have waited for a charger (all
    /// robots together).
    charge_wait_seconds: f64,
    /// The parking places: the nodes of the stations of type `parking` (U13).
    parking_nodes: Vec<NodeIdx>,
    /// Parking place → the robot that is on its way there.
    parking_targets: HashMap<NodeIdx, usize>,
    /// Trips to a parking place started so far (also the id of the stand-in jobs).
    park_trips: usize,
    /// How often a robot without a job got no parking place.
    no_free_parking: usize,
}

impl BaselineDispatcher {
    pub fn new() -> BaselineDispatcher {
        BaselineDispatcher::default()
    }

    /// The memory of a robot; `None` for a number `start` did not cover.
    fn memory(&mut self, robot: usize) -> Option<&mut Memory> {
        self.robots.get_mut(robot)
    }

    /// Step 1: send low robots to chargers; those sent leave `free`, and so do
    /// those below the critical limit.
    fn charge(
        &mut self,
        site: &Site,
        robots: &[Robot],
        cycle: Cycle,
        free: &mut Vec<(usize, NodeIdx)>,
        control: &mut dyn Control,
    ) {
        let Some(policy) = self.charge_policy else {
            return;
        };
        let mut skipped: Vec<usize> = Vec::new();
        let mut dist: HashMap<(usize, NodeIdx), Vec<f64>> = HashMap::new();
        loop {
            let mut best: Option<(f64, usize, NodeIdx)> = None; // (length, index in free, charger)
            for (k, &(i, node)) in free.iter().enumerate() {
                if skipped.contains(&i) || robots[i].charge >= policy.low {
                    continue;
                }
                if self.charger_nodes.contains(&node) {
                    best = Some((-1.0, k, node)); // already on a charger
                    break;
                }
                let t = robots[i].type_idx;
                for &c in &self.charger_nodes {
                    // a charger is free if no robot holds it and none is on its way
                    if control.node_holder(c).is_some() || self.charger_targets.contains_key(&c) {
                        continue;
                    }
                    let d = dist.entry((t, c)).or_insert_with(|| {
                        distances_to(site.graph, &site.types[t].into, &site.types[t].cost, c)
                    })[node];
                    if d.is_finite() && best.is_none_or(|(b, _, _)| d < b) {
                        best = Some((d, k, c));
                    }
                }
            }
            let Some((_, k, charger)) = best else {
                break;
            };
            let (i, node) = free[k];
            if self.start_charge(site, &robots[i], i, node, charger, cycle.tick, control) {
                free.remove(k);
            } else {
                skipped.push(i);
            }
        }
        // robots still free and below the low limit wait for a charger; below the
        // critical limit they take no more jobs
        for &(i, _) in free.iter() {
            if robots[i].charge < policy.low {
                self.charge_wait_seconds += cycle.elapsed;
            }
        }
        free.retain(|&(i, _)| robots[i].charge >= policy.critical);
    }

    /// Send the free robot `i`, standing on `node`, to `charger` and start charging
    /// there; `false` if it cannot be reached (or the controller cannot do it).
    #[allow(clippy::too_many_arguments)]
    fn start_charge(
        &mut self,
        site: &Site,
        robot: &Robot,
        i: usize,
        node: NodeIdx,
        charger: NodeIdx,
        tick: u64,
        control: &mut dyn Control,
    ) -> bool {
        // already on the charger: just start
        if node == charger {
            if let Some(m) = self.memory(i) {
                m.charging = true;
            }
            let trip = self.charge_trips;
            self.charge_trips += 1;
            control.charge_here(i, trip);
            return true;
        }
        let Some(edges) = route(site, robot, i, node, charger, control) else {
            return false;
        };
        let job = Job {
            id: self.charge_trips as u64,
            pickup: node,
            drop: charger,
            created_tick: tick,
        };
        self.charge_trips += 1;
        self.charger_targets.insert(charger, i);
        let task = Task {
            job,
            purpose: Purpose::Charge,
            from: node,
            edges,
            pickup_at: 0,
        };
        let answer = control.give(i, task);
        self.settle(i, Purpose::Charge, answer);
        answer != Answer::Invalid
    }

    /// Note what became of a task given to robot `i`.
    fn settle(&mut self, i: usize, purpose: Purpose, answer: Answer) {
        match answer {
            Answer::Taken => {
                if let Some(m) = self.memory(i) {
                    m.unparked = false;
                    if purpose == Purpose::Job {
                        m.park_refused = false;
                    }
                }
            }
            Answer::Refused => self.given_up(i, purpose),
            // nothing happened: only take back the place it was to go to
            Answer::Invalid => self.forget_target(i, purpose),
        }
    }

    /// Robot `i` is no longer on its way to the goal of a trip.
    fn forget_target(&mut self, i: usize, purpose: Purpose) {
        match purpose {
            Purpose::Job => {}
            Purpose::Charge => self.charger_targets.retain(|_, r| *r != i),
            Purpose::Park => self.parking_targets.retain(|_, r| *r != i),
        }
    }

    /// Robot `i` as a candidate for a job while it is on its way to a parking place
    /// (S144); `None` if it is not, or cannot take one now: it can take no update
    /// (see [`Work::Parking`]), it is not ready, or its battery is below the low
    /// limit (it goes to a charger once it is free).
    fn parking_candidate(&self, site: &Site, i: usize, robot: &Robot) -> Option<Candidate> {
        let Work::Parking {
            from,
            ref ahead,
            arrival,
        } = robot.work
        else {
            return None;
        };
        let low = self.charge_policy.map_or(0.0, |p| p.low);
        if !robot.ready || robot.charge < low {
            return None;
        }
        let cost = &site.types.get(robot.type_idx)?.cost;
        let edges = site.graph.edges.len();
        if from >= site.graph.nodes.len()
            || ahead.iter().any(|&e| e >= edges)
            || arrival.is_some_and(|e| e >= edges)
        {
            return None;
        }
        Some(Candidate {
            robot: i,
            from,
            ahead: ahead.iter().map(|&e| cost[e]).sum(),
            parking: true,
            arrival,
        })
    }

    /// Step 2: give waiting jobs to the candidates; those given one leave the list.
    fn assign_jobs(
        &mut self,
        site: &Site,
        robots: &[Robot],
        candidates: &mut Vec<Candidate>,
        control: &mut dyn Control,
    ) {
        let nodes = site.graph.nodes.len();
        let mut waiting = VecDeque::new();
        while let Some(job) = self.pending.pop_front() {
            // nobody to give it to (no search needed), or a job outside the layout
            if candidates.is_empty() || job.pickup >= nodes || job.drop >= nodes {
                waiting.push_back(job);
                continue;
            }
            // The nearest candidate that can do the job. One search backwards
            // from the pickup gives the route length from every node (S52), so
            // a job costs one search per robot type, not one per free robot. A
            // type may drive only some of the edges (S55), so its lengths differ.
            let mut dist: Vec<Option<Vec<f64>>> = vec![None; site.types.len()];
            let mut order: Vec<(i64, usize, usize)> = Vec::new();
            for (k, c) in candidates.iter().enumerate() {
                let i = c.robot;
                let t = robots[i].type_idx;
                // an order update does not run the actions of its first node
                if c.parking && c.from == job.pickup {
                    continue;
                }
                // A job whose drop cannot be reached from its pickup cannot be done
                // by any robot of the type; the layout does not change, so ask only
                // once (S52: it was retried against every free robot every cycle).
                if self.unserviceable.contains(&(job.id, t)) {
                    continue;
                }
                if dist[t].is_none() {
                    // (however the robot arrives at the pickup: the most it can do)
                    if route_through_waiting(
                        site.graph,
                        &site.types[t].mover(),
                        site.rides,
                        Arrival::Free,
                        job.pickup,
                        job.drop,
                    )
                    .is_none()
                    {
                        self.unserviceable.insert((job.id, t));
                        continue;
                    }
                    dist[t] = Some(distances_to(
                        site.graph,
                        &site.types[t].into,
                        &site.types[t].cost,
                        job.pickup,
                    ));
                }
                let Some(dist_t) = dist[t].as_ref() else {
                    continue; // (computed just above)
                };
                let d = dist_t[c.from] + c.ahead;
                if d.is_finite() {
                    // smallest distance (to a nanometre); equal → the lower robot number
                    order.push(((d * 1e9).round() as i64, i, k));
                }
            }
            order.sort_unstable();
            // the first robot in that order that has a route for the job
            let chosen = order.iter().find_map(|&(_, i, k)| {
                let c = candidates[k];
                let task = job_task(site, &robots[i], &c, job, control)?;
                Some((k, task))
            });
            let Some((k, task)) = chosen else {
                waiting.push_back(job);
                continue;
            };
            let c = candidates.remove(k);
            if c.parking {
                let answer = control.extend(c.robot, task);
                if answer == Answer::Invalid {
                    waiting.push_back(job);
                    continue;
                }
                // taken or refused: the job is gone from the queue, and the robot is
                // no longer on its way to the place
                self.forget_target(c.robot, Purpose::Park);
                if let Some(m) = self.memory(c.robot) {
                    m.park_refused = false;
                }
            } else {
                let answer = control.give(c.robot, task);
                if answer == Answer::Invalid {
                    waiting.push_back(job);
                }
                self.settle(c.robot, Purpose::Job, answer);
            }
        }
        self.pending = waiting;
    }

    /// Step 3: send the robots of `idle` (without a task, with the node they stand
    /// on) that are not on a parking place to free ones, the pair with the shortest
    /// route first.
    fn park(
        &mut self,
        site: &Site,
        robots: &[Robot],
        idle: &[(usize, NodeIdx)],
        tick: u64,
        control: &mut dyn Control,
    ) {
        if self.parking_nodes.is_empty() {
            return;
        }
        let mut waiting: Vec<(usize, NodeIdx)> = idle
            .iter()
            .filter(|&&(i, node)| {
                !self.parking_nodes.contains(&node)
                    && !self.robots.get(i).is_some_and(|m| m.park_refused)
            })
            .copied()
            .collect();
        if waiting.is_empty() {
            return;
        }
        let occupied: HashSet<NodeIdx> = robots.iter().filter_map(|r| r.node).collect();
        let mut dist: HashMap<(usize, NodeIdx), Vec<f64>> = HashMap::new();
        // (robot, place) pairs for which no route was found
        let mut no_route: Vec<(usize, NodeIdx)> = Vec::new();
        loop {
            let mut best: Option<(f64, usize, NodeIdx)> = None; // (length, index, place)
            for (k, &(i, node)) in waiting.iter().enumerate() {
                let t = robots[i].type_idx;
                for &p in &self.parking_nodes {
                    if occupied.contains(&p)
                        || control.node_holder(p).is_some()
                        || self.parking_targets.contains_key(&p)
                        || no_route.contains(&(i, p))
                    {
                        continue;
                    }
                    let d = dist.entry((t, p)).or_insert_with(|| {
                        distances_to(site.graph, &site.types[t].into, &site.types[t].cost, p)
                    })[node];
                    if d.is_finite() && best.is_none_or(|(b, _, _)| d < b) {
                        best = Some((d, k, p));
                    }
                }
            }
            let Some((_, k, place)) = best else {
                break;
            };
            let (i, node) = waiting[k];
            let Some(edges) = route(site, &robots[i], i, node, place, control) else {
                no_route.push((i, place));
                continue;
            };
            let job = Job {
                id: self.park_trips as u64,
                pickup: node,
                drop: place,
                created_tick: tick,
            };
            self.park_trips += 1;
            self.parking_targets.insert(place, i);
            waiting.remove(k);
            let task = Task {
                job,
                purpose: Purpose::Park,
                from: node,
                edges,
                pickup_at: 0,
            };
            let answer = control.give(i, task);
            self.settle(i, Purpose::Park, answer);
        }
        for (i, _) in waiting {
            if let Some(m) = self.memory(i)
                && !m.unparked
            {
                m.unparked = true;
                self.no_free_parking += 1;
            }
        }
    }
}

impl Dispatcher for BaselineDispatcher {
    fn start(&mut self, graph: &Graph, robots: usize) {
        self.robots = vec![Memory::default(); robots];
        self.charger_nodes = station_nodes(graph, "charger");
        self.parking_nodes = station_nodes(graph, "parking");
    }

    fn set_charging(&mut self, policy: ChargePolicy) -> usize {
        self.charge_policy = Some(policy);
        self.charger_nodes.len()
    }

    fn add_job(&mut self, job: Job) {
        self.pending.push_back(job);
    }

    fn pending(&self) -> Box<dyn Iterator<Item = &Job> + '_> {
        Box::new(self.pending.iter())
    }

    fn stop_charging(&mut self, robot: usize, charge: f64) -> bool {
        let Some(policy) = self.charge_policy else {
            return false;
        };
        match self.memory(robot) {
            Some(m) if m.charging && charge >= policy.high => {
                m.charging = false;
                true
            }
            _ => false,
        }
    }

    fn arrived(&mut self, robot: usize, purpose: Purpose) {
        self.forget_target(robot, purpose);
        if purpose == Purpose::Charge
            && let Some(m) = self.memory(robot)
        {
            m.charging = true;
        }
    }

    fn given_up(&mut self, robot: usize, purpose: Purpose) {
        self.forget_target(robot, purpose);
        if purpose == Purpose::Park
            && let Some(m) = self.memory(robot)
        {
            m.park_refused = true;
        }
    }

    fn dispatch(&mut self, site: &Site, robots: &[Robot], cycle: Cycle, control: &mut dyn Control) {
        // free robots with the node they stand on
        let mut free: Vec<(usize, NodeIdx)> = (0..robots.len().min(self.robots.len()))
            .filter(|&i| robots[i].work == Work::None)
            .filter(|&i| !self.robots[i].charging)
            .filter(|&i| robots[i].ready)
            .filter(|&i| robots[i].type_idx < site.types.len())
            .filter_map(|i| Some((i, robots[i].node?)))
            .filter(|&(_, node)| node < site.graph.nodes.len())
            .collect();

        // a free robot with a low battery goes to a charger instead of getting a job
        self.charge(site, robots, cycle, &mut free, control);

        // jobs go to the free robots and to those on their way to a parking place
        let mut candidates: Vec<Candidate> = free
            .iter()
            .map(|&(robot, from)| Candidate {
                robot,
                from,
                ahead: 0.0,
                parking: false,
                arrival: None,
            })
            .collect();
        candidates.extend(
            robots
                .iter()
                .take(self.robots.len())
                .enumerate()
                .filter_map(|(i, r)| self.parking_candidate(site, i, r)),
        );
        self.assign_jobs(site, robots, &mut candidates, control);

        // the free robots that got nothing go to a parking place (U13)
        let idle: Vec<(usize, NodeIdx)> = candidates
            .iter()
            .filter(|c| !c.parking)
            .map(|c| (c.robot, c.from))
            .collect();
        self.park(site, robots, &idle, cycle.tick, control);
    }

    fn counts(&self) -> Counts {
        Counts {
            parking_places: self.parking_nodes.len(),
            park_trips: self.park_trips,
            no_free_parking: self.no_free_parking,
            charge_trips: self.charge_trips,
            charge_wait_seconds: self.charge_wait_seconds,
        }
    }
}

/// The route of a trip of the free robot `i` from the node it stands on to `to`.
fn route(
    site: &Site,
    robot: &Robot,
    i: usize,
    from: NodeIdx,
    to: NodeIdx,
    control: &dyn Control,
) -> Option<Vec<EdgeIdx>> {
    let kind = &site.types[robot.type_idx];
    let loaded = control.route_cost(i);
    let mover = match &loaded {
        Some(c) => kind.mover_with(c),
        None => kind.mover(),
    };
    route_through_waiting(site.graph, &mover, site.rides, heading(robot), from, to)
}

/// The task of `job` for candidate `c`: to the pickup, then to the drop; `None` if
/// one of the two cannot be reached.
fn job_task(
    site: &Site,
    robot: &Robot,
    c: &Candidate,
    job: Job,
    control: &dyn Control,
) -> Option<Task> {
    let kind = &site.types[robot.type_idx];
    let loaded = control.route_cost(c.robot);
    let mover = match &loaded {
        Some(cost) => kind.mover_with(cost),
        None => kind.mover(),
    };
    // the robot's heading matters if it cannot turn on the spot (S73): on a trip,
    // the way it will arrive at the last released node
    let arrival = match c.arrival {
        Some(e) => Arrival::Edge(e),
        None => heading(robot),
    };
    let path = |from, arrival, to| {
        route_through_waiting(site.graph, &mover, site.rides, arrival, from, to)
    };
    let mut edges = path(c.from, arrival, job.pickup)?;
    let pickup_at = edges.len();
    let then = edges.last().map_or(arrival, |&e| Arrival::Edge(e));
    edges.extend(path(job.pickup, then, job.drop)?);
    Some(Task {
        job,
        purpose: Purpose::Job,
        from: c.from,
        edges,
        pickup_at,
    })
}

/// How a standing robot arrives at its node: with its reported heading.
fn heading(robot: &Robot) -> Arrival {
    robot.heading.map_or(Arrival::Free, Arrival::Heading)
}

/// The interaction nodes of the stations of the layout whose type is `kind` (any
/// case), sorted, each once; only nodes of the graph.
fn station_nodes(graph: &Graph, kind: &str) -> Vec<NodeIdx> {
    let mut nodes: Vec<NodeIdx> = graph
        .stations
        .iter()
        .filter(|s| {
            s.station_type
                .as_deref()
                .is_some_and(|t| t.eq_ignore_ascii_case(kind))
        })
        .flat_map(|s| s.interaction_nodes.iter().copied())
        .filter(|&n| n < graph.nodes.len())
        .collect();
    nodes.sort_unstable();
    nodes.dedup();
    nodes
}

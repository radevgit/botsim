//! Whole runs through `botsim::connect` for what the reference runs do not reach: doors
//! and lifts named by their numbers, zone requests and a charge below the critical
//! limit with a message delay. The work gets done, and the route through `vda`
//! translates everything and gives, step by step, the same as the direct one.

use botsim::{connect::Connected, vda_route};
use botsim_dispatcher::ChargePolicy;
use botsim_model::graph::{Graph, GraphStation};
use botsim_model::job::Job;
use botsim_model::time::Millis;
use botsim_model::zones::Zones;
use botsim_sim::actions::ActionTimes;
use botsim_sim::battery::{Battery, BatterySpec};
use botsim_sim::door::DoorSpec;
use botsim_sim::lift::{DoorState, LiftSpec, LiftStop};
use botsim_sim::observer::{Observer, Snapshot};
use botsim_sim::world::World;
use serde_json::json;

mod setup;
use setup::{DT, graph, id, robot_spec};

/// What an observer saw: every snapshot, with what the controller reserved.
#[derive(Debug, Clone, Default)]
struct Seen(std::rc::Rc<std::cell::RefCell<Vec<Snapshot>>>);

impl Observer for Seen {
    fn on_layout(&mut self, _layout: &Graph) {}

    fn on_step(&mut self, snapshot: &Snapshot) {
        self.0.borrow_mut().push(snapshot.clone());
    }
}

/// What a run gives, to compare the two routes by.
#[derive(Debug, PartialEq)]
struct Results {
    /// Every step: the robots (pose, state, order, route, charge, wait), the lifts
    /// and doors, the reservations, who blocks whom, overlaps and deadlocks.
    steps: Vec<Snapshot>,
    /// The state report of every robot at the end, as the robots hold it: orders,
    /// actions, instant actions, zone requests and sets, errors.
    reports: Vec<botsim_link::message::FromRobot>,
    done: String,
    waits: Vec<botsim_sim::waiting::WaitTotal>,
}

/// Run `world` for `seconds` with the jobs given at the start.
fn run(
    mut w: World,
    mut c: Connected,
    jobs: Vec<Job>,
    seconds: f64,
    mut each: impl FnMut(&World),
) -> (World, Connected, Results) {
    let seen = Seen::default();
    w.set_observer(Box::new(seen.clone()));
    let mut jobs = Some(jobs);
    for _ in 0..(seconds / DT) as usize {
        botsim::connect::step(&mut c, &mut w, jobs.take().unwrap_or_default());
        w.step();
        each(&w);
    }
    let results = Results {
        steps: seen.0.take(),
        reports: w.reports(),
        done: format!("{:?}", c.completed()),
        waits: w.wait_totals(),
    };
    (w, c, results)
}

/// The two connections of a controller to `w`: direct, and through `vda`.
fn connect(through_vda: bool, w: &World) -> Connected {
    if through_vda {
        vda_route::controller(w)
    } else {
        botsim::connect::controller(w)
    }
}

/// Nothing was left untranslated on the route through `vda`.
fn translated(c: &Connected, through_vda: bool) {
    assert_eq!(c.untranslated().is_some(), through_vda);
    if let Some(untranslated) = c.untranslated() {
        assert!(untranslated.is_empty(), "{untranslated:?}");
    }
}

/// The two runs are the same; the first difference is shown.
fn assert_same(direct: &Results, through_vda: &Results) {
    assert_eq!(direct.steps.len(), through_vda.steps.len());
    for (a, b) in direct.steps.iter().zip(&through_vda.steps) {
        assert_eq!(a, b, "step {}", a.tick);
    }
    assert_eq!(direct.reports, through_vda.reports);
    assert_eq!(direct.done, through_vda.done);
    assert_eq!(direct.waits, through_vda.waits);
}

/// Two lanes, each from floor 1 over a lift to floor 2, each with a door at its
/// start. The two lifts list their cabin doors the other way round, and each
/// stop uses its own.
fn two_lanes() -> World {
    let g = graph(
        &[
            ("A", "L1", 0.0, 0.0),
            ("B", "L1", 6.0, 0.0),
            ("C", "L1", 12.0, 0.0),
            ("D", "L2", 12.0, 0.0),
            ("E", "L2", 18.0, 0.0),
            ("P", "L1", 0.0, 20.0),
            ("Q", "L1", 6.0, 20.0),
            ("R", "L1", 12.0, 20.0),
            ("S", "L2", 12.0, 20.0),
            ("T", "L2", 18.0, 20.0),
        ],
        &[
            ("A", "B"),
            ("B", "C"),
            ("C", "D"),
            ("D", "E"),
            ("P", "Q"),
            ("Q", "R"),
            ("R", "S"),
            ("S", "T"),
        ],
    );
    let node = |n: &str| g.node_idx(n).unwrap();
    let map = |m: &str| g.map_idx(m).unwrap();
    let lift = |id: &str, doors: [&str; 2], stops: [(&str, usize); 2], start_floor| LiftSpec {
        id: id.into(),
        floors: vec![map("L1"), map("L2")],
        doors: doors.map(String::from).to_vec(),
        stops: stops
            .iter()
            .enumerate()
            .map(|(floor, &(n, door))| LiftStop {
                node: node(n),
                floor,
                door,
                waiting: None,
            })
            .collect(),
        start_floor,
        travel_seconds_per_floor: 5.0,
        door_open_seconds: 2.0,
        door_close_seconds: 2.0,
        door_hold_seconds: None,
    };
    let lifts = [
        lift("north", ["front", "rear"], [("C", 0), ("D", 1)], 0),
        lift("south", ["rear", "front"], [("R", 1), ("S", 0)], 1),
    ];
    let doors = [("first", "A-B"), ("second", "P-Q")].map(|(id, edge)| DoorSpec {
        id: id.into(),
        edges: vec![g.edge_idx(edge).unwrap()],
        passage_width: None,
        open_seconds: 2.0,
        close_seconds: 2.0,
        hold_seconds: None,
    });
    let starts = [node("A"), node("P")];
    let mut w = World::new(g, DT, 0).unwrap();
    for lift in lifts {
        w.add_lift(lift).unwrap();
    }
    for door in doors {
        w.add_door(door).unwrap();
    }
    for (i, start) in starts.into_iter().enumerate() {
        let times = ActionTimes::new(1.0);
        w.add_robot(id(i + 1), robot_spec(), times, start, 0.0)
            .unwrap();
    }
    w
}

fn job(w: &World, id: u64, pickup: &str, drop: &str) -> Job {
    Job {
        id,
        pickup: w.graph().node_idx(pickup).unwrap(),
        drop: w.graph().node_idx(drop).unwrap(),
        created: Millis(w.now_ms()),
    }
}

#[test]
fn each_door_and_lift_gets_its_own_commands_with_and_without_a_delay() {
    for delay in [0, 10] {
        let [direct, through_vda] = [false, true].map(|through_vda| {
            let mut w = two_lanes();
            w.set_message_delay(delay);
            let c = connect(through_vda, &w);
            let jobs = vec![job(&w, 1, "B", "E"), job(&w, 2, "Q", "T")];
            let mut opened = [false; 2];
            let (w, c, results) = run(w, c, jobs, 120.0, |w| {
                for (seen, door) in opened.iter_mut().zip(w.doors()) {
                    *seen |= door.state == DoorState::Open;
                }
            });
            assert_eq!(c.completed().len(), 2, "delay {delay}");
            assert_eq!(opened, [true, true], "delay {delay}");
            // each cabin is on floor 2, and the door used last there is the stop's
            let lifts: Vec<_> = w
                .lifts()
                .into_iter()
                .map(|l| (l.id, l.floor, l.door_id))
                .collect();
            let expected = [("north", 1, "rear"), ("south", 1, "rear")]
                .map(|(id, floor, door)| (id.to_string(), floor, door.to_string()));
            assert_eq!(lifts, expected, "delay {delay}");
            assert!(w.collisions().is_empty(), "delay {delay}");
            translated(&c, through_vda);
            results
        });
        assert_same(&direct, &through_vda);
    }
}

#[test]
fn zone_requests_and_a_critical_charge_work_with_a_delay() {
    let [direct, through_vda] = [false, true].map(zones_and_charging);
    assert_same(&direct, &through_vda);
}

fn zones_and_charging(through_vda: bool) -> Results {
    // a one-way ring with the charger on a side way from N3 to N0
    let mut g = graph(
        &[
            ("N0", "M", 0.0, 0.0),
            ("N1", "M", 20.0, 0.0),
            ("N2", "M", 20.0, 20.0),
            ("N3", "M", 0.0, 20.0),
            ("C", "M", -10.0, 10.0),
        ],
        &[
            ("N0", "N1"),
            ("N1", "N2"),
            ("N2", "N3"),
            ("N3", "N0"),
            ("N3", "C"),
            ("C", "N0"),
        ],
    );
    let charger = g.node_idx("C").unwrap();
    g.stations.push(GraphStation {
        id: "CH".into(),
        name: None,
        station_type: Some("charger".into()),
        interaction_nodes: vec![charger],
    });
    // a RELEASE zone around N1
    let set = serde_json::from_value(json!({"mapId": "M", "zoneSetId": "zs",
        "zones": [{"zoneId": "r", "zoneType": "RELEASE", "vertices": [
            {"x": 15.0, "y": -5.0}, {"x": 25.0, "y": -5.0},
            {"x": 25.0, "y": 5.0}, {"x": 15.0, "y": 5.0}]}]}))
    .unwrap();
    let zones = Zones::new(&g, set).unwrap();
    let (n0, n2) = (g.node_idx("N0").unwrap(), g.node_idx("N2").unwrap());
    let mut w = World::new(g, DT, 0).unwrap();
    w.set_site_zones(zones);
    w.set_message_delay(10);
    let times = ActionTimes::new(1.0);
    for (i, start) in [n0, n2].into_iter().enumerate() {
        w.add_robot(id(i + 1), robot_spec(), times.clone(), start, 0.0)
            .unwrap();
    }
    // R-001 starts below the critical limit; the charger gives 1 % per second
    let battery = BatterySpec {
        capacity_wh: 100.0,
        drive_wh_per_m: 0.0,
        idle_w: 0.0,
        work_w: 0.0,
    };
    w.set_battery(&id(1), Battery::new(battery, 5.0).unwrap())
        .unwrap();
    assert_eq!(w.set_chargers(3600.0).unwrap(), 1);
    let mut c = connect(through_vda, &w);
    c.set_charging(ChargePolicy {
        low: 25.0,
        high: 80.0,
        critical: 10.0,
    });
    let jobs = vec![job(&w, 1, "N1", "N2"), job(&w, 2, "N0", "N2")];
    let mut most = 0.0_f64;
    let (w, c, results) = run(w, c, jobs, 300.0, |w| {
        most = most.max(w.battery_percent(&id(1)).unwrap().unwrap());
    });
    // it charged from 5 % to the upper limit and stopped there (a delay lets it
    // run on a little), so `startCharging` and `stopCharging` both arrived
    assert_eq!(c.charge_trips(), 1);
    assert!((80.0..85.0).contains(&most), "charged up to {most}");
    // both jobs need a robot in the zone, which a robot enters only with a grant
    assert_eq!(c.completed().len(), 2);
    assert!(w.collisions().is_empty());
    translated(&c, through_vda);
    results
}

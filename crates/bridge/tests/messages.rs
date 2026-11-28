//! The messages to robots that reach the world are, field by field, those the 0.1
//! controller built: the controller's messages translated by `botsim-vda`.

use std::sync::Arc;

use botsim_baseline::baseline_policy::BaselinePolicy;
use botsim_baseline::controller::Controller;
use botsim_baseline::cycle::{Input, Message, Output, Time, robot_no};
use botsim_core::actions::ActionTimes;
use botsim_core::world::World;
use botsim_dispatcher::{
    Answer, ChargePolicy, Control, Counts, Cycle, Dispatcher, Purpose, Robot, Site, Task,
};
use botsim_model::graph::Graph;
use botsim_model::job::Job;
use botsim_model::zones::Zones;
use botsim_vda::translator::{RobotMessage, Translator};
use serde_json::{Value, json};

mod setup;
use setup::{DT, id, line, robot_spec};

/// 2026-11-20T10:00:00Z.
const START: i64 = 1_795_168_800_000;

/// A RELEASE zone and an ACTION zone with parameters, away from the robot.
fn zones(g: &Graph) -> Zones {
    let square = |x: f64| {
        json!([{"x": x, "y": 5.0}, {"x": x + 2.0, "y": 5.0},
               {"x": x + 2.0, "y": 7.0}, {"x": x, "y": 7.0}])
    };
    let set = serde_json::from_value(json!({
        "mapId": "M", "zoneSetId": "zs",
        "zones": [
            {"zoneId": "r", "zoneType": "RELEASE", "releaseLossBehavior": "STOP",
             "zoneDescriptor": "crossing", "vertices": square(0.0)},
            {"zoneId": "a", "zoneType": "ACTION", "vertices": square(10.0),
             "entryActions": [{"actionType": "beep", "blockingType": "NONE",
                 "actionParameters": [{"key": "loud", "value": [1.5, true, "x"]}]}],
             "duringActions": [], "exitActions": []},
        ],
    }))
    .unwrap();
    Zones::new(g, set).unwrap()
}

/// A world with the robot ACME/R-001 on N0 of the line.
fn world() -> World {
    let g = line();
    let n0 = g.node_idx("N0").unwrap();
    let z = zones(&g);
    let mut w = World::new(g, DT, START).unwrap();
    w.set_site_zones(z);
    w.add_robot(id(1), robot_spec(), ActionTimes::new(1.0), n0, 0.0)
        .unwrap();
    w
}

/// The robot's state message with some fields replaced.
fn state_with(w: &mut World, fields: Value) -> vda5050::state::State {
    let mut message = serde_json::to_value(w.state(&id(1)).unwrap()).unwrap();
    for (key, value) in fields.as_object().unwrap() {
        message[key] = value.clone();
    }
    serde_json::from_value(message).unwrap()
}

/// What the world gets of the messages of a cycle.
fn written(w: &World, out: &Output) -> Vec<RobotMessage> {
    let mut translator = Translator::new(Arc::new(w.graph().clone()));
    translator.add_robot(robot_no(0), id(1)).unwrap();
    out.messages
        .iter()
        .map(|m| {
            let Message::Robot(m) = m else {
                panic!("a message to a robot expected: {m:?}");
            };
            let translated = translator.to_robot_to_vda(m).unwrap();
            assert!(translated.issues.is_empty(), "{:?}", translated.issues);
            translated.message
        })
        .collect()
}

/// The header fields every message has, as 0.1 wrote them, with `rest`.
fn as_in_0_1(w: &World, header_id: u32, rest: Value) -> Value {
    let mut message = json!({
        "headerId": header_id,
        "timestamp": w.timestamp(),
        "version": "3.0.1",
        "manufacturer": "ACME",
        "serialNumber": "R-001",
    });
    for (key, value) in rest.as_object().unwrap() {
        message[key] = value.clone();
    }
    message
}

fn instant(w: &World, header_id: u32, action: Value) -> RobotMessage {
    let message = as_in_0_1(w, header_id, json!({"actions": [action]}));
    RobotMessage::InstantActions(serde_json::from_value(message).unwrap())
}

/// One cycle of a controller of the world's site with the state `message`.
fn cycle(
    c: &mut botsim_bridge::Bridged,
    w: &World,
    message: &vda5050::state::State,
    jobs: Vec<Job>,
) -> Output {
    let states = [c.read_state(message, w.tick()).unwrap()];
    let input = Input {
        time: Time {
            tick: w.tick(),
            unix_ms: w.now_ms(),
        },
        states: &states,
        lifts: &[],
        watched: false,
    };
    c.step(&input, jobs)
}

#[test]
fn zones_orders_answers_and_cancel_are_written_as_before() {
    let mut w = world();
    let mut c = botsim_bridge::controller(&w);

    // the site's zone set, to a robot that has none
    let message = state_with(&mut w, json!({}));
    let out = cycle(&mut c, &w, &message, Vec::new());
    let z = zones(w.graph());
    let set = vda5050::zone_set::ZoneSet {
        map_id: "M".into(),
        zone_set_id: z.zone_set_id.clone(),
        zone_set_descriptor: None,
        zones: z.zones.clone(),
    };
    let expected = as_in_0_1(&w, 0, json!({"zoneSet": set}));
    let expected = serde_json::from_value(expected).unwrap();
    assert_eq!(written(&w, &out), [RobotMessage::ZoneSet(expected)]);

    // enabling it, when the robot shows it
    w.step();
    let sets = |status: &str| json!({"zoneSets": [{"zoneSetId": "zs", "mapId": "M", "zoneSetStatus": status}]});
    let message = state_with(&mut w, sets("DISABLED"));
    let out = cycle(&mut c, &w, &message, Vec::new());
    let enable = json!({"actionType": "enableZoneSet", "actionId": "enable-zs",
        "blockingType": "NONE",
        "actionParameters": [{"key": "zoneSetId", "value": "zs"}]});
    assert_eq!(written(&w, &out), [instant(&w, 0, enable)]);

    // the answer to a zone request, then a new order
    w.step();
    let mut fields = sets("ENABLED");
    fields["zoneRequests"] = json!([{"requestId": "req-1", "requestType": "ACCESS",
        "zoneId": "r", "zoneSetId": "zs", "requestStatus": "REQUESTED"}]);
    let message = state_with(&mut w, fields.clone());
    let node = |n: &str| w.graph().node_idx(n).unwrap();
    let job = Job {
        id: 1,
        pickup: node("N1"),
        drop: node("N3"),
        created_tick: w.tick(),
    };
    let out = cycle(&mut c, &w, &message, vec![job]);
    let answers = as_in_0_1(
        &w,
        0,
        json!({"responses": [{"requestId": "req-1", "grantType": "GRANTED"}]}),
    );
    let node = |n: u32, released: bool, actions: Value| {
        json!({"nodeId": format!("N{n}"), "sequenceId": 2 * n, "released": released,
               "actions": actions})
    };
    let edge = |n: u32, released: bool| {
        json!({"edgeId": format!("N{n}-N{}", n + 1), "sequenceId": 2 * n + 1,
               "released": released, "actions": []})
    };
    let action = |kind: &str, at: u32| {
        json!([{"actionType": kind, "actionId": format!("{kind}-{at}"),
                "blockingType": "HARD"}])
    };
    let order = as_in_0_1(
        &w,
        0,
        json!({"orderId": "job-1", "orderUpdateId": 0,
            "nodes": [node(0, true, json!([])), node(1, true, action("pick", 1)),
                      node(2, true, json!([])), node(3, false, action("drop", 3))],
            "edges": [edge(0, true), edge(1, true), edge(2, false)]}),
    );
    assert_eq!(
        written(&w, &out),
        [
            RobotMessage::Responses(serde_json::from_value(answers).unwrap()),
            RobotMessage::Order(serde_json::from_value(order).unwrap()),
        ]
    );

    // a refused order update: cancelOrder
    w.step();
    fields["orderId"] = json!("job-1");
    fields["lastNodeId"] = json!("N0");
    let message = state_with(&mut w, fields.clone());
    cycle(&mut c, &w, &message, Vec::new());
    fields["lastNodeId"] = json!("N1");
    fields["lastNodeSequenceId"] = json!(2);
    let message = state_with(&mut w, fields.clone());
    let out = cycle(&mut c, &w, &message, Vec::new());
    assert!(
        matches!(&written(&w, &out)[..], [RobotMessage::Order(o)]
            if o.header_id == 1 && o.order_update_id == 1),
        "{out:?}"
    );
    fields["errors"] = json!([{"errorType": "VALIDATION_FAILURE", "errorLevel": "WARNING",
        "errorReferences": [{"referenceKey": "orderId", "referenceValue": "job-1"}]}]);
    let message = state_with(&mut w, fields);
    let out = cycle(&mut c, &w, &message, Vec::new());
    let cancel = json!({"actionType": "cancelOrder", "actionId": "cancel-job-1",
        "blockingType": "NONE"});
    assert_eq!(written(&w, &out), [instant(&w, 1, cancel)]);
    assert!(c.untranslated().is_empty(), "{:?}", c.untranslated());
}

/// Sends the robot to a charger on N1, has it charge where it stands, and stops
/// the charging once.
#[derive(Debug, Default)]
struct Charging {
    stopped: bool,
}

impl Dispatcher for Charging {
    fn start(&mut self, _graph: &Graph, _robots: usize) {}
    fn set_charging(&mut self, _policy: ChargePolicy) -> usize {
        0
    }
    fn add_job(&mut self, _job: Job) {}
    fn pending(&self) -> Box<dyn Iterator<Item = &Job> + '_> {
        Box::new(std::iter::empty())
    }
    fn stop_charging(&mut self, _robot: usize, charge: f64) -> bool {
        let stop = charge > 90.0 && !self.stopped;
        self.stopped |= stop;
        stop
    }
    fn arrived(&mut self, _robot: usize, _purpose: Purpose) {}
    fn given_up(&mut self, _robot: usize, _purpose: Purpose) {}
    fn counts(&self) -> Counts {
        Counts::default()
    }
    fn dispatch(&mut self, site: &Site, _robots: &[Robot], cycle: Cycle, c: &mut dyn Control) {
        let node = |n: &str| site.graph.node_idx(n).unwrap();
        match cycle.tick {
            1 => c.charge_here(0, 7),
            2 => {
                let task = Task {
                    job: Job {
                        id: 4,
                        pickup: node("N0"),
                        drop: node("N1"),
                        created_tick: 0,
                    },
                    purpose: Purpose::Charge,
                    from: node("N0"),
                    edges: vec![site.graph.edge_idx("N0-N1").unwrap()],
                    pickup_at: 0,
                };
                assert_eq!(c.give(0, task), Answer::Taken);
            }
            _ => {}
        }
    }
}

#[test]
fn the_charging_actions_are_written_as_before() {
    let g = line();
    let n0 = g.node_idx("N0").unwrap();
    let mut w = World::new(g, DT, START).unwrap();
    w.add_robot(id(1), robot_spec(), ActionTimes::new(1.0), n0, 0.0)
        .unwrap();
    let mut c = botsim_bridge::with_controller(&w, |config| {
        let policy = BaselinePolicy::new(&config.graph);
        Controller::with_dispatcher(&config, policy, Box::new(Charging::default()))
    });
    let half = json!({"powerSupply": {"stateOfCharge": 50.0, "charging": false}});
    let start =
        |id: &str| json!({"actionType": "startCharging", "actionId": id, "blockingType": "NONE"});

    // stopCharging
    let message = state_with(&mut w, json!({}));
    let out = cycle(&mut c, &w, &message, Vec::new());
    let stop = json!({"actionType": "stopCharging", "actionId": "stop-charge-0",
        "blockingType": "NONE"});
    assert_eq!(written(&w, &out), [instant(&w, 0, stop)]);

    // startCharging where the robot stands
    w.step();
    let message = state_with(&mut w, half.clone());
    let out = cycle(&mut c, &w, &message, Vec::new());
    assert_eq!(written(&w, &out), [instant(&w, 1, start("charge-here-7"))]);

    // and after a trip to a charger
    w.step();
    let message = state_with(&mut w, half.clone());
    let out = cycle(&mut c, &w, &message, Vec::new());
    assert!(matches!(&written(&w, &out)[..], [RobotMessage::Order(_)]));
    w.step();
    let mut arrived = half;
    arrived["orderId"] = json!("charge-4");
    arrived["lastNodeId"] = json!("N1");
    arrived["lastNodeSequenceId"] = json!(2);
    let message = state_with(&mut w, arrived);
    let out = cycle(&mut c, &w, &message, Vec::new());
    assert_eq!(
        written(&w, &out),
        [instant(&w, 2, start("charge-charge-4"))]
    );
    assert!(c.untranslated().is_empty(), "{:?}", c.untranslated());
}

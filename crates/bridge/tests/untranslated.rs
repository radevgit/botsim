//! Whatever the route through `vda` cannot translate is counted, never dropped
//! silently, reaches the robot's errors where VDA 5050 has an error for it, and
//! the run goes on safely. Doors and lifts are outside VDA 5050: what names
//! nothing there, the world refuses.

use std::sync::Arc;

use botsim_bridge::vda_route;
use botsim_bridge::vda_route::report::{Entry, What};
use botsim_core::actions::ActionTimes;
use botsim_core::lift::{LiftSpec, LiftStop};
use botsim_core::world::{World, WorldError};
use botsim_link::device::{DeviceHeader, DoorCommand, DoorNo, LiftCommand, LiftNo, ToDoor, ToLift};
use botsim_link::ids::{ActionId, OrderId, RequestId, ZoneSetId};
use botsim_link::instant::{InstantAction, InstantKind};
use botsim_link::message::{Header, Millis, ToRobot, ToRobotBody};
use botsim_link::order::Order;
use botsim_link::response::{Grant, Response};
use botsim_link::state::ActionStatus;
use botsim_link::zone_set::{Zone, ZoneKind, ZoneSet};
use botsim_model::job::Job;
use botsim_model::zones::Zones;
use botsim_server::cycle::{Message, robot_no};
use serde_json::json;

mod setup;
use setup::{DT, graph, id, line, robot_spec};
#[path = "../../core/tests/vda_port/mod.rs"]
mod vda_port;
use vda_port::VdaWorld;

/// A world with the robot ACME/R-001 on N0 of the line, started at `start_ms`.
fn world(start_ms: i64) -> World {
    let g = line();
    let n0 = g.node_idx("N0").unwrap();
    let mut w = World::new(g, DT, start_ms).unwrap();
    w.add_robot(id(1), robot_spec(), ActionTimes::new(1.0), n0, 0.0)
        .unwrap();
    w
}

fn job(w: &World) -> Job {
    let node = |n: &str| w.graph().node_idx(n).unwrap();
    Job {
        id: 1,
        pickup: node("N1"),
        drop: node("N2"),
        created_tick: 0,
    }
}

fn to_robot(body: ToRobotBody) -> Message {
    let header = Header {
        robot: robot_no(0),
        seq: 0,
        time: Millis(0),
    };
    Message::Robot(ToRobot { header, body })
}

/// What was counted: (what, of whom, refused, how often).
fn counted(entries: &[Entry]) -> Vec<(What, &str, bool, u64)> {
    entries
        .iter()
        .map(|e| (e.what, e.name.as_str(), e.refused, e.count))
        .collect()
}

/// What the route of `c` has counted.
fn entries(c: &botsim_bridge::Bridged) -> &[Entry] {
    c.untranslated().expect("on the route").entries()
}

#[test]
fn with_a_state_refused_the_controller_only_takes_the_jobs() {
    // a world in the year 11476, which a VDA 5050 timestamp cannot say
    let mut w = world(300_000_000_000_000);
    let mut c = vda_route::controller(&w);
    let mut jobs = Some(vec![job(&w)]);
    for _ in 0..40 {
        botsim_bridge::step(&mut c, &mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    assert_eq!((c.pending().count(), c.busy()), (1, 0));
    // (the robot never had an order)
    assert_eq!(w.report(robot_no(0)).unwrap().state.order_id, None);
    assert_eq!(
        counted(entries(&c)),
        [(What::State, "ACME/R-001", true, 40)]
    );
    let entry = &entries(&c)[0];
    assert_eq!(entry.first_tick, 0);
    assert!(entry.reason.to_string().contains("timestamp"), "{entry:?}");
}

#[test]
fn a_part_of_a_state_that_cannot_be_read_is_left_out_and_counted() {
    let mut w = world(0);
    let mut c = vda_route::controller(&w);
    let good = w.state(&id(1)).unwrap();

    let mut message = good.clone();
    message.mobile_robot_position.as_mut().unwrap().map_id = "nowhere".into();
    let report = c.read_state(&message, 3).unwrap();
    assert_eq!(report.state.position, None);
    assert!(report.state.velocity.is_some(), "the rest is there");

    // a charge a hair above full is full, and nothing to count
    let mut message = good.clone();
    message.power_supply.state_of_charge = 100.0 * 0.69 / 0.69;
    assert!(message.power_supply.state_of_charge > 100.0);
    assert_eq!(c.read_state(&message, 4).unwrap().state.charge, 100.0);

    let mut message = good.clone();
    message.power_supply.state_of_charge = 101.0;
    assert_eq!(c.read_state(&message, 5), None);

    // and a robot that is not of the fleet
    let mut message = good;
    message.serial_number = "R-009".into();
    assert_eq!(c.read_state(&message, 6), None);
    assert_eq!(
        counted(entries(&c)),
        [
            (What::State, "ACME/R-001", false, 1),
            (What::State, "ACME/R-001", true, 1),
            (What::State, "ACME/R-009", true, 1),
        ]
    );
}

#[test]
fn a_refused_message_is_not_sent_and_of_a_list_the_other_entries_go() {
    let mut w = world(0);
    let mut c = vda_route::controller(&w);

    // an order from a node the layout does not have: the robot says why it did
    // not get it
    let order = Order::new(OrderId::new("o"), 0, 0, 99, Vec::new(), 0, Vec::new()).unwrap();
    c.send(&mut w, &to_robot(ToRobotBody::Order(order)));
    w.step();
    let state = w.state(&id(1)).unwrap();
    assert!(state.order_id.is_empty());
    let [error] = &state.errors[..] else {
        panic!("one error: {:?}", state.errors);
    };
    assert_eq!(error.error_type, "NO_ROUTE_TO_TARGET");
    let reference = &error.error_references.as_ref().unwrap()[0];
    assert_eq!(
        (
            reference.reference_key.as_str(),
            reference.reference_value.as_str()
        ),
        ("orderId", "o")
    );

    // an instant action without an id beside a good one
    let actions = ["", "s"].map(|id| InstantAction {
        id: ActionId::new(id),
        kind: InstantKind::StopCharging,
    });
    let body = ToRobotBody::InstantActions(actions.to_vec());
    c.send(&mut w, &to_robot(body));
    w.step();
    let state = w.state(&id(1)).unwrap();
    let ids: Vec<_> = state
        .instant_action_states
        .iter()
        .map(|a| a.action_id.as_str())
        .collect();
    assert_eq!(ids, ["s"]);
    // (the robot lists the one without an id too, as failed, which a VDA 5050
    // state cannot say)
    let report = w.report(robot_no(0)).unwrap().state;
    let listed: Vec<_> = report
        .instant_actions
        .iter()
        .map(|a| (a.id.as_str(), a.status))
        .collect();
    assert_eq!(
        listed,
        [("", ActionStatus::Failed), ("s", ActionStatus::Finished)]
    );
    assert_eq!(report.errors.len(), 2);

    // a response without a request id beside a good one
    let responses = ["", "r"].map(|id| Response {
        request: RequestId::new(id),
        grant: Grant::Granted,
        lease_expiry: None,
    });
    let body = ToRobotBody::Responses(responses.to_vec());
    c.send(&mut w, &to_robot(body));

    // a zone set that the schema forbids
    let set = ZoneSet {
        id: ZoneSetId::new("zs"),
        map: 0,
        descriptor: None,
        zones: vec![Zone {
            id: botsim_link::ids::ZoneId::new("z"),
            descriptor: None,
            vertices: vec![(0.0, 0.0), (1.0, 0.0)],
            kind: ZoneKind::Blocked,
        }],
    };
    c.send(&mut w, &to_robot(ToRobotBody::ZoneSet(Arc::new(set))));
    w.step();
    assert_eq!(w.state(&id(1)).unwrap().zone_sets, None);

    assert_eq!(
        counted(entries(&c)),
        [
            (What::Order, "ACME/R-001", true, 1),
            (What::InstantActions, "ACME/R-001", false, 1),
            (What::Responses, "ACME/R-001", false, 1),
            (What::ZoneSet, "ACME/R-001", true, 1),
        ]
    );
}

#[test]
fn without_its_zone_set_a_site_with_zones_gets_no_robot_a_task() {
    let g = line();
    // a zone with an empty id: the zones load, the translator refuses them
    let set = serde_json::from_value(json!({"mapId": "M", "zoneSetId": "zs",
        "zones": [{"zoneId": "", "zoneType": "RELEASE", "vertices": [
            {"x": 40.0, "y": 5.0}, {"x": 42.0, "y": 5.0}, {"x": 42.0, "y": 7.0}]}]}))
    .unwrap();
    let zones = Zones::new(&g, set).unwrap();
    let n0 = g.node_idx("N0").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.set_site_zones(zones);
    w.add_robot(id(1), robot_spec(), ActionTimes::new(1.0), n0, 0.0)
        .unwrap();
    let config = vda_route::config(&w);
    assert!(config.zones.is_some() && config.zone_set.is_none());

    let mut c = vda_route::controller(&w);
    let mut jobs = Some(vec![job(&w)]);
    for _ in 0..200 {
        botsim_bridge::step(&mut c, &mut w, jobs.take().unwrap_or_default());
        w.step();
    }
    assert_eq!((c.pending().count(), c.busy()), (1, 0));
    let state = w.state(&id(1)).unwrap();
    assert!(state.order_id.is_empty() && state.zone_sets.is_none());
    assert_eq!(counted(entries(&c)), [(What::SiteZoneSet, "zs", true, 1)]);
}

/// Doors and lifts are outside VDA 5050, so there is nothing to translate or to
/// count: the world refuses a command that names nothing, and nothing happens.
#[test]
fn lift_and_door_messages_that_name_nothing_are_counted() {
    let g = graph(
        &[("A", "L1", 0.0, 0.0), ("B", "L2", 0.0, 0.0)],
        &[("A", "B")],
    );
    let (a, b) = (g.node_idx("A").unwrap(), g.node_idx("B").unwrap());
    let floors = vec![g.map_idx("L1").unwrap(), g.map_idx("L2").unwrap()];
    let mut w = World::new(g, DT, 0).unwrap();
    let stops = [a, b]
        .iter()
        .enumerate()
        .map(|(floor, &node)| LiftStop {
            node,
            floor,
            door: 0,
            waiting: None,
        })
        .collect();
    w.add_lift(LiftSpec {
        id: "L".into(),
        floors,
        doors: vec!["front".into()],
        stops,
        start_floor: 0,
        travel_seconds_per_floor: 1.0,
        door_open_seconds: 1.0,
        door_close_seconds: 1.0,
        door_hold_seconds: None,
    })
    .unwrap();
    let time = Millis(0);
    let door = ToDoor {
        header: DeviceHeader {
            device: DoorNo(7),
            seq: 0,
            time,
        },
        command: DoorCommand::Open,
    };
    let to_lift = |device, command| {
        let header = DeviceHeader {
            device: LiftNo(device),
            seq: 0,
            time,
        };
        ToLift { header, command }
    };
    let call = |map, car_door| to_lift(0, LiftCommand::Call { map, car_door });
    let no_lift = to_lift(7, LiftCommand::Release);

    assert_eq!(
        w.command_door(&door),
        Err(WorldError::UnknownDoorNo(DoorNo(7)))
    );
    assert_eq!(
        w.command_lift(&no_lift),
        Err(WorldError::UnknownLiftNo(LiftNo(7)))
    );
    assert!(matches!(
        w.command_lift(&call(99, 0)),
        Err(WorldError::UnknownMap(_))
    ));
    assert!(matches!(
        w.command_lift(&call(1, 9)),
        Err(WorldError::UnknownDoor(_))
    ));

    // the controller's connection drops them, on either route
    for mut c in [botsim_bridge::controller(&w), vda_route::controller(&w)] {
        c.send(&mut w, &Message::Door(door));
        for command in [no_lift, call(99, 0), call(1, 9), call(1, 9)] {
            c.send(&mut w, &Message::Lift(command));
        }
        assert!(c.untranslated().is_none_or(|u| u.is_empty()));
    }
    assert!(w.lifts()[0].queue.is_empty() && w.lifts()[0].moving_to.is_none());
}

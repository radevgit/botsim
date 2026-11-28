//! Lifts and doors whose owner has no new report, on hand-written data.

use botsim_link::device::{DeviceHeader, DoorCommand, DoorPhase, FromLift, LiftCommand, LiftNo};
use botsim_link::message::Millis;
use botsim_model::graph::Graph;
use botsim_server::cycle::{Known, Message, robot_no};
use botsim_server::doors::{Door, DoorDesk};
use botsim_server::lifts::{Lift, LiftDesk, LiftRobot, LiftStop};
use botsim_server::order_builder::Plan;

#[allow(dead_code)] // (not every part of the helper is used here)
mod hand;
use hand::report;

#[allow(dead_code)] // (only the graph of the layout builder is used here)
#[path = "../../runner/tests/setup/mod.rs"]
mod setup;

/// P → E1 on floor 1, the ride E1 → E2, E2 → Q on floor 2.
fn two_floors() -> (Graph, Plan) {
    let g = setup::graph(
        &[
            ("P", "F1", 0.0, 0.0),
            ("E1", "F1", 10.0, 0.0),
            ("E2", "F2", 10.0, 0.0),
            ("Q", "F2", 20.0, 0.0),
        ],
        &[("P", "E1"), ("E1", "E2"), ("E2", "Q")],
    );
    let edge = |id: &str| g.edge_idx(id).unwrap();
    let edges = vec![edge("P-E1"), edge("E1-E2"), edge("E2-Q")];
    let plan = Plan::new(&g, g.node_idx("P").unwrap(), edges);
    (g, plan)
}

fn lift_desk(g: &Graph) -> LiftDesk {
    let stop = |node: &str| LiftStop {
        node: g.node_idx(node).unwrap(),
        door: 0,
        waiting: None,
    };
    let lift = Lift {
        id: "L".into(),
        doors: vec!["front".into()],
        stops: vec![stop("E1"), stop("E2")],
    };
    LiftDesk::new(g, &[lift])
}

/// The lift at `ms`, with robot 0 in its cabin or nobody.
fn lift_state(ms: i64, inside: bool) -> FromLift {
    FromLift {
        header: DeviceHeader {
            device: LiftNo(0),
            seq: 0,
            time: Millis(ms),
        },
        map: 0,
        car_door: 0,
        door: DoorPhase::Open,
        occupant: inside.then(|| robot_no(0)),
    }
}

/// One update of the lift desk for robot 0; the commands to the lift.
fn update(desk: &mut LiftDesk, states: &[FromLift], robot: LiftRobot) -> Vec<LiftCommand> {
    let mut out = Vec::new();
    desk.update(states, &[robot], Millis(0), &mut out);
    out.into_iter()
        .map(|m| match m {
            Message::Lift(m) => m.command,
            other => panic!("to the lift: {other:?}"),
        })
        .collect()
}

#[test]
fn a_lift_of_a_silent_robot_is_not_sent_on_or_let_go() {
    let (g, plan) = two_floors();
    let node = |n: &str| g.node_idx(n);
    let call = |floor: &str| LiftCommand::Call {
        map: g.map_idx(floor).unwrap(),
        car_door: 0,
    };
    let mut desk = lift_desk(&g);
    assert_eq!(desk.window(0, &plan, 0, 0, 2), 2, "it gets the lift");
    let robot = |at, node, new| LiftRobot { at, node, new };
    let at_gate = robot(Some(0), node("P"), true);
    assert_eq!(update(&mut desk, &[], at_gate), [call("F1")]);

    // silent: whatever is said of it, the lift is only called again and stays
    // its own
    let inside = [lift_state(1, true)];
    for silent in [robot(Some(1), node("E1"), false), robot(None, None, false)] {
        assert_eq!(update(&mut desk, &inside, silent), [call("F1")]);
        assert!(desk.owns(0));
    }
    // it reports; of two lift states the newer one counts, wherever it is
    let boarding = robot(Some(1), node("E1"), true);
    for states in [
        [lift_state(5, false), lift_state(4, true)],
        [lift_state(4, true), lift_state(5, false)],
    ] {
        assert_eq!(update(&mut desk, &states, boarding), [call("F1")]);
    }
    assert_eq!(
        update(&mut desk, &[lift_state(6, true)], boarding),
        [LiftCommand::Release, call("F2")]
    );
    // riding, silent: nothing to say
    let silent = robot(Some(2), node("E2"), false);
    assert_eq!(update(&mut desk, &[], silent), []);
    // out of the cabin, still on the stop: the door stays; then silent, even with
    // a report that says it is gone: the lift is not let go
    assert_eq!(update(&mut desk, &[], robot(Some(2), node("E2"), true)), []);
    let gone = robot(Some(3), node("Q"), false);
    assert_eq!(update(&mut desk, &[], gone), []);
    assert!(desk.owns(0));
    let gone = robot(Some(3), node("Q"), true);
    assert_eq!(update(&mut desk, &[], gone), [LiftCommand::Release]);
    assert!(!desk.owns(0));
}

#[test]
fn a_door_of_a_silent_robot_is_asked_open_and_not_let_go() {
    let (g, plan) = two_floors();
    let first = g.edge_idx("P-E1").unwrap();
    let mut desk = DoorDesk::new(&[Door {
        id: "D".into(),
        edges: vec![first],
    }]);
    assert_eq!(desk.window(0, &plan, 0, 0, 2), 2);
    let p = g.node_idx("P").unwrap();
    let e1 = g.node_idx("E1").unwrap();
    let mut update = |at: Option<usize>, new: bool| -> Vec<DoorCommand> {
        let known = Known {
            report: report(&g, 0, "", &[p, e1], 0, false),
            new,
        };
        let mut out = Vec::new();
        desk.update(&[at], &[known], Millis(0), &mut out);
        out.into_iter()
            .map(|m| match m {
                Message::Door(m) => m.command,
                other => panic!("to the door: {other:?}"),
            })
            .collect()
    };
    assert_eq!(update(Some(0), true), [DoorCommand::Open]);
    // silent: a kept report that says it has passed, or has no job, changes
    // nothing
    for at in [Some(2), None] {
        assert_eq!(update(at, false), [DoorCommand::Open]);
    }
    assert_eq!(update(Some(1), true), [DoorCommand::Release]);
    assert_eq!(update(Some(1), true), []);
}

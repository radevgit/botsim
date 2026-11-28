use botsim_core::actions::ActionTimes;
use botsim_core::drive::{Driver, Phase};
use botsim_link::state::ActionStatus::{self, Finished, Running, Waiting};
use botsim_model::graph::Graph;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vdma_lif::Lif;

mod vda_port;
use vda_port::route_from_order;

const DT: f64 = 0.05;

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap()
}

fn spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

/// Node actions as `(actionId, actionType, blockingType)`.
type Acts<'a> = &'a [(&'a str, &'a str, &'a str)];

fn actions_json(acts: Acts) -> String {
    let v: Vec<String> = acts
        .iter()
        .map(|(id, t, b)| {
            format!(r#"{{"actionId":"{id}","actionType":"{t}","blockingType":"{b}"}}"#)
        })
        .collect();
    format!("[{}]", v.join(","))
}

/// Order A→B→C, fully released, with the given actions on A, B and C.
fn order_abc(update_id: u32, a: Acts, b: Acts, c: Acts) -> Order {
    let json = format!(
        r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-001","orderId":"o1","orderUpdateId":{update_id},
        "nodes":[
          {{"nodeId":"A","sequenceId":0,"released":true,"actions":{}}},
          {{"nodeId":"B","sequenceId":2,"released":true,"actions":{}}},
          {{"nodeId":"C","sequenceId":4,"released":true,"actions":{}}}],
        "edges":[
          {{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]}},
          {{"edgeId":"B-C","sequenceId":3,"released":true,"actions":[]}}]}}"#,
        actions_json(a),
        actions_json(b),
        actions_json(c)
    );
    serde_json::from_str(&json).unwrap()
}

fn times() -> ActionTimes {
    // every action takes 2 s, "pick" 3 s
    ActionTimes::new(2.0).with("pick", 3.0)
}

fn driver(o: &Order) -> Driver {
    let g = graph();
    let r = route_from_order(&g, o).unwrap();
    Driver::from_order_route(&g, &spec(), 0.0, &r, times()).unwrap()
}

/// Seconds until the order is finished.
fn finish_time(o: &Order) -> f64 {
    let mut d = driver(o);
    let mut steps = 0;
    while !d.is_finished() {
        d.step(DT);
        steps += 1;
        assert!(steps < 100_000);
    }
    steps as f64 * DT
}

fn statuses(d: &Driver) -> Vec<(String, ActionStatus)> {
    d.action_states()
        .map(|(id, _, s)| (id.to_string(), s))
        .collect()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 2.0 * DT
}

#[test]
fn hard_action_at_a_node_stops_the_robot_for_its_duration() {
    let base = finish_time(&order_abc(0, &[], &[], &[]));
    let t = finish_time(&order_abc(0, &[], &[("p", "pick", "HARD")], &[]));
    assert!(close(t - base, 3.0), "delay {}", t - base);

    // and while it acts it stands still at B
    let o = order_abc(0, &[], &[("p", "pick", "HARD")], &[]);
    let mut d = driver(&o);
    let mut seen_acting = false;
    while !d.is_finished() {
        d.step(DT);
        if d.phase() == Phase::Acting {
            seen_acting = true;
            assert_eq!(d.speed(), 0.0);
            let p = d.pose();
            assert!((p.x - 10.0).abs() < 1e-9 && p.y.abs() < 1e-9);
            assert_eq!(statuses(&d).len(), 1);
        }
    }
    assert!(seen_acting);
    assert_eq!(statuses(&d), vec![("p".to_string(), Finished)]);
}

#[test]
fn soft_action_also_stops_driving() {
    let base = finish_time(&order_abc(0, &[], &[], &[]));
    let t = finish_time(&order_abc(0, &[], &[("s", "beep", "SOFT")], &[]));
    assert!(close(t - base, 2.0), "delay {}", t - base);
}

#[test]
fn none_and_single_actions_do_not_delay_driving() {
    let base = finish_time(&order_abc(0, &[], &[], &[]));
    // 2 s actions at B while a 5 s drive B-C follows: no delay
    for blocking in ["NONE", "SINGLE"] {
        let t = finish_time(&order_abc(0, &[], &[("x", "beep", blocking)], &[]));
        assert!(close(t, base), "{blocking}: delay {}", t - base);
    }

    // the action keeps running while the robot drives away
    let o = order_abc(0, &[], &[("x", "beep", "NONE")], &[]);
    let mut d = driver(&o);
    let mut driving_while_running = false;
    while !d.is_finished() {
        d.step(DT);
        if d.phase() == Phase::Driving && statuses(&d) == vec![("x".to_string(), Running)] {
            driving_while_running = true;
        }
    }
    assert!(driving_while_running);
}

#[test]
fn the_order_is_not_finished_before_the_actions_at_the_last_node_are() {
    let base = finish_time(&order_abc(0, &[], &[], &[]));
    // a NONE action at the last node does not block driving but must finish first
    let t = finish_time(&order_abc(0, &[], &[], &[("x", "beep", "NONE")]));
    assert!(close(t - base, 2.0), "delay {}", t - base);

    let o = order_abc(0, &[], &[], &[("x", "beep", "NONE")]);
    let mut d = driver(&o);
    let mut acting_at_end = false;
    while !d.is_finished() {
        d.step(DT);
        if d.phase() == Phase::Acting {
            acting_at_end = true;
            let p = d.pose();
            assert!((p.x - 10.0).abs() < 1e-9 && (p.y - 5.0).abs() < 1e-9);
        }
    }
    assert!(acting_at_end);
}

#[test]
fn actions_at_the_start_node_run_before_driving() {
    let base = finish_time(&order_abc(0, &[], &[], &[]));
    let t = finish_time(&order_abc(0, &[("p", "pick", "HARD")], &[], &[]));
    assert!(close(t - base, 3.0), "delay {}", t - base);

    let o = order_abc(0, &[("p", "pick", "HARD")], &[], &[]);
    let d = driver(&o);
    assert_eq!(d.phase(), Phase::Acting, "acts at once, before it moves");
}

#[test]
fn action_list_order_and_blocking_types_are_respected() {
    // NONE (2 s) then HARD (2 s) at B: the HARD waits for the NONE → 4 s stop
    let base = finish_time(&order_abc(0, &[], &[], &[]));
    let acts: Acts = &[("n", "beep", "NONE"), ("h", "beep", "HARD")];
    let t = finish_time(&order_abc(0, &[], acts, &[]));
    assert!(close(t - base, 4.0), "delay {}", t - base);

    // at the moment the robot arrives: n running, h waiting
    let o = order_abc(0, &[], acts, &[]);
    let mut d = driver(&o);
    while d.phase() != Phase::Acting {
        d.step(DT);
    }
    assert_eq!(
        statuses(&d),
        vec![("n".to_string(), Running), ("h".to_string(), Waiting)]
    );
}

#[test]
fn an_update_does_not_run_the_actions_of_the_decision_node_again() {
    let g = graph();
    // first order: only A-B released, B has a HARD action; C is horizon
    let first: Order = {
        let json = format!(
            r#"{{"headerId":1,"timestamp":"2026-11-20T10:00:00.000Z","version":"3.0.1",
            "manufacturer":"ACME","serialNumber":"R-001","orderId":"o1","orderUpdateId":0,
            "nodes":[
              {{"nodeId":"A","sequenceId":0,"released":true,"actions":[]}},
              {{"nodeId":"B","sequenceId":2,"released":true,"actions":{}}},
              {{"nodeId":"C","sequenceId":4,"released":false,"actions":[]}}],
            "edges":[
              {{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]}},
              {{"edgeId":"B-C","sequenceId":3,"released":false,"actions":[]}}]}}"#,
            actions_json(&[("p", "pick", "HARD")])
        );
        serde_json::from_str(&json).unwrap()
    };
    let r = route_from_order(&g, &first).unwrap();
    let mut d = Driver::from_order_route(&g, &spec(), 0.0, &r, times()).unwrap();
    while d.phase() != Phase::Waiting {
        d.step(DT);
    }
    assert_eq!(statuses(&d), vec![("p".to_string(), Finished)]);

    // the update re-sends B with the same (finished) action and releases B-C and C
    let update = order_abc(1, &[], &[], &[]);
    let update: Order = {
        let mut u = update;
        u.nodes.remove(0);
        u.edges.remove(0);
        u.nodes[0].actions = first.nodes[1].actions.clone();
        u
    };
    let ur = route_from_order(&g, &update).unwrap();
    assert!(
        ur.node_actions[0].is_empty(),
        "decision node actions are not repeated"
    );
    d.extend(&g, &ur).unwrap();
    while !d.is_finished() {
        d.step(DT);
    }
    assert_eq!(statuses(&d), vec![("p".to_string(), Finished)], "ran once");
}

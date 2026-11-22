use botsim_record::{
    LayoutLine, NodeRecord, RobotRecord, RouteRecord, ShapeRecord, SnapshotLine, StatusRecord,
};
use botsim_viewer::describe;

fn layout() -> LayoutLine {
    let node = |id: &str| NodeRecord {
        id: id.into(),
        map: 0,
        x: 0.0,
        y: 0.0,
    };
    LayoutLine {
        version: 1,
        maps: vec!["M".into()],
        nodes: vec![node("A"), node("B"), node("C"), node("D")],
        edges: vec![],
        geometry: vec![],
        robots: vec![],
    }
}

fn robot(id: &str, status: StatusRecord) -> RobotRecord {
    RobotRecord {
        id: id.into(),
        map: 0,
        x: 0.0,
        y: 0.0,
        heading: 0.0,
        shape: ShapeRecord::Circle([0.0, 0.0, 0.4]),
        status,
        speed: 0.0,
        order: String::new(),
        last_node: None,
        released: RouteRecord::default(),
        planned: RouteRecord::default(),
    }
}

fn snapshot(robots: Vec<RobotRecord>, blocking: Vec<[usize; 2]>) -> SnapshotLine {
    SnapshotLine {
        tick: 1,
        time: 0.05,
        robots,
        reservations: vec![],
        blocking,
        overlapping: vec![],
        deadlocks: vec![],
        lifts: vec![],
        doors: vec![],
    }
}

#[test]
fn it_describes_a_robot_in_words() {
    let mut r = robot("ACME/R-001", StatusRecord::WaitingForRelease);
    r.speed = -0.25;
    r.order = "job-7".into();
    r.last_node = Some(1);
    r.released = RouteRecord {
        nodes: vec![2],
        edges: vec![0],
    };
    r.planned = RouteRecord {
        nodes: vec![3, 0],
        edges: vec![1, 2],
    };
    let snap = snapshot(vec![robot("ACME/R-000", StatusRecord::Idle), r], vec![]);
    let info = describe(&layout(), &snap, 1).unwrap();
    assert_eq!(info.id, "ACME/R-001");
    assert_eq!(info.status, "WaitingForRelease");
    assert_eq!((info.speed, info.order.as_str()), (-0.25, "job-7"));
    assert_eq!(info.last_node.as_deref(), Some("B"));
    assert_eq!(info.released, ["C"]);
    assert_eq!(info.planned, ["D", "A"]);
    assert!(info.waits_for.is_empty() && info.waited_for_by.is_empty());
}

#[test]
fn it_says_who_waits_for_whom() {
    let robots = vec![
        robot("ACME/R-000", StatusRecord::WaitingForRelease),
        robot("ACME/R-001", StatusRecord::WaitingForRelease),
        robot("ACME/R-002", StatusRecord::Driving),
    ];
    // 0 waits for 1; 1 waits for 0 (a deadlock pair); 2 waits for 1
    let snap = snapshot(robots, vec![[0, 1], [1, 0], [2, 1]]);
    let layout = layout();
    let one = describe(&layout, &snap, 1).unwrap();
    assert_eq!(one.waits_for, ["ACME/R-000"]);
    assert_eq!(one.waited_for_by, ["ACME/R-000", "ACME/R-002"]);
    let two = describe(&layout, &snap, 2).unwrap();
    assert_eq!(two.waits_for, ["ACME/R-001"]);
    assert!(two.waited_for_by.is_empty());
}

#[test]
fn it_copes_with_missing_things() {
    let layout = layout();
    // no robot at that index
    assert_eq!(describe(&layout, &snapshot(vec![], vec![]), 0), None);
    // a node number that the layout does not have, a blocker that is not a robot
    let mut r = robot("ACME/R-000", StatusRecord::Idle);
    r.last_node = Some(99);
    let snap = snapshot(vec![r], vec![[0, 5]]);
    let info = describe(&layout, &snap, 0).unwrap();
    assert_eq!(info.last_node.as_deref(), Some("#99"));
    assert_eq!(info.waits_for, ["#5"]);
}

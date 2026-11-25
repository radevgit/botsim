use botsim_record::{
    EdgeRecord, LayoutLine, NodeRecord, Recording, ReservationRecord, RobotRecord, RouteRecord,
    ShapeRecord, SnapshotLine, StatusRecord,
};
use botsim_viewer::{
    busiest_edges, group_table, groups_from_json, most_waited_nodes, usage, usage_between,
};

fn layout() -> LayoutLine {
    let node = |id: &str| NodeRecord {
        id: id.into(),
        map: 0,
        x: 0.0,
        y: 0.0,
    };
    let edge = |id: &str, start, end| EdgeRecord {
        id: id.into(),
        start,
        end,
        length: 1.0,
        map_transition: false,
        paths: vec![],
    };
    LayoutLine {
        version: 1,
        maps: vec!["M".into()],
        nodes: vec![node("A"), node("B"), node("C")],
        edges: vec![edge("A-B", 0, 1), edge("B-C", 1, 2)],
        geometry: vec![],
        robots: vec![],
    }
}

fn robot(status: StatusRecord, last_node: Option<usize>) -> RobotRecord {
    RobotRecord {
        id: "ACME/R-000".into(),
        map: 0,
        x: 0.0,
        y: 0.0,
        heading: 0.0,
        shape: ShapeRecord::Circle([0.0, 0.0, 0.4]),
        status,
        speed: 0.0,
        order: String::new(),
        last_node,
        released: RouteRecord::default(),
        planned: RouteRecord::default(),
        charge: None,
        charging: false,
        waiting: None,
    }
}

fn snapshot(time: f64, edges: Vec<usize>, status: StatusRecord) -> SnapshotLine {
    SnapshotLine {
        tick: 0,
        time,
        robots: vec![robot(status, Some(1))],
        reservations: vec![ReservationRecord {
            robot: 0,
            nodes: vec![],
            edges,
        }],
        blocking: vec![],
        overlapping: vec![],
        deadlocks: vec![],
        lifts: vec![],
        doors: vec![],
    }
}

fn recording() -> Recording {
    // 0 s - 1 s: edge 0 reserved, driving; 1 s - 3 s: edge 1 reserved, waiting at node 1;
    // the last snapshot (3 s) counts for as long as the one before (2 s)
    Recording {
        layout: layout(),
        snapshots: vec![
            snapshot(0.0, vec![0], StatusRecord::Driving),
            snapshot(1.0, vec![1], StatusRecord::WaitingForRelease),
            snapshot(3.0, vec![1], StatusRecord::Obstructed),
        ],
    }
}

#[test]
fn it_counts_how_long_each_edge_was_reserved() {
    let u = usage(&recording());
    // (the last snapshot counts for as long as the one before: 1 + 2 + 2 s)
    assert_eq!(u.duration, 5.0);
    assert_eq!(u.edge_seconds, vec![1.0, 4.0]);
    assert!((u.edge_share(0) - 0.2).abs() < 1e-9);
    assert!((u.max_edge_share() - 0.8).abs() < 1e-9);
}

#[test]
fn it_counts_only_the_time_of_a_window() {
    let rec = recording();
    // 1 s to 2 s: only the second snapshot (edge 1, waiting at node 1), cut to 1 s
    let u = usage_between(&rec, 1.0, 2.0);
    assert_eq!(u.duration, 1.0);
    assert_eq!(u.edge_seconds, vec![0.0, 1.0]);
    assert_eq!(u.node_wait_seconds, vec![0.0, 1.0, 0.0]);
    // a window with nothing in it
    let u = usage_between(&rec, 10.0, 20.0);
    assert_eq!(u.duration, 0.0);
    assert_eq!(u.edge_share(1), 0.0);
}

#[test]
fn it_counts_the_waiting_at_the_last_node() {
    let u = usage(&recording());
    assert_eq!(u.node_wait_seconds, vec![0.0, 4.0, 0.0]);
}

#[test]
fn it_lists_the_busiest_edges_and_the_most_waited_nodes() {
    let rec = recording();
    let u = usage(&rec);
    let edges = busiest_edges(&rec.layout, &u, 5);
    assert_eq!(edges[0].0, "B-C");
    assert_eq!(edges[1].0, "A-B");
    let nodes = most_waited_nodes(&rec.layout, &u, 5);
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0].0, "B");
}

#[test]
fn it_sums_up_the_use_by_group_of_roads() {
    // the nodes are all at (0, 0): put the groups round it
    let rec = recording();
    let u = usage(&rec);
    let groups = groups_from_json(
        r#"[{"name": "near", "boxes": [[-1, -1, 1, 1]]}, {"name": "far", "boxes": [[5, 5, 6, 6]]}]"#,
    )
    .unwrap();
    let table = group_table(&rec.layout, &u, &groups);
    assert_eq!(table.len(), 1); // "far" and "other" have no edges
    assert_eq!(table[0].name, "near");
    assert_eq!(table[0].edges, 2);
    assert!((table[0].mean_share - (0.2 + 0.8) / 2.0).abs() < 1e-9);
    assert_eq!(table[0].seconds, 5.0);
    assert!((table[0].max_share - 0.8).abs() < 1e-9);
    assert!(groups_from_json("not json").is_err());
}

#[test]
fn it_copes_with_a_recording_without_time() {
    let rec = Recording {
        layout: layout(),
        snapshots: vec![snapshot(0.0, vec![0], StatusRecord::Driving)],
    };
    let u = usage(&rec);
    assert_eq!(u.edge_share(0), 0.0);
}

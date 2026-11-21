//! LIF stations in the graph, and jobs between places (S60).

use botsim_core::graph::{Graph, GraphError};
use botsim_core::jobs::{JobError, JobGenerator};
use vdma_lif::Lif;

/// Three nodes A, B, C on a line; `stations` is the JSON array of the layout's
/// stations (or empty for none).
fn layout(stations: &str) -> Result<Graph, GraphError> {
    let node = |id: &str, x: f64| {
        format!(
            r#"{{"nodeId":"{id}","mapId":"M","nodePosition":{{"x":{x},"y":0.0}},
            "mobileRobotTypeNodeProperties":[{{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]}}]}}"#
        )
    };
    let stations = if stations.is_empty() {
        String::new()
    } else {
        format!(r#","stations":{stations}"#)
    };
    let json = format!(
        r#"{{"metaInformation":{{"lifId":"t","creator":"t","exportTimestamp":"2026-11-21T10:00:00.000Z","lifVersion":"2.0.0"}},
        "origins":[{{"originId":"O","layouts":[{{"layoutId":"L","layoutVersion":"1",
        "nodes":[{},{},{}],
        "edges":[{{"edgeId":"A-B","startNodeId":"A","endNodeId":"B","mobileRobotTypeEdgeProperties":[
          {{"mobileRobotTypes":[{{"manufacturer":"ACME","seriesName":"X1"}}]}}]}}]{stations}}}]}}]}}"#,
        node("A", 0.0),
        node("B", 5.0),
        node("C", 10.0)
    );
    let lif: Lif = serde_json::from_str(&json).unwrap();
    Graph::from_lif(&lif)
}

#[test]
fn stations_are_read_with_their_interaction_nodes() {
    let g = layout(
        r#"[{"stationId":"S1","stationName":"Dock","stationType":"DOCK","interactionNodeIds":["A"]},
           {"stationId":"S2","interactionNodeIds":["B","C","B"]}]"#,
    )
    .unwrap();
    assert_eq!(g.stations.len(), 2);
    let s1 = &g.stations[g.station_idx("S1").unwrap()];
    assert_eq!(s1.name.as_deref(), Some("Dock"));
    assert_eq!(s1.station_type.as_deref(), Some("DOCK"));
    assert_eq!(s1.interaction_nodes, vec![g.node_idx("A").unwrap()]);
    // several interaction nodes, a repeated one only once; no name is fine
    let s2 = &g.stations[g.station_idx("S2").unwrap()];
    assert_eq!(
        s2.interaction_nodes,
        vec![g.node_idx("B").unwrap(), g.node_idx("C").unwrap()]
    );
    assert_eq!(s2.name, None);
    assert_eq!(g.station_idx("S9"), None);
    // a layout without stations has none
    assert!(layout("").unwrap().stations.is_empty());
}

#[test]
fn wrong_stations_are_refused_with_the_station_named() {
    let dup = layout(
        r#"[{"stationId":"S1","interactionNodeIds":["A"]},{"stationId":"S1","interactionNodeIds":["B"]}]"#,
    );
    assert_eq!(dup.unwrap_err(), GraphError::DuplicateStation("S1".into()));

    let unknown = layout(r#"[{"stationId":"S1","interactionNodeIds":["Z"]}]"#).unwrap_err();
    assert_eq!(
        unknown,
        GraphError::BadStation {
            station: "S1".into(),
            node: Some("Z".into())
        }
    );
    assert!(unknown.to_string().contains("S1") && unknown.to_string().contains('Z'));

    let none = layout(r#"[{"stationId":"S1","interactionNodeIds":[]}]"#).unwrap_err();
    assert_eq!(
        none,
        GraphError::BadStation {
            station: "S1".into(),
            node: None
        }
    );
}

const DT: f64 = 0.05;

fn jobs(mut g: JobGenerator, steps: u64) -> Vec<(usize, usize)> {
    (0..steps)
        .filter_map(|t| g.step(t))
        .map(|j| (j.pickup, j.drop))
        .collect()
}

#[test]
fn places_with_one_node_give_the_same_jobs_as_plain_nodes() {
    let nodes = [3, 5, 8, 9];
    let places: Vec<Vec<usize>> = nodes.iter().map(|&n| vec![n]).collect();
    for seed in 1..=4 {
        let a = jobs(JobGenerator::new(seed, &nodes, 600.0, DT).unwrap(), 20000);
        let b = jobs(
            JobGenerator::between_places(seed, &places, 600.0, DT).unwrap(),
            20000,
        );
        assert!(a.len() > 100);
        assert_eq!(a, b, "seed {seed}");
    }
}

#[test]
fn a_job_goes_between_two_places_at_one_of_their_nodes() {
    // place 0 = {1, 2}, place 1 = {3}, place 2 = {4, 5}
    let places = vec![vec![1, 2], vec![3], vec![4, 5]];
    let list = jobs(
        JobGenerator::between_places(7, &places, 1200.0, DT).unwrap(),
        60000,
    );
    assert!(list.len() > 500);
    let place = |n: usize| places.iter().position(|p| p.contains(&n)).unwrap();
    for &(pickup, drop) in &list {
        assert_ne!(
            place(pickup),
            place(drop),
            "a job needs two different places"
        );
    }
    // both nodes of a station are used, and nothing else
    let used: std::collections::BTreeSet<usize> = list.iter().flat_map(|&(p, d)| [p, d]).collect();
    assert_eq!(used.into_iter().collect::<Vec<_>>(), vec![1, 2, 3, 4, 5]);
    // and the same seed gives the same jobs
    let again = jobs(
        JobGenerator::between_places(7, &places, 1200.0, DT).unwrap(),
        60000,
    );
    assert_eq!(list, again);
}

#[test]
fn at_least_two_different_places_are_needed() {
    let one = JobGenerator::between_places(1, &[vec![1, 2]], 60.0, DT).err();
    assert_eq!(one, Some(JobError::TooFewNodes));
    // the same list twice is one place; an empty list is no place
    let same = JobGenerator::between_places(1, &[vec![1, 2], vec![1, 2], vec![]], 60.0, DT).err();
    assert_eq!(same, Some(JobError::TooFewNodes));
    assert!(JobGenerator::between_places(1, &[vec![1], vec![1, 2]], 60.0, DT).is_ok());
}

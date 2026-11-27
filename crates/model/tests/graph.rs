use botsim_model::graph::{Graph, GraphError};
use vdma_lif::Lif;

const THREE_NODES: &str = include_str!("data/three_nodes.lif.json");

fn lif() -> Lif {
    serde_json::from_str(THREE_NODES).unwrap()
}

#[test]
fn builds_graph_from_layout() {
    let lif = lif();
    let g = Graph::from_layout(&lif.origins[0].layouts[0]).unwrap();

    assert_eq!(g.nodes.len(), 3);
    assert_eq!(g.edges.len(), 3);

    let (a, b, c) = (
        g.node_idx("A").unwrap(),
        g.node_idx("B").unwrap(),
        g.node_idx("C").unwrap(),
    );
    assert_eq!((g.nodes[b].x, g.nodes[b].y), (10.0, 0.0));

    let ab = g.edge_idx("A-B").unwrap();
    assert_eq!((g.edges[ab].start, g.edges[ab].end), (a, b));
    assert_eq!(g.edges[ab].length, 10.0);
    assert_eq!(g.edges[g.edge_idx("B-C").unwrap()].length, 5.0);

    assert_eq!(g.out_edges(a), &[ab]);
    assert_eq!(g.out_edges(b), &[g.edge_idx("B-C").unwrap()]);
    assert_eq!(g.out_edges(c), &[g.edge_idx("C-B").unwrap()]);
}

#[test]
fn rejects_unknown_node_in_edge() {
    let mut lif = lif();
    lif.origins[0].layouts[0].edges[0].end_node_id = "Z".into();
    let err = Graph::from_layout(&lif.origins[0].layouts[0]).unwrap_err();
    assert_eq!(
        err,
        GraphError::UnknownNode {
            edge: "A-B".into(),
            node: "Z".into()
        }
    );
}

#[test]
fn rejects_duplicate_ids() {
    let mut lif = lif();
    let layout = &mut lif.origins[0].layouts[0];
    layout.nodes.push(layout.nodes[0].clone());
    assert_eq!(
        Graph::from_layout(layout).unwrap_err(),
        GraphError::DuplicateNode("A".into())
    );

    let mut lif = self::lif();
    let layout = &mut lif.origins[0].layouts[0];
    layout.edges.push(layout.edges[0].clone());
    assert_eq!(
        Graph::from_layout(layout).unwrap_err(),
        GraphError::DuplicateEdge("A-B".into())
    );
}

#[test]
fn nodes_carry_their_map_and_edges_across_maps_are_transitions() {
    let mut lif = lif();
    {
        let g = Graph::from_layout(&lif.origins[0].layouts[0]).unwrap();
        assert_eq!(g.maps, vec!["M".to_string()]);
        assert_eq!(g.map_idx("M"), Some(0));
        assert!(g.nodes.iter().all(|n| n.map == 0));
        assert!(g.edges.iter().all(|e| !e.map_transition));
    }

    // put node C on a second map
    lif.origins[0].layouts[0].nodes[2].map_id = "M2".into();
    let g = Graph::from_layout(&lif.origins[0].layouts[0]).unwrap();
    assert_eq!(g.maps, vec!["M".to_string(), "M2".to_string()]);
    assert_eq!(g.nodes[g.node_idx("C").unwrap()].map, 1);

    let ab = &g.edges[g.edge_idx("A-B").unwrap()];
    assert!(!ab.map_transition);
    assert_eq!(ab.length, 10.0);
    // B-C and C-B cross maps: no length, flagged
    for id in ["B-C", "C-B"] {
        let e = &g.edges[g.edge_idx(id).unwrap()];
        assert!(e.map_transition, "{id}");
        assert_eq!(e.length, 0.0);
    }
}

/// Official LIF example 14: two levels, each its own layout and origin; the lift
/// edges `N2-N102` and `N102-N2` lead from one layout into the other.
#[test]
fn builds_one_graph_from_a_lif_with_two_levels() {
    let lif: Lif = serde_json::from_str(include_str!("data/two_levels.lif.json")).unwrap();

    // one layout alone cannot be built: its lift edge ends in the other layout
    assert!(matches!(
        Graph::from_layout(&lif.origins[0].layouts[0]),
        Err(GraphError::UnknownNode { .. })
    ));

    let g = Graph::from_lif(&lif).unwrap();
    assert_eq!(g.nodes.len(), 4);
    assert_eq!(g.edges.len(), 5);
    assert_eq!(g.maps, vec!["Map_Z-Level_1", "Map_Z-Level_2"]);

    let n2 = g.node_idx("N2").unwrap();
    let n102 = g.node_idx("N102").unwrap();
    assert_eq!(g.nodes[n2].map, 0);
    assert_eq!(g.nodes[n102].map, 1);

    // the two lift edges are map transitions, the others are ordinary
    for id in ["N2-N102", "N102-N2"] {
        let e = &g.edges[g.edge_idx(id).unwrap()];
        assert!(e.map_transition, "{id}");
        assert_eq!(e.length, 0.0);
    }
    for id in ["N1-N2", "N102-N101", "N101-N102"] {
        let e = &g.edges[g.edge_idx(id).unwrap()];
        assert!(!e.map_transition, "{id}");
        assert!(e.length > 0.0);
    }
    // edges of both layouts hang on the right nodes
    assert_eq!(g.out_edges(n2).len(), 1);
    assert_eq!(g.out_edges(n102).len(), 2);
}

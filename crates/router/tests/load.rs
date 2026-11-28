//! The route cost with the load and waiting penalties (S91, S93).

use botsim_model::graph::Graph;
use botsim_router::load::{Load, loaded_cost};

mod common;
use common::graph;

/// A → B → C → D: edges 0, 1 and 2 end at the nodes B, C and D.
fn chain() -> Graph {
    graph(
        &[
            ("A", 0.0, 0.0),
            ("B", 10.0, 0.0),
            ("C", 20.0, 0.0),
            ("D", 30.0, 0.0),
        ],
        &[("A", "B"), ("B", "C"), ("C", "D")],
    )
}

/// Nothing held, nobody waiting, both weights 0.
const NOTHING: Load = Load {
    load_weight: 0.0,
    wait_weight: 0.0,
    edge_held: &[],
    node_held: &[],
    node_wait: &[],
};

const COST: [f64; 3] = [10.0, 20.0, 30.0];

#[test]
fn a_held_edge_and_a_held_end_node_each_add_the_load_weight() {
    let load = Load {
        load_weight: 5.0,
        edge_held: &[false, true, true],
        // A is only the start of an edge: holding it costs nothing
        node_held: &[true, false, false, true],
        ..NOTHING
    };
    assert_eq!(
        loaded_cost(&chain(), &COST, &load),
        Some(vec![10.0, 25.0, 40.0])
    );
}

#[test]
fn the_waiting_average_of_the_end_node_adds_the_wait_weight() {
    let load = Load {
        wait_weight: 4.0,
        // without a load weight, who holds what does not matter
        edge_held: &[true, true, true],
        node_held: &[true, true, true, true],
        node_wait: &[9.0, 0.5, 0.0, 2.0],
        ..NOTHING
    };
    assert_eq!(
        loaded_cost(&chain(), &COST, &load),
        Some(vec![12.0, 20.0, 38.0])
    );
}

#[test]
fn without_a_weight_above_zero_there_is_no_other_cost() {
    let load = Load {
        edge_held: &[true, true, true],
        node_wait: &[1.0, 1.0, 1.0, 1.0],
        ..NOTHING
    };
    assert_eq!(loaded_cost(&chain(), &COST, &load), None);
    let load = Load {
        load_weight: -1.0,
        wait_weight: -1.0,
        ..load
    };
    assert_eq!(loaded_cost(&chain(), &COST, &load), None);
}

#[test]
fn lists_that_do_not_fit_the_graph_are_no_error() {
    let g = chain();
    let load = Load {
        load_weight: 5.0,
        wait_weight: 4.0,
        ..NOTHING
    };
    // nothing named: the plain cost
    assert_eq!(loaded_cost(&g, &COST, &load), Some(COST.to_vec()));
    // short lists: what they do not name is not held, and nobody waits there
    let short = Load {
        edge_held: &[true],
        node_held: &[false, true],
        node_wait: &[0.0, 1.0],
        ..load
    };
    assert_eq!(loaded_cost(&g, &COST, &short), Some(vec![24.0, 20.0, 30.0]));
    // one result for each cost: fewer than the graph has edges, and more (an edge
    // the graph does not have has no end node)
    assert_eq!(loaded_cost(&g, &COST[..1], &short), Some(vec![24.0]));
    let more = Load {
        edge_held: &[false, false, false, true],
        node_held: &[true, true, true, true],
        node_wait: &[1.0, 1.0, 1.0, 1.0],
        ..load
    };
    assert_eq!(
        loaded_cost(&g, &[10.0, 20.0, 30.0, 40.0], &more),
        Some(vec![19.0, 29.0, 39.0, 45.0])
    );
    assert_eq!(loaded_cost(&g, &[], &more), Some(Vec::new()));
}

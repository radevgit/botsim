//! The search for the node at a reported position takes any numbers.

use botsim_server::node_grid::NodeGrid;

mod common;
use common::graph;

#[test]
fn an_enormous_or_impossible_position_is_on_no_node() {
    let g = graph(&[("N0", 0.0, 0.0), ("N1", 10.0, 0.0)], &[("N0", "N1")]);
    let grid = NodeGrid::new(&g);
    assert_eq!(grid.node_near(0, 10.0, 0.0, 0.5), Some(1));
    let wild = [
        f64::MAX,
        -f64::MAX,
        1e300,
        -1e300,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NAN,
    ];
    for x in wild {
        for y in wild.into_iter().chain([0.0]) {
            assert_eq!(grid.node_near(0, x, y, 0.5), None, "{x} {y}");
            assert_eq!(grid.node_near(0, y, x, 0.5), None, "{y} {x}");
        }
    }
    assert_eq!(grid.node_near(usize::MAX, 0.0, 0.0, 0.5), None);
}

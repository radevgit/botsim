//! What each robot type may drive, prepared once for the searches.

use botsim_model::graph::{EdgeIdx, Graph};
use botsim_model::spec::Kinematics;

use crate::route::{Mover, Turns, in_edges, travel_lengths};

/// What a robot type may drive (S55): the layout lists the types that may use
/// each node and edge.
#[derive(Debug)]
pub struct TypeRouting {
    /// An omnidirectional type drives off in any direction without turning.
    pub omnidirectional: bool,
    /// `ok[e]`: the type may drive edge `e` (the edge and both its nodes allow it).
    pub ok: Vec<bool>,
    /// The turns the type cannot make (S73); `None` if it can make them all.
    pub turns: Option<Turns>,
    /// The distance the type drives on each edge (S74): a curve is longer than the
    /// straight line between its nodes.
    pub cost: Vec<f64>,
    /// The allowed edges into each node.
    pub into: Vec<Vec<EdgeIdx>>,
}

impl TypeRouting {
    /// For the robot type `manufacturer` / `series` with `kinematics` on `graph`.
    pub fn new(
        graph: &Graph,
        kinematics: Kinematics,
        manufacturer: &str,
        series: &str,
    ) -> TypeRouting {
        let ok: Vec<bool> = (0..graph.edges.len())
            .map(|e| graph.edge_allows(e, manufacturer, series))
            .collect();
        TypeRouting {
            omnidirectional: kinematics == Kinematics::Omnidirectional,
            into: in_edges(graph, &ok),
            turns: Turns::for_type(graph, kinematics, manufacturer, series),
            cost: travel_lengths(graph, manufacturer, series),
            ok,
        }
    }

    /// What the type can drive, for the route search.
    pub fn mover(&self) -> Mover<'_> {
        Mover {
            ok: &self.ok,
            cost: &self.cost,
            turns: self.turns.as_ref(),
        }
    }

    /// Like [`mover`](Self::mover), with another cost per edge (the length plus the
    /// load penalty).
    pub fn mover_with<'a>(&'a self, cost: &'a [f64]) -> Mover<'a> {
        Mover {
            ok: &self.ok,
            cost,
            turns: self.turns.as_ref(),
        }
    }
}

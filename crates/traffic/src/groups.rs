//! Groups (D5, amended): the conflicts of two routes that must share one order.
//!
//! A robot takes a piece when it is released and gives it back when it reaches the
//! next node: node `k` from the release of node `k` until the robot is at node
//! `k + 1`, the edge before it from the same release until the robot is at node `k`.
//! Two pieces of a route **touch** if these times overlap: a node and an edge that
//! starts or ends there, or two nodes joined by an edge.
//!
//! Two conflicts of the same two robots are neighbours if their pieces are the same
//! or touch in **both** routes. Giving neighbours opposite orders is always a
//! deadlock, so a connected run of conflicts, head-on or following, is one order
//! decision.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;
use std::ops::RangeInclusive;

use crate::spots::{Piece, Spots};

/// A robot's route: node, edge, node, … The index of a piece is its VDA 5050
/// `sequenceId` in an order that starts at 0.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Route {
    /// `manufacturer/seriesName` of the robot.
    type_name: String,
    pieces: Vec<Piece>,
}

impl Route {
    /// `None` unless there is one node more than there are edges.
    pub fn new<S: AsRef<str>>(type_name: &str, nodes: &[S], edges: &[S]) -> Option<Route> {
        if nodes.len() != edges.len() + 1 {
            return None;
        }
        let mut pieces = Vec::with_capacity(nodes.len() + edges.len());
        for (i, n) in nodes.iter().enumerate() {
            pieces.push(Piece::Node(n.as_ref().to_string()));
            if let Some(e) = edges.get(i) {
                pieces.push(Piece::Edge(e.as_ref().to_string()));
            }
        }
        Some(Route {
            type_name: type_name.to_string(),
            pieces,
        })
    }

    pub fn type_name(&self) -> &str {
        &self.type_name
    }

    pub fn pieces(&self) -> &[Piece] {
        &self.pieces
    }

    /// Drop everything after node `node` (its index among the nodes).
    pub(crate) fn cut(&mut self, node: usize) {
        self.pieces
            .truncate(node.saturating_mul(2).saturating_add(1));
    }
}

/// A route names something the layout or the conflict map does not have.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UnknownName {
    Type(String),
    Piece(Piece),
}

impl fmt::Display for UnknownName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            UnknownName::Type(t) => write!(f, "unknown robot type {t}"),
            UnknownName::Piece(p) => write!(f, "unknown {p}"),
        }
    }
}

impl Error for UnknownName {}

/// The conflicts that share one order, as pairs (piece index in the first route,
/// piece index in the second), sorted.
pub type Group = Vec<(usize, usize)>;

/// The groups of conflicts between the routes of two robots, in the order of their
/// first pair. No groups: the two never get in each other's way.
pub fn groups(spots: &Spots, a: &Route, b: &Route) -> Result<Vec<Group>, UnknownName> {
    groups_where(spots, a, b, |_, _| true)
}

/// Like [`groups`], but only of the conflicts `live` keeps: a pass that is over
/// takes no part in an order.
pub(crate) fn groups_where(
    spots: &Spots,
    a: &Route,
    b: &Route,
    live: impl Fn(usize, usize) -> bool,
) -> Result<Vec<Group>, UnknownName> {
    check(spots, a)?;
    check(spots, b)?;

    let mut in_b: BTreeMap<&Piece, Vec<usize>> = BTreeMap::new();
    for (q, piece) in b.pieces.iter().enumerate() {
        in_b.entry(piece).or_default().push(q);
    }
    // the conflicts not yet in a group
    let mut open = BTreeSet::new();
    for (p, piece) in a.pieces.iter().enumerate() {
        let others = spots.conflicting(&a.type_name, piece, &b.type_name);
        for other in others.into_iter().flatten() {
            for &q in in_b.get(other).into_iter().flatten() {
                if live(p, q) {
                    open.insert((p, q));
                }
            }
        }
    }

    let mut groups = Vec::new();
    while let Some(first) = open.pop_first() {
        let mut group = vec![first];
        let mut next = 0;
        while let Some(&(p, q)) = group.get(next) {
            next += 1;
            for p2 in touching(p) {
                for q2 in touching(q) {
                    if open.remove(&(p2, q2)) {
                        group.push((p2, q2));
                    }
                }
            }
        }
        group.sort_unstable();
        groups.push(group);
    }
    Ok(groups)
}

/// Does the layout and the conflict map have everything the route names?
pub(crate) fn check(spots: &Spots, route: &Route) -> Result<(), UnknownName> {
    if !spots.has_type(&route.type_name) {
        return Err(UnknownName::Type(route.type_name.clone()));
    }
    match route.pieces.iter().find(|p| !spots.has_piece(p)) {
        Some(p) => Err(UnknownName::Piece(p.clone())),
        None => Ok(()),
    }
}

/// The piece indices that touch piece `p` of a route, `p` too: for a node (even
/// index) the edges and nodes on both sides, for an edge only its two nodes.
fn touching(p: usize) -> RangeInclusive<usize> {
    let reach = if p.is_multiple_of(2) { 2 } else { 1 };
    p.saturating_sub(reach)..=p + reach
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{LARGE, SMALL, edge, edge_pair, empty_map, lif, node};
    use botsim_sweep::ConflictMap;
    use vdma_lif::Lif;

    fn route(type_name: &str, nodes: &[&str], edges: &[&str]) -> Route {
        Route::new(type_name, nodes, edges).expect("route")
    }

    /// A two-way corridor X – M – Y.
    fn corridor() -> (Lif, ConflictMap) {
        let l = lif(
            &["X", "M", "Y"],
            &[
                ("XM", "X", "M"),
                ("MY", "M", "Y"),
                ("YM", "Y", "M"),
                ("MX", "M", "X"),
            ],
        );
        let mut map = empty_map();
        map.pairs.push(edge_pair(SMALL, "XM", SMALL, "MX"));
        map.pairs.push(edge_pair(SMALL, "MY", SMALL, "YM"));
        (l, map)
    }

    /// Two lines of two edges each, without a common node.
    fn two_lines() -> Lif {
        lif(
            &["A0", "A1", "A2", "B0", "B1", "B2"],
            &[
                ("E1", "A0", "A1"),
                ("E2", "A1", "A2"),
                ("F1", "B0", "B1"),
                ("F2", "B1", "B2"),
            ],
        )
    }

    #[test]
    fn route_alternates_nodes_and_edges() {
        let r = route(SMALL, &["X", "M"], &["XM"]);
        assert_eq!(r.pieces(), [node("X"), edge("XM"), node("M")]);
        assert!(Route::new(SMALL, &["X", "M"], &[]).is_none());
        assert!(Route::new::<&str>(SMALL, &[], &[]).is_none());
    }

    #[test]
    fn head_on_corridor_is_one_group_with_its_ends() {
        let (l, map) = corridor();
        let spots = Spots::new(&l, &map).expect("setup");
        let a = route(SMALL, &["X", "M", "Y"], &["XM", "MY"]);
        let b = route(SMALL, &["Y", "M", "X"], &["YM", "MX"]);
        let all = vec![(0, 4), (1, 3), (2, 2), (3, 1), (4, 0)];
        assert_eq!(groups(&spots, &a, &b), Ok(vec![all]));
    }

    #[test]
    fn head_on_is_one_group_with_node_conflicts_alone() {
        let (l, _) = corridor();
        let spots = Spots::new(&l, &empty_map()).expect("setup");
        let a = route(SMALL, &["X", "M", "Y"], &["XM", "MY"]);
        let b = route(SMALL, &["Y", "M", "X"], &["YM", "MX"]);
        assert_eq!(
            groups(&spots, &a, &b),
            Ok(vec![vec![(0, 4), (2, 2), (4, 0)]])
        );
    }

    #[test]
    fn following_is_one_group() {
        let (l, map) = corridor();
        let spots = Spots::new(&l, &map).expect("setup");
        let a = route(SMALL, &["X", "M", "Y"], &["XM", "MY"]);
        let same = vec![(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)];
        assert_eq!(groups(&spots, &a, &a), Ok(vec![same]));
    }

    #[test]
    fn two_separate_crossings_are_two_groups() {
        let l = lif(
            &["A0", "A1", "A2", "B0", "B1", "B2", "P", "Q"],
            &[
                ("A0P", "A0", "P"),
                ("PA1", "P", "A1"),
                ("A1Q", "A1", "Q"),
                ("QA2", "Q", "A2"),
                ("B0P", "B0", "P"),
                ("PB1", "P", "B1"),
                ("B1Q", "B1", "Q"),
                ("QB2", "Q", "B2"),
            ],
        );
        let spots = Spots::new(&l, &empty_map()).expect("setup");
        let a = route(
            SMALL,
            &["A0", "P", "A1", "Q", "A2"],
            &["A0P", "PA1", "A1Q", "QA2"],
        );
        let b = route(
            SMALL,
            &["B0", "P", "B1", "Q", "B2"],
            &["B0P", "PB1", "B1Q", "QB2"],
        );
        assert_eq!(groups(&spots, &a, &b), Ok(vec![vec![(2, 2)], vec![(6, 6)]]));
    }

    #[test]
    fn consecutive_edges_alone_do_not_share_an_order() {
        // E1 beside F1 and E2 beside F2, the nodes between them clear: one robot may
        // go first at the one place and second at the other
        let mut map = empty_map();
        map.pairs.push(edge_pair(SMALL, "E1", SMALL, "F1"));
        map.pairs.push(edge_pair(SMALL, "E2", SMALL, "F2"));
        let spots = Spots::new(&two_lines(), &map).expect("setup");
        let a = route(SMALL, &["A0", "A1", "A2"], &["E1", "E2"]);
        let b = route(SMALL, &["B0", "B1", "B2"], &["F1", "F2"]);
        assert_eq!(groups(&spots, &a, &b), Ok(vec![vec![(1, 1)], vec![(3, 3)]]));
    }

    #[test]
    fn groups_depend_on_the_robot_types() {
        let mut map = empty_map();
        map.pairs.push(edge_pair(SMALL, "E1", LARGE, "F1"));
        let spots = Spots::new(&two_lines(), &map).expect("setup");
        let large = route(LARGE, &["B0", "B1"], &["F1"]);
        let on_a = |ty| route(ty, &["A0", "A1"], &["E1"]);
        assert_eq!(groups(&spots, &on_a(SMALL), &large), Ok(vec![vec![(1, 1)]]));
        assert_eq!(groups(&spots, &on_a(LARGE), &large), Ok(Vec::new()));
    }

    #[test]
    fn unknown_names_in_a_route_are_reported() {
        let spots = Spots::new(&two_lines(), &empty_map()).expect("setup");
        let good = route(SMALL, &["A0", "A1"], &["E1"]);
        let lost = route(SMALL, &["A0", "Z"], &["E1"]);
        assert_eq!(
            groups(&spots, &good, &lost),
            Err(UnknownName::Piece(node("Z")))
        );
        let other = route("ACME/Other", &["A0"], &[]);
        assert_eq!(
            groups(&spots, &other, &good),
            Err(UnknownName::Type("ACME/Other".into()))
        );
    }
}

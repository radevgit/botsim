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

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;
use std::ops::RangeInclusive;

use crate::spots::{Piece, Spots};

/// A robot's route: node, edge, node, … The index of a piece is its VDA 5050
/// `sequenceId` in an order that starts at 0.
#[derive(Debug, Clone)]
pub struct Route {
    /// `manufacturer/seriesName` of the robot.
    type_name: String,
    pieces: Vec<Piece>,
    /// The same as numbers, once [`number`](Route::number) has looked them up.
    numbers: Option<Numbers>,
}

/// Two routes are the same if their type and pieces are.
impl PartialEq for Route {
    fn eq(&self, other: &Route) -> bool {
        self.type_name == other.type_name && self.pieces == other.pieces
    }
}

impl Eq for Route {}

/// The type and the pieces of a route as the numbers the spots have for them.
#[derive(Debug, Clone)]
struct Numbers {
    type_number: usize,
    pieces: Vec<usize>,
}

impl Numbers {
    /// An error for the type, else for the first piece, the spots do not have.
    fn of(spots: &Spots, route: &Route) -> Result<Numbers, UnknownName> {
        let type_number = spots
            .type_number(&route.type_name)
            .ok_or_else(|| UnknownName::Type(route.type_name.clone()))?;
        let pieces = route.pieces.iter().map(|piece| {
            spots
                .piece_number(piece)
                .ok_or_else(|| UnknownName::Piece(piece.clone()))
        });
        Ok(Numbers {
            type_number,
            pieces: pieces.collect::<Result<_, _>>()?,
        })
    }
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
            numbers: None,
        })
    }

    /// Keep the numbers these spots have for the route, so that its names are not
    /// looked up again; for a route that stays with them. With a name they do not
    /// have, it keeps none.
    pub(crate) fn number(&mut self, spots: &Spots) -> Result<(), UnknownName> {
        if self.numbers.is_none() {
            self.numbers = Some(Numbers::of(spots, self)?);
        }
        Ok(())
    }

    /// The numbers: those kept, else looked up now.
    fn numbers(&self, spots: &Spots) -> Result<Cow<'_, Numbers>, UnknownName> {
        match &self.numbers {
            Some(numbers) => Ok(Cow::Borrowed(numbers)),
            None => Numbers::of(spots, self).map(Cow::Owned),
        }
    }

    pub fn type_name(&self) -> &str {
        &self.type_name
    }

    pub fn pieces(&self) -> &[Piece] {
        &self.pieces
    }

    /// Drop everything after node `node` (its index among the nodes).
    pub(crate) fn cut(&mut self, node: usize) {
        let length = node.saturating_mul(2).saturating_add(1);
        self.pieces.truncate(length);
        if let Some(numbers) = &mut self.numbers {
            numbers.pieces.truncate(length);
        }
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
    // an unknown name in `a` is told first
    a.numbers(spots)?;
    Against::new(spots, b)?.groups_where(a, live)
}

/// The one group of [`groups_where`] that holds the pair `at`; empty if that is
/// not a conflict `live` keeps. Only the group is walked, not the whole routes.
pub(crate) fn group_where(
    spots: &Spots,
    a: &Route,
    b: &Route,
    at: (usize, usize),
    live: impl Fn(usize, usize) -> bool,
) -> Result<Group, UnknownName> {
    let a = a.numbers(spots)?;
    let b = b.numbers(spots)?;
    let open = |p: usize, q: usize| match (a.pieces.get(p), b.pieces.get(q)) {
        (Some(&x), Some(&y)) => {
            spots.numbers_conflict(b.type_number, y, a.type_number, x) && live(p, q)
        }
        _ => false,
    };
    if !open(at.0, at.1) {
        return Ok(Vec::new());
    }
    let mut found = BTreeSet::from([at]);
    let mut group = vec![at];
    let mut next = 0;
    while let Some(&(p, q)) = group.get(next) {
        next += 1;
        for p2 in touching(p) {
            for q2 in touching(q) {
                if !found.contains(&(p2, q2)) && open(p2, q2) {
                    found.insert((p2, q2));
                    group.push((p2, q2));
                }
            }
        }
    }
    group.sort_unstable();
    Ok(group)
}

/// One route, to find the groups of others against it. Which pieces of a type of
/// robot conflict with which of the route's is worked out once for the type; then
/// each piece of another route takes one look-up.
pub(crate) struct Against<'a> {
    spots: &'a Spots,
    route: Cow<'a, Numbers>,
    /// By the other type: (its piece, index in the route), sorted.
    by_type: BTreeMap<usize, Vec<(usize, usize)>>,
}

impl<'a> Against<'a> {
    pub(crate) fn new(spots: &'a Spots, route: &'a Route) -> Result<Against<'a>, UnknownName> {
        Ok(Against {
            spots,
            route: route.numbers(spots)?,
            by_type: BTreeMap::new(),
        })
    }

    /// As [`groups_where`] gives them for `a` and this route.
    pub(crate) fn groups_where(
        &mut self,
        a: &Route,
        live: impl Fn(usize, usize) -> bool,
    ) -> Result<Vec<Group>, UnknownName> {
        let theirs = a.numbers(self.spots)?;
        let found = self
            .by_type
            .entry(theirs.type_number)
            .or_insert_with(|| conflicts(self.spots, &self.route, theirs.type_number));
        // the conflicts not yet in a group
        let mut open = BTreeSet::new();
        for (p, &piece) in theirs.pieces.iter().enumerate() {
            let from = found.partition_point(|&(other, _)| other < piece);
            let same = found.iter().skip(from);
            for &(_, q) in same.take_while(|&&(other, _)| other == piece) {
                if live(p, q) {
                    open.insert((p, q));
                }
            }
        }
        Ok(connected(open))
    }
}

/// For a robot of type `other`: each of its pieces that conflicts with a piece of
/// the route, with that piece's index in the route; sorted.
fn conflicts(spots: &Spots, route: &Numbers, other: usize) -> Vec<(usize, usize)> {
    let mut found = Vec::new();
    for (q, &piece) in route.pieces.iter().enumerate() {
        found.push((piece, q)); // the same piece
        // the list goes both ways
        let listed = spots.listed_against(route.type_number, piece, other);
        found.extend(listed.map(|theirs| (theirs, q)));
    }
    found.sort_unstable();
    found
}

/// These conflicts in their groups.
fn connected(mut open: BTreeSet<(usize, usize)>) -> Vec<Group> {
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
    groups
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
    use botsim_space::{ConflictMap, NodeConflict, NodeTypePair};
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
    fn groups_hold_just_the_conflicts_the_spots_name() {
        // maps and routes drawn by a small generator; the routes need not be
        // drivable
        let mut state = 1u64;
        let mut draw = |below: usize| {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (state >> 33) as usize % below
        };
        let layout = two_lines();
        let nodes = ["A0", "A1", "A2", "B0", "B1", "B2"];
        let edges = ["E1", "E2", "F1", "F2"];
        let types = [SMALL, LARGE];
        let mut conflicts = 0;
        for _ in 0..2_000 {
            let mut map = empty_map();
            for _ in 0..draw(6) {
                let (a, b) = (types[draw(2)], types[draw(2)]);
                map.pairs
                    .push(edge_pair(a, edges[draw(4)], b, edges[draw(4)]));
            }
            for _ in 0..draw(6) {
                map.node_pairs.push(NodeTypePair {
                    type_node: types[draw(2)].into(),
                    type_edge: types[draw(2)].into(),
                    conflicts: vec![NodeConflict {
                        node: nodes[draw(6)].into(),
                        edge: edges[draw(4)].into(),
                        junction_nodes: None,
                        depth: 0.2,
                        distance: 0.0,
                    }],
                });
            }
            let spots = Spots::new(&layout, &map).expect("setup");
            let mut drawn = || {
                let length = draw(4);
                let on: Vec<&str> = (0..=length).map(|_| nodes[draw(6)]).collect();
                let by: Vec<&str> = (0..length).map(|_| edges[draw(4)]).collect();
                route(types[draw(2)], &on, &by)
            };
            let (a, b) = (drawn(), drawn());
            let mut named = Vec::new();
            for (p, x) in a.pieces().iter().enumerate() {
                for (q, y) in b.pieces().iter().enumerate() {
                    if spots.conflict(a.type_name(), x, b.type_name(), y) == Some(true) {
                        named.push((p, q));
                    }
                }
            }
            conflicts += named.len();
            let found = groups(&spots, &a, &b).expect("names");
            let mut all = found.concat();
            all.sort_unstable();
            assert_eq!(all, named);
            // the same with the numbers kept
            let (mut a_kept, mut b_kept) = (a.clone(), b.clone());
            assert_eq!(a_kept.number(&spots), Ok(()));
            assert_eq!(b_kept.number(&spots), Ok(()));
            assert_eq!(groups(&spots, &a_kept, &b_kept), Ok(found));
        }
        assert!(conflicts > 1_000);
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

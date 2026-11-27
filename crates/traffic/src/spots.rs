//! Spots (D2): which pieces of two routes two robots must not use at the same time.
//!
//! Two passes conflict if they use the same node, the same edge, or a pair the
//! conflict map lists for the two robot types (edge–edge, node–edge or
//! node–node), or two pieces of one RELEASE zone ([`Spots::add_zone`]).

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

use botsim_space::ConflictMap;
use vdma_lif::Lif;

/// A node or an edge of the layout, by its LIF id.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum Piece {
    Node(String),
    Edge(String),
}

impl fmt::Display for Piece {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Piece::Node(id) => write!(f, "node {id}"),
            Piece::Edge(id) => write!(f, "edge {id}"),
        }
    }
}

/// Why the layout, the conflict map or a zone cannot be used.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SetupError {
    /// The layout has two nodes or two edges with the same id.
    DuplicatePiece(Piece),
    /// The conflict map or a zone names a piece the layout does not have.
    UnknownPiece(Piece),
    /// The conflict map or a zone lists something for a type that is not in the
    /// map's type list.
    UnknownType(String),
    /// A zone was added before.
    DuplicateZone {
        zone_set_id: String,
        zone_id: String,
    },
}

impl fmt::Display for SetupError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SetupError::DuplicatePiece(p) => write!(f, "the layout has {p} twice"),
            SetupError::UnknownPiece(p) => {
                write!(
                    f,
                    "the conflict map or a zone names {p}, which the layout does not have"
                )
            }
            SetupError::UnknownType(t) => {
                write!(f, "the conflict map or a zone names the unknown type {t}")
            }
            SetupError::DuplicateZone {
                zone_set_id,
                zone_id,
            } => write!(f, "zone {zone_id} of zone set {zone_set_id} is there twice"),
        }
    }
}

impl Error for SetupError {}

/// The lookup "route piece → conflicting pieces" for every pair of robot types.
///
/// Robot types are named `manufacturer/seriesName`, as in the conflict map.
#[derive(Debug, Clone)]
pub struct Spots {
    types: BTreeMap<String, usize>,
    /// All nodes, then all edges, in layout order.
    pieces: Vec<Piece>,
    index: BTreeMap<Piece, usize>,
    /// (type, piece, other type) → the pieces of the other type listed against it.
    listed: BTreeMap<(usize, usize, usize), BTreeSet<usize>>,
    /// (zone set id, zone id) → by type name, the pieces inside the zone.
    zones: BTreeMap<(String, String), Inside>,
}

/// By robot type, the pieces on which a robot of that type is inside a zone.
type Inside = BTreeMap<String, BTreeSet<Piece>>;

impl Spots {
    pub fn new(lif: &Lif, map: &ConflictMap) -> Result<Spots, SetupError> {
        let mut spots = Spots {
            types: BTreeMap::new(),
            pieces: Vec::new(),
            index: BTreeMap::new(),
            listed: BTreeMap::new(),
            zones: BTreeMap::new(),
        };
        for (i, name) in map.types.iter().enumerate() {
            spots.types.insert(name.clone(), i);
        }

        let layouts = || lif.origins.iter().flat_map(|o| &o.layouts);
        for n in layouts().flat_map(|l| &l.nodes) {
            spots.add_piece(Piece::Node(n.node_id.clone()))?;
        }
        for e in layouts().flat_map(|l| &l.edges) {
            spots.add_piece(Piece::Edge(e.edge_id.clone()))?;
        }

        for pair in &map.pairs {
            let a = spots.setup_type(&pair.type_a)?;
            let b = spots.setup_type(&pair.type_b)?;
            for c in &pair.conflicts {
                let p = spots.setup_piece(Piece::Edge(c.edge_a.clone()))?;
                let q = spots.setup_piece(Piece::Edge(c.edge_b.clone()))?;
                spots.list(a, p, b, q);
            }
        }
        for pair in &map.node_pairs {
            let a = spots.setup_type(&pair.type_node)?;
            let b = spots.setup_type(&pair.type_edge)?;
            for c in &pair.conflicts {
                let p = spots.setup_piece(Piece::Node(c.node.clone()))?;
                let q = spots.setup_piece(Piece::Edge(c.edge.clone()))?;
                spots.list(a, p, b, q);
            }
        }
        for pair in &map.node_node_pairs {
            let a = spots.setup_type(&pair.type_a)?;
            let b = spots.setup_type(&pair.type_b)?;
            for c in &pair.conflicts {
                let p = spots.setup_piece(Piece::Node(c.node_a.clone()))?;
                let q = spots.setup_piece(Piece::Node(c.node_b.clone()))?;
                spots.list(a, p, b, q);
            }
        }
        Ok(spots)
    }

    /// One more spot: a RELEASE zone, given as the pieces on which a robot of each
    /// type is inside it. The caller works them out; list every such piece, also
    /// the nodes between listed edges. From now on any two robots on pieces of this
    /// zone conflict. The two ids are those zone requests name it by. A zone with
    /// an unknown name, or one added before, changes nothing.
    pub fn add_zone(
        &mut self,
        zone_set_id: &str,
        zone_id: &str,
        inside: &BTreeMap<String, Vec<Piece>>,
    ) -> Result<(), SetupError> {
        let id = (zone_set_id.to_string(), zone_id.to_string());
        if self.zones.contains_key(&id) {
            return Err(SetupError::DuplicateZone {
                zone_set_id: id.0,
                zone_id: id.1,
            });
        }
        let mut found = Vec::new();
        for (name, pieces) in inside {
            let t = self.setup_type(name)?;
            for piece in pieces {
                found.push((t, self.setup_piece(piece.clone())?));
            }
        }
        for &(a, p) in &found {
            for &(b, q) in &found {
                self.list(a, p, b, q);
            }
        }
        let inside = inside
            .iter()
            .map(|(name, pieces)| (name.clone(), pieces.iter().cloned().collect()))
            .collect();
        self.zones.insert(id, inside);
        Ok(())
    }

    /// The pieces inside a zone, by type name; `None` for an unknown zone.
    pub(crate) fn zone(&self, zone_set_id: &str, zone_id: &str) -> Option<&Inside> {
        self.zones
            .get(&(zone_set_id.to_string(), zone_id.to_string()))
    }

    /// The pieces a robot of `other_type` must not use while a robot of `type_name`
    /// uses `piece`: the same piece first, then the listed ones in layout order.
    /// `None` if a type or the piece is unknown.
    pub fn conflicting(
        &self,
        type_name: &str,
        piece: &Piece,
        other_type: &str,
    ) -> Option<Vec<&Piece>> {
        let a = *self.types.get(type_name)?;
        let b = *self.types.get(other_type)?;
        let p = *self.index.get(piece)?;
        let listed = self.listed.get(&(a, p, b)).into_iter().flatten();
        Some(
            std::iter::once(p)
                .chain(listed.copied())
                .filter_map(|q| self.pieces.get(q))
                .collect(),
        )
    }

    /// Must two robots of these types not use `piece_a` and `piece_b` at the same
    /// time? `None` if a type or a piece is unknown.
    pub fn conflict(
        &self,
        type_a: &str,
        piece_a: &Piece,
        type_b: &str,
        piece_b: &Piece,
    ) -> Option<bool> {
        let a = *self.types.get(type_a)?;
        let b = *self.types.get(type_b)?;
        let p = *self.index.get(piece_a)?;
        let q = *self.index.get(piece_b)?;
        Some(self.numbers_conflict(a, p, b, q))
    }

    /// Is this `manufacturer/seriesName` in the conflict map's type list?
    pub fn has_type(&self, type_name: &str) -> bool {
        self.types.contains_key(type_name)
    }

    /// Is this node or edge in the layout?
    pub fn has_piece(&self, piece: &Piece) -> bool {
        self.index.contains_key(piece)
    }

    /// The number a type has here, so that its name is looked up only once.
    pub(crate) fn type_number(&self, type_name: &str) -> Option<usize> {
        self.types.get(type_name).copied()
    }

    /// The number a node or edge has here.
    pub(crate) fn piece_number(&self, piece: &Piece) -> Option<usize> {
        self.index.get(piece).copied()
    }

    /// By their numbers: the pieces listed against piece `p` of type `a` for a
    /// robot of type `b`. The piece itself, which conflicts too, is not among them.
    pub(crate) fn listed_against(
        &self,
        a: usize,
        p: usize,
        b: usize,
    ) -> impl Iterator<Item = usize> {
        self.listed.get(&(a, p, b)).into_iter().flatten().copied()
    }

    /// By their numbers: do piece `p` of type `a` and piece `q` of type `b`
    /// conflict, being the same or listed against each other?
    pub(crate) fn numbers_conflict(&self, a: usize, p: usize, b: usize, q: usize) -> bool {
        p == q
            || self
                .listed
                .get(&(a, p, b))
                .is_some_and(|listed| listed.contains(&q))
    }

    fn add_piece(&mut self, piece: Piece) -> Result<(), SetupError> {
        if self
            .index
            .insert(piece.clone(), self.pieces.len())
            .is_some()
        {
            return Err(SetupError::DuplicatePiece(piece));
        }
        self.pieces.push(piece);
        Ok(())
    }

    fn setup_type(&self, name: &str) -> Result<usize, SetupError> {
        self.types
            .get(name)
            .copied()
            .ok_or_else(|| SetupError::UnknownType(name.to_string()))
    }

    fn setup_piece(&self, piece: Piece) -> Result<usize, SetupError> {
        self.index
            .get(&piece)
            .copied()
            .ok_or(SetupError::UnknownPiece(piece))
    }

    /// A robot of type `a` on `p` and one of type `b` on `q` conflict, both ways round.
    fn list(&mut self, a: usize, p: usize, b: usize, q: usize) {
        if p != q {
            self.listed.entry((a, p, b)).or_default().insert(q);
            self.listed.entry((b, q, a)).or_default().insert(p);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{LARGE, SMALL, edge, edge_pair, empty_map, lif, node};
    use botsim_space::{NodeConflict, NodeNodeConflict, NodeNodeTypePair, NodeTypePair};

    /// Two edges without a common node: E1 from A to B, E2 from C to D.
    fn layout() -> Lif {
        lif(&["A", "B", "C", "D"], &[("E1", "A", "B"), ("E2", "C", "D")])
    }

    #[test]
    fn same_piece_always_conflicts() {
        let spots = Spots::new(&layout(), &empty_map()).expect("setup");
        for a in [SMALL, LARGE] {
            for b in [SMALL, LARGE] {
                for p in [node("A"), edge("E1")] {
                    assert_eq!(spots.conflict(a, &p, b, &p), Some(true));
                    assert_eq!(spots.conflicting(a, &p, b), Some(vec![&p]));
                }
                assert_eq!(spots.conflict(a, &edge("E1"), b, &edge("E2")), Some(false));
            }
        }
    }

    #[test]
    fn edge_pair_counts_only_for_its_types_in_both_directions() {
        let mut map = empty_map();
        map.pairs.push(edge_pair(SMALL, "E1", LARGE, "E2"));
        let spots = Spots::new(&layout(), &map).expect("setup");
        let (e1, e2) = (edge("E1"), edge("E2"));
        assert_eq!(spots.conflict(SMALL, &e1, LARGE, &e2), Some(true));
        assert_eq!(spots.conflict(LARGE, &e2, SMALL, &e1), Some(true));
        // the other way round: the large robot on E1, the small one on E2
        assert_eq!(spots.conflict(LARGE, &e1, SMALL, &e2), Some(false));
        assert_eq!(spots.conflict(SMALL, &e1, SMALL, &e2), Some(false));
        assert_eq!(spots.conflicting(SMALL, &e1, LARGE), Some(vec![&e1, &e2]));
        assert_eq!(spots.conflicting(LARGE, &e2, SMALL), Some(vec![&e2, &e1]));
    }

    #[test]
    fn pair_of_one_type_counts_both_ways_round() {
        // the map lists a type with itself only once per edge pair
        let mut map = empty_map();
        map.pairs.push(edge_pair(LARGE, "E1", LARGE, "E2"));
        let spots = Spots::new(&layout(), &map).expect("setup");
        let (e1, e2) = (edge("E1"), edge("E2"));
        assert_eq!(spots.conflict(LARGE, &e1, LARGE, &e2), Some(true));
        assert_eq!(spots.conflict(LARGE, &e2, LARGE, &e1), Some(true));
    }

    #[test]
    fn node_pair_is_standing_robot_against_driving_robot() {
        let mut map = empty_map();
        map.node_pairs.push(NodeTypePair {
            type_node: LARGE.into(),
            type_edge: SMALL.into(),
            conflicts: vec![NodeConflict {
                node: "C".into(),
                edge: "E1".into(),
                junction_nodes: None,
                depth: 0.2,
                distance: 0.0,
            }],
        });
        let spots = Spots::new(&layout(), &map).expect("setup");
        let (c, e1) = (node("C"), edge("E1"));
        assert_eq!(spots.conflict(LARGE, &c, SMALL, &e1), Some(true));
        assert_eq!(spots.conflict(SMALL, &e1, LARGE, &c), Some(true));
        assert_eq!(spots.conflict(SMALL, &c, LARGE, &e1), Some(false));
    }

    #[test]
    fn node_node_pair_counts_for_its_types_in_both_directions() {
        let mut map = empty_map();
        map.node_node_pairs.push(NodeNodeTypePair {
            type_a: SMALL.into(),
            type_b: LARGE.into(),
            conflicts: vec![NodeNodeConflict {
                node_a: "A".into(),
                node_b: "C".into(),
                depth: 0.2,
                distance: 0.0,
            }],
        });
        let spots = Spots::new(&layout(), &map).expect("setup");
        let (a, c) = (node("A"), node("C"));
        assert_eq!(spots.conflict(SMALL, &a, LARGE, &c), Some(true));
        assert_eq!(spots.conflict(LARGE, &c, SMALL, &a), Some(true));
        // the other way round: the large robot on A, the small one on C
        assert_eq!(spots.conflict(LARGE, &a, SMALL, &c), Some(false));
        assert_eq!(spots.conflict(SMALL, &a, SMALL, &c), Some(false));
        assert_eq!(spots.conflicting(SMALL, &a, LARGE), Some(vec![&a, &c]));
    }

    fn inside(types: &[(&str, &[Piece])]) -> BTreeMap<String, Vec<Piece>> {
        types
            .iter()
            .map(|(t, pieces)| (t.to_string(), pieces.to_vec()))
            .collect()
    }

    #[test]
    fn pieces_of_one_zone_conflict() {
        let mut spots = Spots::new(&layout(), &empty_map()).expect("setup");
        let (a, e1, e2) = (node("A"), edge("E1"), edge("E2"));
        // a small robot is inside on A and E1, a large one on E2
        let zone = inside(&[
            (SMALL, &[a.clone(), e1.clone()]),
            (LARGE, std::slice::from_ref(&e2)),
        ]);
        assert_eq!(spots.add_zone("S", "Z", &zone), Ok(()));
        // the same ids once only; the same zone id in another set is another zone
        assert_eq!(
            spots.add_zone("S", "Z", &inside(&[(SMALL, std::slice::from_ref(&e2))])),
            Err(SetupError::DuplicateZone {
                zone_set_id: "S".into(),
                zone_id: "Z".into(),
            })
        );
        assert_eq!(spots.add_zone("T", "Z", &inside(&[])), Ok(()));
        assert_eq!(spots.conflict(SMALL, &e1, LARGE, &e2), Some(true));
        assert_eq!(spots.conflict(LARGE, &e2, SMALL, &a), Some(true));
        // two small robots, in both directions
        assert_eq!(spots.conflict(SMALL, &a, SMALL, &e1), Some(true));
        assert_eq!(spots.conflict(SMALL, &e1, SMALL, &a), Some(true));
        // the large robot on E1 is not inside
        assert_eq!(spots.conflict(LARGE, &e1, SMALL, &a), Some(false));
        assert_eq!(spots.conflict(SMALL, &e1, SMALL, &e2), Some(false));
    }

    #[test]
    fn a_zone_with_an_unknown_name_changes_nothing() {
        let mut spots = Spots::new(&layout(), &empty_map()).expect("setup");
        let (a, e1) = (node("A"), edge("E1"));
        let piece = inside(&[(SMALL, &[a.clone(), e1.clone(), edge("E9")])]);
        assert_eq!(
            spots.add_zone("S", "Z", &piece),
            Err(SetupError::UnknownPiece(edge("E9")))
        );
        let known: &[Piece] = &[a.clone(), e1.clone()];
        let other = inside(&[(SMALL, known), ("ACME/Z", known)]);
        assert_eq!(
            spots.add_zone("S", "Z", &other),
            Err(SetupError::UnknownType("ACME/Z".into()))
        );
        assert_eq!(spots.conflict(SMALL, &a, SMALL, &e1), Some(false));
    }

    #[test]
    fn unknown_names_are_reported() {
        let mut map = empty_map();
        map.pairs.push(edge_pair(SMALL, "E1", LARGE, "E9"));
        assert_eq!(
            Spots::new(&layout(), &map).err(),
            Some(SetupError::UnknownPiece(edge("E9")))
        );
        let mut map = empty_map();
        map.pairs.push(edge_pair(SMALL, "E1", "ACME/Other", "E2"));
        assert_eq!(
            Spots::new(&layout(), &map).err(),
            Some(SetupError::UnknownType("ACME/Other".into()))
        );
        let spots = Spots::new(&layout(), &empty_map()).expect("setup");
        assert_eq!(spots.conflict(SMALL, &node("Z"), SMALL, &node("A")), None);
        assert_eq!(spots.conflicting("ACME/Other", &node("A"), SMALL), None);
    }

    #[test]
    fn duplicate_id_is_reported() {
        let mut lif = layout();
        let twice = lif.origins[0].layouts[0].nodes[0].clone();
        lif.origins[0].layouts[0].nodes.push(twice);
        assert_eq!(
            Spots::new(&lif, &empty_map()).err(),
            Some(SetupError::DuplicatePiece(node("A")))
        );
    }
}

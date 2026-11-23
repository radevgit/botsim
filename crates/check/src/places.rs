//! Looking up the position of layout objects for findings.

use std::collections::HashMap;

use vdma_lif::Layout;
use vdma_lif::edge::Edge;
use vdma_lif::node::Node;
use vdma_lif::station::Station;

use crate::finding::{FindingPlace, PlaceKind};

/// Index of one layout by id. If an id occurs twice, the first object counts.
pub struct LayoutIndex<'a> {
    nodes: HashMap<&'a str, &'a Node>,
    edges: HashMap<&'a str, &'a Edge>,
    stations: HashMap<&'a str, &'a Station>,
}

impl<'a> LayoutIndex<'a> {
    pub fn new(layout: &'a Layout) -> Self {
        let mut nodes = HashMap::new();
        for n in &layout.nodes {
            nodes.entry(n.node_id.as_str()).or_insert(n);
        }
        let mut edges = HashMap::new();
        for e in &layout.edges {
            edges.entry(e.edge_id.as_str()).or_insert(e);
        }
        let mut stations = HashMap::new();
        for s in layout.stations.iter().flatten() {
            stations.entry(s.station_id.as_str()).or_insert(s);
        }
        LayoutIndex {
            nodes,
            edges,
            stations,
        }
    }

    pub fn node(&self, id: &str) -> Option<FindingPlace> {
        let n = self.nodes.get(id)?;
        Some(FindingPlace {
            kind: PlaceKind::Node,
            id: id.to_string(),
            map: Some(n.map_id.clone()),
            points: vec![[n.node_position.x, n.node_position.y]],
        })
    }

    /// Start and end point; a missing end node leaves out that point. `None` if
    /// neither end exists.
    pub fn edge(&self, id: &str) -> Option<FindingPlace> {
        let e = self.edges.get(id)?;
        let start = self.nodes.get(e.start_node_id.as_str());
        let end = self.nodes.get(e.end_node_id.as_str());
        let points: Vec<[f64; 2]> = [start, end]
            .into_iter()
            .flatten()
            .map(|n| [n.node_position.x, n.node_position.y])
            .collect();
        if points.is_empty() {
            return None;
        }
        let map = start.or(end).map(|n| n.map_id.clone());
        Some(FindingPlace {
            kind: PlaceKind::Edge,
            id: id.to_string(),
            map,
            points,
        })
    }

    /// The station's own position if it has one, else its first interaction node
    /// that exists. The map is the one of the first interaction node.
    pub fn station(&self, id: &str) -> Option<FindingPlace> {
        let s = self.stations.get(id)?;
        let node = s
            .interaction_node_ids
            .iter()
            .find_map(|i| self.nodes.get(i.as_str()));
        let point = match (&s.station_position, node) {
            (Some(p), _) => [p.x, p.y],
            (None, Some(n)) => [n.node_position.x, n.node_position.y],
            (None, None) => return None,
        };
        Some(FindingPlace {
            kind: PlaceKind::Station,
            id: id.to_string(),
            map: node.map(|n| n.map_id.clone()),
            points: vec![point],
        })
    }

    pub fn nodes<'i>(&self, ids: impl IntoIterator<Item = &'i str>) -> Vec<FindingPlace> {
        ids.into_iter().filter_map(|i| self.node(i)).collect()
    }

    pub fn edges<'i>(&self, ids: impl IntoIterator<Item = &'i str>) -> Vec<FindingPlace> {
        ids.into_iter().filter_map(|i| self.edge(i)).collect()
    }

    pub fn stations<'i>(&self, ids: impl IntoIterator<Item = &'i str>) -> Vec<FindingPlace> {
        ids.into_iter().filter_map(|i| self.station(i)).collect()
    }
}

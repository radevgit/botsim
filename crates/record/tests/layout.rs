use botsim_model::graph::Graph;
use botsim_record::{FORMAT_VERSION, LayoutLine};
use vdma_lif::Lif;

fn two_floors() -> Graph {
    let text = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/two_floors.lif.json"
    ))
    .unwrap();
    let lif: Lif = serde_json::from_str(&text).unwrap();
    Graph::from_lif(&lif).unwrap()
}

#[test]
fn layout_line_copies_the_graph() {
    let graph = two_floors();
    let line = LayoutLine::from_graph(&graph);
    assert_eq!(line.version, FORMAT_VERSION);
    assert_eq!(line.maps, graph.maps);
    assert_eq!(line.nodes.len(), graph.nodes.len());
    assert_eq!(line.edges.len(), graph.edges.len());
    assert!(line.maps.len() >= 2);
    assert!(line.edges.iter().any(|e| e.map_transition));
}

#[test]
fn layout_line_json_round_trip() {
    let line = LayoutLine::from_graph(&two_floors());
    let text = serde_json::to_string(&line).unwrap();
    assert!(!text.contains('\n'), "one line");
    assert!(
        text.starts_with(r#"{"type":"layout","version":1,"#),
        "{text}"
    );
    let back: LayoutLine = serde_json::from_str(&text).unwrap();
    assert_eq!(back, line);
}

mod common;

use botsim_record::{
    GeometryError, GeometryFile, GeometryKind, LayoutLine, Outline, StaticShape, read_recording,
};

const EXAMPLE: &str = "corridor_deadlock.geometry.json";

fn parse(text: &str) -> Result<GeometryFile, GeometryError> {
    GeometryFile::from_json_str(text)
}

fn one(shape: &str) -> String {
    format!(r#"{{"version":1,"shapes":[{shape}]}}"#)
}

#[test]
fn the_example_file_loads_and_fits_the_corridor_layout() {
    let file = parse(&common::read(EXAMPLE)).unwrap();
    assert_eq!(file.shapes.len(), 9);
    let kinds: Vec<GeometryKind> = file.shapes.iter().map(|s| s.kind).collect();
    for k in [
        GeometryKind::Wall,
        GeometryKind::Rack,
        GeometryKind::Shelf,
        GeometryKind::Station,
    ] {
        assert!(kinds.contains(&k), "{k:?}");
    }
    assert_eq!(file.shapes[0].shape, Outline::Polyline);
    assert_eq!(file.shapes[4].shape, Outline::Polygon);

    let layout = LayoutLine::from_graph(&common::corridor_graph())
        .with_geometry(&file)
        .unwrap();
    assert_eq!(layout.geometry.len(), 9);
    assert!(
        layout.geometry.iter().all(|s| s.map == 0),
        "map M is index 0"
    );
    // file order is kept
    assert_eq!(layout.geometry[0].id.as_deref(), Some("wall-north"));
}

#[test]
fn load_reads_a_file_and_reports_a_missing_one() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(EXAMPLE);
    assert_eq!(GeometryFile::load(&path).unwrap().shapes.len(), 9);
    let err = GeometryFile::load(std::path::Path::new("nope.json")).unwrap_err();
    assert!(matches!(err, GeometryError::Read { .. }));
    assert!(err.to_string().starts_with("cannot read nope.json"));
}

#[test]
fn the_layout_line_carries_the_geometry_through_a_file() {
    let layout = LayoutLine::from_graph(&common::corridor_graph())
        .with_geometry(&parse(&common::read(EXAMPLE)).unwrap())
        .unwrap();
    let text = serde_json::to_string(&layout).unwrap();
    assert!(
        text.contains(r#""geometry":[{"id":"wall-north","map":0,"kind":"wall","shape":"polyline""#)
    );
    assert_eq!(serde_json::from_str::<LayoutLine>(&text).unwrap(), layout);

    // a layout line without a geometry field (no geometry) still reads
    let plain = r#"{"type":"layout","version":1,"maps":["M"],"nodes":[],"edges":[]}"#;
    let rec = read_recording(plain.as_bytes()).unwrap();
    assert!(rec.layout.geometry.is_empty());
}

#[test]
fn bad_geometry_files_are_rejected_with_a_reason() {
    let wall = |v: &str| {
        one(&format!(
            r#"{{"id":"w","mapId":"M","kind":"wall","shape":"polyline","vertices":{v}}}"#
        ))
    };

    assert!(matches!(parse("not json"), Err(GeometryError::Parse(_))));
    assert!(matches!(
        parse(r#"{"version":2,"shapes":[]}"#),
        Err(GeometryError::UnsupportedVersion {
            found: 2,
            supported: 1
        })
    ));
    // a polyline needs 2 vertices
    assert!(matches!(
        parse(&wall(r#"[{"x":0,"y":0}]"#)),
        Err(GeometryError::TooFewVertices {
            need: 2,
            found: 1,
            ..
        })
    ));
    // a polygon needs 3
    let polygon = one(
        r#"{"mapId":"M","kind":"rack","shape":"polygon","vertices":[{"x":0,"y":0},{"x":1,"y":0}]}"#,
    );
    let err = parse(&polygon).unwrap_err();
    assert!(matches!(
        err,
        GeometryError::TooFewVertices {
            need: 3,
            found: 2,
            ..
        }
    ));
    assert_eq!(err.to_string(), "geometry #1: needs 3 vertices, has 2");
    // an unknown kind, an unknown field and a typo are caught
    for bad in [
        r#"{"mapId":"M","kind":"door","shape":"polyline","vertices":[{"x":0,"y":0},{"x":1,"y":0}]}"#,
        r#"{"mapId":"M","kind":"wall","shape":"circle","vertices":[]}"#,
        r#"{"mapId":"M","kind":"wall","shape":"polyline","colour":"red","vertices":[{"x":0,"y":0},{"x":1,"y":0}]}"#,
        r#"{"map":"M","kind":"wall","shape":"polyline","vertices":[{"x":0,"y":0},{"x":1,"y":0}]}"#,
    ] {
        assert!(
            matches!(parse(&one(bad)), Err(GeometryError::Parse(_))),
            "{bad}"
        );
    }
    // JSON cannot hold NaN, so check the validation on a value built in code
    let mut file = parse(&wall(r#"[{"x":0,"y":0},{"x":1,"y":0}]"#)).unwrap();
    file.shapes[0].vertices[1].x = f64::NAN;
    assert!(matches!(
        file.validate(),
        Err(GeometryError::NotFinite { .. })
    ));
}

#[test]
fn a_shape_on_an_unknown_map_is_an_error() {
    let file = parse(&one(
        r#"{"id":"w","mapId":"Level_9","kind":"wall","shape":"polyline","vertices":[{"x":0,"y":0},{"x":1,"y":0}]}"#,
    ))
    .unwrap();
    let err = LayoutLine::from_graph(&common::corridor_graph())
        .with_geometry(&file)
        .unwrap_err();
    assert_eq!(
        err,
        GeometryError::UnknownMap {
            shape: "'w'".into(),
            map_id: "Level_9".into()
        }
    );
    assert_eq!(
        err.to_string(),
        "geometry 'w': the layout has no map 'Level_9'"
    );
}

#[test]
fn the_second_floor_gets_its_own_map_index() {
    // two_floors.lif.json has two maps; a shape on the second one gets index 1
    let text = common::read("two_floors.lif.json");
    let lif: vdma_lif::Lif = serde_json::from_str(&text).unwrap();
    let graph = botsim_core::graph::Graph::from_lif(&lif).unwrap();
    assert!(graph.maps.len() >= 2);
    let second = &graph.maps[1];
    let file = parse(&one(&format!(
        r#"{{"mapId":"{second}","kind":"other","shape":"polygon","vertices":[{{"x":0,"y":0}},{{"x":1,"y":0}},{{"x":1,"y":1}}]}}"#
    )))
    .unwrap();
    let shapes: Vec<StaticShape> = file.resolve(&graph.maps).unwrap();
    assert_eq!(shapes[0].map, 1);
    assert_eq!(shapes[0].id, None);
}

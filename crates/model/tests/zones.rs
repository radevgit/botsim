use botsim_model::graph::Graph;
use botsim_model::zones::{ZoneError, Zones};
use vda5050::zone_set::{ZoneKind, ZoneSet};
use vdma_lif::Lif;

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap() // map "M"
}

/// A zone set for map `map` with the given zone JSON objects.
fn set(map: &str, zones: &[String]) -> ZoneSet {
    serde_json::from_str(&format!(
        r#"{{"mapId":"{map}","zoneSetId":"zs1","zones":[{}]}}"#,
        zones.join(",")
    ))
    .unwrap()
}

/// A zone with the given vertices and the extra JSON after them.
fn zone(id: &str, pts: &[(f64, f64)], extra: &str) -> String {
    let v: Vec<String> = pts
        .iter()
        .map(|(x, y)| format!(r#"{{"x":{x},"y":{y}}}"#))
        .collect();
    format!(
        r#"{{"zoneId":"{id}","vertices":[{}],{extra}}}"#,
        v.join(",")
    )
}

const SQUARE: [(f64, f64); 4] = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)];

fn blocked(id: &str, pts: &[(f64, f64)]) -> String {
    zone(id, pts, r#""zoneType":"BLOCKED""#)
}

fn error(zones: &[String]) -> ZoneError {
    Zones::new(&graph(), set("M", zones)).unwrap_err()
}

#[test]
fn a_good_zone_set_loads_with_all_types() {
    let zs = vec![
        zone(
            "rel",
            &SQUARE,
            r#""zoneType":"RELEASE","releaseLossBehavior":"STOP""#,
        ),
        blocked("blk", &[(10.0, 0.0), (14.0, 0.0), (12.0, 3.0)]),
        zone(
            "slow",
            &SQUARE,
            r#""zoneType":"SPEED_LIMIT","maximumSpeed":0.5"#,
        ),
        zone(
            "prio",
            &SQUARE,
            r#""zoneType":"PRIORITY","priorityFactor":1.0"#,
        ),
        zone(
            "pen",
            &SQUARE,
            r#""zoneType":"PENALTY","penaltyFactor":0.0"#,
        ),
        zone(
            "dir",
            &SQUARE,
            r#""zoneType":"DIRECTED","direction":1.5,"directedLimitation":"SOFT""#,
        ),
        zone("line", &SQUARE, r#""zoneType":"LINE_GUIDED""#),
    ];
    let z = Zones::new(&graph(), set("M", &zs)).unwrap();
    assert_eq!(
        (z.zone_set_id.as_str(), z.map, z.zones.len()),
        ("zs1", 0, 7)
    );
    assert!(matches!(
        z.zone("rel").unwrap().kind,
        ZoneKind::Release { .. }
    ));
    assert!(z.zone("nope").is_none());
}

#[test]
fn the_map_must_be_in_the_layout() {
    let e = Zones::new(&graph(), set("Z", &[blocked("a", &SQUARE)])).unwrap_err();
    assert_eq!(e, ZoneError::UnknownMap("Z".into()));
    assert!(e.to_string().contains("\"Z\""));
}

#[test]
fn zone_ids_must_be_unique() {
    let e = error(&[blocked("a", &SQUARE), blocked("a", &SQUARE)]);
    assert_eq!(e, ZoneError::DuplicateZone("a".into()));
}

#[test]
fn bad_polygons_are_refused() {
    // fewer than 3 vertices
    assert_eq!(
        error(&[blocked("a", &[(0.0, 0.0), (1.0, 0.0)])]),
        ZoneError::TooFewVertices("a".into())
    );
    // a vertex twice in a row, also first = last
    assert_eq!(
        error(&[blocked(
            "a",
            &[(0.0, 0.0), (0.0, 0.0), (1.0, 1.0), (2.0, 0.0)]
        )]),
        ZoneError::RepeatedVertex("a".into())
    );
    assert_eq!(
        error(&[blocked(
            "a",
            &[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]
        )]),
        ZoneError::RepeatedVertex("a".into())
    );
    // all on a line: no area
    assert_eq!(
        error(&[blocked("a", &[(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)])]),
        ZoneError::NoArea("a".into())
    );
    // a bow tie crosses itself
    assert_eq!(
        error(&[blocked(
            "a",
            &[(0.0, 0.0), (2.0, 2.0), (2.0, 0.0), (0.0, 2.0)]
        )]),
        ZoneError::SelfIntersecting("a".into())
    );
    // a side that touches another side
    assert_eq!(
        error(&[blocked(
            "a",
            &[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (2.0, 0.0), (0.0, 4.0)]
        )]),
        ZoneError::SelfIntersecting("a".into())
    );
}

#[test]
fn a_concave_polygon_is_fine() {
    // an L shape
    let l = [
        (0.0, 0.0),
        (4.0, 0.0),
        (4.0, 1.0),
        (1.0, 1.0),
        (1.0, 4.0),
        (0.0, 4.0),
    ];
    assert!(Zones::new(&graph(), set("M", &[blocked("l", &l)])).is_ok());
}

#[test]
fn type_parameters_are_checked() {
    let bad = |extra: &str| error(&[zone("a", &SQUARE, extra)]);
    let what = |e: ZoneError| match e {
        ZoneError::BadParameter { what, .. } => what,
        other => panic!("{other:?}"),
    };
    assert!(what(bad(r#""zoneType":"SPEED_LIMIT","maximumSpeed":0.0"#)).contains("maximumSpeed"));
    assert!(what(bad(r#""zoneType":"SPEED_LIMIT","maximumSpeed":-1.0"#)).contains("maximumSpeed"));
    assert!(what(bad(r#""zoneType":"PRIORITY","priorityFactor":1.5"#)).contains("priorityFactor"));
    assert!(what(bad(r#""zoneType":"PENALTY","penaltyFactor":-0.1"#)).contains("penaltyFactor"));
}

#[test]
fn points_on_the_border_are_inside_and_the_zones_at_a_point_are_listed() {
    let l = [
        (0.0, 0.0),
        (4.0, 0.0),
        (4.0, 1.0),
        (1.0, 1.0),
        (1.0, 4.0),
        (0.0, 4.0),
    ];
    let zs = vec![
        blocked("sq", &SQUARE),
        blocked("l", &l),
        blocked("far", &[(10.0, 10.0), (12.0, 10.0), (11.0, 12.0)]),
    ];
    let z = Zones::new(&graph(), set("M", &zs)).unwrap();
    let at = |x, y| -> Vec<&str> {
        z.at(x, y)
            .iter()
            .map(|&i| z.zones[i].zone_id.as_str())
            .collect()
    };

    assert_eq!(at(2.0, 2.0), vec!["sq"], "inside the square, outside the L");
    assert_eq!(at(0.5, 0.5), vec!["sq", "l"]);
    assert_eq!(at(3.0, 0.5), vec!["sq", "l"], "the arm of the L");
    // border and corner count as inside
    assert_eq!(at(4.0, 2.0), vec!["sq"]);
    assert_eq!(at(0.0, 0.0), vec!["sq", "l"]);
    assert_eq!(at(2.5, 1.0), vec!["sq", "l"], "on the inner side of the L");
    // outside everything
    assert!(at(-0.1, 2.0).is_empty());
    assert!(at(5.0, 5.0).is_empty());
    assert_eq!(at(11.0, 10.5), vec!["far"]);
    assert!(Zones::contains(z.zone("sq").unwrap(), 1.0, 1.0));
}

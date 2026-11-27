use std::collections::HashSet;

use botsim_core::drive::{DriveError, Driver, RouteEdge};
use botsim_core::errors::{ErrorType, RobotError};
use botsim_core::order_route::OrderError;
use botsim_core::state_report::{Header, build_state};
use botsim_model::graph::Graph;
use botsim_model::orientation::OrientationError;
use botsim_model::spec::RobotSpec;
use vda5050::factsheet::Factsheet;
use vda5050::state::ErrorLevel::{self, Critical, Fatal, Urgent, Warning};
use vdma_lif::Lif;

/// Table 9 of VDA 5050 (error type, level), copied from the specification.
const TABLE_9: [(&str, ErrorLevel); 28] = [
    ("UNSUPPORTED_PARAMETER", Critical),
    ("NO_ORDER_TO_CANCEL", Warning),
    ("VALIDATION_FAILURE", Warning),
    ("INVALID_ORDER_ACTION", Warning),
    ("INVALID_INSTANT_ACTION", Warning),
    ("OUTDATED_ORDER_UPDATE", Warning),
    ("SAME_ORDER_UPDATE_ID", Warning),
    ("ORDER_UPDATE_FOLLOWING_CANCEL", Warning),
    ("OUTSIDE_OF_CORRIDOR", Critical),
    ("INSUFFICIENT_MEMORY", Urgent),
    ("DUPLICATE_MAP", Warning),
    ("BLOCKED_ZONE_VIOLATION", Critical),
    ("DUPLICATE_ZONE_SET", Warning),
    ("ZONE_RELEASE_LOSS_HANDLING", Warning),
    ("ZONE_RELEASE_LOST", Critical),
    ("CORRIDOR_RELEASE_LOSS_HANDLING", Warning),
    ("CORRIDOR_RELEASE_LOST", Critical),
    ("ZONE_ACTION_CONFLICT", Critical),
    ("NODE_UNREACHABLE", Critical),
    ("LOCALIZATION_ERROR", Fatal),
    ("NO_ROUTE_TO_TARGET", Warning),
    ("OTHER_ORDER_ACTIVE", Warning),
    ("START_NODE_OUT_OF_RANGE", Warning),
    ("MOBILE_ROBOT_NOT_AVAILABLE", Warning),
    ("UNKNOWN_MAP_ID", Warning),
    ("ORIENTATION_UNREACHABLE", Warning),
    ("INSTANT_ACTION_STATES_FULL", Urgent),
    ("ZONE_ACTION_STATES_FULL", Urgent),
];

#[test]
fn the_enum_matches_table_9_of_the_standard() {
    let ours: Vec<(&str, ErrorLevel)> = ErrorType::ALL
        .iter()
        .map(|t| (t.as_str(), t.level()))
        .collect();
    assert_eq!(ours, TABLE_9.to_vec());
    let unique: HashSet<_> = ErrorType::ALL.iter().map(|t| t.as_str()).collect();
    assert_eq!(unique.len(), 28);
}

#[test]
fn order_errors_map_to_predefined_types_with_the_prescribed_references() {
    let r = RobotError::from_order_error(&OrderError::FirstNodeNotReleased, "o1", 3);
    assert_eq!(r.error_type, ErrorType::ValidationFailure);
    assert_eq!(
        r.references,
        vec![
            ("orderId".to_string(), "o1".to_string()),
            ("orderUpdateId".to_string(), "3".to_string())
        ]
    );

    let r = RobotError::from_order_error(
        &OrderError::Unknown {
            what: "node",
            id: "Z".into(),
        },
        "o1",
        3,
    );
    assert_eq!(r.error_type, ErrorType::NoRouteToTarget);
    assert_eq!(
        r.references,
        vec![("orderId".to_string(), "o1".to_string())]
    );

    let r =
        RobotError::from_order_error(&OrderError::EdgeDoesNotConnect { edge: "e".into() }, "o", 0);
    assert_eq!(r.error_type, ErrorType::NoRouteToTarget);
}

#[test]
fn drive_errors_map_to_predefined_types() {
    let r = RobotError::from_drive_error(
        &DriveError::Orientation {
            edge: "A-B".into(),
            error: OrientationError::GlobalNeedsOmnidirectional,
        },
        "o1",
        0,
    );
    assert_eq!(r.error_type, ErrorType::OrientationUnreachable);
    assert_eq!(
        r.references,
        vec![("edgeId".to_string(), "A-B".to_string())]
    );

    let r = RobotError::from_drive_error(
        &DriveError::NotContinuation {
            expected: "B".into(),
            found: "C".into(),
        },
        "o1",
        2,
    );
    assert_eq!(r.error_type, ErrorType::ValidationFailure);

    let r = RobotError::from_drive_error(&DriveError::MapTransition { edge: "e".into() }, "o1", 0);
    assert_eq!(r.error_type, ErrorType::NoRouteToTarget);
}

#[test]
fn errors_appear_in_a_schema_valid_state_message() {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    let g = Graph::from_lif(&lif).unwrap();
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    let spec = RobotSpec::from_factsheet(&f).unwrap();

    // a real rejection: A-B with an orientation of π/2
    let route = [RouteEdge::tangential(g.edge_idx("A-B").unwrap(), 1.5)];
    let err = Driver::new(&g, &spec, g.node_idx("A").unwrap(), 0.0, &route).unwrap_err();
    let robot_error = RobotError::from_drive_error(&err, "o1", 0);

    // the robot stands idle at A; the state carries the error
    let d = Driver::new(&g, &spec, g.node_idx("A").unwrap(), 0.0, &[]).unwrap();
    let header = Header {
        header_id: 1,
        timestamp: "2026-11-20T10:00:00.000Z".into(),
        version: "3.0.1".into(),
        manufacturer: "ACME".into(),
        serial_number: "R-001".into(),
    };
    let s = build_state(
        &g,
        &header,
        &d.progress(),
        std::slice::from_ref(&robot_error),
        &[],
    );

    assert_eq!(s.errors.len(), 1);
    let e = &s.errors[0];
    assert_eq!(e.error_type, "ORIENTATION_UNREACHABLE");
    assert_eq!(e.error_level, Warning);
    let refs = e.error_references.as_ref().unwrap();
    assert_eq!(
        (
            refs[0].reference_key.as_str(),
            refs[0].reference_value.as_str()
        ),
        ("edgeId", "A-B")
    );

    let schema: serde_json::Value =
        serde_json::from_str(include_str!("data/vda5050_state.schema.json")).unwrap();
    let validator = jsonschema::options()
        .should_validate_formats(true)
        .build(&schema)
        .unwrap();
    let instance = serde_json::to_value(&s).unwrap();
    let errors: Vec<String> = validator
        .iter_errors(&instance)
        .map(|e| e.to_string())
        .collect();
    assert!(errors.is_empty(), "{errors:?}");
}

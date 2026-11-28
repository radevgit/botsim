use botsim_core::actions::ActionTimes;
use botsim_core::world::World;
use botsim_core::zones::Zones;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::RobotSpec;
use botsim_model::time::unix_ms_from_iso8601;
use vda5050::factsheet::Factsheet;
use vda5050::order::Order;
use vda5050::responses::{GrantType, Responses};
use vda5050::state::RequestStatus;
use vda5050::zone_set::ZoneSet;
use vdma_lif::Lif;

const DT: f64 = 0.05;

fn graph() -> Graph {
    let lif: Lif = serde_json::from_str(include_str!("data/three_nodes.lif.json")).unwrap();
    Graph::from_lif(&lif).unwrap() // A (0,0) → B (10,0) → C (10,5)
}

fn spec() -> RobotSpec {
    let f: Factsheet =
        serde_json::from_str(include_str!("data/diff_robot.factsheet.json")).unwrap();
    RobotSpec::from_factsheet(&f).unwrap()
}

fn id() -> RobotId {
    RobotId::new("ACME", "R-1")
}

/// A RELEASE zone on the way A → B (x 4 … 7) with the given `releaseLossBehavior`
/// (`None`: the zone says nothing).
fn world(loss: Option<&str>) -> World {
    let behavior = loss
        .map(|b| format!(r#","releaseLossBehavior":"{b}""#))
        .unwrap_or_default();
    let set: ZoneSet = serde_json::from_str(&format!(
        r#"{{"mapId":"M","zoneSetId":"zs1","zones":[{{"zoneId":"gate","zoneType":"RELEASE"{behavior},
        "vertices":[{{"x":4,"y":-1}},{{"x":7,"y":-1}},{{"x":7,"y":1}},{{"x":4,"y":1}}]}}]}}"#
    ))
    .unwrap();
    let g = graph();
    let zones = Zones::new(&g, set).unwrap();
    let a = g.node_idx("A").unwrap();
    let mut w = World::new(g, DT, 0).unwrap(); // tick 0 is 1970-01-01T00:00:00Z
    w.set_zones(zones);
    w.add_robot(id(), spec(), ActionTimes::new(1.0), a, 0.0)
        .unwrap();
    // A → B → C, everything released
    let o: Order = serde_json::from_str(
        r#"{"headerId":1,"timestamp":"1970-01-01T00:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"A","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"B","sequenceId":2,"released":true,"actions":[]},
                 {"nodeId":"C","sequenceId":4,"released":true,"actions":[]}],
        "edges":[{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]},
                 {"edgeId":"B-C","sequenceId":3,"released":true,"actions":[]}]}"#,
    )
    .unwrap();
    w.send_order(&o).unwrap();
    w
}

fn respond(w: &mut World, request: &str, grant: GrantType, lease: Option<&str>) {
    let mut r = serde_json::json!({"requestId": request, "grantType": grant});
    if let Some(l) = lease {
        r["leaseExpiry"] = l.into();
    }
    let msg: Responses = serde_json::from_value(serde_json::json!({
        "headerId": 1, "timestamp": "1970-01-01T00:00:00.000Z", "version": "3.0.1",
        "manufacturer": "ACME", "serialNumber": "R-1", "responses": [r]
    }))
    .unwrap();
    w.send_responses(&msg).unwrap();
}

fn run_ticks(w: &mut World, n: u32) {
    for _ in 0..n {
        w.step();
    }
}

fn x(w: &World) -> f64 {
    w.pose(&id()).unwrap().x
}

fn requests(w: &World) -> Vec<(String, RequestStatus)> {
    w.zone_requests(&id())
        .unwrap()
        .iter()
        .map(|r| (r.request_id.clone(), r.request_status))
        .collect()
}

fn errors(w: &mut World) -> Vec<(String, String)> {
    w.state(&id())
        .unwrap()
        .errors
        .iter()
        .map(|e| (e.error_type.clone(), format!("{:?}", e.error_level)))
        .collect()
}

/// Grant the zone and drive until the robot is inside it (x between 5 and 6).
fn into_the_zone(w: &mut World) {
    respond(w, "req-1", GrantType::Granted, None);
    for _ in 0..3000 {
        w.step();
        if (5.0..6.0).contains(&x(w)) {
            return;
        }
    }
    panic!("never got into the zone");
}

// ---- before entering ----

#[test]
fn a_revoked_grant_before_entering_removes_the_request_and_the_robot_waits() {
    let mut w = world(Some("STOP"));
    respond(&mut w, "req-1", GrantType::Granted, None);
    run_ticks(&mut w, 20); // on its way, not in the zone
    assert!(x(&w) > 0.0 && x(&w) < 4.0);
    respond(&mut w, "req-1", GrantType::Revoked, None);

    // it brakes and does not go in; it states a new request
    run_ticks(&mut w, 600);
    assert!(x(&w) < 4.0, "stopped before the zone at x = {}", x(&w));
    assert_eq!(
        requests(&w),
        vec![("req-2".into(), RequestStatus::Requested)]
    );
    assert!(
        errors(&mut w).is_empty(),
        "no release was lost: it had not entered"
    );
    let at = x(&w);
    run_ticks(&mut w, 200);
    assert_eq!(x(&w), at, "stands still");

    // a new grant: it goes on
    respond(&mut w, "req-2", GrantType::Granted, None);
    run_ticks(&mut w, 3000);
    assert_eq!(w.pose(&id()).unwrap().y, 5.0, "reached C");
}

#[test]
fn an_expired_lease_before_entering_stops_the_robot_the_same_way() {
    let mut w = world(None);
    respond(
        &mut w,
        "req-1",
        GrantType::Granted,
        Some("1970-01-01T00:00:01.500Z"),
    );
    run_ticks(&mut w, 20); // 1 s: valid still
    assert!(x(&w) > 0.0);
    run_ticks(&mut w, 400);
    assert!(x(&w) < 4.0, "did not enter: x = {}", x(&w));
    assert_eq!(requests(&w)[0].1, RequestStatus::Requested, "a new request");
    assert_ne!(requests(&w)[0].0, "req-1");
}

#[test]
fn a_grant_without_a_lease_never_expires() {
    let mut w = world(None);
    respond(&mut w, "req-1", GrantType::Granted, None);
    run_ticks(&mut w, 3000); // 150 s
    assert_eq!(w.pose(&id()).unwrap().y, 5.0, "drove all the way");
}

#[test]
fn a_lease_can_be_extended_by_a_new_response() {
    let mut w = world(None);
    respond(
        &mut w,
        "req-1",
        GrantType::Granted,
        Some("1970-01-01T00:00:02.000Z"),
    );
    run_ticks(&mut w, 20);
    respond(
        &mut w,
        "req-1",
        GrantType::Granted,
        Some("1970-01-01T00:10:00.000Z"),
    );
    run_ticks(&mut w, 3000);
    assert_eq!(
        w.pose(&id()).unwrap().y,
        5.0,
        "the extended lease did not run out"
    );
}

// ---- inside the zone: release loss behaviour ----

#[test]
fn stop_stands_still_inside_reports_a_critical_error_and_goes_on_when_granted_again() {
    // an unset behaviour is STOP as well
    for loss in [Some("STOP"), None] {
        let mut w = world(loss);
        into_the_zone(&mut w);
        respond(&mut w, "req-1", GrantType::Revoked, None);
        run_ticks(&mut w, 200);

        let stopped_at = x(&w);
        assert!(
            (4.0..=7.0).contains(&stopped_at),
            "stopped inside: {stopped_at}"
        );
        run_ticks(&mut w, 400);
        assert_eq!(x(&w), stopped_at, "{loss:?}: stands still");
        assert_eq!(requests(&w), vec![("req-1".into(), RequestStatus::Revoked)]);
        assert_eq!(
            errors(&mut w),
            vec![("ZONE_RELEASE_LOST".into(), "Critical".into())]
        );

        // the error names the zone and the request
        let s = w.state(&id()).unwrap();
        let refs: Vec<_> = s.errors[0]
            .error_references
            .as_ref()
            .unwrap()
            .iter()
            .map(|r| (r.reference_key.as_str(), r.reference_value.as_str()))
            .collect();
        assert_eq!(refs, vec![("zoneId", "gate"), ("requestId", "req-1")]);
        assert!(!s.driving);

        // granted again: the error goes and the robot continues
        respond(&mut w, "req-1", GrantType::Granted, None);
        assert!(errors(&mut w).is_empty());
        run_ticks(&mut w, 3000);
        assert_eq!(w.pose(&id()).unwrap().y, 5.0, "reached C");
        assert!(requests(&w).is_empty());
    }
}

#[test]
fn continue_and_evacuate_drive_on_with_a_warning_until_the_zone_is_left() {
    for loss in ["CONTINUE", "EVACUATE"] {
        let mut w = world(Some(loss));
        into_the_zone(&mut w);
        respond(&mut w, "req-1", GrantType::Revoked, None);
        assert_eq!(
            errors(&mut w),
            vec![("ZONE_RELEASE_LOSS_HANDLING".into(), "Warning".into())]
        );
        assert_eq!(requests(&w)[0].1, RequestStatus::Revoked);

        // it keeps going: out of the zone, the warning and the request are gone
        let mut left_the_zone = false;
        for _ in 0..400 {
            w.step();
            if x(&w) > 7.0 {
                left_the_zone = true;
                break;
            }
            assert!(
                !errors_now(&mut w).is_empty(),
                "{loss}: the warning lasts while it is inside"
            );
        }
        assert!(left_the_zone, "{loss}: drove out");
        run_ticks(&mut w, 2);
        assert!(errors(&mut w).is_empty());
        assert!(requests(&w).is_empty());
        run_ticks(&mut w, 3000);
        assert_eq!(w.pose(&id()).unwrap().y, 5.0, "{loss}: finished the order");
    }
}

fn errors_now(w: &mut World) -> Vec<(String, String)> {
    errors(w)
}

#[test]
fn an_expired_lease_inside_the_zone_is_a_loss_with_status_expired() {
    let mut w = world(Some("STOP"));
    // the robot reaches the zone (x = 4) after about 4.2 s and is out of it (x = 7)
    // after about 6.2 s; a lease of 4.6 s runs out just inside, early enough that
    // it can brake (about 1.4 m) before the far side
    respond(
        &mut w,
        "req-1",
        GrantType::Granted,
        Some("1970-01-01T00:00:04.600Z"),
    );
    let mut expired_inside = false;
    for _ in 0..1000 {
        w.step();
        let r = requests(&w);
        if r.first().is_some_and(|r| r.1 == RequestStatus::Expired) {
            expired_inside = (4.0..=7.0).contains(&x(&w));
            break;
        }
    }
    assert!(
        expired_inside,
        "the lease ran out while the robot was in the zone"
    );
    assert_eq!(
        errors(&mut w),
        vec![("ZONE_RELEASE_LOST".into(), "Critical".into())]
    );
    // it brakes (about 1.4 m) and then stands still, still inside the zone
    run_ticks(&mut w, 100);
    let at = x(&w);
    assert!((4.0..=7.0).contains(&at), "stopped inside the zone: {at}");
    run_ticks(&mut w, 200);
    assert_eq!(x(&w), at, "stands still");
}

#[test]
fn continue_with_the_order_ending_inside_the_zone_drops_the_warning() {
    // the order ends at B (x = 10) — outside; use a zone that contains B instead
    let set: ZoneSet = serde_json::from_str(
        r#"{"mapId":"M","zoneSetId":"zs1","zones":[{"zoneId":"gate","zoneType":"RELEASE",
        "releaseLossBehavior":"CONTINUE",
        "vertices":[{"x":8,"y":-1},{"x":12,"y":-1},{"x":12,"y":1},{"x":8,"y":1}]}]}"#,
    )
    .unwrap();
    let g = graph();
    let zones = Zones::new(&g, set).unwrap();
    let a = g.node_idx("A").unwrap();
    let mut w = World::new(g, DT, 0).unwrap();
    w.set_zones(zones);
    w.add_robot(id(), spec(), ActionTimes::new(1.0), a, 0.0)
        .unwrap();
    let o: Order = serde_json::from_str(
        r#"{"headerId":1,"timestamp":"1970-01-01T00:00:00.000Z","version":"3.0.1",
        "manufacturer":"ACME","serialNumber":"R-1","orderId":"o1","orderUpdateId":0,
        "nodes":[{"nodeId":"A","sequenceId":0,"released":true,"actions":[]},
                 {"nodeId":"B","sequenceId":2,"released":true,"actions":[]}],
        "edges":[{"edgeId":"A-B","sequenceId":1,"released":true,"actions":[]}]}"#,
    )
    .unwrap();
    w.send_order(&o).unwrap();
    respond(&mut w, "req-1", GrantType::Granted, None);
    while x(&w) < 8.5 {
        w.step();
    }
    respond(&mut w, "req-1", GrantType::Revoked, None);
    assert_eq!(errors(&mut w).len(), 1, "warning while the order goes on");
    run_ticks(&mut w, 2000); // the order ends at B, inside the zone
    assert_eq!(x(&w), 10.0);
    assert!(
        errors(&mut w).is_empty(),
        "the order ended inside the zone: no warning"
    );
    assert_eq!(
        requests(&w)[0].1,
        RequestStatus::Revoked,
        "the request stays"
    );
}

// ---- the timestamp reader ----

#[test]
fn iso_timestamps_are_read() {
    assert_eq!(unix_ms_from_iso8601("1970-01-01T00:00:00.000Z"), Some(0));
    assert_eq!(unix_ms_from_iso8601("1970-01-01T00:00:01Z"), Some(1000));
    assert_eq!(unix_ms_from_iso8601("1970-01-01T00:00:01.5Z"), Some(1500));
    assert_eq!(
        unix_ms_from_iso8601("1970-01-01T00:00:01.123456Z"),
        Some(1123)
    );
    assert_eq!(
        unix_ms_from_iso8601("2026-11-20T10:00:00.250Z"),
        Some(1_795_168_800_250)
    );
}

#[test]
fn bad_timestamps_are_refused() {
    for bad in [
        "",
        "2026-11-20",
        "2026-11-20T10:00:00", // no Z
        "2026-11-20T10:00:00+01:00",
        "2026-13-20T10:00:00Z", // month 13
        "2026-11-32T10:00:00Z",
        "2026-11-20T25:00:00Z",
        "2026-11-20T10:61:00Z",
        "2026-11-20T10:00Z",
        "2026-11-20T10:00:00.Z",
        "2026-11-20T10:00:00.abcZ",
        "yesterday",
    ] {
        assert_eq!(unix_ms_from_iso8601(bad), None, "{bad}");
    }
}

// the lease of a response that cannot be read counts as no lease: the grant holds

#[test]
fn an_unreadable_lease_is_treated_as_no_lease() {
    let mut w = world(None);
    respond(&mut w, "req-1", GrantType::Granted, Some("soon"));
    run_ticks(&mut w, 3000);
    assert_eq!(w.pose(&id()).unwrap().y, 5.0);
}

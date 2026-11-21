//! Writing a run to a file and reading it back.

mod common;

use std::cell::RefCell;
use std::fs::File;
use std::io::BufReader;
use std::rc::Rc;

use botsim_core::graph::Graph;
use botsim_core::observer::{Observer, Snapshot};
use botsim_record::{
    FORMAT_VERSION, LayoutLine, ReadError, Reader, Recorder, SnapshotLine, read_recording,
};

/// Sees the same run as the recorder, so the test can compare the two.
#[derive(Debug)]
struct Collect(Rc<RefCell<Vec<Snapshot>>>);

impl Observer for Collect {
    fn on_layout(&mut self, _: &Graph) {}
    fn on_step(&mut self, s: &Snapshot) {
        self.0.borrow_mut().push(s.clone());
    }
}

/// Two observers are not possible on one world, so run it twice: the run is
/// deterministic, and the second run is the one that is recorded.
fn run_collected(steps: usize, interval: f64) -> Vec<Snapshot> {
    let mut w = common::corridor_world();
    let log = Rc::new(RefCell::new(Vec::new()));
    w.set_observer_at(Box::new(Collect(log.clone())), interval)
        .unwrap();
    for _ in 0..steps {
        w.step();
    }
    log.borrow().clone()
}

fn run_recorded(path: &std::path::Path, steps: usize, interval: f64) {
    let mut w = common::corridor_world();
    let (recorder, handle) = Recorder::new(File::create(path).unwrap());
    w.set_observer_at(Box::new(recorder), interval).unwrap();
    for _ in 0..steps {
        w.step();
    }
    handle.finish().unwrap();
}

fn tmp(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(name)
}

#[test]
fn a_recorded_run_reads_back_the_same() {
    let path = tmp("run.jsonl");
    run_recorded(&path, 400, 0.5);
    let expected = run_collected(400, 0.5);
    assert!(expected.len() > 10 && expected.len() < 400, "interval used");

    let rec = read_recording(BufReader::new(File::open(&path).unwrap())).unwrap();
    assert_eq!(
        rec.layout,
        LayoutLine::from_graph(&common::corridor_graph())
    );
    assert_eq!(rec.layout.version, FORMAT_VERSION);
    let lines: Vec<SnapshotLine> = expected.iter().map(SnapshotLine::from).collect();
    assert_eq!(rec.snapshots, lines);

    // the file is JSON Lines: the layout first, then one snapshot per line
    let text = std::fs::read_to_string(&path).unwrap();
    assert_eq!(text.lines().count(), 1 + expected.len());
    assert!(
        text.lines()
            .next()
            .unwrap()
            .starts_with(r#"{"type":"layout""#)
    );
    assert!(text.ends_with('\n'));
}

#[test]
fn the_same_run_gives_the_same_file() {
    let (a, b) = (tmp("same_a.jsonl"), tmp("same_b.jsonl"));
    run_recorded(&a, 200, 0.1);
    run_recorded(&b, 200, 0.1);
    assert_eq!(std::fs::read(&a).unwrap(), std::fs::read(&b).unwrap());
}

#[test]
fn the_reader_streams_the_snapshots() {
    let path = tmp("stream.jsonl");
    run_recorded(&path, 100, 0.1);
    let mut reader = Reader::new(BufReader::new(File::open(&path).unwrap())).unwrap();
    assert_eq!(reader.layout().nodes.len(), 3);
    let first = reader.next().unwrap().unwrap();
    assert!(first.time > 0.0);
    // 100 steps of 0.05 s, a snapshot every 0.1 s: 50 in all, 49 after the first
    assert_eq!(reader.count(), 49);
}

fn read_text(text: &str) -> Result<botsim_record::Recording, ReadError> {
    read_recording(text.as_bytes())
}

const LAYOUT: &str = r#"{"type":"layout","version":1,"maps":["m"],"nodes":[],"edges":[]}"#;

#[test]
fn the_reader_rejects_bad_files() {
    assert!(matches!(read_text(""), Err(ReadError::Empty)));
    assert!(matches!(read_text("\n \n"), Err(ReadError::Empty)));
    assert!(matches!(
        read_text(r#"{"type":"snapshot","tick":1}"#),
        Err(ReadError::NotALayout { line: 1 })
    ));
    assert!(matches!(
        read_text(r#"{"type":"layout","version":2,"maps":[],"nodes":[],"edges":[]}"#),
        Err(ReadError::UnsupportedVersion {
            found: 2,
            supported: 1
        })
    ));
    assert!(matches!(
        read_text(r#"{"type":"layout","maps":[],"nodes":[],"edges":[]}"#),
        Err(ReadError::Line { line: 1, .. })
    ));
    assert!(matches!(
        read_text("not json"),
        Err(ReadError::Line { line: 1, .. })
    ));
    // a broken snapshot is reported with its line number (blank lines count)
    let text = format!("{LAYOUT}\n\n{{\"type\":\"snapshot\",\"tick\":1}}\n");
    assert!(matches!(
        read_text(&text),
        Err(ReadError::Line { line: 3, .. })
    ));
    // a layout only is a valid recording of zero snapshots
    assert!(read_text(LAYOUT).unwrap().snapshots.is_empty());
}

#[test]
fn the_geometry_is_in_the_layout_line_of_the_file() {
    let geometry = botsim_record::GeometryFile::from_json_str(&common::read(
        "corridor_deadlock.geometry.json",
    ))
    .unwrap();
    let path = tmp("geometry.jsonl");
    let mut w = common::corridor_world();
    let (recorder, handle) =
        Recorder::with_geometry(File::create(&path).unwrap(), geometry.clone());
    w.set_observer_at(Box::new(recorder), 0.5).unwrap();
    for _ in 0..100 {
        w.step();
    }
    handle.finish().unwrap();

    let rec = read_recording(BufReader::new(File::open(&path).unwrap())).unwrap();
    assert_eq!(
        rec.layout.geometry,
        geometry.resolve(&rec.layout.maps).unwrap()
    );
    assert_eq!(rec.layout.geometry.len(), 9);
    assert_eq!(rec.snapshots.len(), 10);
}

#[test]
fn geometry_on_an_unknown_map_writes_nothing_and_finish_reports_it() {
    let geometry = botsim_record::GeometryFile::from_json_str(
        r#"{"version":1,"shapes":[{"id":"w","mapId":"Nope","kind":"wall","shape":"polyline",
            "vertices":[{"x":0,"y":0},{"x":1,"y":0}]}]}"#,
    )
    .unwrap();
    let path = tmp("bad_geometry.jsonl");
    let mut w = common::corridor_world();
    let (recorder, handle) = Recorder::with_geometry(File::create(&path).unwrap(), geometry);
    w.set_observer(Box::new(recorder));
    for _ in 0..10 {
        w.step();
    }
    let err = handle.finish().unwrap_err();
    assert_eq!(err.kind(), std::io::ErrorKind::InvalidData);
    assert_eq!(
        err.to_string(),
        "geometry 'w': the layout has no map 'Nope'"
    );
    assert_eq!(std::fs::read(&path).unwrap(), b"", "nothing written");
}

/// A writer that fails, to see the error come out of `finish`.
struct Failing;

impl std::io::Write for Failing {
    fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
        Err(std::io::Error::other("disk full"))
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Err(std::io::Error::other("disk full"))
    }
}

#[test]
fn a_write_error_is_reported_by_finish() {
    let mut w = common::corridor_world();
    let (recorder, handle) = Recorder::new(Failing);
    w.set_observer(Box::new(recorder));
    for _ in 0..2000 {
        w.step(); // more than the buffer holds, so the failing writer is hit
    }
    let err = handle.finish().unwrap_err();
    assert_eq!(err.to_string(), "disk full");
}

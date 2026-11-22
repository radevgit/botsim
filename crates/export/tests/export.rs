//! Writing frames: the library function and the command line.

use std::path::{Path, PathBuf};
use std::process::Command;

use botsim_export::{
    ExportError, ExportOptions, ExportSummary, export_frames, frame_count, frame_path, palette,
};
use botsim_record::{
    EdgeRecord, GeometryFile, LayoutLine, NodeRecord, Recording, RobotRecord, RouteRecord,
    ShapeRecord, SnapshotLine, StatusRecord,
};
use tiny_skia::Pixmap;

/// The corridor A - B - C with its geometry; a second map "N" with nothing on it.
fn layout() -> LayoutLine {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/corridor_deadlock.geometry.json"
    );
    let node = |id: &str, x: f64| NodeRecord {
        id: id.into(),
        map: 0,
        x,
        y: 0.0,
    };
    let edge = |id: &str, start, end| EdgeRecord {
        id: id.into(),
        start,
        end,
        length: 10.0,
        map_transition: false,
        paths: vec![],
    };
    LayoutLine {
        version: 1,
        maps: vec!["M".into(), "N".into()],
        nodes: vec![node("A", 0.0), node("B", 10.0), node("C", 20.0)],
        edges: vec![
            edge("A-B", 0, 1),
            edge("B-A", 1, 0),
            edge("B-C", 1, 2),
            edge("C-B", 2, 1),
        ],
        geometry: vec![],
        robots: vec![],
    }
    .with_geometry(&GeometryFile::load(Path::new(path)).unwrap())
    .unwrap()
}

/// One robot driving along the corridor at 5 m/s, a snapshot every 0.5 s from
/// t = 0 to t = 4 s.
fn snapshots() -> Vec<SnapshotLine> {
    (0..=8)
        .map(|i| {
            let t = i as f64 * 0.5;
            SnapshotLine {
                tick: i as u64 * 10,
                time: t,
                robots: vec![RobotRecord {
                    id: "ACME/R-1".into(),
                    map: 0,
                    x: 5.0 * t,
                    y: 0.0,
                    heading: 0.0,
                    shape: ShapeRecord::Rect([0.0, 0.0, 0.6, 0.4]),
                    status: StatusRecord::Driving,
                    speed: 5.0,
                    order: "o".into(),
                    last_node: None,
                    released: RouteRecord::default(),
                    planned: RouteRecord::default(),
                }],
                reservations: vec![],
                blocking: vec![],
                overlapping: vec![],
                deadlocks: vec![],
                lifts: vec![],
                doors: vec![],
            }
        })
        .collect()
}

fn recording() -> Recording {
    Recording {
        layout: layout(),
        snapshots: snapshots(),
    }
}

fn tmp(name: &str) -> PathBuf {
    Path::new(env!("CARGO_TARGET_TMPDIR")).join(name)
}

/// A fresh, empty directory.
fn fresh(name: &str) -> PathBuf {
    let dir = tmp(name);
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

fn small() -> ExportOptions {
    ExportOptions {
        width: 192,
        height: 108,
        ..ExportOptions::default()
    }
}

fn files(dir: &Path) -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap())
        .collect();
    names.sort();
    names
}

#[test]
fn the_frame_count_follows_the_time_range_and_the_speed() {
    assert_eq!(frame_count(0.0, 4.0, 30, 1.0), 121);
    assert_eq!(frame_count(0.0, 4.0, 30, 10.0), 13);
    assert_eq!(
        frame_count(2.0, 2.0, 30, 1.0),
        1,
        "a single moment is a frame"
    );
    assert_eq!(frame_count(3.0, 2.0, 30, 1.0), 0);
    // 0.4 - 0.1 is not exactly 0.3; still 10 frames (0.1 … 0.4 at 30 fps)
    assert_eq!(frame_count(0.1, 0.4, 30, 1.0), 10);
}

#[test]
fn the_frames_are_numbered_with_six_digits() {
    assert_eq!(
        frame_path(Path::new("out"), 0),
        Path::new("out/frame_000001.png")
    );
    assert_eq!(
        frame_path(Path::new("out"), 123456),
        Path::new("out/frame_123457.png")
    );
}

#[test]
fn it_writes_the_frames_and_the_robot_moves_between_them() {
    let dir = fresh("frames_a");
    let options = ExportOptions {
        to: Some(1.0),
        fps: 4,
        ..small()
    };
    let s = export_frames(&recording(), &dir, &options).unwrap();
    assert_eq!(
        s,
        ExportSummary {
            frames: 5,
            width: 192,
            height: 108,
            map_id: "M".into(),
            from: 0.0,
            to: 1.0,
            fps: 4,
            speed: 1.0
        }
    );
    assert_eq!(
        files(&dir),
        [
            "frame_000001.png",
            "frame_000002.png",
            "frame_000003.png",
            "frame_000004.png",
            "frame_000005.png"
        ]
    );

    // frame k is t = k / 4, so the robot is at x = 1.25 k; the view is the same
    // in every frame, so look at the same body point (0.3 m behind and 0.2 m left
    // of the centre) in each
    let layout = layout();
    let view = botsim_export::View::fit(&layout, 0, 192, 108).unwrap();
    for k in 0..5 {
        let frame = Pixmap::load_png(frame_path(&dir, k)).unwrap();
        assert_eq!((frame.width(), frame.height()), (192, 108));
        let (px, py) = view.to_px(1.25 * k as f64 - 0.3, 0.2);
        let p = frame.pixel(px as u32, py as u32).unwrap();
        assert_eq!(
            [p.red(), p.green(), p.blue()],
            palette::status(StatusRecord::Driving),
            "frame {k}"
        );
    }
}

#[test]
fn speed_and_range_change_which_moments_are_drawn() {
    // 4 s at speed 4 and 2 fps: a frame every 2 s → t = 0, 2, 4
    let dir = fresh("frames_b");
    let options = ExportOptions {
        fps: 2,
        speed: 4.0,
        ..small()
    };
    let s = export_frames(&recording(), &dir, &options).unwrap();
    assert_eq!((s.frames, s.from, s.to), (3, 0.0, 4.0));
    assert_eq!(files(&dir).len(), 3);
    // the last frame is at t = 4 s: the robot is at x = 20 (the node C)
    let layout = layout();
    let view = botsim_export::View::fit(&layout, 0, 192, 108).unwrap();
    let frame = Pixmap::load_png(frame_path(&dir, 2)).unwrap();
    let (px, py) = view.to_px(19.7, 0.2);
    let p = frame.pixel(px as u32, py as u32).unwrap();
    assert_eq!(
        [p.red(), p.green(), p.blue()],
        palette::status(StatusRecord::Driving)
    );
}

#[test]
fn the_same_input_gives_the_same_files_and_other_files_stay() {
    let (a, b) = (fresh("frames_c1"), fresh("frames_c2"));
    let options = ExportOptions {
        fps: 2,
        speed: 2.0,
        ..small()
    };
    export_frames(&recording(), &a, &options).unwrap();
    // an older, longer export is replaced where the names are the same
    export_frames(&recording(), &b, &small()).unwrap();
    std::fs::write(b.join("notes.txt"), "mine").unwrap();
    export_frames(&recording(), &b, &options).unwrap();
    for name in files(&a) {
        assert_eq!(
            std::fs::read(a.join(&name)).unwrap(),
            std::fs::read(b.join(&name)).unwrap(),
            "{name}"
        );
    }
    assert!(b.join("notes.txt").exists(), "other files are not deleted");
    assert!(
        b.join("frame_000100.png").exists(),
        "so are leftover frames"
    );
}

#[test]
fn bad_input_is_reported() {
    let dir = fresh("frames_d");
    let run = |r: &Recording, o: &ExportOptions| export_frames(r, &dir, o).unwrap_err();

    let empty = Recording {
        layout: layout(),
        snapshots: vec![],
    };
    assert!(matches!(run(&empty, &small()), ExportError::NoSnapshots));

    let unknown = ExportOptions {
        map: Some("Z".into()),
        ..small()
    };
    let err = run(&recording(), &unknown);
    assert_eq!(err.to_string(), "the recording has no map 'Z' (maps: M, N)");

    let empty_floor = ExportOptions {
        map: Some("N".into()),
        ..small()
    };
    let err = run(&recording(), &empty_floor);
    assert_eq!(err.to_string(), "map number 1 has no nodes and no geometry");

    for (options, text) in [
        (
            ExportOptions {
                width: 191,
                ..small()
            },
            "even",
        ),
        (
            ExportOptions {
                height: 0,
                ..small()
            },
            "at least 1",
        ),
        (ExportOptions { fps: 0, ..small() }, "fps"),
        (
            ExportOptions {
                speed: 0.0,
                ..small()
            },
            "speed",
        ),
        (
            ExportOptions {
                speed: f64::NAN,
                ..small()
            },
            "speed",
        ),
        (
            ExportOptions {
                from: Some(f64::NAN),
                ..small()
            },
            "from",
        ),
        (
            ExportOptions {
                from: Some(3.0),
                to: Some(1.0),
                ..small()
            },
            "before from",
        ),
    ] {
        let err = run(&recording(), &options);
        assert!(matches!(err, ExportError::BadOption(_)), "{err}");
        assert!(err.to_string().contains(text), "{err}");
    }
    assert!(!dir.exists(), "nothing was created for bad input");

    // the output directory cannot be created below a file
    let file = tmp("a_file");
    std::fs::write(&file, "x").unwrap();
    let err = export_frames(&recording(), &file.join("sub"), &small()).unwrap_err();
    assert!(matches!(err, ExportError::Write { .. }), "{err}");
}

// the command line ---------------------------------------------------------

fn cli(args: &[&str]) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_botsim-export"))
        .args(args)
        .output()
        .unwrap()
}

/// The recording as a JSONL file, as `botsim-record` writes it.
fn write_recording(name: &str) -> PathBuf {
    let path = tmp(name);
    let mut text = serde_json::to_string(&layout()).unwrap();
    for s in snapshots() {
        text.push('\n');
        text.push_str(&serde_json::to_string(&s).unwrap());
    }
    text.push('\n');
    std::fs::write(&path, text).unwrap();
    path
}

#[test]
fn the_cli_writes_frames_and_says_what_it_did() {
    let rec = write_recording("cli.jsonl");
    let dir = fresh("cli_frames");
    let o = cli(&[
        rec.to_str().unwrap(),
        dir.to_str().unwrap(),
        "--size",
        "192x108",
        "--fps",
        "2",
        "--speed",
        "4",
        "--map",
        "M",
        "--from",
        "0",
        "--to",
        "4",
    ]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    assert!(o.stderr.is_empty());
    let out = String::from_utf8(o.stdout).unwrap();
    assert!(
        out.starts_with("wrote 3 frames 192x108 of map M to "),
        "{out}"
    );
    assert!(
        out.contains("simulated 0.0 s to 4.0 s, 2 fps, speed 4"),
        "{out}"
    );
    assert_eq!(files(&dir).len(), 3);
    // default size when --size is left out is 1920x1080
    let dir2 = fresh("cli_frames_default");
    let o = cli(&[
        rec.to_str().unwrap(),
        dir2.to_str().unwrap(),
        "--from",
        "1",
        "--to",
        "1",
    ]);
    assert!(o.status.success());
    let frame = Pixmap::load_png(frame_path(&dir2, 0)).unwrap();
    assert_eq!((frame.width(), frame.height()), (1920, 1080));
}

#[test]
fn the_cli_reports_errors_with_exit_codes() {
    let rec = write_recording("cli_err.jsonl");
    let rec = rec.to_str().unwrap();
    let out = fresh("cli_err_out");
    let out = out.to_str().unwrap();

    // wrong command lines: usage, exit 2
    for args in [
        vec![],
        vec![rec],
        vec![rec, out, "extra"],
        vec![rec, out, "--nope"],
        vec![rec, out, "--fps"],
        vec![rec, out, "--fps", "many"],
        vec![rec, out, "--size", "big"],
        vec![rec, out, "--size", "1920"],
        vec![rec, out, "--speed", "fast"],
    ] {
        let o = cli(&args);
        assert_eq!(o.status.code(), Some(2), "{args:?}");
        assert!(
            String::from_utf8_lossy(&o.stderr).contains("usage"),
            "{args:?}"
        );
        assert!(o.stdout.is_empty());
    }

    // values that parse but cannot work, a missing or a broken recording: exit 1
    let broken = tmp("cli_broken.jsonl");
    std::fs::write(&broken, "not json\n").unwrap();
    for (args, text) in [
        (
            vec![rec, out, "--size", "191x108"],
            "error: width and height must be even",
        ),
        (
            vec![rec, out, "--map", "Z"],
            "error: the recording has no map 'Z'",
        ),
        (
            vec!["missing.jsonl", out],
            "error: cannot read missing.jsonl",
        ),
        (vec![broken.to_str().unwrap(), out], "line 1"),
    ] {
        let o = cli(&args);
        assert_eq!(o.status.code(), Some(1), "{args:?}");
        assert!(o.stdout.is_empty());
        assert!(
            String::from_utf8_lossy(&o.stderr).contains(text),
            "{args:?}"
        );
    }
}

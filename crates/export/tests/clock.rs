use std::path::{Path, PathBuf};
use std::process::Command;

use botsim_export::{Clock, ExportOptions, Renderer, export_frames};
use botsim_record::{EdgeRecord, LayoutLine, NodeRecord, Recording, SnapshotLine};
use tiny_skia::Pixmap;

fn layout() -> LayoutLine {
    let node = |id: &str, x: f64| NodeRecord {
        id: id.into(),
        map: 0,
        x,
        y: 0.0,
    };
    LayoutLine {
        version: 1,
        maps: vec!["M".into()],
        nodes: vec![node("A", 0.0), node("B", 20.0)],
        edges: vec![EdgeRecord {
            id: "A-B".into(),
            start: 0,
            end: 1,
            length: 20.0,
            map_transition: true, // not drawn, so the picture is empty but for the label
            paths: vec![],
        }],
        geometry: vec![],
        robots: vec![],
    }
}

fn at_time(time: f64) -> SnapshotLine {
    SnapshotLine {
        tick: 0,
        time,
        robots: vec![],
        reservations: vec![],
        blocking: vec![],
        overlapping: vec![],
        deadlocks: vec![],
        lifts: vec![],
        doors: vec![],
    }
}

/// The pixels that differ between two pictures of the same size.
fn differing(a: &Pixmap, b: &Pixmap) -> Vec<(u32, u32)> {
    assert_eq!((a.width(), a.height()), (b.width(), b.height()));
    (0..a.height())
        .flat_map(|y| (0..a.width()).map(move |x| (x, y)))
        .filter(|&(x, y)| a.pixel(x, y) != b.pixel(x, y))
        .collect()
}

fn inside(area: (f32, f32, f32, f32), (x, y): (u32, u32)) -> bool {
    let (ax, ay, w, h) = area;
    (x as f32) >= ax - 1.0
        && (x as f32) <= ax + w + 1.0
        && (y as f32) >= ay - 1.0
        && (y as f32) <= ay + h + 1.0
}

#[test]
fn the_text_is_the_simulated_time() {
    assert_eq!(Clock::text(100.0), "100.0 s");
    assert_eq!(Clock::text(0.0), "0.0 s");
    assert_eq!(Clock::text(1234.56), "1234.6 s");
}

#[test]
fn the_label_changes_only_its_own_corner() {
    let layout = layout();
    let plain = Renderer::new(&layout, 0, 1920, 1080).unwrap();
    let with = Renderer::new(&layout, 0, 1920, 1080)
        .unwrap()
        .with_clock(true)
        .unwrap();
    let snap = at_time(100.0);
    let (a, b) = (plain.frame(&snap), with.frame(&snap));

    let area = with.clock_area(100.0).unwrap();
    let changed = differing(&a, &b);
    assert!(changed.len() > 1000, "a plate and text: {}", changed.len());
    assert!(
        changed.iter().all(|&p| inside(area, p)),
        "outside the label area"
    );
    // it is at the top, away from the very edge, and begins 3 m inside the left wall
    // of the building (the test layout's nodes are at x = 0 and x = 20 m), but never
    // more than 5% of the width of the image inside it
    assert!(area.1 > 10.0);
    let left_wall = with.view().to_px(0.0, 0.0).0;
    let inset = (3.0 * with.view().scale).min(0.05 * 1920.0) as f32;
    assert!(
        (area.0 - left_wall - inset).abs() < 0.5,
        "begins {} px inside the wall, expected {inset}",
        area.0 - left_wall
    );
    // no clock, no area
    assert_eq!(plain.clock_area(100.0), None);
    // some pixels are dark text colour (the plate itself is light)
    let dark = changed
        .iter()
        .filter(|&&(x, y)| {
            let p = b.pixel(x, y).unwrap();
            p.red() < 120 && p.green() < 120 && p.blue() < 130
        })
        .count();
    assert!(dark > 100, "text pixels: {dark}");
}

#[test]
fn the_label_follows_the_time_and_never_the_empty_frame() {
    let layout = layout();
    let r = Renderer::new(&layout, 0, 1920, 1080)
        .unwrap()
        .with_clock(true)
        .unwrap();
    let (a, b) = (r.frame(&at_time(100.0)), r.frame(&at_time(200.0)));
    // the plate is as wide as the longer text, so it is the area of the later time
    let area = r.clock_area(200.0).unwrap();
    let changed = differing(&a, &b);
    assert!(!changed.is_empty());
    assert!(changed.iter().all(|&p| inside(area, p)));
    // the same time twice: the same picture
    assert!(differing(&a, &r.frame(&at_time(100.0))).is_empty());
    // the floor alone has no label
    let plain = Renderer::new(&layout, 0, 1920, 1080).unwrap();
    assert!(differing(&r.empty_frame(), &plain.empty_frame()).is_empty());
    // and with_clock(false) is no clock
    let off = Renderer::new(&layout, 0, 1920, 1080)
        .unwrap()
        .with_clock(true)
        .unwrap()
        .with_clock(false)
        .unwrap();
    assert!(differing(&off.frame(&at_time(5.0)), &plain.frame(&at_time(5.0))).is_empty());
}

#[test]
fn the_label_grows_with_the_text_and_with_the_image() {
    // whatever font the system has: more characters, a wider plate
    let clock = Clock::new().unwrap();
    let w = |t: f64| clock.area(0.0, 1080, t).2;
    assert!(w(1.0) < w(10.0) && w(10.0) < w(100.0));
    // it grows with the image
    assert!(clock.area(0.0, 2160, 1.0).2 > 1.9 * clock.area(0.0, 1080, 1.0).2);
}

#[test]
fn a_label_that_does_not_fit_the_image_is_cut_off_not_an_error() {
    let layout = layout();
    for (w, h) in [(192, 108), (32, 32), (4, 4)] {
        let r = Renderer::new(&layout, 0, w, h)
            .unwrap()
            .with_clock(true)
            .unwrap();
        let frame = r.frame(&at_time(1200.0));
        assert_eq!((frame.width(), frame.height()), (w, h));
    }
}

// the export options ---------------------------------------------------------

fn tmp(name: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

fn recording() -> Recording {
    Recording {
        layout: layout(),
        snapshots: vec![at_time(0.0), at_time(1.0)],
    }
}

#[test]
fn the_export_has_the_clock_unless_told_not_to() {
    let (with, without) = (tmp("clock_on"), tmp("clock_off"));
    let size = ExportOptions {
        width: 640,
        height: 360,
        to: Some(0.0),
        ..ExportOptions::default()
    };
    assert!(size.clock, "on by default");
    export_frames(&recording(), &with, &size).unwrap();
    export_frames(
        &recording(),
        &without,
        &ExportOptions {
            clock: false,
            ..size
        },
    )
    .unwrap();
    let a = Pixmap::load_png(with.join("frame_000001.png")).unwrap();
    let b = Pixmap::load_png(without.join("frame_000001.png")).unwrap();
    let changed = differing(&a, &b);
    assert!(!changed.is_empty());
    let area = Renderer::new(&layout(), 0, 640, 360)
        .unwrap()
        .with_clock(true)
        .unwrap()
        .clock_area(0.0)
        .unwrap();
    assert!(changed.iter().all(|&p| inside(area, p)));
}

#[test]
fn the_command_line_has_no_clock() {
    let rec = Path::new(env!("CARGO_TARGET_TMPDIR")).join("clock_cli.jsonl");
    let text = format!(
        "{}\n{}\n",
        serde_json::to_string(&layout()).unwrap(),
        serde_json::to_string(&at_time(3.0)).unwrap()
    );
    std::fs::write(&rec, text).unwrap();
    let run = |extra: &[&str], dir: &Path| {
        let o = Command::new(env!("CARGO_BIN_EXE_botsim-export"))
            .arg(&rec)
            .arg(dir)
            .args(["--size", "320x180"])
            .args(extra)
            .output()
            .unwrap();
        assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    };
    let (with, without) = (tmp("clock_cli_on"), tmp("clock_cli_off"));
    run(&[], &with);
    run(&["--no-clock"], &without);
    let a = Pixmap::load_png(with.join("frame_000001.png")).unwrap();
    let b = Pixmap::load_png(without.join("frame_000001.png")).unwrap();
    // only the clock differs: the name is there in both
    let clock_area = Renderer::new(&layout(), 0, 320, 180)
        .unwrap()
        .with_clock(true)
        .unwrap()
        .clock_area(3.0)
        .unwrap();
    let changed = differing(&a, &b);
    assert!(!changed.is_empty());
    assert!(changed.iter().all(|&p| inside(clock_area, p)));
}

#[test]
fn the_name_is_in_the_bottom_right_corner_and_nowhere_else() {
    let layout = layout();
    let (w, h) = (1920, 1080);
    let plain = Renderer::new(&layout, 0, w, h).unwrap();
    let branded = Renderer::new(&layout, 0, w, h)
        .unwrap()
        .with_brand(true)
        .unwrap();
    let snap = at_time(7.0);
    let (a, b) = (plain.frame(&snap), branded.frame(&snap));

    let area = branded.brand_area().unwrap();
    let changed = differing(&a, &b);
    assert!(changed.len() > 300, "text pixels: {}", changed.len());
    assert!(
        changed.iter().all(|&p| inside(area, p)),
        "outside the name's area"
    );
    // at the bottom, inside the image
    assert!(area.0 + area.2 < w as f32 && area.1 + area.3 < h as f32);
    assert!(area.1 > h as f32 / 2.0);
    // its right end is a few metres inside the right wall of the building: the
    // test layout's nodes are at x = 0 and x = 20 m, the right wall is x = 20 m
    let right_wall = branded.view().to_px(20.0, 0.0).0;
    let right_end = area.0 + area.2;
    assert!(
        right_end < right_wall,
        "{right_end} is not inside {right_wall}"
    );
    // it ends 3 m inside the wall, but never more than 5% of the width of the image
    // (in this small layout, 3 m is a lot of pixels)
    let inset = (3.0 * branded.view().scale).min(0.05 * w as f64) as f32;
    assert!(
        (right_wall - right_end - inset).abs() < 0.5,
        "ends {} px inside the wall, expected {inset}",
        right_wall - right_end
    );
    // without the name, nothing has an area
    assert_eq!(plain.brand_area(), None);
    // the name does not depend on the time
    assert!(differing(&b, &branded.frame(&at_time(900.0))).is_empty());
    // with_brand(false) is no name; the empty floor never has it
    let off = Renderer::new(&layout, 0, w, h)
        .unwrap()
        .with_brand(true)
        .unwrap()
        .with_brand(false)
        .unwrap();
    assert!(differing(&off.frame(&snap), &a).is_empty());
    assert!(differing(&branded.empty_frame(), &plain.empty_frame()).is_empty());
}

#[test]
fn the_clock_and_the_name_are_independent() {
    let layout = layout();
    let (w, h) = (1920, 1080);
    let both = Renderer::new(&layout, 0, w, h)
        .unwrap()
        .with_clock(true)
        .unwrap()
        .with_brand(true)
        .unwrap();
    let clock_only = Renderer::new(&layout, 0, w, h)
        .unwrap()
        .with_clock(true)
        .unwrap();
    let snap = at_time(42.0);
    let name = both.brand_area().unwrap();
    // both = clock only + the name
    let changed = differing(&both.frame(&snap), &clock_only.frame(&snap));
    assert!(!changed.is_empty());
    assert!(changed.iter().all(|&p| inside(name, p)));
    // the two plates do not meet: the clock is at the top, the name at the bottom
    let c = both.clock_area(42.0).unwrap();
    assert!(c.1 + c.3 < name.1);
}

#[test]
fn every_exported_frame_has_the_name() {
    let dir = tmp("brand_export");
    let options = ExportOptions {
        width: 640,
        height: 360,
        to: Some(0.0),
        clock: false,
        ..ExportOptions::default()
    };
    export_frames(&recording(), &dir, &options).unwrap();
    // the same frame without any text: only the name's corner differs
    let exported = Pixmap::load_png(dir.join("frame_000001.png")).unwrap();
    let layout = layout();
    let bare = Renderer::new(&layout, 0, 640, 360)
        .unwrap()
        .frame(&at_time(0.0));
    let area = Renderer::new(&layout, 0, 640, 360)
        .unwrap()
        .with_brand(true)
        .unwrap()
        .brand_area()
        .unwrap();
    let changed = differing(&exported, &bare);
    assert!(!changed.is_empty(), "the name is drawn");
    assert!(changed.iter().all(|&p| inside(area, p)));
}

//! `botsim-viewer <recording.jsonl>`: replay a recording in a window.
//!
//! The picture is the one `botsim-export` draws for a video (same colours, same
//! drawing), made for the size of the window. Below it: play / pause, a slider to
//! scrub through the run, and the speed.

use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use botsim_check::{Finding, Level, check_scenario};
use botsim_export::{Renderer, View, sample};
use botsim_record::{LayoutLine, Recording, SnapshotLine, read_recording_partial};
use botsim_viewer::{Playback, SPEEDS, describe, pick_robot};
use eframe::egui;

const USAGE: &str = "usage: botsim-viewer [--check <scenario.toml>] <recording.jsonl>";

/// One finding of the layout check, placed on the floor.
struct Mark {
    map: usize,
    x: f64,
    y: f64,
    level: Level,
    /// The finding's text with its check name.
    text: String,
    /// Index into the findings.
    finding: usize,
}

/// Place the findings on the floor, one mark for each place of a finding.
fn marks_for(layout: &LayoutLine, findings: &[Finding]) -> Vec<Mark> {
    let mut marks = Vec::new();
    for (finding, f) in findings.iter().enumerate() {
        let text = format!("[{}] {}", f.check, f.text);
        for p in &f.places {
            // a point, or the middle of an edge's first and last point
            let (Some(first), Some(last)) = (p.points.first(), p.points.last()) else {
                continue;
            };
            let map = p
                .map
                .as_ref()
                .and_then(|m| layout.maps.iter().position(|id| id == m))
                .unwrap_or(0);
            marks.push(Mark {
                map,
                x: (first[0] + last[0]) / 2.0,
                y: (first[1] + last[1]) / 2.0,
                level: f.level,
                text: text.clone(),
                finding,
            });
        }
    }
    marks
}

struct ViewerApp {
    /// What the layout check found, placed on the floor (empty without `--check`).
    findings: Vec<Finding>,
    marks: Vec<Mark>,
    /// The finding whose text is in the side panel (index into `findings`).
    selected_finding: Option<usize>,
    /// Which severities are shown: error, warning, info.
    show_levels: [bool; 3],
    recording: Recording,
    /// The floor that is shown (an index into the layout's maps).
    map: usize,
    playback: Playback,
    /// The renderer for the current view (window size, zoom, where).
    renderer: Option<(Renderer, View)>,
    /// Why there is no picture, if there is none.
    problem: Option<String>,
    texture: Option<egui::TextureHandle>,
    /// What the texture shows: time, view and selected robot. The picture is drawn
    /// again when this changes.
    shown: Option<(f64, View, Option<String>)>,
    /// Zoomed in: how many times closer than the whole floor, and the world point in
    /// the middle of the picture. `None`: the whole floor.
    camera: Option<(f64, (f64, f64))>,
    /// The robot whose details are in the side panel (`manufacturer/serial`).
    selected: Option<String>,
    /// The file ended in the middle of a line: the run was stopped while it was written.
    cut_off: bool,
}

impl ViewerApp {
    fn new(recording: Recording, cut_off: bool, findings: Vec<Finding>) -> ViewerApp {
        let first = recording.snapshots.first().map_or(0.0, |s| s.time);
        let last = recording.snapshots.last().map_or(0.0, |s| s.time);
        let marks = marks_for(&recording.layout, &findings);
        ViewerApp {
            findings,
            marks,
            selected_finding: None,
            show_levels: [true; 3],
            recording,
            map: 0,
            playback: Playback::new(first, last),
            renderer: None,
            problem: None,
            texture: None,
            shown: None,
            camera: None,
            selected: None,
            cut_off,
        }
    }

    /// The view of an image of this size: the whole floor, or zoomed in.
    fn view_for(&self, size: [u32; 2]) -> Option<View> {
        let fit = View::fit(&self.recording.layout, self.map, size[0], size[1]).ok()?;
        Some(match self.camera {
            Some((zoom, centre)) => View {
                scale: fit.scale * zoom,
                centre,
                ..fit
            },
            None => fit,
        })
    }

    /// Zoom in by `factor` (out if below 1) round the pixel `anchor`, and move the view
    /// by `drag` pixels. Not out further than the whole floor, nor in further than
    /// `MAX_ZOOM`, and the middle stays on the floor.
    fn move_camera(&mut self, size: [u32; 2], factor: f64, anchor: (f32, f32), drag: (f32, f32)) {
        let layout = &self.recording.layout;
        let (Ok(fit), Some(view)) = (
            View::fit(layout, self.map, size[0], size[1]),
            self.view_for(size),
        ) else {
            return;
        };
        let zoom = (view.scale / fit.scale * factor).clamp(1.0, MAX_ZOOM);
        let view = view
            .zoomed(zoom * fit.scale / view.scale, anchor)
            .panned(drag.0, drag.1);
        self.camera = if zoom <= 1.0 {
            None
        } else {
            let (min_x, min_y, max_x, max_y) =
                View::content_bounds(layout, self.map).unwrap_or((0.0, 0.0, 0.0, 0.0));
            Some((
                zoom,
                (
                    view.centre.0.clamp(min_x, max_x),
                    view.centre.1.clamp(min_y, max_y),
                ),
            ))
        };
    }

    /// The renderer for this view; made again when the view changes.
    fn prepare_renderer(&mut self, view: View) {
        if self.renderer.as_ref().is_none_or(|(_, v)| *v != view) {
            self.shown = None;
            let layout = &self.recording.layout;
            let build = |text: bool| {
                let r = Renderer::with_view(layout, self.map, view).map_err(|e| e.to_string())?;
                if text {
                    r.with_clock(true)
                        .and_then(|r| r.with_brand(true))
                        .map_err(|e| e.to_string())
                } else {
                    Ok(r)
                }
            };
            // the same text as in the videos; without a font, no text
            match build(true).or_else(|_| build(false)) {
                Ok(r) => {
                    self.renderer = Some((r, view));
                    self.problem = None;
                }
                Err(e) => {
                    self.renderer = None;
                    self.problem = Some(e.to_string());
                }
            }
        }
    }
}

/// The keys: Space plays / pauses, Left / Right step 1 s back / forward (with
/// Shift 10 s), Home / End or Ctrl + Left / Right jump to the start / end, `[` and
/// `]` change the speed.
///
/// A key is taken away from the widgets (`consume_key`), so that a button or the
/// slider that has the focus does not act on it a second time.
fn handle_keys(ui: &egui::Ui, playback: &mut Playback) {
    use egui::{Key, Modifiers};
    ui.input_mut(|i| {
        if i.consume_key(Modifiers::NONE, Key::Space) {
            playback.toggle();
        }
        // Ctrl (Cmd on a Mac) and Shift first, or the plain arrow would take the
        // key too. Ctrl + arrow is for keyboards without Home and End.
        for (key, sign) in [(Key::ArrowLeft, -1.0), (Key::ArrowRight, 1.0)] {
            if i.consume_key(Modifiers::COMMAND, key) {
                if sign < 0.0 {
                    playback.to_start();
                } else {
                    playback.to_end();
                }
            } else if i.consume_key(Modifiers::SHIFT, key) {
                playback.step(10.0 * sign);
            } else if i.consume_key(Modifiers::NONE, key) {
                playback.step(sign);
            }
        }
        if i.consume_key(Modifiers::NONE, Key::Home) {
            playback.to_start();
        }
        if i.consume_key(Modifiers::NONE, Key::End) {
            playback.to_end();
        }
        if i.consume_key(Modifiers::NONE, Key::OpenBracket) {
            playback.slower();
        }
        if i.consume_key(Modifiers::NONE, Key::CloseBracket) {
            playback.faster();
        }
    });
}

/// The most the picture can be zoomed in (times the whole floor).
const MAX_ZOOM: f64 = 80.0;

/// How much bigger than egui's default the text of the window is.
const TEXT_SCALE: f32 = 1.35;

/// A white window, whatever the theme of the desktop is, and bigger text.
fn style(ctx: &egui::Context) {
    ctx.set_theme(egui::ThemePreference::Light);
    ctx.all_styles_mut(|style| {
        style.visuals.panel_fill = egui::Color32::WHITE;
        style.visuals.window_fill = egui::Color32::WHITE;
        for font in style.text_styles.values_mut() {
            font.size *= TEXT_SCALE;
        }
    });
}

/// The side panel: what the selected robot is doing now.
fn robot_panel(
    ui: &mut egui::Ui,
    layout: &LayoutLine,
    snapshot: &SnapshotLine,
    selected: Option<&str>,
) {
    let info = selected
        .and_then(|id| snapshot.robots.iter().position(|r| r.id == id))
        .and_then(|index| describe(layout, snapshot, index));
    let Some(info) = info else {
        ui.add_space(8.0);
        ui.label("Click a robot to see what it is doing.");
        return;
    };
    let list = |items: &[String], separator: &str| {
        if items.is_empty() {
            "–".to_string()
        } else {
            items.join(separator)
        }
    };
    // One field below the other, the text wrapping at the width of the panel: the
    // panel can be made narrow and the values (routes, an order name) can be long.
    ui.add_space(8.0);
    ui.add(egui::Label::new(egui::RichText::new(&info.id).heading()).wrap());
    ui.add_space(8.0);
    let mut field = |name: &str, value: String| {
        ui.label(egui::RichText::new(name).weak());
        ui.add(egui::Label::new(value).wrap());
        ui.add_space(6.0);
    };
    field("Status", info.status.clone());
    field("Speed", format!("{:.2} m/s", info.speed));
    field("Last node", info.last_node.clone().unwrap_or("–".into()));
    field("Released", list(&info.released, " → "));
    field("Planned", list(&info.planned, " → "));
    field("Waits for", list(&info.waits_for, ", "));
    field("Waited for by", list(&info.waited_for_by, ", "));
    // the order last: its name can be of any length
    field(
        "Order",
        if info.order.is_empty() {
            "–".to_string()
        } else {
            info.order.clone()
        },
    );
}

impl eframe::App for ViewerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // time
        let dt = ui.input(|i| i.stable_dt) as f64;
        handle_keys(ui, &mut self.playback);
        self.playback.advance(dt);
        if self.playback.is_playing() {
            ui.ctx().request_repaint();
        }

        // the controls
        egui::Panel::bottom("controls").show(ui, |ui| {
            if self.cut_off {
                ui.label(format!(
                    "The run was stopped: the recording ends at {:.1} s.",
                    self.playback.end()
                ));
            }
            ui.horizontal(|ui| {
                let label = if self.playback.is_playing() {
                    "Pause"
                } else {
                    "Play"
                };
                if ui.button(label).clicked() {
                    self.playback.toggle();
                }
                let mut speed = self.playback.speed();
                egui::ComboBox::from_id_salt("speed")
                    .selected_text(format!("{speed}×"))
                    .show_ui(ui, |ui| {
                        for s in SPEEDS {
                            ui.selectable_value(&mut speed, s, format!("{s}×"));
                        }
                    });
                self.playback.set_speed(speed);
                // the zoom: the wheel zooms, a drag moves, a double click shows it all
                let zoom = self.camera.map_or(1.0, |(z, _)| z);
                if ui
                    .add_enabled(self.camera.is_some(), egui::Button::new("Show all"))
                    .on_hover_text("Wheel: zoom, drag: move, double click: show all")
                    .clicked()
                {
                    self.camera = None;
                }
                ui.label(format!("{zoom:.1}×"));
                if !self.findings.is_empty() {
                    for (i, level) in [Level::Error, Level::Warning, Level::Info]
                        .into_iter()
                        .enumerate()
                    {
                        let n = self.findings.iter().filter(|f| f.level == level).count();
                        ui.checkbox(&mut self.show_levels[i], format!("{level} {n}"));
                    }
                }
                let mut time = self.playback.time();
                ui.spacing_mut().slider_width = (ui.available_width() - 90.0).max(50.0);
                let range = self.playback.start()..=self.playback.end();
                if ui
                    .add(egui::Slider::new(&mut time, range).suffix(" s"))
                    .changed()
                {
                    self.playback.seek(time);
                }
            });
        });

        // the world at this moment
        let time = self.playback.time();
        let snapshot = sample(&self.recording.snapshots, time).expect("has snapshots");

        // the selected robot
        egui::Panel::right("robot")
            .resizable(true)
            .default_size(360.0)
            .size_range(200.0..=900.0)
            .show(ui, |ui| {
                if let Some(f) = self.selected_finding.and_then(|i| self.findings.get(i)) {
                    ui.add_space(8.0);
                    ui.strong(format!("{} [{}]", f.level, f.check));
                    if let Some(r) = &f.robot_type {
                        ui.label(format!("robot type: {r}"));
                    }
                    ui.label(&f.text);
                    let places: Vec<&str> = f.places.iter().map(|p| p.id.as_str()).collect();
                    ui.label(format!("{} place(s): {}", places.len(), places.join(", ")));
                    ui.separator();
                }
                robot_panel(
                    ui,
                    &self.recording.layout,
                    &snapshot,
                    self.selected.as_deref(),
                );
            });

        // the picture, for the size of what is left of the window
        egui::CentralPanel::default().show(ui, |ui| {
            let ppp = ui.ctx().pixels_per_point();
            let points = ui.available_size();
            let size = [
                (points.x * ppp).round().max(1.0) as u32,
                (points.y * ppp).round().max(1.0) as u32,
            ];
            // the wheel (or a pinch) zooms round the pointer, a drag moves the picture, a
            // double click shows the whole floor
            let rect = ui.available_rect_before_wrap();
            let id = ui.make_persistent_id("picture");
            let response = ui.interact(rect, id, egui::Sense::click_and_drag());
            if response.double_clicked() {
                self.camera = None;
            } else {
                let (scroll, pinch, hover) = ui.input(|i| {
                    (
                        i.smooth_scroll_delta.y,
                        i.zoom_delta(),
                        i.pointer.hover_pos(),
                    )
                });
                let factor = f64::from(pinch) * (f64::from(scroll) * 0.0015).exp();
                let drag = response.drag_delta() * ppp;
                if (factor - 1.0).abs() > 1e-9 || drag != egui::Vec2::ZERO {
                    let at = hover.filter(|p| rect.contains(*p)).unwrap_or(rect.center());
                    let anchor = (at - rect.min) * ppp;
                    self.move_camera(size, factor, (anchor.x, anchor.y), (drag.x, drag.y));
                    ui.ctx().request_repaint();
                }
            }
            let Some(view) = self.view_for(size) else {
                ui.label("There is nothing to show on this floor.");
                return;
            };
            self.prepare_renderer(view);
            let Some((renderer, _)) = &self.renderer else {
                ui.label(self.problem.clone().unwrap_or_default());
                return;
            };
            let wanted = (time, view, self.selected.clone());
            if self.shown.as_ref() != Some(&wanted) {
                // the selected robot's own curves are drawn strong
                let pixmap = renderer.frame_selected(&snapshot, self.selected.as_deref());
                let image = egui::ColorImage::from_rgba_unmultiplied(
                    [size[0] as usize, size[1] as usize],
                    pixmap.data(),
                );
                match &mut self.texture {
                    Some(texture) => texture.set(image, egui::TextureOptions::NEAREST),
                    None => {
                        self.texture = Some(ui.ctx().load_texture(
                            "frame",
                            image,
                            egui::TextureOptions::NEAREST,
                        ));
                    }
                }
                self.shown = Some(wanted);
            }
            if let Some(texture) = &self.texture {
                egui::Image::new(texture)
                    .fit_to_exact_size(points)
                    .paint_at(ui, rect);
                // the layout check: a ring for each placed finding (red error, orange
                // warning, blue info); the text shows when the pointer is near
                {
                    let painter = ui.painter_at(rect);
                    let hover = response.hover_pos();
                    let mut tip: Option<&Mark> = None;
                    let click = if response.clicked() {
                        response.interact_pointer_pos()
                    } else {
                        None
                    };
                    let mut clicked_finding: Option<usize> = None;
                    let shown = |l: Level| match l {
                        Level::Error => self.show_levels[0],
                        Level::Warning => self.show_levels[1],
                        Level::Info => self.show_levels[2],
                    };
                    for m in self
                        .marks
                        .iter()
                        .filter(|m| m.map == self.map && shown(m.level))
                    {
                        let (px, py) = renderer.view().to_px(m.x, m.y);
                        let at = rect.min + egui::vec2(px, py) / ppp;
                        let colour = match m.level {
                            Level::Error => egui::Color32::from_rgb(220, 30, 30),
                            Level::Warning => egui::Color32::from_rgb(240, 150, 0),
                            Level::Info => egui::Color32::from_rgb(60, 120, 220),
                        };
                        // the selected finding's places are drawn bigger and thicker
                        if self.selected_finding == Some(m.finding) {
                            painter.circle_stroke(at, 10.0, egui::Stroke::new(4.0, colour));
                        } else {
                            painter.circle_stroke(at, 7.0, egui::Stroke::new(2.0, colour));
                        }
                        if hover.is_some_and(|h| h.distance(at) < 9.0) {
                            tip = Some(m);
                        }
                        if click.is_some_and(|c| c.distance(at) < 9.0) {
                            clicked_finding = Some(m.finding);
                        }
                    }
                    if let Some(m) = tip {
                        response.clone().on_hover_text(&m.text);
                    }
                    if click.is_some() {
                        self.selected_finding = clicked_finding;
                    }
                }
                // a click that misses every ring picks the robot under it, or none
                if response.clicked()
                    && self.selected_finding.is_none()
                    && let Some(pos) = response.interact_pointer_pos()
                {
                    let pixel = (pos - rect.min) * ppp;
                    let picked = pick_robot(
                        &snapshot.robots,
                        self.map,
                        renderer.view(),
                        (pixel.x, pixel.y),
                    );
                    self.selected = picked.map(|i| snapshot.robots[i].id.clone());
                }
            }
        });
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (check, path) = match args.as_slice() {
        [path] => (None, path),
        [flag, scenario, path] if flag == "--check" => (Some(scenario), path),
        _ => {
            eprintln!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    if path.starts_with("--") {
        eprintln!("unknown option {path}\n{USAGE}");
        return ExitCode::from(2);
    }
    let findings = match check {
        Some(scenario) => match check_scenario(Path::new(scenario)) {
            Ok(f) => f,
            Err(e) => {
                eprintln!("error: check {scenario}: {e}");
                return ExitCode::from(1);
            }
        },
        None => Vec::new(),
    };
    let path = PathBuf::from(path);
    let (recording, cut_off) = match File::open(&path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))
        .and_then(|f| {
            read_recording_partial(BufReader::new(f))
                .map_err(|e| format!("{}: {e}", path.display()))
        }) {
        Ok((r, _)) if r.snapshots.is_empty() => {
            eprintln!("error: {}: the recording has no snapshots", path.display());
            return ExitCode::from(1);
        }
        Ok((r, cut_off)) => {
            if cut_off {
                eprintln!("note: the last line is cut off (the run was stopped); showing the rest");
            }
            (r, cut_off)
        }
        Err(message) => {
            eprintln!("error: {message}");
            return ExitCode::from(1);
        }
    };
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1280.0, 800.0]),
        ..eframe::NativeOptions::default()
    };
    let title = format!("botsim viewer: {}", path.display());
    let result = eframe::run_native(
        &title,
        options,
        Box::new(|cc| {
            style(&cc.egui_ctx);
            Ok(Box::new(ViewerApp::new(recording, cut_off, findings)))
        }),
    );
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(1)
        }
    }
}

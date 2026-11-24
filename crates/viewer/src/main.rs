//! `botsim-viewer <recording.jsonl>`: replay a recording in a window.
//!
//! The picture is the one `botsim-export` draws for a video (same colours, same
//! drawing), made for the size of the window. Below it: play / pause, a slider to
//! scrub through the run, and the speed.

use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;
use std::process::ExitCode;

use botsim_check::{Finding, Level, Options, run_scenario};
use botsim_export::{Renderer, View, sample};
use botsim_record::{LayoutLine, Recording, SnapshotLine, read_recording_partial};
use botsim_viewer::{
    ALL_STATUSES, Group, Playback, SPEEDS, StatusPoint, Usage, busiest_edges, describe,
    group_table, groups_from_json, most_waited_nodes, pick_robot, point_at, status_colour,
    status_series, usage_between,
};
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

/// The layout check running in a thread: its findings, or why it failed.
type CheckResult = std::sync::mpsc::Receiver<Result<CheckOutput, String>>;

/// What the check gives the viewer: the findings, and the pairs of edge ids whose swept
/// outlines overlap although the two edges have no node in common (the conflicts that
/// the node reservation does not cover), each pair once.
struct CheckOutput {
    findings: Vec<Finding>,
    open_conflicts: Vec<(String, String)>,
}

/// A conflict drawn as a line between the middles of its two edges.
struct ConflictLine {
    map: usize,
    from: (f64, f64),
    to: (f64, f64),
}

/// The conflicts as lines between the edges' middles (edges that are not in the layout
/// are left out).
fn conflict_lines(layout: &LayoutLine, pairs: &[(String, String)]) -> Vec<ConflictLine> {
    let middle = |id: &str| {
        let e = layout.edges.iter().find(|e| e.id == id)?;
        let a = layout.nodes.get(e.start)?;
        let b = layout.nodes.get(e.end)?;
        Some((a.map, ((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)))
    };
    pairs
        .iter()
        .filter_map(|(a, b)| {
            let (map, from) = middle(a)?;
            let (_, to) = middle(b)?;
            Some(ConflictLine { map, from, to })
        })
        .collect()
}

struct ViewerApp {
    /// The check still running, if any.
    check: Option<CheckResult>,
    /// Why the check failed.
    check_error: Option<String>,
    /// What the layout check found, placed on the floor (empty without `--check`).
    findings: Vec<Finding>,
    marks: Vec<Mark>,
    /// The finding whose text is in the side panel (index into `findings`).
    selected_finding: Option<usize>,
    /// Which severities are shown: error, warning, info.
    show_levels: [bool; 3],
    /// Show only the heat map of the use of the roads (see `usage`), not the picture.
    show_heat: bool,
    /// The use of the roads over the whole recording, counted when the heat map is first shown.
    usage: Option<Usage>,
    /// The window of time of the usage (seconds from, to): the start of the run at first.
    heat_window: (f64, f64),
    /// The groups of roads for the table of the use (from `<scenario>.groups.json` next to the
    /// scenario given with `--check`; none: no table).
    groups: Vec<Group>,
    /// The conflicts without a common node, as lines (see [`CheckOutput`]).
    conflicts: Vec<ConflictLine>,
    show_conflicts: bool,
    /// How many robots are in each status at each snapshot (the strip under the slider).
    statuses: Vec<StatusPoint>,
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
    fn new(
        recording: Recording,
        cut_off: bool,
        check: Option<CheckResult>,
        groups: Vec<Group>,
    ) -> ViewerApp {
        let first = recording.snapshots.first().map_or(0.0, |s| s.time);
        let last = recording.snapshots.last().map_or(0.0, |s| s.time);
        ViewerApp {
            check,
            check_error: None,
            findings: Vec::new(),
            marks: Vec::new(),
            selected_finding: None,
            show_levels: [true; 3],
            show_heat: false,
            usage: None,
            heat_window: (first, last.min(first + 500.0)),
            groups,
            conflicts: Vec::new(),
            show_conflicts: true,
            statuses: status_series(&recording),
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
/// The colour of a road in the heat map: `t` from 0 (not used: light grey) over yellow to 1
/// (the most used: dark red).
fn heat_colour(t: f64) -> egui::Color32 {
    let t = t.clamp(0.0, 1.0);
    let lerp = |a: f64, b: f64, k: f64| (a + (b - a) * k).round() as u8;
    if t < 0.5 {
        let k = t / 0.5;
        egui::Color32::from_rgb(lerp(205.0, 245.0, k), lerp(208.0, 200.0, k), lerp(215.0, 40.0, k))
    } else {
        let k = (t - 0.5) / 0.5;
        egui::Color32::from_rgb(lerp(245.0, 170.0, k), lerp(200.0, 20.0, k), lerp(40.0, 20.0, k))
    }
}

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
        // the layout check: wait for it without blocking the window
        if let Some(rx) = &self.check {
            match rx.try_recv() {
                Ok(Ok(output)) => {
                    self.marks = marks_for(&self.recording.layout, &output.findings);
                    self.conflicts =
                        conflict_lines(&self.recording.layout, &output.open_conflicts);
                    self.findings = output.findings;
                    self.check = None;
                }
                Ok(Err(e)) => {
                    self.check_error = Some(e);
                    self.check = None;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {
                    ui.ctx()
                        .request_repaint_after(std::time::Duration::from_millis(200));
                }
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.check_error = Some("the check stopped without a result".to_string());
                    self.check = None;
                }
            }
        }
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
                let mut heat_changed = ui
                    .checkbox(&mut self.show_heat, "Usage")
                    .on_hover_text(
                        "Only a heat map of how much each road was used (time reserved) in \
                         the window, and where robots waited",
                    )
                    .changed();
                if self.show_heat {
                    // the window of time: a run that freezes says most in its first part
                    let end = self.playback.end();
                    ui.label("from");
                    heat_changed |= ui
                        .add(
                            egui::DragValue::new(&mut self.heat_window.0)
                                .range(0.0..=end)
                                .speed(5.0)
                                .suffix(" s"),
                        )
                        .changed();
                    ui.label("to");
                    heat_changed |= ui
                        .add(
                            egui::DragValue::new(&mut self.heat_window.1)
                                .range(0.0..=end)
                                .speed(5.0)
                                .suffix(" s"),
                        )
                        .changed();
                    if ui.button("all").clicked() {
                        self.heat_window = (0.0, end);
                        heat_changed = true;
                    }
                }
                if self.show_heat && (heat_changed || self.usage.is_none()) {
                    self.usage = Some(usage_between(
                        &self.recording,
                        self.heat_window.0,
                        self.heat_window.1,
                    ));
                }
                if self.check.is_some() {
                    ui.label("checking the layout…");
                }
                if let Some(e) = &self.check_error {
                    ui.colored_label(egui::Color32::from_rgb(220, 30, 30), e);
                }
                if !self.findings.is_empty() {
                    for (i, level) in [Level::Error, Level::Warning, Level::Info]
                        .into_iter()
                        .enumerate()
                    {
                        let n = self.findings.iter().filter(|f| f.level == level).count();
                        ui.checkbox(&mut self.show_levels[i], format!("{level} {n}"));
                    }
                    if !self.conflicts.is_empty() {
                        ui.checkbox(
                            &mut self.show_conflicts,
                            format!("conflicts {}", self.conflicts.len()),
                        )
                        .on_hover_text(
                            "Pairs of edges whose swept outlines overlap and that have no node in common",
                        );
                    }
                }
            });
            // the time line: a stacked histogram of the statuses over time (colours as in the
            // table at the right) that is also the scrubber: click or drag to jump there
            let bar_width = (ui.available_width() - 100.0).max(50.0);
            ui.horizontal(|ui| {
                let now = self.playback.time();
                let (start, end) = (self.playback.start(), self.playback.end());
                let size = egui::vec2(bar_width, 56.0);
                let (response, painter) = ui.allocate_painter(size, egui::Sense::click_and_drag());
                let rect = response.rect;
                painter.rect_filled(rect, 0.0, ui.visuals().extreme_bg_color);
                let span = (end - start).max(1e-9);
                let x_of = |t: f64| rect.left() + (((t - start) / span) as f32) * rect.width();
                // one column of pixels at a time: the status counts at the time of the column
                let total = self
                    .statuses
                    .iter()
                    .map(|p| p.counts.iter().sum::<usize>())
                    .max()
                    .unwrap_or(1)
                    .max(1) as f32;
                let columns = rect.width().floor().max(1.0) as usize;
                for c in 0..columns {
                    let t = start + span * (c as f64 + 0.5) / columns as f64;
                    let Some(p) = point_at(&self.statuses, t) else {
                        break;
                    };
                    let x0 = rect.left() + c as f32;
                    let mut y = rect.bottom();
                    for (k, &status) in ALL_STATUSES.iter().enumerate() {
                        if p.counts[k] == 0 {
                            continue;
                        }
                        let h = rect.height() * p.counts[k] as f32 / total;
                        let [r, g, b] = status_colour(status);
                        painter.rect_filled(
                            egui::Rect::from_min_max(
                                egui::pos2(x0, y - h),
                                egui::pos2(x0 + 1.0, y),
                            ),
                            0.0,
                            egui::Color32::from_rgb(r, g, b),
                        );
                        y -= h;
                    }
                }
                let x = x_of(now);
                painter.line_segment(
                    [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
                    egui::Stroke::new(1.5, ui.visuals().strong_text_color()),
                );
                if let (true, Some(pos)) = (
                    response.clicked() || response.dragged(),
                    response.interact_pointer_pos(),
                ) {
                    let f = ((pos.x - rect.left()) / rect.width()).clamp(0.0, 1.0) as f64;
                    self.playback.seek(start + f * span);
                }
                // the time, in a box of fixed size (so the line does not move)
                ui.add_sized(
                    [90.0, 20.0],
                    egui::Label::new(
                        egui::RichText::new(format!("{:.1} s", self.playback.time())).monospace(),
                    ),
                );
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
                // at the bottom: every status a robot can have, and how many robots have it now
                egui::Panel::bottom("status_counts").show(ui, |ui| {
                    ui.add_space(4.0);
                    ui.strong(format!("Statuses at {time:.0} s"));
                    egui::Grid::new("status_grid").num_columns(2).show(ui, |ui| {
                        for status in ALL_STATUSES {
                            let n = snapshot.robots.iter().filter(|r| r.status == status).count();
                            let [r, g, b] = status_colour(status);
                            ui.colored_label(
                                egui::Color32::from_rgb(r, g, b),
                                format!("■ {status:?}"),
                            );
                            ui.label(format!("{n}"));
                            ui.end_row();
                        }
                        ui.strong("all robots");
                        ui.strong(format!("{}", snapshot.robots.len()));
                        ui.end_row();
                    });
                });
                if let (true, Some(u)) = (self.show_heat, &self.usage) {
                    ui.add_space(8.0);
                    ui.strong("Use of the roads");
                    ui.label(format!(
                        "reserved share of {:.0} s; busiest edge {:.0} %",
                        u.duration,
                        100.0 * u.max_edge_share()
                    ));
                    if !self.groups.is_empty() {
                        ui.label("by group of roads (mean share reserved):");
                        for r in group_table(&self.recording.layout, u, &self.groups) {
                            ui.label(format!(
                                "  {}  {:.1} %  ({} edges, busiest {:.0} %)",
                                r.name,
                                100.0 * r.mean_share,
                                r.edges,
                                100.0 * r.max_share
                            ));
                        }
                    }
                    ui.label("busiest edges:");
                    for (id, share) in busiest_edges(&self.recording.layout, u, 15) {
                        ui.label(format!("  {id}  {:.0} %", 100.0 * share));
                    }
                    ui.label("most waiting (robot-seconds / run):");
                    for (id, share) in most_waited_nodes(&self.recording.layout, u, 10) {
                        ui.label(format!("  {id}  {:.2}", share));
                    }
                    ui.separator();
                }
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
                // the heat map: only the roads, coloured by how long they were reserved (grey
                // little, yellow, red most; the width goes with it), and a red disc at the nodes
                // where robots waited (by how long)
                if let (true, Some(u)) = (self.show_heat, &self.usage) {
                    let painter = ui.painter_at(rect);
                    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(250, 250, 252));
                    let layout = &self.recording.layout;
                    let view = renderer.view();
                    let at = |x: f64, y: f64| {
                        let (px, py) = view.to_px(x, y);
                        rect.min + egui::vec2(px, py) / ppp
                    };
                    let max = u.max_edge_share().max(1e-9);
                    let mut order: Vec<usize> = (0..layout.edges.len()).collect();
                    order.sort_by(|&a, &b| u.edge_share(a).total_cmp(&u.edge_share(b)));
                    for e in order {
                        let edge = &layout.edges[e];
                        let (Some(a), Some(b)) = (layout.nodes.get(edge.start), layout.nodes.get(edge.end))
                        else {
                            continue;
                        };
                        if a.map != self.map {
                            continue;
                        }
                        let t = (u.edge_share(e) / max).sqrt();
                        let colour = heat_colour(t);
                        let stroke = egui::Stroke::new(1.0 + 3.5 * t as f32, colour);
                        let points: Vec<egui::Pos2> = match edge.paths.first() {
                            Some(p) if p.points.len() >= 2 => {
                                p.points.iter().map(|q| at(q[0], q[1])).collect()
                            }
                            _ => vec![at(a.x, a.y), at(b.x, b.y)],
                        };
                        painter.add(egui::Shape::line(points, stroke));
                    }
                    let wait_max = u.max_node_wait_share().max(1e-9);
                    for (n, node) in layout.nodes.iter().enumerate() {
                        let share = u.node_wait_share(n);
                        if node.map != self.map || share <= 0.0 {
                            continue;
                        }
                        let t = (share / wait_max).sqrt() as f32;
                        painter.circle_filled(
                            at(node.x, node.y),
                            2.0 + 9.0 * t,
                            egui::Color32::from_rgba_unmultiplied(200, 30, 30, 120),
                        );
                    }
                }
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
                    if self.show_conflicts {
                        let stroke =
                            egui::Stroke::new(1.5, egui::Color32::from_rgba_unmultiplied(200, 40, 160, 160));
                        for c in self.conflicts.iter().filter(|c| c.map == self.map) {
                            let (ax, ay) = renderer.view().to_px(c.from.0, c.from.1);
                            let (bx, by) = renderer.view().to_px(c.to.0, c.to.1);
                            painter.line_segment(
                                [
                                    rect.min + egui::vec2(ax, ay) / ppp,
                                    rect.min + egui::vec2(bx, by) / ppp,
                                ],
                                stroke,
                            );
                        }
                    }
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
    // the groups of roads for the table of the use: `<scenario>.groups.json` next to the scenario
    let groups = check
        .and_then(|scenario| {
            let p = PathBuf::from(scenario).with_extension("groups.json");
            std::fs::read_to_string(&p).ok().map(|t| (p, t))
        })
        .map(|(p, t)| match groups_from_json(&t) {
            Ok(g) => g,
            Err(e) => {
                eprintln!("note: {}: {e}", p.display());
                Vec::new()
            }
        })
        .unwrap_or_default();
    // the check runs in a thread (the conflict map takes seconds); the window opens at once
    let check_result = check.map(|scenario| {
        let scenario = PathBuf::from(scenario);
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            // the geometry file next to the scenario (`warehouse4.geometry.json`) gives the
            // clearance check its walls and racks
            let geometry = scenario.with_extension("geometry.json");
            let options = Options {
                geometry: geometry.exists().then_some(geometry),
                ..Options::default()
            };
            let result = run_scenario(&scenario, &options)
                .map(|report| {
                    let mut seen = std::collections::HashSet::new();
                    let mut open_conflicts = Vec::new();
                    for pair in report.conflict_map.iter().flat_map(|m| &m.pairs) {
                        for c in pair.conflicts.iter().filter(|c| !c.shares_node) {
                            let key = if c.edge_a <= c.edge_b {
                                (c.edge_a.clone(), c.edge_b.clone())
                            } else {
                                (c.edge_b.clone(), c.edge_a.clone())
                            };
                            if seen.insert(key.clone()) {
                                open_conflicts.push(key);
                            }
                        }
                    }
                    CheckOutput {
                        findings: report.findings,
                        open_conflicts,
                    }
                })
                .map_err(|e| format!("check {}: {e}", scenario.display()));
            let _ = tx.send(result);
        });
        rx
    });
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
            Ok(Box::new(ViewerApp::new(recording, cut_off, check_result, groups)))
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

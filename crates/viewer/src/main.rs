//! `botsim-viewer <recording.jsonl>`: replay a recording in a window.
//!
//! The picture is the one `botsim-export` draws for a video (same colours, same
//! drawing), made for the size of the window. Below it: play / pause, a slider to
//! scrub through the run, and the speed.

use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;
use std::process::ExitCode;

use botsim_export::{Renderer, sample};
use botsim_record::{LayoutLine, Recording, SnapshotLine, read_recording};
use botsim_viewer::{Playback, SPEEDS, describe, pick_robot};
use eframe::egui;

const USAGE: &str = "usage: botsim-viewer <recording.jsonl>";

struct ViewerApp {
    recording: Recording,
    /// The floor that is shown (an index into the layout's maps).
    map: usize,
    playback: Playback,
    /// The renderer for the current window size, and that size in pixels.
    renderer: Option<(Renderer, [u32; 2])>,
    /// Why there is no picture, if there is none.
    problem: Option<String>,
    texture: Option<egui::TextureHandle>,
    /// What the texture shows: time and size. The picture is drawn again when this
    /// changes.
    shown: Option<(f64, [u32; 2], Option<String>)>,
    /// The robot whose details are in the side panel (`manufacturer/serial`).
    selected: Option<String>,
}

impl ViewerApp {
    fn new(recording: Recording) -> ViewerApp {
        let first = recording.snapshots.first().map_or(0.0, |s| s.time);
        let last = recording.snapshots.last().map_or(0.0, |s| s.time);
        ViewerApp {
            recording,
            map: 0,
            playback: Playback::new(first, last),
            renderer: None,
            problem: None,
            texture: None,
            shown: None,
            selected: None,
        }
    }

    /// The renderer for an image of this size; made again when the size changes.
    fn prepare_renderer(&mut self, size: [u32; 2]) {
        if self.renderer.as_ref().is_none_or(|(_, s)| *s != size) {
            self.shown = None;
            let layout = &self.recording.layout;
            let build = |text: bool| {
                let r =
                    Renderer::new(layout, self.map, size[0], size[1]).map_err(|e| e.to_string())?;
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
                    self.renderer = Some((r, size));
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
            self.prepare_renderer(size);
            let Some((renderer, _)) = &self.renderer else {
                ui.label(self.problem.clone().unwrap_or_default());
                return;
            };
            let wanted = (time, size, self.selected.clone());
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
                let picture = ui.add(
                    egui::Image::new(texture)
                        .fit_to_exact_size(points)
                        .sense(egui::Sense::click()),
                );
                // a click picks the robot under it, or none
                if picture.clicked()
                    && let Some(pos) = picture.interact_pointer_pos()
                {
                    let pixel = (pos - picture.rect.min) * ppp;
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
    let [path] = args.as_slice() else {
        eprintln!("{USAGE}");
        return ExitCode::from(2);
    };
    if path.starts_with("--") {
        eprintln!("unknown option {path}\n{USAGE}");
        return ExitCode::from(2);
    }
    let path = PathBuf::from(path);
    let recording = match File::open(&path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))
        .and_then(|f| {
            read_recording(BufReader::new(f)).map_err(|e| format!("{}: {e}", path.display()))
        }) {
        Ok(r) if r.snapshots.is_empty() => {
            eprintln!("error: {}: the recording has no snapshots", path.display());
            return ExitCode::from(1);
        }
        Ok(r) => r,
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
            Ok(Box::new(ViewerApp::new(recording)))
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

//! The text in a frame: the clock (the simulated time, top-left) and the name
//! `botsim` (at the bottom, a few metres inside the right wall of the building).

use std::fmt;

use ab_glyph::{Font, FontVec, PxScale, ScaleFont, point};
use tiny_skia::{
    Color, FillRule, Paint, PathBuilder, Pixmap, PremultipliedColorU8, Rect, Transform,
};

use crate::palette::{self, Rgb};

/// Fonts to look for, in this order: monospace ones, so the label does not
/// jitter when the digits change. Any monospace, then any sans-serif font of the
/// system is the fallback.
const FONT_NAMES: &[&str] = &[
    "DejaVu Sans Mono",
    "Liberation Mono",
    "Noto Sans Mono",
    "Ubuntu Mono",
    "Menlo",
    "Consolas",
    "Courier New",
];

/// Text size as a share of the image height.
const TEXT_SIZE: f32 = 0.035;
/// Distance of the label from the corner, as a share of the image height.
const MARGIN: f32 = 0.03;
/// Space between the text and the edge of its plate, as a share of the text size.
const PADDING: f32 = 0.4;
const TEXT: Rgb = [0x37, 0x41, 0x51];
/// The name in the corner of every frame; a little smaller and lighter than the
/// clock.
pub const BRAND: &str = "botsim";
const BRAND_SIZE: f32 = 0.028;
const BRAND_TEXT: Rgb = [0x6B, 0x72, 0x80];
/// The plate behind the text: the background colour, mostly opaque.
const PLATE_ALPHA: u8 = 217;

/// Draws the label.
pub struct Clock {
    font: FontVec,
}

/// No usable font on the system.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NoFont;

impl fmt::Display for NoFont {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "no usable font found on this system for the text in the frames"
        )
    }
}

impl std::error::Error for NoFont {}

impl Clock {
    /// Find a font on the system (see [`FONT_NAMES`]).
    pub fn new() -> Result<Clock, NoFont> {
        let mut db = fontdb::Database::new();
        db.load_system_fonts();
        let mut families: Vec<fontdb::Family> =
            FONT_NAMES.iter().map(|n| fontdb::Family::Name(n)).collect();
        families.push(fontdb::Family::Monospace);
        families.push(fontdb::Family::SansSerif);
        let query = fontdb::Query {
            families: &families,
            ..fontdb::Query::default()
        };
        let id = db.query(&query).ok_or(NoFont)?;
        let font = db
            .with_face_data(id, |data, index| {
                FontVec::try_from_vec_and_index(data.to_vec(), index).ok()
            })
            .flatten()
            .ok_or(NoFont)?;
        Ok(Clock { font })
    }

    /// The words: `100.0 s`.
    pub fn text(seconds: f64) -> String {
        format!("{seconds:.1} s")
    }

    /// The plate of the clock for an image of this height: `(x, y, width,
    /// height)` in pixels, at the top, its left edge at `left` (pixels from the left
    /// of the image; not nearer to the image edge than the margin).
    pub fn area(&self, left: f32, image_height: u32, seconds: f64) -> (f32, f32, f32, f32) {
        let size = (image_height as f32 * TEXT_SIZE).max(6.0);
        let (w, h) = self.plate_size(&Clock::text(seconds), size);
        let margin = image_height as f32 * MARGIN;
        (left.max(margin), margin, w, h)
    }

    /// The plate of the botsim name for an image of this height: at the bottom, its
    /// right edge at `right` (pixels from the left of the image).
    pub fn brand_area(&self, right: f32, image_height: u32) -> (f32, f32, f32, f32) {
        let size = (image_height as f32 * BRAND_SIZE).max(6.0);
        let (w, h) = self.plate_size(BRAND, size);
        let margin = image_height as f32 * MARGIN;
        (right - w, image_height as f32 - margin - h, w, h)
    }

    /// Size of the plate around `text` at a text size.
    fn plate_size(&self, text: &str, size: f32) -> (f32, f32) {
        let scaled = self.font.as_scaled(PxScale::from(size));
        let text_width: f32 = text
            .chars()
            .map(|c| scaled.h_advance(scaled.glyph_id(c)))
            .sum();
        let pad = size * PADDING;
        (text_width + 2.0 * pad, scaled.height() + 2.0 * pad)
    }

    /// Draw the clock for `seconds` at the top of the picture, its left edge at `left`.
    pub fn draw(&self, pixmap: &mut Pixmap, left: f32, seconds: f64) {
        let size = (pixmap.height() as f32 * TEXT_SIZE).max(6.0);
        let (x, y, ..) = self.area(left, pixmap.height(), seconds);
        self.draw_text(pixmap, &Clock::text(seconds), (x, y), size, TEXT);
    }

    /// Draw the name `botsim` at the bottom of the picture, its right edge at `right`.
    pub fn draw_brand(&self, pixmap: &mut Pixmap, right: f32) {
        let size = (pixmap.height() as f32 * BRAND_SIZE).max(6.0);
        let (x, y, ..) = self.brand_area(right, pixmap.height());
        self.draw_text(pixmap, BRAND, (x, y), size, BRAND_TEXT);
    }

    /// Text on a plate; `(x, y)` is the top-left corner of the plate.
    fn draw_text(&self, pixmap: &mut Pixmap, text: &str, (x, y): (f32, f32), size: f32, c: Rgb) {
        let scale = PxScale::from(size);
        let scaled = self.font.as_scaled(scale);
        let (w, h) = self.plate_size(text, size);

        if let Some(rect) = Rect::from_xywh(x, y, w, h) {
            let mut paint = Paint::default();
            let [r, g, b] = palette::BACKGROUND;
            paint.set_color(Color::from_rgba8(r, g, b, PLATE_ALPHA));
            pixmap.fill_path(
                &PathBuilder::from_rect(rect),
                &paint,
                FillRule::Winding,
                Transform::identity(),
                None,
            );
        }

        let pad = size * PADDING;
        let baseline = y + pad + scaled.ascent();
        let mut caret = x + pad;
        for ch in text.chars() {
            let id = scaled.glyph_id(ch);
            let glyph = id.with_scale_and_position(scale, point(caret, baseline));
            caret += scaled.h_advance(id);
            if let Some(outlined) = self.font.outline_glyph(glyph) {
                let bounds = outlined.px_bounds();
                outlined.draw(|gx, gy, coverage| {
                    let px = bounds.min.x as i64 + gx as i64;
                    let py = bounds.min.y as i64 + gy as i64;
                    blend(pixmap, px, py, c, coverage);
                });
            }
        }
    }
}

/// Put `colour` over the pixel (opaque pictures only: the alpha stays 255).
fn blend(pixmap: &mut Pixmap, x: i64, y: i64, colour: Rgb, coverage: f32) {
    if x < 0 || y < 0 || x >= pixmap.width() as i64 || y >= pixmap.height() as i64 {
        return;
    }
    let coverage = coverage.clamp(0.0, 1.0);
    let width = pixmap.width() as usize;
    let pixel = &mut pixmap.pixels_mut()[y as usize * width + x as usize];
    let mix =
        |old: u8, new: u8| (old as f32 * (1.0 - coverage) + new as f32 * coverage).round() as u8;
    if let Some(p) = PremultipliedColorU8::from_rgba(
        mix(pixel.red(), colour[0]),
        mix(pixel.green(), colour[1]),
        mix(pixel.blue(), colour[2]),
        255,
    ) {
        *pixel = p;
    }
}

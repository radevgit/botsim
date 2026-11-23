//! Which part of a floor the picture shows, and where a point lands in it.

use std::fmt;

use botsim_record::LayoutLine;

/// Margin around the content, as a share of the image (each side).
const MARGIN: f64 = 0.05;

/// The smallest extent of the content, metres; a floor with one node or a
/// straight line of nodes would otherwise have no size.
const MIN_EXTENT: f64 = 1.0;

#[derive(Debug, Clone, PartialEq)]
pub enum ViewError {
    /// The map index is not in the layout.
    NoSuchMap(usize),
    /// Nothing on this floor to fit the view to (no node, no geometry).
    EmptyMap(usize),
    /// The image has no pixels.
    NoPixels,
}

impl fmt::Display for ViewError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ViewError::NoSuchMap(m) => write!(f, "the layout has no map number {m}"),
            ViewError::EmptyMap(m) => write!(f, "map number {m} has no nodes and no geometry"),
            ViewError::NoPixels => write!(f, "the image must be at least 1×1 pixels"),
        }
    }
}

impl std::error::Error for ViewError {}

/// A fixed view of one floor: equal scale in x and y, the content centred, a 5%
/// margin. World x points right, world y points up; image y points down.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct View {
    pub width: u32,
    pub height: u32,
    /// Pixels per metre.
    pub scale: f64,
    /// The world point in the middle of the image.
    pub centre: (f64, f64),
}

impl View {
    /// The box `(min_x, min_y, max_x, max_y)` round everything on floor `map`: its
    /// nodes, its geometry (the walls of the building) and its curves; `None` if
    /// there is nothing.
    pub fn content_bounds(layout: &LayoutLine, map: usize) -> Option<(f64, f64, f64, f64)> {
        let nodes = layout
            .nodes
            .iter()
            .filter(|n| n.map == map)
            .map(|n| (n.x, n.y));
        let corners = layout
            .geometry
            .iter()
            .filter(|s| s.map == map)
            .flat_map(|s| s.vertices.iter().map(|v| (v.x, v.y)));
        // a curved edge can bulge out of the box round its nodes
        let curves = layout
            .edges
            .iter()
            .filter(|e| {
                !e.map_transition && layout.nodes.get(e.start).is_some_and(|n| n.map == map)
            })
            .flat_map(|e| {
                e.paths
                    .iter()
                    .flat_map(|p| p.points.iter().map(|q| (q[0], q[1])))
            });
        let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
        let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        for (x, y) in nodes.chain(corners).chain(curves) {
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
        }
        (min_x <= max_x).then_some((min_x, min_y, max_x, max_y))
    }

    /// The view that shows all nodes and geometry of `map` in a
    /// `width` × `height` image. Use the same view for every frame of a video.
    pub fn fit(
        layout: &LayoutLine,
        map: usize,
        width: u32,
        height: u32,
    ) -> Result<View, ViewError> {
        if width == 0 || height == 0 {
            return Err(ViewError::NoPixels);
        }
        if map >= layout.maps.len() {
            return Err(ViewError::NoSuchMap(map));
        }
        let Some((min_x, min_y, max_x, max_y)) = View::content_bounds(layout, map) else {
            return Err(ViewError::EmptyMap(map));
        };
        let extent_x = (max_x - min_x).max(MIN_EXTENT);
        let extent_y = (max_y - min_y).max(MIN_EXTENT);
        let usable = 1.0 - 2.0 * MARGIN;
        let scale = (width as f64 * usable / extent_x).min(height as f64 * usable / extent_y);
        Ok(View {
            width,
            height,
            scale,
            centre: ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0),
        })
    }

    /// Where a world point (metres) is in the image (pixels).
    pub fn to_px(&self, x: f64, y: f64) -> (f32, f32) {
        (
            (self.width as f64 / 2.0 + (x - self.centre.0) * self.scale) as f32,
            (self.height as f64 / 2.0 - (y - self.centre.1) * self.scale) as f32,
        )
    }

    /// The world point (metres) that is at an image pixel: the inverse of
    /// [`View::to_px`].
    pub fn to_world(&self, px: f32, py: f32) -> (f64, f64) {
        (
            self.centre.0 + (px as f64 - self.width as f64 / 2.0) / self.scale,
            self.centre.1 - (py as f64 - self.height as f64 / 2.0) / self.scale,
        )
    }

    /// This view zoomed in by `factor` (zoomed out if below 1) round the image pixel
    /// `anchor`: the world point under it stays under it.
    pub fn zoomed(&self, factor: f64, anchor: (f32, f32)) -> View {
        let (wx, wy) = self.to_world(anchor.0, anchor.1);
        let scale = self.scale * factor;
        View {
            scale,
            centre: (
                wx - (anchor.0 as f64 - self.width as f64 / 2.0) / scale,
                wy + (anchor.1 as f64 - self.height as f64 / 2.0) / scale,
            ),
            ..*self
        }
    }

    /// This view with the content moved by `(dx, dy)` image pixels (a drag).
    pub fn panned(&self, dx: f32, dy: f32) -> View {
        View {
            centre: (
                self.centre.0 - dx as f64 / self.scale,
                self.centre.1 + dy as f64 / self.scale,
            ),
            ..*self
        }
    }

    /// A length in metres as pixels.
    pub fn px(&self, metres: f64) -> f32 {
        (metres * self.scale) as f32
    }
}

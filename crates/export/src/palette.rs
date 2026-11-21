//! The colours of botsim pictures. The viewer uses the same ones, so a frame
//! from the exporter looks like the viewer's window.

use botsim_record::{GeometryKind, StatusRecord};

/// Red, green, blue.
pub type Rgb = [u8; 3];

pub const BACKGROUND: Rgb = [0xF7, 0xF7, 0xF4];

/// Track graph: edges and the rim of the nodes.
pub const GRAPH: Rgb = [0x9C, 0xA3, 0xAF];
/// The centre of a node.
pub const NODE_CENTRE: Rgb = [0xFF, 0xFF, 0xFF];

/// The fill colour of a robot in a status.
pub fn status(s: StatusRecord) -> Rgb {
    match s {
        StatusRecord::Idle => [0x9A, 0xA0, 0xA6],
        StatusRecord::Driving => [0x2E, 0x9E, 0x57],
        StatusRecord::Turning => [0x1B, 0x9A, 0xAA],
        StatusRecord::Acting => [0x3B, 0x6F, 0xD4],
        StatusRecord::WaitingForRelease => [0xF2, 0xB1, 0x34],
        StatusRecord::WaitingForZone => [0xE8, 0x80, 0x3A],
        StatusRecord::WaitingForLift => [0xA0, 0x5B, 0xD1],
        StatusRecord::RidingLift => [0x6B, 0x3F, 0xA0],
        StatusRecord::Held => [0x6D, 0x5A, 0x4A],
        StatusRecord::Obstructed => [0xD6, 0x45, 0x45],
    }
}

/// How a geometry kind is drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KindStyle {
    /// Inside of a polygon; `None` for a kind that is only a line (walls).
    pub fill: Option<Rgb>,
    /// The outline of a polygon, or the colour of a polyline.
    pub line: Rgb,
}

pub fn kind(k: GeometryKind) -> KindStyle {
    match k {
        GeometryKind::Wall => KindStyle {
            fill: None,
            line: [0x4A, 0x4F, 0x57],
        },
        GeometryKind::Rack => KindStyle {
            fill: Some([0xE3, 0xD5, 0xB8]),
            line: [0xA3, 0x8F, 0x68],
        },
        GeometryKind::Shelf => KindStyle {
            fill: Some([0xD3, 0xE0, 0xCE]),
            line: [0x8D, 0xA8, 0x86],
        },
        GeometryKind::Station => KindStyle {
            fill: Some([0xD6, 0xE4, 0xF5]),
            line: [0x7F, 0xA6, 0xD6],
        },
        GeometryKind::Other => KindStyle {
            fill: Some([0xE2, 0xE2, 0xE2]),
            line: [0xA0, 0xA0, 0xA0],
        },
    }
}

/// A darker shade, for the outline of a robot.
pub fn darker(c: Rgb) -> Rgb {
    c.map(|v| (v as f32 * 0.65) as u8)
}

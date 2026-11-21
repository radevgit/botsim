//! Static geometry (V5): walls, racks, shelves and station outlines that the
//! viewer draws under the track graph. LIF has no walls, so they live in a file
//! of their own: [`GeometryFile`]. A recording carries them in the layout line as
//! [`StaticShape`]s, with the map given as an index.
//!
//! The file is JSON, metres, in the LIF frame of each shape's `mapId`:
//!
//! ```json
//! {"version": 1, "shapes": [
//!   {"id": "wall-north", "mapId": "M", "kind": "wall", "shape": "polyline",
//!    "vertices": [{"x": 0.0, "y": 8.0}, {"x": 30.0, "y": 8.0}]}
//! ]}
//! ```
//!
//! Shapes are drawn in file order, later ones on top.

use std::fmt;
use std::path::Path;

use serde::{Deserialize, Serialize};

/// The version of the geometry file format.
pub const GEOMETRY_VERSION: u32 = 1;

/// What a shape is; sets its colour. Other kinds can be added later.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum GeometryKind {
    Wall,
    Rack,
    Shelf,
    Station,
    Other,
}

/// Closed and filled, or an open line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Outline {
    Polygon,
    Polyline,
}

impl Outline {
    /// The fewest vertices the outline needs.
    pub fn min_vertices(self) -> usize {
        match self {
            Outline::Polygon => 3,
            Outline::Polyline => 2,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vertex {
    pub x: f64,
    pub y: f64,
}

/// One shape in the geometry file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GeometryShape {
    /// For messages; optional.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    /// The VDA 5050 `mapId` whose frame the vertices are in.
    #[serde(rename = "mapId")]
    pub map_id: String,
    pub kind: GeometryKind,
    pub shape: Outline,
    pub vertices: Vec<Vertex>,
}

/// The geometry file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GeometryFile {
    pub version: u32,
    pub shapes: Vec<GeometryShape>,
}

/// A shape as the recording's layout line carries it: the map is an index into
/// the layout's `maps`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StaticShape {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    pub map: usize,
    pub kind: GeometryKind,
    pub shape: Outline,
    pub vertices: Vec<Vertex>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum GeometryError {
    /// The file cannot be read.
    Read {
        path: String,
        message: String,
    },
    /// The text is not a geometry file (bad JSON, unknown field or kind).
    Parse(String),
    UnsupportedVersion {
        found: u32,
        supported: u32,
    },
    /// `shape` is the shape's id, or `#n` (counted from 1) if it has none.
    TooFewVertices {
        shape: String,
        need: usize,
        found: usize,
    },
    NotFinite {
        shape: String,
    },
    /// The shape is on a map that the layout does not have.
    UnknownMap {
        shape: String,
        map_id: String,
    },
}

impl fmt::Display for GeometryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GeometryError::Read { path, message } => write!(f, "cannot read {path}: {message}"),
            GeometryError::Parse(m) => write!(f, "bad geometry file: {m}"),
            GeometryError::UnsupportedVersion { found, supported } => write!(
                f,
                "geometry file version {found} is not supported (this reader: {supported})"
            ),
            GeometryError::TooFewVertices { shape, need, found } => {
                write!(f, "geometry {shape}: needs {need} vertices, has {found}")
            }
            GeometryError::NotFinite { shape } => {
                write!(f, "geometry {shape}: a vertex is not a finite number")
            }
            GeometryError::UnknownMap { shape, map_id } => {
                write!(f, "geometry {shape}: the layout has no map '{map_id}'")
            }
        }
    }
}

impl std::error::Error for GeometryError {}

impl GeometryFile {
    /// Parse and check a geometry file.
    pub fn from_json_str(text: &str) -> Result<GeometryFile, GeometryError> {
        let file: GeometryFile =
            serde_json::from_str(text).map_err(|e| GeometryError::Parse(e.to_string()))?;
        file.validate()?;
        Ok(file)
    }

    pub fn load(path: &Path) -> Result<GeometryFile, GeometryError> {
        let text = std::fs::read_to_string(path).map_err(|e| GeometryError::Read {
            path: path.display().to_string(),
            message: e.to_string(),
        })?;
        GeometryFile::from_json_str(&text)
    }

    /// Version, vertex counts and finite coordinates. (Whether the maps exist is
    /// checked by [`GeometryFile::resolve`], which needs the layout.)
    pub fn validate(&self) -> Result<(), GeometryError> {
        if self.version != GEOMETRY_VERSION {
            return Err(GeometryError::UnsupportedVersion {
                found: self.version,
                supported: GEOMETRY_VERSION,
            });
        }
        for (i, s) in self.shapes.iter().enumerate() {
            let name = shape_name(s, i);
            let need = s.shape.min_vertices();
            if s.vertices.len() < need {
                return Err(GeometryError::TooFewVertices {
                    shape: name,
                    need,
                    found: s.vertices.len(),
                });
            }
            if s.vertices
                .iter()
                .any(|v| !(v.x.is_finite() && v.y.is_finite()))
            {
                return Err(GeometryError::NotFinite { shape: name });
            }
        }
        Ok(())
    }

    /// The shapes for a recording: each `mapId` becomes its index in `maps`
    /// (the layout's map ids). A shape on an unknown map is an error.
    pub fn resolve(&self, maps: &[String]) -> Result<Vec<StaticShape>, GeometryError> {
        self.shapes
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let map = maps.iter().position(|m| *m == s.map_id).ok_or_else(|| {
                    GeometryError::UnknownMap {
                        shape: shape_name(s, i),
                        map_id: s.map_id.clone(),
                    }
                })?;
                Ok(StaticShape {
                    id: s.id.clone(),
                    map,
                    kind: s.kind,
                    shape: s.shape,
                    vertices: s.vertices.clone(),
                })
            })
            .collect()
    }
}

fn shape_name(s: &GeometryShape, index: usize) -> String {
    match &s.id {
        Some(id) => format!("'{id}'"),
        None => format!("#{}", index + 1),
    }
}

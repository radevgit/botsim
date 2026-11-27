//! Findings: what the checker reports.

use std::fmt;

use serde::Serialize;

/// Severity of a finding.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Level {
    Info,
    Warning,
    Error,
}

impl fmt::Display for Level {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Level::Info => "info",
            Level::Warning => "warning",
            Level::Error => "error",
        })
    }
}

/// A layout object a finding is about, with its position (kind, id, map, points),
/// so a viewer can draw it without looking the id up. Defined in `botsim-space`.
pub use botsim_space::{Place as FindingPlace, PlaceKind};

/// How many ids the text output shows per finding.
const TEXT_IDS: usize = 10;

/// One finding of one check.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Finding {
    pub level: Level,
    /// Check name, e.g. `structure`, `reachability`.
    pub check: String,
    /// Layout id the finding belongs to, if any.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub layout_id: Option<String>,
    /// Map id, if any.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub map_id: Option<String>,
    /// Robot type as `manufacturer/seriesName`, if the finding is specific to one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robot_type: Option<String>,
    /// Ids of the layout objects concerned (nodes, edges, stations).
    pub ids: Vec<String>,
    /// The same objects with kind, map and position. All of them, in JSON.
    pub places: Vec<FindingPlace>,
    pub text: String,
}

impl Finding {
    pub fn new(level: Level, check: &str, text: impl Into<String>) -> Self {
        Finding {
            level,
            check: check.to_string(),
            layout_id: None,
            map_id: None,
            robot_type: None,
            ids: Vec::new(),
            places: Vec::new(),
            text: text.into(),
        }
    }

    pub fn layout(mut self, id: &str) -> Self {
        self.layout_id = Some(id.to_string());
        self
    }

    pub fn map(mut self, id: &str) -> Self {
        self.map_id = Some(id.to_string());
        self
    }

    pub fn robot(mut self, t: &str) -> Self {
        self.robot_type = Some(t.to_string());
        self
    }

    pub fn with_ids<I, S>(mut self, ids: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        self.ids = ids.into_iter().map(Into::into).collect();
        self
    }
}

impl Finding {
    pub fn with_places(mut self, places: Vec<FindingPlace>) -> Self {
        self.places = places;
        self
    }
}

impl fmt::Display for Finding {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} [{}]", self.level, self.check)?;
        if let Some(l) = &self.layout_id {
            write!(f, " layout={l}")?;
        }
        if let Some(m) = &self.map_id {
            write!(f, " map={m}")?;
        }
        if let Some(r) = &self.robot_type {
            write!(f, " robot={r}")?;
        }
        if !self.ids.is_empty() {
            // The text form is for reading: first ids only. JSON has all of them.
            let shown = self.ids.iter().take(TEXT_IDS).cloned().collect::<Vec<_>>();
            write!(f, " ids={}", shown.join(","))?;
            if self.ids.len() > TEXT_IDS {
                write!(f, ",... (+{} more)", self.ids.len() - TEXT_IDS)?;
            }
        }
        write!(f, ": {}", self.text)
    }
}

/// Text output, one finding per line.
pub fn to_text(findings: &[Finding]) -> String {
    findings.iter().map(|f| format!("{f}\n")).collect()
}

/// JSON output (array of findings).
pub fn to_json(findings: &[Finding]) -> String {
    // Serialising plain structs of strings cannot fail; fall back to an empty array anyway.
    serde_json::to_string_pretty(findings).unwrap_or_else(|_| "[]".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn text_and_json() {
        let f = Finding::new(Level::Error, "structure", "edge refers to missing node")
            .layout("L1")
            .robot("ACME/X1")
            .with_ids(["E1", "N9"]);
        assert_eq!(
            f.to_string(),
            "error [structure] layout=L1 robot=ACME/X1 ids=E1,N9: edge refers to missing node"
        );
        let j = to_json(&[f]);
        assert!(j.contains("\"level\": \"error\""));
        assert!(!j.contains("map_id"));
    }
}

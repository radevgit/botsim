//! Reading a recording.

use std::fmt;
use std::io::{self, BufRead};

use serde::Deserialize;

use crate::{FORMAT_VERSION, LayoutLine, SnapshotLine};

#[derive(Debug)]
pub enum ReadError {
    Io(io::Error),
    /// The file has no line at all.
    Empty,
    /// A line is not valid JSON of the expected kind.
    Line {
        line: usize,
        message: String,
    },
    /// Line 1 is not a layout line.
    NotALayout {
        line: usize,
    },
    /// The recording was written with another format version.
    UnsupportedVersion {
        found: u32,
        supported: u32,
    },
}

impl fmt::Display for ReadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ReadError::Io(e) => write!(f, "read error: {e}"),
            ReadError::Empty => write!(f, "the recording is empty"),
            ReadError::Line { line, message } => write!(f, "line {line}: {message}"),
            ReadError::NotALayout { line } => {
                write!(f, "line {line}: the first line must be the layout")
            }
            ReadError::UnsupportedVersion { found, supported } => write!(
                f,
                "recording format version {found} is not supported (this reader: {supported})"
            ),
        }
    }
}

impl std::error::Error for ReadError {}

/// What is read first from line 1, before the rest of the line is understood.
#[derive(Deserialize)]
struct Header {
    #[serde(rename = "type")]
    kind: String,
    version: Option<u32>,
}

/// A recording read line by line: the layout is read when the reader is opened,
/// the snapshots come as an iterator. Blank lines are skipped.
pub struct Reader<R: BufRead> {
    lines: io::Lines<R>,
    line_no: usize,
    layout: LayoutLine,
}

impl<R: BufRead> Reader<R> {
    /// Read and check line 1.
    pub fn new(input: R) -> Result<Reader<R>, ReadError> {
        let mut lines = input.lines();
        let mut line = 0;
        let first = next_line(&mut lines, &mut line)?.ok_or(ReadError::Empty)?;
        let header: Header = parse(&first, line)?;
        if header.kind != "layout" {
            return Err(ReadError::NotALayout { line });
        }
        let found = header.version.ok_or_else(|| ReadError::Line {
            line,
            message: "missing field `version`".into(),
        })?;
        if found != FORMAT_VERSION {
            return Err(ReadError::UnsupportedVersion {
                found,
                supported: FORMAT_VERSION,
            });
        }
        Ok(Reader {
            lines,
            line_no: line,
            layout: parse(&first, line)?,
        })
    }

    pub fn layout(&self) -> &LayoutLine {
        &self.layout
    }
}

/// The next line that is not blank, counting lines in `line_no`.
fn next_line<R: BufRead>(
    lines: &mut io::Lines<R>,
    line_no: &mut usize,
) -> Result<Option<String>, ReadError> {
    for line in lines.by_ref() {
        *line_no += 1;
        let line = line.map_err(ReadError::Io)?;
        if !line.trim().is_empty() {
            return Ok(Some(line));
        }
    }
    Ok(None)
}

impl<R: BufRead> Iterator for Reader<R> {
    type Item = Result<SnapshotLine, ReadError>;

    fn next(&mut self) -> Option<Self::Item> {
        match next_line(&mut self.lines, &mut self.line_no) {
            Ok(Some(text)) => Some(parse(&text, self.line_no)),
            Ok(None) => None,
            Err(e) => Some(Err(e)),
        }
    }
}

fn parse<'a, T: Deserialize<'a>>(text: &'a str, line: usize) -> Result<T, ReadError> {
    serde_json::from_str(text).map_err(|e| ReadError::Line {
        line,
        message: e.to_string(),
    })
}

/// A whole recording in memory.
#[derive(Debug, Clone, PartialEq)]
pub struct Recording {
    pub layout: LayoutLine,
    pub snapshots: Vec<SnapshotLine>,
}

/// Read a whole recording.
pub fn read_recording<R: BufRead>(input: R) -> Result<Recording, ReadError> {
    let reader = Reader::new(input)?;
    let layout = reader.layout.clone();
    let snapshots = reader.collect::<Result<Vec<_>, _>>()?;
    Ok(Recording { layout, snapshots })
}

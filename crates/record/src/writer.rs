//! The observer adapter that writes a recording.

use std::cell::RefCell;
use std::fmt;
use std::io::{self, BufWriter, Write};
use std::rc::Rc;

use botsim_core::graph::Graph;
use botsim_core::observer::{Observer, Snapshot};
use serde::Serialize;

use crate::{GeometryFile, LayoutLine, LayoutRobot, SnapshotLine};

struct Shared<W: Write> {
    out: BufWriter<W>,
    /// The first write error. The [`Observer`] methods cannot return errors, so
    /// the recorder keeps it, writes nothing more, and [`RecordHandle::finish`]
    /// reports it.
    error: Option<io::Error>,
}

impl<W: Write> Shared<W> {
    fn write_line<T: Serialize>(&mut self, line: &T) {
        if self.error.is_some() {
            return;
        }
        let result = serde_json::to_writer(&mut self.out, line)
            .map_err(io::Error::from)
            .and_then(|()| self.out.write_all(b"\n"));
        if let Err(e) = result {
            self.error = Some(e);
        }
    }
}

/// An [`Observer`] that writes the layout (line 1) and one snapshot line per
/// step to `W`. Give it to `World::set_observer` / `set_observer_at`; keep the
/// [`RecordHandle`] to flush and to see write errors after the run.
pub struct Recorder<W: Write> {
    shared: Rc<RefCell<Shared<W>>>,
    geometry: Option<GeometryFile>,
    robots: Vec<LayoutRobot>,
}

/// Stays with the caller while the [`Recorder`] is inside the world.
pub struct RecordHandle<W: Write> {
    shared: Rc<RefCell<Shared<W>>>,
}

impl<W: Write> Recorder<W> {
    pub fn new(out: W) -> (Recorder<W>, RecordHandle<W>) {
        Recorder::build(out, None)
    }

    /// Like [`Recorder::new`], and the layout line carries the static geometry.
    /// If a shape is on a map that the layout does not have, nothing is written
    /// and [`RecordHandle::finish`] reports it; check beforehand with
    /// [`LayoutLine::with_geometry`] to fail early.
    pub fn with_geometry(out: W, geometry: GeometryFile) -> (Recorder<W>, RecordHandle<W>) {
        Recorder::build(out, Some(geometry))
    }

    fn build(out: W, geometry: Option<GeometryFile>) -> (Recorder<W>, RecordHandle<W>) {
        let shared = Rc::new(RefCell::new(Shared {
            out: BufWriter::new(out),
            error: None,
        }));
        (
            Recorder {
                shared: shared.clone(),
                geometry,
                robots: Vec::new(),
            },
            RecordHandle { shared },
        )
    }
}

impl<W: Write> Recorder<W> {
    /// The layout line also lists the type of each robot, so that a viewer can
    /// tell which curve a robot follows on an edge.
    pub fn with_robots(mut self, robots: Vec<LayoutRobot>) -> Recorder<W> {
        self.robots = robots;
        self
    }
}

impl<W: Write> RecordHandle<W> {
    /// Flush the output. Returns the first error that happened while writing,
    /// if any (after an error nothing more is written).
    pub fn finish(&self) -> io::Result<()> {
        let mut s = self.shared.borrow_mut();
        if let Some(e) = s.error.take() {
            return Err(e);
        }
        s.out.flush()
    }
}

impl<W: Write> fmt::Debug for Recorder<W> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("Recorder")
    }
}

impl<W: Write> Observer for Recorder<W> {
    fn on_layout(&mut self, layout: &Graph) {
        let line = LayoutLine::from_graph(layout).with_robots(self.robots.clone());
        let mut shared = self.shared.borrow_mut();
        match &self.geometry {
            None => shared.write_line(&line),
            Some(geometry) => match line.with_geometry(geometry) {
                Ok(line) => shared.write_line(&line),
                Err(e) => {
                    shared
                        .error
                        .get_or_insert(io::Error::new(io::ErrorKind::InvalidData, e.to_string()));
                }
            },
        }
    }

    fn on_step(&mut self, snapshot: &Snapshot) {
        self.shared
            .borrow_mut()
            .write_line(&SnapshotLine::from(snapshot));
    }
}

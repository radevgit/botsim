//! Records what the botsim fleet server decides on, and plays it back.
//!
//! The server ([`botsim_server`]) reads no clock and no file: it is made once from
//! a setup, then run in cycles with the time and what has arrived. A recording
//! holds exactly that ([`Setup`], then the time and the inbox of every cycle), so
//! that the same decisions can be made again from the file alone: no simulator,
//! scenario, layout or factsheet file is needed. For every cycle it also holds two
//! checksums, of the messages that came out and of the state the controller was
//! left in ([`botsim_server::checksum`]), by which a replay finds the first cycle
//! where it goes another way.
//!
//! - **Record:** [`Recorder`] runs each cycle of a controller made by
//!   [`controller`] and writes it down.
//! - **Replay:** [`replay`] makes the controller of a file and runs its cycles.
//! - **Read or rewrite a file:** [`Reader`] and [`Writer`].
//!
//! The same decisions are promised only for **the same build on the same kind of
//! machine**; another build is replayed all the same and may differ. A file that
//! is damaged, cut off or something else gives an [`Error`].
//!
//! The format is described in `file.rs` ([`file`](mod@file)); the checksum of a
//! state is not stable across builds, that of the messages only as long as their
//! types stay.

use std::fmt;
use std::io;

/// A pair of functions that write an enum without fields as one byte and read it
/// back. The `match` of the writer names each variant, so a new one stops the
/// build.
macro_rules! tags {
    ($write:ident, $read:ident, $type:ident, $($variant:ident = $tag:literal),+) => {
        fn $write(e: &mut $crate::codec::Encoder, value: $type) {
            e.byte(match value {
                $($type::$variant => $tag,)+
            });
        }

        fn $read(d: &mut $crate::codec::Decoder) -> Result<$type, $crate::codec::Bad> {
            match d.byte()? {
                $($tag => Ok($type::$variant),)+
                _ => Err(concat!("an unknown ", stringify!($type))),
            }
        }
    };
}

pub mod codec;
pub mod file;
mod messages;
mod record;
mod replay;
pub mod setup;

pub use file::{Cycle, Reader, VERSION, Writer};
pub use record::Recorder;
pub use replay::{Difference, Outcome, replay};
pub use setup::{Call, Dispatch, Rules, Setup, apply, controller};

/// Why a recording cannot be written or replayed.
#[derive(Debug)]
pub enum Error {
    /// Reading or writing failed.
    Io(io::Error),
    /// The file is no recording of the server's inputs.
    Foreign,
    /// The file is written in a format of this number, which this build does not
    /// read.
    Format(u32),
    /// The file ends too early: the run that wrote it did not finish, or part of
    /// the file is lost.
    CutOff,
    /// The file is damaged; the text says what was found.
    Damaged(&'static str),
    /// No controller can be made from the setup; the text says why.
    Setup(String),
    /// A part is too large for the format.
    TooLarge,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => e.fmt(f),
            Error::Foreign => write!(f, "not a recording of botsim server inputs"),
            Error::Format(n) => write!(
                f,
                "a recording in format {n}; this build reads format {}",
                file::FORMAT
            ),
            Error::CutOff => write!(f, "the recording is cut off"),
            Error::Damaged(what) => write!(f, "the recording is damaged: {what}"),
            Error::Setup(why) => write!(f, "the setup cannot be used: {why}"),
            Error::TooLarge => write!(f, "too much for one record of a recording"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Error {
        Error::Io(e)
    }
}

/// What the content of a record gets wrong.
impl From<codec::Bad> for Error {
    fn from(what: codec::Bad) -> Error {
        Error::Damaged(what)
    }
}

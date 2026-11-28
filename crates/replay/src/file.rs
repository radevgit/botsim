//! The recording file: writing and reading it.
//!
//! **Layout.** A header, then records. Numbers of fixed width are least
//! significant byte first.
//!
//! | Part | Bytes |
//! |---|---|
//! | header | `BOTSIMIN`; the format number (4); the length of the version (1) and the botsim version that wrote the file; the check value (8) of all this |
//! | each record | kind (1); length of the content (4); content; check value (8) of the three |
//!
//! The records are one **setup** (kind 1), then one **cycle** (kind 2) for every
//! cycle of the controller, then one **end** (kind 3) with the number of cycles,
//! after which the file ends. A file without its end was cut off: the run did not
//! finish. A check value is a [`Checksum`] of the bytes, so a changed byte is
//! always found.

use std::io::{self, Read, Write};

use botsim_model::checksum::Checksum;
use botsim_model::time::Millis;
use botsim_server::cycle::Inbox;

use crate::Error;
use crate::codec::{Decoder, Encoder};
use crate::messages::{self, Previous};
use crate::setup::{self, Setup};

/// The bytes a recording starts with.
pub const MAGIC: [u8; 8] = *b"BOTSIMIN";

/// The number of the format this build writes and reads.
pub const FORMAT: u32 = 1;

/// The botsim version of this build.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

const SETUP: u8 = 1;
const CYCLE: u8 = 2;
const END: u8 = 3;

/// How many bytes of a record are read at a time.
const CHUNK: usize = 64 * 1024;

/// One cycle of a recording.
#[derive(Debug, Clone, PartialEq)]
pub struct Cycle {
    /// The time the controller was given.
    pub now: Millis,
    /// What had arrived.
    pub inbox: Inbox,
    /// The checksum of what came out ([`botsim_server::checksum::of_outbox`]).
    pub messages: u64,
    /// The checksum of the state afterwards
    /// ([`Controller::state_checksum`](botsim_server::controller::Controller::state_checksum)).
    pub state: u64,
}

/// A cycle being written: its inputs, the checksums still to come.
#[derive(Debug)]
pub(crate) struct Begun {
    content: Encoder,
}

/// Writes a recording. After an error it is of no further use.
#[derive(Debug)]
pub struct Writer<W: Write> {
    out: W,
    previous: Previous,
    /// The time of the cycle written last (before the first: the start).
    time: Millis,
    cycles: u64,
}

impl<W: Write> Writer<W> {
    /// Start a recording of a controller made from `setup`: writes the header and
    /// the setup. Buffer `out` if it writes slowly in small pieces.
    ///
    /// An error if the graph of the setup is not that of its layout text, or if
    /// writing fails.
    pub fn new(out: W, setup: &Setup) -> Result<Writer<W>, Error> {
        Writer::with_version(out, setup, VERSION)
    }

    /// [`Writer::new`], naming `version` as the botsim version that wrote the file
    /// (for tools that rewrite recordings, and tests).
    pub fn with_version(mut out: W, setup: &Setup, version: &str) -> Result<Writer<W>, Error> {
        let length = u8::try_from(version.len()).map_err(|_| Error::TooLarge)?;
        let mut header = MAGIC.to_vec();
        header.extend_from_slice(&FORMAT.to_le_bytes());
        header.push(length);
        header.extend_from_slice(version.as_bytes());
        let check = Checksum::of_bytes(&header);
        header.extend_from_slice(&check.to_le_bytes());
        out.write_all(&header)?;
        let mut content = Encoder::default();
        setup::write(&mut content, setup)?;
        write_record(&mut out, SETUP, &content.bytes)?;
        Ok(Writer {
            out,
            previous: Previous::default(),
            time: setup.config.start,
            cycles: 0,
        })
    }

    /// Write one cycle.
    pub fn cycle(&mut self, cycle: &Cycle) -> Result<(), Error> {
        let Cycle {
            now,
            inbox,
            messages,
            state,
        } = cycle;
        let begun = self.begin(*now, inbox);
        self.complete(begun, *messages, *state)
    }

    /// The first part of a cycle, from what the controller is given (which it then
    /// takes); [`Writer::complete`] must follow.
    pub(crate) fn begin(&mut self, now: Millis, inbox: &Inbox) -> Begun {
        let mut content = Encoder::default();
        let Millis(time) = now;
        content.signed(time.wrapping_sub(self.time.0));
        self.time = now;
        let Inbox {
            robots,
            lifts,
            jobs,
            watched,
        } = inbox;
        content.bool(*watched);
        content.list(robots, |e, report| {
            messages::write_report(e, report, now, &mut self.previous);
        });
        content.list(lifts, |e, lift| messages::write_lift(e, lift, now));
        content.list(jobs, |e, job| messages::write_job(e, job, now));
        Begun { content }
    }

    pub(crate) fn complete(
        &mut self,
        begun: Begun,
        messages: u64,
        state: u64,
    ) -> Result<(), Error> {
        let Begun { mut content } = begun;
        content.fixed(messages);
        content.fixed(state);
        write_record(&mut self.out, CYCLE, &content.bytes)?;
        self.cycles += 1;
        Ok(())
    }

    /// Write the end of the recording and flush.
    pub fn finish(mut self) -> Result<(), Error> {
        let mut content = Encoder::default();
        content.unsigned(self.cycles);
        write_record(&mut self.out, END, &content.bytes)?;
        self.out.flush()?;
        Ok(())
    }
}

/// The check value of a record.
fn check_of(head: &[u8], content: &[u8]) -> u64 {
    use std::hash::Hasher;
    let mut sum = Checksum::new();
    sum.write(head);
    sum.write(content);
    sum.finish()
}

fn write_record(out: &mut impl Write, kind: u8, content: &[u8]) -> Result<(), Error> {
    let length = u32::try_from(content.len()).map_err(|_| Error::TooLarge)?;
    let mut head = [kind, 0, 0, 0, 0];
    head[1..].copy_from_slice(&length.to_le_bytes());
    out.write_all(&head)?;
    out.write_all(content)?;
    out.write_all(&check_of(&head, content).to_le_bytes())?;
    Ok(())
}

/// Fill `buffer`; the file ending first means it was cut off.
fn fill(input: &mut impl Read, buffer: &mut [u8]) -> Result<(), Error> {
    input.read_exact(buffer).map_err(|e| {
        if e.kind() == io::ErrorKind::UnexpectedEof {
            Error::CutOff
        } else {
            Error::Io(e)
        }
    })
}

/// Reads a recording, cycle by cycle. After an error it is of no further use.
#[derive(Debug)]
pub struct Reader<R: Read> {
    input: R,
    chunk: Vec<u8>,
    version: String,
    setup: Setup,
    previous: Previous,
    time: Millis,
    cycles: u64,
    ended: bool,
}

impl<R: Read> Reader<R> {
    /// Read the header and the setup. Buffer `input` if it reads slowly in small
    /// pieces.
    pub fn new(mut input: R) -> Result<Reader<R>, Error> {
        // a file too short to hold the first bytes is no recording either
        let mut magic = [0; 8];
        fill(&mut input, &mut magic).map_err(|e| match e {
            Error::CutOff => Error::Foreign,
            other => other,
        })?;
        if magic != MAGIC {
            return Err(Error::Foreign);
        }
        let mut format = [0; 4];
        fill(&mut input, &mut format)?;
        let format_number = u32::from_le_bytes(format);
        if format_number != FORMAT {
            return Err(Error::Format(format_number));
        }
        let mut length = [0; 1];
        fill(&mut input, &mut length)?;
        // (at most 255 bytes)
        let mut version = vec![0; usize::from(length[0])];
        fill(&mut input, &mut version)?;
        let mut check = [0; 8];
        fill(&mut input, &mut check)?;
        let header = [&magic[..], &format, &length, &version].concat();
        if Checksum::of_bytes(&header) != u64::from_le_bytes(check) {
            return Err(Error::Damaged("the header does not fit its check value"));
        }
        let version =
            String::from_utf8(version).map_err(|_| Error::Damaged("the version is not a text"))?;

        let mut chunk = vec![0; CHUNK];
        let (kind, content) = read_record(&mut input, &mut chunk)?;
        if kind != SETUP {
            return Err(Error::Damaged("the first record is not the setup"));
        }
        let mut d = Decoder::new(&content);
        let setup = setup::read(&mut d)?;
        d.end()?;
        Ok(Reader {
            input,
            chunk,
            version,
            time: setup.config.start,
            setup,
            previous: Previous::default(),
            cycles: 0,
            ended: false,
        })
    }

    /// The botsim version that wrote the file.
    pub fn version(&self) -> &str {
        &self.version
    }

    /// What the recorded controller was made from.
    pub fn setup(&self) -> &Setup {
        &self.setup
    }

    /// The next cycle; `None` after the last, once the end has been read and
    /// found right.
    pub fn next_cycle(&mut self) -> Result<Option<Cycle>, Error> {
        if self.ended {
            return Ok(None);
        }
        let (kind, content) = read_record(&mut self.input, &mut self.chunk)?;
        let mut d = Decoder::new(&content);
        match kind {
            CYCLE => {
                let now = Millis(self.time.0.wrapping_add(d.signed()?));
                self.time = now;
                let watched = d.bool()?;
                let previous = &mut self.previous;
                let robots = d.list(|d| messages::read_report(d, now, previous))?;
                let lifts = d.list(|d| messages::read_lift(d, now))?;
                let jobs = d.list(|d| messages::read_job(d, now))?;
                let messages = d.fixed()?;
                let state = d.fixed()?;
                d.end()?;
                self.cycles += 1;
                Ok(Some(Cycle {
                    now,
                    inbox: Inbox {
                        robots,
                        lifts,
                        jobs,
                        watched,
                    },
                    messages,
                    state,
                }))
            }
            END => {
                let cycles = d.unsigned()?;
                d.end()?;
                if cycles != self.cycles {
                    return Err(Error::Damaged("cycles are missing or there are too many"));
                }
                match fill(&mut self.input, &mut [0]) {
                    Err(Error::CutOff) => {}
                    Ok(()) => return Err(Error::Damaged("there are bytes after the end")),
                    Err(other) => return Err(other),
                }
                self.ended = true;
                Ok(None)
            }
            _ => Err(Error::Damaged("a record of an unknown or misplaced kind")),
        }
    }
}

/// The kind and the content of the next record, its check value found right. The
/// content grows as it is read, `chunk` bytes at a time: a length that the file
/// does not hold sets no memory aside.
fn read_record(input: &mut impl Read, chunk: &mut [u8]) -> Result<(u8, Vec<u8>), Error> {
    let mut head = [0; 5];
    fill(input, &mut head)?;
    let [kind, a, b, c, d] = head;
    let mut left = u32::from_le_bytes([a, b, c, d]) as usize;
    let mut content = Vec::new();
    while left > 0 {
        let part = &mut chunk[..left.min(CHUNK)];
        fill(input, part)?;
        content.extend_from_slice(part);
        left -= part.len();
    }
    let mut check = [0; 8];
    fill(input, &mut check)?;
    if check_of(&head, &content) != u64::from_le_bytes(check) {
        return Err(Error::Damaged("a record does not fit its check value"));
    }
    Ok((kind, content))
}

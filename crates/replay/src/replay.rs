//! Playing a recording back.

use std::io::Read;

use botsim_model::time::Millis;
use botsim_server::checksum;

use crate::Error;
use crate::file::Reader;
use crate::setup::controller;

/// The first cycle where a replay did not do what was recorded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Difference {
    /// The number of the cycle, the first being 1.
    pub cycle: u64,
    /// The time the controller was given in it.
    pub time: Millis,
    /// The messages, what is held back or the reservations differ.
    pub messages: bool,
    /// The controller was left in another state.
    pub state: bool,
}

/// What came of a replay.
#[derive(Debug)]
pub struct Outcome {
    /// The botsim version that wrote the file.
    pub recorded_with: String,
    /// How many cycles were run and compared, the one that differs among them.
    pub cycles: u64,
    /// Where the replay went another way; it stops there.
    pub difference: Option<Difference>,
    /// What stopped the reading of the file after those cycles, if it could not be
    /// read to its end.
    pub broken: Option<Error>,
}

impl Outcome {
    /// Did every cycle of the whole file give what was recorded?
    pub fn is_same(&self) -> bool {
        self.difference.is_none() && self.broken.is_none()
    }
}

/// Make the controller of the recording in `input` and run its cycles, comparing
/// the checksums of each, up to the first that differs.
///
/// An error if not even the setup can be used. A file that breaks later is replayed
/// as far as it can be read ([`Outcome::broken`]).
pub fn replay<R: Read>(input: R) -> Result<Outcome, Error> {
    let mut reader = Reader::new(input)?;
    let mut controller = controller(reader.setup())?;
    let mut outcome = Outcome {
        recorded_with: reader.version().to_string(),
        cycles: 0,
        difference: None,
        broken: None,
    };
    loop {
        let cycle = match reader.next_cycle() {
            Ok(Some(cycle)) => cycle,
            Ok(None) => break,
            Err(error) => {
                outcome.broken = Some(error);
                break;
            }
        };
        let outbox = controller.step(cycle.now, cycle.inbox);
        outcome.cycles += 1;
        let messages = checksum::of_outbox(&outbox) != cycle.messages;
        let state = controller.state_checksum() != cycle.state;
        if messages || state {
            outcome.difference = Some(Difference {
                cycle: outcome.cycles,
                time: cycle.now,
                messages,
                state,
            });
            break;
        }
    }
    Ok(outcome)
}

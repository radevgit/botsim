//! Recording a run: each cycle of the controller is written down as it is run.

use std::io::Write;

use botsim_model::time::Millis;
use botsim_server::checksum;
use botsim_server::controller::Controller;
use botsim_server::cycle::{Inbox, Outbox};
use botsim_server::policy::TrafficPolicy;

use crate::Error;
use crate::file::Writer;
use crate::setup::Setup;

/// Writes down the cycles of a controller. A cycle goes to the file after it has
/// run, so the file of a program that stops in one ends with the cycle before.
#[derive(Debug)]
pub struct Recorder<W: Write> {
    /// `None` once writing has failed.
    writer: Option<Writer<W>>,
    error: Option<Error>,
}

impl<W: Write> Recorder<W> {
    /// Start the recording of a controller made from `setup` (by
    /// [`controller`](crate::controller), or the same way), before its first cycle.
    pub fn new(out: W, setup: &Setup) -> Result<Recorder<W>, Error> {
        Ok(Recorder {
            writer: Some(Writer::new(out, setup)?),
            error: None,
        })
    }

    /// Run one cycle of `controller` ([`Controller::step`]) and write it down. A
    /// failure to write does not stop the controller: nothing more is written, and
    /// [`Recorder::finish`] says what failed.
    pub fn step<P: TrafficPolicy>(
        &mut self,
        controller: &mut Controller<P>,
        now: Millis,
        inbox: Inbox,
    ) -> Outbox {
        let begun = self.writer.as_mut().map(|w| w.begin(now, &inbox));
        let outbox = controller.step(now, inbox);
        if let (Some(writer), Some(begun)) = (self.writer.as_mut(), begun) {
            let messages = checksum::of_outbox(&outbox);
            let state = controller.state_checksum();
            if let Err(error) = writer.complete(begun, messages, state) {
                self.writer = None;
                self.error = Some(error);
            }
        }
        outbox
    }

    /// Write the end of the recording and flush; the first thing that failed, if
    /// anything did.
    pub fn finish(self) -> Result<(), Error> {
        match (self.error, self.writer) {
            (Some(error), _) => Err(error),
            (None, Some(writer)) => writer.finish(),
            // (without a writer there is an error)
            (None, None) => Ok(()),
        }
    }
}

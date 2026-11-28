//! What every message carries: the protocol version, the time and who the robot
//! is. Also the checks shared by the messages.

use std::f64::consts::PI;

use botsim_link::message::{Header, Millis};
use botsim_model::angle::normalize_angle;
use botsim_model::robot::RobotId;
use botsim_model::time::{iso8601, unix_ms_from_iso8601, unix_ms_from_utc};

use crate::report::Problem;
use crate::translator::Translator;

/// The VDA 5050 version written in the headers.
pub const VERSION: &str = botsim_model::protocol::PROTOCOL_VERSION;

/// The range of the angles in the schemas of order and state (rad). It is a hair
/// wider than −π..π.
#[allow(clippy::approx_constant)] // (the schemas' number, not π)
const ANGLE_LIMIT: f64 = 3.1416;
const _: () = assert!(ANGLE_LIMIT > PI);

/// The header fields of an outgoing message.
pub(crate) struct Outgoing {
    pub header_id: u32,
    pub timestamp: String,
    pub version: String,
    pub manufacturer: String,
    pub serial_number: String,
}

/// The header fields of an incoming message.
pub(crate) struct Incoming<'a> {
    pub header_id: u32,
    pub timestamp: &'a str,
    pub version: &'a str,
    pub manufacturer: &'a str,
    pub serial_number: &'a str,
}

impl Translator {
    pub(crate) fn outgoing(&self, header: &Header) -> Result<Outgoing, Problem> {
        let id = self.identity(header.robot)?;
        Ok(Outgoing {
            header_id: header.seq,
            timestamp: timestamp(header.time, "timestamp")?,
            version: VERSION.to_string(),
            manufacturer: id.manufacturer.clone(),
            serial_number: id.serial_number.clone(),
        })
    }

    pub(crate) fn incoming(&self, header: Incoming<'_>) -> Result<Header, Problem> {
        let id = RobotId::new(header.manufacturer, header.serial_number);
        let Some(robot) = self.robot_no(&id) else {
            return Err(Problem::UnknownRobot(id));
        };
        if header.version.split('.').next() != Some("3") {
            return Err(Problem::Version(header.version.to_string()));
        }
        let time = unix_ms_from_iso8601(header.timestamp)
            .ok_or(Problem::BadTime { field: "timestamp" })?;
        Ok(Header {
            robot,
            seq: header.header_id,
            time: Millis(time),
        })
    }
}

/// The time as an ISO 8601 text.
pub(crate) fn timestamp(time: Millis, field: &'static str) -> Result<String, Problem> {
    // the years that have four digits
    let first = unix_ms_from_utc(0, 1, 1, 0, 0, 0);
    let last = unix_ms_from_utc(9999, 12, 31, 23, 59, 59) + 999;
    if (first..=last).contains(&time.0) {
        Ok(iso8601(time.0))
    } else {
        Err(Problem::BadTime { field })
    }
}

/// An id that is not empty.
pub(crate) fn usable<'a>(id: &'a str, field: &'static str) -> Result<&'a str, Problem> {
    if id.is_empty() {
        Err(Problem::UnusableId { field })
    } else {
        Ok(id)
    }
}

pub(crate) fn finite(value: f64, field: &'static str) -> Result<f64, Problem> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(Problem::NotFinite { field })
    }
}

/// An angle as a message may carry it: `angle` itself, or, if it is outside the
/// range of the schemas, the same direction in −π..π together with the report of
/// the change.
pub(crate) fn angle_to_vda(
    angle: f64,
    field: &'static str,
) -> Result<(f64, Option<Problem>), Problem> {
    let angle = finite(angle, field)?;
    if angle.abs() <= ANGLE_LIMIT {
        Ok((angle, None))
    } else {
        let changed = Problem::AngleNormalized { field, from: angle };
        Ok((normalize_angle(angle), Some(changed)))
    }
}

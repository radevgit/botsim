//! `responses`. A bad response is left out, the others pass. VDA 5050 has no
//! error type for them.

use botsim_link::ids::RequestId;
use botsim_link::message::{Header, Millis, ToRobot, ToRobotBody};
use botsim_link::response::{Grant, Response};
use botsim_model::time::unix_ms_from_iso8601;
use vda5050::responses as vda;

use crate::header::{Incoming, timestamp, usable};
use crate::report::{Issue, Problem, Translated};
use crate::translator::Translator;

const REQUEST_ID: &str = "responses[].requestId";
const LEASE_EXPIRY: &str = "responses[].leaseExpiry";

impl Translator {
    pub(crate) fn responses_to_vda(
        &self,
        header: &Header,
        responses: &[Response],
    ) -> Result<Translated<vda::Responses>, Issue> {
        let head = self.outgoing(header).map_err(Issue::new)?;
        let mut message = vda::Responses {
            header_id: head.header_id,
            timestamp: head.timestamp,
            version: head.version,
            manufacturer: head.manufacturer,
            serial_number: head.serial_number,
            responses: Vec::with_capacity(responses.len()),
        };
        let mut issues = Vec::new();
        for (i, r) in responses.iter().enumerate() {
            // (a grant whose end cannot be written is not sent: without the end it
            // would hold for ever)
            match response_to_vda(r) {
                Ok(response) => message.responses.push(response),
                Err(problem) => issues.push(Issue::new(problem).at(i)),
            }
        }
        Ok(Translated { message, issues })
    }

    /// What the `responses` message says, as a robot receives it. A
    /// `leaseExpiry` that cannot be read counts as none and is in the issues.
    pub fn responses_from_vda(
        &self,
        message: &vda::Responses,
    ) -> Result<Translated<ToRobot>, Issue> {
        let header = self
            .incoming(Incoming {
                header_id: message.header_id,
                timestamp: &message.timestamp,
                version: &message.version,
                manufacturer: &message.manufacturer,
                serial_number: &message.serial_number,
            })
            .map_err(Issue::new)?;
        let mut responses = Vec::with_capacity(message.responses.len());
        let mut issues = Vec::new();
        for (i, r) in message.responses.iter().enumerate() {
            let request = match usable(&r.request_id, REQUEST_ID) {
                Ok(id) => RequestId::new(id),
                Err(problem) => {
                    issues.push(Issue::new(problem).at(i));
                    continue;
                }
            };
            let lease_expiry = r.lease_expiry.as_deref().and_then(|text| {
                let time = unix_ms_from_iso8601(text).map(Millis);
                if time.is_none() {
                    let problem = Problem::BadTime {
                        field: LEASE_EXPIRY,
                    };
                    issues.push(Issue::new(problem).at(i));
                }
                time
            });
            responses.push(Response {
                request,
                grant: grant_from_vda(r.grant_type),
                lease_expiry,
            });
        }
        Ok(Translated {
            message: ToRobot {
                header,
                body: ToRobotBody::Responses(responses),
            },
            issues,
        })
    }
}

fn response_to_vda(r: &Response) -> Result<vda::Response, Problem> {
    Ok(vda::Response {
        request_id: usable(r.request.as_str(), REQUEST_ID)?.to_string(),
        grant_type: grant_to_vda(r.grant),
        lease_expiry: r
            .lease_expiry
            .map(|t| timestamp(t, LEASE_EXPIRY))
            .transpose()?,
    })
}

fn grant_to_vda(g: Grant) -> vda::GrantType {
    match g {
        Grant::Granted => vda::GrantType::Granted,
        Grant::Queued => vda::GrantType::Queued,
        Grant::Revoked => vda::GrantType::Revoked,
        Grant::Rejected => vda::GrantType::Rejected,
    }
}

fn grant_from_vda(g: vda::GrantType) -> Grant {
    match g {
        vda::GrantType::Granted => Grant::Granted,
        vda::GrantType::Queued => Grant::Queued,
        vda::GrantType::Revoked => Grant::Revoked,
        vda::GrantType::Rejected => Grant::Rejected,
    }
}

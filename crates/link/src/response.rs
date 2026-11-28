//! Answers to the requests a robot states in its reports.

use crate::ids::RequestId;
use crate::message::Millis;

/// The answer to one request.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Response {
    pub request: RequestId,
    pub grant: Grant,
    /// Until when a grant holds; `None`: without end.
    pub lease_expiry: Option<Millis>,
}

/// VDA 5050 `grantType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Grant {
    Granted,
    Queued,
    Revoked,
    Rejected,
}

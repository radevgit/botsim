//! Ids that VDA 5050 writes as text and that are not names of the layout.
//!
//! A copy of an id shares its text. Any text is accepted, also an empty one.

use std::fmt;
use std::sync::Arc;

macro_rules! text_id {
    ($(#[$doc:meta])* $name:ident) => {
        $(#[$doc])*
        #[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub struct $name(Arc<str>);

        impl $name {
            pub fn new(text: impl Into<Arc<str>>) -> $name {
                $name(text.into())
            }

            /// The text, which is all there is to an id.
            pub fn as_str(&self) -> &str {
                let $name(text) = self;
                text
            }
        }

        impl From<&str> for $name {
            fn from(text: &str) -> $name {
                $name::new(text)
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                fmt::Display::fmt(self.as_str(), f)
            }
        }
    };
}

text_id!(
    /// `orderId`.
    OrderId
);
text_id!(
    /// `actionId`.
    ActionId
);
text_id!(
    /// `actionType`: `pick`, `drop` and any other.
    ActionType
);
text_id!(
    /// `requestId`, made by the robot.
    RequestId
);
text_id!(
    /// `zoneId`, unique in its zone set.
    ZoneId
);
text_id!(
    /// `zoneSetId`.
    ZoneSetId
);

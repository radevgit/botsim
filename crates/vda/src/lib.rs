//! The translation between the compact messages of `botsim-link` and the VDA 5050
//! 3.0.1 messages of the `vda5050` crate, in both directions. It is the one place
//! where botsim translates: inside the program the fleet server and the robots use
//! `link`, real robots get only standard VDA 5050.
//!
//! A [`Translator`](translator::Translator) holds the layout, for the names of
//! nodes, edges and maps, and the table that says which VDA 5050 identity a
//! [`RobotNo`](botsim_model::robot::RobotNo) stands for.
//!
//! | Side | To VDA 5050 | From VDA 5050 |
//! |---|---|---|
//! | Fleet server | [`to_robot_to_vda`](translator::Translator::to_robot_to_vda) | [`state_from_vda`](translator::Translator::state_from_vda) |
//! | Robot | [`state_to_vda`](translator::Translator::state_to_vda) | [`order_from_vda`](translator::Translator::order_from_vda), [`instant_actions_from_vda`](translator::Translator::instant_actions_from_vda), [`responses_from_vda`](translator::Translator::responses_from_vda), [`zone_set_from_vda`](translator::Translator::zone_set_from_vda) |
//!
//! A zone set that is not in a message, such as the one of a site's configuration,
//! is read with
//! [`zone_set_content_from_vda`](translator::Translator::zone_set_content_from_vda).
//!
//! Whatever cannot be translated is reported as an [`Issue`](report::Issue), never
//! a panic. A message is either refused whole (`Err`) or translated with the parts
//! that had to be left out or changed listed beside it
//! ([`Translated`](report::Translated)).
//!
//! # What is refused, what is left out
//!
//! - **Order, zone set**: refused whole.
//! - **Instant actions, responses**: a bad entry is left out; the others pass.
//! - **State**: refused whole for a bad header or a charge that is no number from
//!   0 to 100. Otherwise a bad part is left out: the last node (then nothing is
//!   ahead either), the route ahead from the first bad step on, the position, the
//!   velocity, the type of an action, and single entries of the action states, zone
//!   requests, zone sets and errors.
//!
//! An id is unusable if it is empty. An empty `orderId` in a state means that the
//! robot never had an order.
//!
//! A robot's `manufacturer` and `serialNumber` are taken as they are. Whether they
//! can be levels of an MQTT topic is a rule of MQTT, which is not handled here:
//! [`identity_fits_topic`](translator::identity_fits_topic) says it.
//!
//! A charge at most 1e-9 above 100 is rounding: it is written and read as 100,
//! without a report.
//!
//! Outgoing, an edge `orientation` or a position `theta` outside the range the
//! schemas allow (±3.1416 rad) is brought into −π..π, the same direction, and
//! reported. Incoming, any finite angle is taken.
//!
//! # What is not carried
//!
//! `link` carries what botsim writes or reads. These fields are dropped without a
//! report when a message is read, and are absent when one is written:
//!
//! - all messages: `version` (checked: major version 3; written as 3.0.1);
//! - order: `orderDescription`; node and edge descriptors; `nodePosition`; the
//!   actions' `actionDescriptor`, `actionParameters` and `retriable`; edge
//!   `actions` (written empty), `maximumSpeed`, `maximumMobileRobotHeight`,
//!   `minimumLoadHandlingDeviceHeight`, `direction`,
//!   `reachOrientationBeforeEntering`, `maximumRotationSpeed`, `trajectory`,
//!   `length`, `corridor`;
//! - instant actions: `actionDescriptor`, `retriable`, `blockingType` (only
//!   `NONE` exists), parameters other than `zoneSetId` and `orderId`;
//! - state: `maps`; node and edge descriptors, `nodePosition`, `trajectory`;
//!   `plannedPath`, `intermediatePath`; `localizationScore`, `deviationRange`;
//!   `loads`; `paused`; `newBaseRequest`; `edgeRequests`; `distanceSinceLastNode`;
//!   `actionDescriptor`, `actionResult`; zone request `trajectory`;
//!   `zoneActionStates`; battery voltage, current, health and `range`;
//!   translations of error texts; `information`.
//!
//! Doors and lifts are outside VDA 5050 and are not translated. JSON text and MQTT
//! topics are not handled here.

mod header;
mod instant;
mod order;
pub mod report;
mod response;
mod state;
pub mod translator;
mod zone_set;

pub use header::VERSION;

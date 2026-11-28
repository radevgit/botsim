# botsim-vda

The translation between the compact messages of `botsim-link` and VDA 5050 3.0.1
messages, in both directions. Inside the program the fleet server and the robots use
`link`; real robots get only standard VDA 5050, and this crate is the one place that
translates. It adds no custom fields.

Library only; of the botsim crates it depends on `botsim-link` and `botsim-model`
only, reads no files and uses no network. The fleet server and the simulator talk
to each other in `link` without it. The command line crate
[`botsim`](../runner/README.md), which connects the two, has a route through
`vda` that tests the translation without real robots.

The VDA 5050 side is the message types of the
[`vda5050`](https://crates.io/crates/vda5050) crate. JSON text and MQTT topics are
not handled here.

## Use

A `Translator` holds the layout graph, for the names of nodes, edges and maps, and
the table that says which `RobotId` a `RobotNo` stands for (`add_robot`,
`remove_robot`).

| Side | To VDA 5050 | From VDA 5050 |
|---|---|---|
| Fleet server | `to_robot_to_vda` | `state_from_vda` |
| Robot | `state_to_vda` | `order_from_vda`, `instant_actions_from_vda`, `responses_from_vda`, `zone_set_from_vda` |

A zone set outside a message is read with `zone_set_content_from_vda`, with the
checks of a message. The zone set of a site's configuration, which has its own
checks, is made into `link` types as it is with `zone_set_of_setup`.

## What cannot be translated

It is reported, never a panic. A translation gives the message with the parts that
were left out or changed (`Translated`), or an `Issue` that says why the whole
message was refused. An issue carries the VDA 5050 error type and references with
which a robot refuses it, where there is one.

| Message | On a bad part |
|---|---|
| order, zone set | refused whole |
| instant actions, responses | the entry is left out |
| state: header, charge | refused whole |
| state: last node | left out, and with it all that is ahead |
| state: route ahead | cut at the first bad step |
| state: position, velocity, action type, single list entries | left out |

An id is unusable if it is empty. An empty `orderId` in a state means no order.

A robot's `manufacturer` and `serialNumber` are taken as they are. Whether they can
be levels of an MQTT topic (not empty, none of `/ + # $`) is for whoever uses MQTT:
`identity_fits_topic`.

A charge at most 1e-9 above 100 is rounding: it is written and read as 100, without a
report.

Outgoing, an edge `orientation` or a position `theta` outside ±3.1416 rad (the range
of the schemas) is brought into −π..π and reported.

An incoming order is checked for what a robot refuses before it looks at the layout
and itself: node and edge counts, `sequenceId`s, release rules
(`VALIDATION_FAILURE`) and unknown names (`NO_ROUTE_TO_TARGET`). Whether edges
connect the nodes, whether the robot may drive them and whether the order fits what
it is doing is for the robot. So are the polygons and number ranges of an incoming
zone set.

## Fields that are not carried

`link` carries what botsim writes or reads; the rest of VDA 5050 is dropped without
a report. The list is in the crate documentation (`src/lib.rs`).

## Tests

Every kind of message the crate produces is validated against the official schemas
in `tests/data` (source and licence: `tests/data/SOURCE.md`), and `link` to VDA 5050
and back gives the same value.

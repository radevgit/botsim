# botsim-link

The messages between the fleet server and its robots, doors and lifts, used inside
the program. Those of robots are compact data with VDA 5050 3.0.1 meaning; they are
never sent to a robot, and the translation to and from VDA 5050 messages is not in
this crate.

Library only; of the botsim crates it depends on `botsim-model` only, reads no
files and uses no network. No crate uses it yet.

- Layout nodes, edges and maps are the numbers of the layout graph; a robot is a
  `RobotNo` (`botsim_model::robot`). Names are looked up elsewhere.
- Every message names the robot (for a door or lift: the device), a sequence number
  and a time in milliseconds, and is complete in itself.
- The types promise shape only. Whether a number lies in the layout, a float is
  finite or an id is usable is for whoever reads a message to check.

## Modules

| Module | |
|---|---|
| `message` | the messages to and from robots, their header and the time |
| `order` | an order: start node, legs, how many are released, actions |
| `instant` | instant actions |
| `response` | answers to the requests of a robot |
| `zone_set` | a zone set with its zones |
| `state` | the state report |
| `device` | doors and lifts: commands and states, and their header |
| `ids` | the ids that are text: order, action, action type, request, zone, zone set |
| `shape` | why an order or the route of a report cannot be made |

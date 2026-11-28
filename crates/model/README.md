# botsim-model

What the botsim crates share about a site and its robots. Library only; it
depends on no other botsim crate, reads no files and uses no network.

## Modules

| Module | |
|---|---|
| `graph` | the track graph of a LIF layout (`Graph::from_lif`): nodes on one or more maps, directed edges, per-robot-type edge rules and curves, stations |
| `spec` | a robot type from a VDA 5050 factsheet (`RobotSpec`): kinematics, speed and acceleration limits, body outline |
| `path` | the geometry of an edge as a line to drive along: straight, or a sampled LIF trajectory (NURBS) |
| `orientation` | the heading a robot must have on an edge (VDA 5050 `orientation`, `OrientationType`) |
| `safety` | the protective field ahead of a moving robot |
| `job` | a transport job (`Job`): from a pickup node to a drop node |
| `robot` | who a robot is: its VDA 5050 identity (`RobotId`) and the small number that stands for it (`RobotNo`) |
| `angle` | normalising and subtracting angles |
| `zones` | the zones of a map (VDA 5050 `zoneSet`), checked (`Zones`); the same checks and geometry on plain points, and a zone in plain data (`Area`) |
| `collision` | robot bodies placed in the world: overlap and gap |
| `waiting` | when a robot counts as waiting (`stands_waiting`), and why a controller holds one back (`HeldBack`, `WaitReason`) |
| `reservation` | what a controller has reserved for a robot (`Reservation`) |
| `protocol` | the VDA 5050 version in the message headers |
| `errors` | the predefined VDA 5050 error types (`ErrorType`): names and levels (`ErrorLevel`) |
| `time` | a time in whole milliseconds since 1970 (`Millis`); VDA 5050 timestamps (ISO 8601 UTC) from and to it |

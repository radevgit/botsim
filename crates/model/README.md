# botsim-model

What the botsim crates share about a site and its robots. Library only; it
depends on no other botsim crate, reads no files and uses no network.

## Modules

| Module | |
|---|---|
| `graph` | the track graph of a LIF layout (`Graph::from_lif`): nodes on one or more maps, directed edges, per-robot-type edge rules and curves, stations |
| `spec` | a robot type from a VDA 5050 factsheet (`RobotSpec`): kinematics, speed and acceleration limits, body outline |
| `path` | the geometry of an edge as a line to drive along: straight, or a sampled LIF trajectory (NURBS) |
| `orientation` | the heading a robot must have on an edge (VDA 5050 `orientation`) |
| `safety` | the protective field ahead of a moving robot |
| `job` | a transport job (`Job`): from a pickup node to a drop node |
| `robot` | who a robot is: its VDA 5050 identity (`RobotId`) and the small number that stands for it (`RobotNo`) |
| `angle` | normalising and subtracting angles |
| `errors` | the predefined VDA 5050 error types (`ErrorType`): names and levels |
| `time` | VDA 5050 timestamps (ISO 8601 UTC) from and to milliseconds since 1970 |

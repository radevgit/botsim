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
| `angle` | normalising and subtracting angles |

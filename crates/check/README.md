# botsim-check

Static checks of a LIF layout for the robot fleet of a botsim scenario. No simulation
is run. Meant to be used as a **library** (the viewer calls it); the `botsim-check`
binary is a thin wrapper.

```sh
cargo run --release -p botsim-check -- --geometry scenarios/warehouse4.geometry.json scenarios/warehouse4.toml
cargo run --release -p botsim-check -- --json --conflicts conflicts.json scenarios/warehouse4.toml
```

```rust
let report = botsim_check::run_scenario(path, &botsim_check::Options::default())?;
// report.findings, report.conflict_map, report.to_json()
```

The crate docs (`cargo doc -p botsim-check --open`) describe the library API.
How to show the results in a viewer: `discover/briefs/checker_for_viewer.md`.
The problems behind many checks were found by simulation: `docs/graph_problems.md`
(G1 to G12).

## Checks

A finding has a level (`info`, `warning`, `error`), a check name, a text, the ids of the
objects concerned and their places (kind, id, map, points). Checks run per robot type
of the fleet (types of the scenario's factsheets), on the whole LIF file.

| check (name in findings) | what it looks for | level | source |
|---|---|---|---|
| `structure` | edge refers to a node that exists in no layout of the file; node, edge or station id used twice | error | brief check 1 |
| `structure` | robot type named in edge properties that is not in the fleet | warning | |
| `structure` | edges that have no properties for a fleet type (it cannot use them) | info | |
| `factsheet` | a factsheet the checks cannot use (e.g. unsupported kinematics) | error | |
| `reachability` | nodes and stations a robot type cannot reach **and get out of again** (biggest strongly connected set of edges, with the turn rules of its kinematics: three-wheel cannot turn on the spot, differential needs the allowed rotation); stations the type does not serve | warning / info | G1, G7 |
| `turn` | edges after which no edge the type may drive can follow (the turn rule blocks it); the text gives an example with the headings | warning | G1 |
| `start` | scenario start nodes: node missing or not allowed for the type (error); robot cannot drive off in its start heading, start node outside the biggest connected set, start nodes less than 6 m apart (warning); start node used twice (error) | error / warning | G8 |
| `spacing` | edges whose end nodes are closer than the longest allowed robot plus its protective field (from the factsheet's safety envelope, else the simulator's 0.3 m) | warning | G3 |
| `length` | straight edges longer than 2.5 times the longest allowed robot (a convoy cannot form); the text gives the share of the road | warning | G4 |
| `conflict` | pairs of edges **without a common node** whose swept robot outlines overlap, per pair of robot types (including a type with itself); overlaps shallower than `min_overlap` (0.05 m) are left out | warning | check 5, G5, G6, G11 |
| `clearance` | needs the geometry file (`--geometry`, or `Options::geometry` / `Inputs::geometry`): the swept outline of a robot type against walls (polyline segments), racks and shelves (filled polygons) on the same map; station outlines and `other` shapes are not obstacles; the finding gives the deepest overlap (m), hits not deeper than 1 mm or than `min_overlap` (0.05 m) are left out; shapes on a map the layout does not have are ignored (info) | warning / info | check 4, G9 (partly) |

| `exits` | nodes of a type's biggest connected set that have only one edge the type may drive out of them (a held-up robot cannot be re-routed); count and share in the text, the nodes as places | info | G12 |
| `exit-block` | needs the conflict map: a **place** (a station's interaction node, or a dead-end node with one neighbour) with one edge out, whose **way out** (the only edge that may follow, up to the first node with two or more choices) a robot can block from a node where it **waits** for the place: both can stand there at once (as they arrived), the node is on the way or conflicts with it in the map, and on a shortest way from it to the place the next edge or its end node conflicts with the robot on the place, or ends at it. One finding per layout and pair of types. It names the risk of a deadlock; whether a controller lets a robot wait there it cannot say | warning | brief U6 |
| `loop` | closed one-way loops (simple cycles of drivable edges, turn rules apply, up to 150 m, two-way roads excluded) that hold fewer robots (length / (robot length + protective field)) than the scenario has robots of that type; needs a scenario (robot counts) | warning | G10 |
| `shelving` | needs the geometry file: an interaction node of a station of type `rack` or `shelf` with no rack or shelf within 2.5 m on its map | warning | G9 |
| `corner` | a node within 3 m of the corner of a turn arc (an edge with a trajectory turning 25 to 100 degrees; the corner is where the tangents at its two ends meet) that is not the arc's start or end; a node exactly at the corner (< 5 cm) is the junction's own corner node and not reported (that case is G6, found by the node-edge `conflict`). `warning` only if a robot standing on the node overlaps the swept outline of that arc (a node-edge conflict in the conflict map), else `info` (also `info` without a conflict map) | warning / info | G6b, viewer request |
| `door` | scenario `[[door]]`: edge missing in the layout (error); a robot type wider than `passage_width` on a door edge it may drive (warning). `[[lift]]`: floor that is not a map, stop node missing or not on a floor of the lift (error). The cabin size is not in the scenario, so it is not checked | error / warning | check 3 |
| | the two kinds of `conflict` finding (edge-edge and node-edge) are **classified**: *near a junction* (a diverge or merge: one node of one edge and one node of the other are at most 2 edges apart in the graph, or closer than the longer of the two robots plus its protective field) is `info`, with the junction nodes as places; the rest (*parallel*) is `warning`. In the map: `junction_nodes` on a conflict | info / warning | viewer request |
| `conflict` (node-edge) | a robot **standing on a node** (with the heading of an edge to or from it) against an edge that does not touch that node, driven by another robot; per ordered pair of robot types; the finding lists the nodes and the edges as places and the deepest case in the text | warning | check 5, G6 |

The swept outlines and the conflict map are computed by the library crate `botsim-sweep`
(`crates/sweep`); the checker turns the map into findings. `Options::clearance` (default
0.10 m, CLI `--clearance <m>`): in the map two bodies **nearer** than this count as conflicting
(overlap too; `depth` 0 and `distance` > 0 for a near miss). The findings keep to real overlaps
(`min_overlap`), so the clearance changes only the map. `check_all` and `check_inputs` use
clearance 0 (as before).

Also computed, not a finding: the **conflict map** (`report.conflict_map`): every pair of
different edges on the same map whose swept outlines overlap, per pair of robot types, with
the overlap depth and whether the two edges share a node (`pairs`), and the node-edge conflicts
with their depth (`node_pairs`). Ids only; see the note for the viewer for the format.

## How the swept outline is made

`Graph::edge_path` of `botsim-core` (the LIF trajectory, a NURBS curve, sampled; or the
straight line) is sampled every 0.25 m, curves much more finely (no corner moves more than 2 cm
between samples); at a node the body is swept over the turns on the spot (see
`crates/sweep/README.md`). At each sample the body outline of the factsheet
(`envelopes2d` without safety fields, else length × width) is placed at the kinematic center
and turned to the direction of travel. An omnidirectional robot gets a circle round its
kinematic center that holds the body in any heading (more conflicts than real). Overlap and
depth: separating axis test for rectangles, distance for circles.

## Official LIF examples

The 28 examples of the `vdma-lif` crate (`tests/examples`) run without panics or errors
(`structure` is clean). They have no scenario and no factsheets, so the example
`examples/official_lifs.rs` gives every robot type named in a LIF a copy of one template
factsheet (a differential robot, 1.2 m × 0.8 m) and calls `run_layout` (LIF + fleet, no scenario):

```sh
cargo run --release -p botsim-check --example official_lifs -- scenarios/diff_robot.factsheet.json \
    ~/.cargo/registry/src/*/vdma-lif-0.1.0/tests/examples/*.json
```

What comes out is expected for demonstration fragments, not a sign of wrong checks:
- `reachability` warnings in most of them: a layout of two nodes and one one-way edge has no
  connected set a robot can get out of again.
- `length` warnings: the edges are 5 to 11 m (limit 3 m for this robot).
- `turn` in 03, 04, 11, 12, 13: the rotation rules of the example (e.g. `NONE` at node N1 in
  03: the robot cannot turn round there, so it is stuck at the end of the edge) or a U-turn for
  the template's differential kinematics (04 is meant for an omnidirectional robot).
- `spacing` in 05, 09, 14: very short edges (0 to 0.4 m); `conflict` in 07 to 10, 16: stations whose
  nodes lie 1 to 2 m apart.
- 15 and 17 (a rack station of three stations, an edge with a trajectory): no findings.

## Not done yet

- Clearance: only the deepest hit per edge (no position of the hit along the edge); depth is
  exact for convex obstacles only.
- Two robots on two different nodes: in the conflict map (`node_node_pairs`), but no finding yet.
- The protective field and driving backwards in the swept outline.
- G2 (turn arc does not fit): needs an arc radius, which the layout and the factsheet do not give
  (open question). G5 (branch inside a siding): no separate check; the uncovered conflicts of
  `conflict` find the overlaps it causes.
- Lift cabin size against the robot (the scenario has none); turning ability (the factsheet has
  no turning radius).
- Node pairs that are not joined by an edge in the spacing check.

## Rules for the code

No panics on anything the input files cause: unusable files are a `LoadError`, layout
problems are findings. Touch only this crate; `botsim-core` and `botsim-scenario` are used
read-only.

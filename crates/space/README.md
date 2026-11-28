# botsim-space

Space on a layout: swept robot bodies along LIF edges and the **conflict map** of a layout
for a mixed fleet. Library only; it knows nothing about findings or scenario files. Used by
`botsim-check` (findings), the runner `botsim` (makes the map for the baseline's rules),
`botsim-server` (makes the map for the traffic module) and `botsim-traffic` (which pieces of
two routes cannot be used at the same time).

```rust
use botsim_space::{MapOptions, TypeRef, compute};

// one TypeRef per robot type, from its VDA 5050 factsheet
let types: Vec<TypeRef> = factsheets.iter().map(TypeRef::from_factsheet).collect::<Result<_, _>>()?;
let map = compute(&lif, &types, &MapOptions::default()).expect("layout is a graph");
// map.pairs      edge against edge, per pair of types (shares_node, junction_nodes, depth, distance)
// map.node_pairs a robot on a node against an edge that does not touch it
// map.node_node_pairs  two robots on two different nodes (no junction_nodes)
let json = botsim_space::to_json(&map);
```

- `MapOptions { sample_step: 0.25, min_overlap: 0.05, clearance: 0.10, protective_field: false }`:
  two bodies count as conflicting when they overlap by at least `min_overlap` **or** are nearer than `clearance`
  (a controller keeps a gap; positions vary from run to run). `MapOptions::overlap_only(step,
  min_overlap)` is the map of overlaps only. Per conflict: `depth` (deepest overlap, 0 for a
  near miss) and `distance` (smallest gap, 0 for an overlap).
- `junction_nodes` is set for a conflict near a junction (a diverge or merge: nodes at most
  `JUNCTION_HOPS` = 2 edges apart, or closer than the longer robot plus its protective field).
- How the outline is made: the edge path (LIF trajectory or straight line) sampled every
  `sample_step` (never more than the body length); the body outline at each sample turned to
  the path direction; an omnidirectional robot gets a circle that holds the body in any
  heading.
- **Curves** are sampled until no corner moves more than `CORNER_STEP` (2 cm) between
  neighbours, however short the curve (the edge path is a polyline: at its points the
  direction jumps, poses 5 mm apart).
- **Nodes:** the body in every heading a robot can have there: the directions of the edges in
  and out and, for a differential robot, the whole turn on the spot from each edge in to each
  edge out that may follow it, in the direction the simulator takes (both for a U-turn). Not
  covered: a turn from a start heading the layout does not give; driving backwards.
- `LayoutIndex` / `Place`: position (kind, id, map, points) of nodes, edges and stations by id.

## Protective field (option, off by default)

`protective_field: true` also lists a pair when the simulator's **standstill protective field**
(`botsim_model::safety`: ahead of the body in the direction of travel, 0.3 m long, 0.1 m wider on
each side) of one robot along its edge meets the other robot's body: such a robot stops and stands
until the other has left. Either side's field is enough; a robot standing on a node has none. Any
overlap counts (`clearance` and `min_overlap` do not apply to it). On curves the field is
placed as finely as the body, with the field's corners as the measure.
`ConflictMap::by_field(depth, distance)` says whether an entry is there only because of the
field; the JSON has no such mark, and the map does not record the option.

## Size and time

```sh
cargo run --release -p botsim-space --example map_stats -- <layout.lif.json> <factsheet.json>...
```

Release build, default options (2026-11-26; warehouse4 one run, the others best of three);
"off" and "on" are the protective field option:

| layout | off | on | edge pairs a controller waits for, off → on |
|---|---|---|---|
| warehouse4 (5 types) | 7.2 s | 17.6 s | 2822 → 3880 |
| M1 (3 types) | 0.07 s | 0.08 s | 0 → 12 |
| W_wrhs_1, 2, 3, L (1 type) | 0.50–0.77 s | 1.2–1.6 s | about 180 → about 1000 |

Before curves were sampled finely and turns were swept: warehouse4 1.0 s, M1 0.009 s,
W_wrhs about 0.02 s (off). Not optimised.

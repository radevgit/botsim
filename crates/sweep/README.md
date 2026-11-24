# botsim-sweep

Swept robot bodies along LIF edges and the **conflict map** of a layout for a mixed fleet.
Library only; it knows nothing about findings or scenario files. Used by `botsim-check`
(findings) and meant for the runner (reservations of the baseline controller) and later
the traffic module.

```rust
use botsim_sweep::{MapOptions, TypeRef, compute};

// one TypeRef per robot type, from its VDA 5050 factsheet
let types: Vec<TypeRef> = factsheets.iter().map(TypeRef::from_factsheet).collect::<Result<_, _>>()?;
let map = compute(&lif, &types, &MapOptions::default()).expect("layout is a graph");
// map.pairs      edge against edge, per pair of types (shares_node, junction_nodes, depth, distance)
// map.node_pairs a robot standing on a node against an edge that does not touch it
let json = botsim_sweep::to_json(&map);
```

- `MapOptions { sample_step: 0.25, min_overlap: 0.05, clearance: 0.10 }`: two bodies count as
  conflicting when they overlap by at least `min_overlap` **or** are nearer than `clearance`
  (a controller keeps a gap; positions vary from run to run). `MapOptions::overlap_only(step,
  min_overlap)` is the map of overlaps only. Per conflict: `depth` (deepest overlap, 0 for a
  near miss) and `distance` (smallest gap, 0 for an overlap).
- `junction_nodes` is set for a conflict near a junction (a diverge or merge: nodes at most
  `JUNCTION_HOPS` = 2 edges apart, or closer than the longer robot plus its protective field).
- How the outline is made: the edge path (LIF trajectory or straight line) sampled every
  `sample_step`; the body outline at each sample turned to the path direction; an
  omnidirectional robot gets a circle that holds the body in any heading.
- `LayoutIndex` / `Place`: position (kind, id, map, points) of nodes, edges and stations by id.
- The first call on warehouse 4 takes about 1.5 s (release build).

# botsim-record

The recording of a run: a JSON Lines file that the [viewer](../viewer/README.md)
and the [PNG exporter](../export/README.md) replay. This crate has the line
types, the writer and the reader.

```sh
cargo run --release -p botsim -- --record run.jsonl scenarios/ring_jobs.toml
```

## Using it

| | |
|---|---|
| Write | `Recorder::new(output)` gives the recorder and a handle; attach it with `World::set_observer_at(Box::new(recorder), interval_seconds)`; call `handle.finish()` after the run to flush and to get write errors |
| Read | `read_recording` (whole file), `Reader` (snapshot by snapshot), `read_recording_partial` (accepts a last line cut off by a stopped run) |

## Format (version 1)

Line 1 is the layout, every other line a snapshot. Maps, nodes, edges and robots
are referred to by their position in the lists.

**Layout** (`"type": "layout"`): `version`; `maps`; `nodes` (id, map, x, y);
`edges` (id, start, end, length, map_transition, and the curves per robot type
where there are any); `geometry` (static shapes); `robots` (id and type).

**Snapshot** (`"type": "snapshot"`):

| Field | |
|---|---|
| `tick`, `time` | step number and simulated seconds |
| `robots` | per robot: id, map, x, y, heading, shape, status, speed, order, last_node, released and planned route; if present: `charge`, `charging`, `waiting` |
| `reservations` | the nodes and edges each robot holds, as the controller published them |
| `blocking` | `[waiter, blocker]` pairs, as the deadlock detector sees them |
| `overlapping` | pairs of robots whose bodies overlap |
| `deadlocks` | the rings of waiting robots that last |
| `lifts`, `doors` | their state |

`waiting` is `{"waiting_for": <robot or null>, "reason": ...}` with the reason
`Release`, `Zone`, `Lift`, `Door` or `SafetyStop`. It is written only while a
robot waits for others, not during its own stop.

Readers accept older recordings without the optional fields, and refuse another
format version.

## Static geometry

Walls and racks are not in LIF. They come from a separate file
(`GeometryFile`, given to `botsim --geometry`) and are stored in the layout line:

```json
{"version": 1, "shapes": [
  {"id": "wall-north", "mapId": "M", "kind": "wall", "shape": "polyline",
   "vertices": [{"x": 0.0, "y": 8.0}, {"x": 30.0, "y": 8.0}]}
]}
```

`kind`: `wall`, `rack`, `shelf`, `station`, `other`; `shape`: `polygon` or
`polyline`. Coordinates are metres on the map `mapId`.

## What it does not do

- A snapshot is taken at the chosen interval; what happens in between is not
  recorded.
- No jobs, messages, zones, LIF stations or run figures: those are in the run's
  summary or the layout file.
- It only stores a run; it cannot replay it into the simulator.

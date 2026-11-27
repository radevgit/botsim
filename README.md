# botsim

Headless simulator for mixed robot fleets: robots execute VDA 5050 orders on a
LIF layout, and a controller is measured on throughput, waiting, deadlocks and
collisions. Work in progress.

[![botsim warehouse simulation](https://img.youtube.com/vi/wbVRCgwhrG8/maxresdefault.jpg)](https://www.youtube.com/watch?v=wbVRCgwhrG8)

A longer run on a more complex warehouse: [video, 6:40](https://www.youtube.com/watch?v=WVp4MgZwQKg)

## Workspace

```text
botsim/
├── crates/
│   ├── model/         layout graph, robot types, path and safety geometry
│   ├── link/          compact messages: fleet server, robots, doors, lifts
│   ├── core/          simulation: world, robots, orders, collisions, jobs
│   ├── scenario/      scenario file (TOML): loading, building the world
│   ├── baseline/      bundled baseline fleet controller
│   ├── traffic/       passing order at shared spots (--policy traffic)
│   ├── space/         swept robot outlines and the conflict map
│   ├── runner/        command line and JSON summary (package botsim)
│   ├── check/         static layout checker (checks: see its README)
│   ├── record/        recording of a run (JSON Lines)
│   ├── export/        PNG frames for videos, layout generators
│   └── viewer/        replay window, check overlay, usage heat map
└── scenarios/         example scenarios and layouts
```

Each crate's package is `botsim-<folder>`, except the runner, which is `botsim`.

Details: [run](crates/runner/README.md), [check](crates/check/README.md),
[conflict map](crates/space/README.md), [viewer](crates/viewer/README.md),
[video](crates/export/README.md).

## Quick start

```sh
# run a scenario, print a JSON summary
cargo run --release -p botsim -- scenarios/ring_jobs.toml

# check a layout without simulating
cargo run --release -p botsim-check -- scenarios/warehouse4.toml

# record a run and watch it
cargo run --release -p botsim -- --record run.jsonl scenarios/warehouse.toml
cargo run --release -p botsim-viewer -- run.jsonl
```

## Licence

MIT

# botsim

Headless simulator for mixed robot fleets. Robots of different sizes and types
execute VDA 5050 orders on a LIF track layout, so that a controller can be
measured on throughput, waiting, deadlocks and collisions.

Status: work in progress.

## Try it

```sh
cargo run --release -p botsim-runner -- scenarios/ring_jobs.toml
```

It runs the scenario to its end and prints a JSON summary (jobs, throughput,
waiting, collisions, where each robot ended). The same file always gives the
same output. Edit `scenarios/ring_jobs.toml` to change robots, jobs and seed.

## Workspace

- `crates/core` — simulation core (`botsim-core`): world, robots, orders, collisions, jobs, metrics
- `crates/baseline` — the bundled baseline controller (`botsim-baseline`)
- `crates/runner` — scenario files, CLI and JSON summary (`botsim-runner`)
- `docs/rules.md` — the rules the simulator and the baseline follow, with the decisions behind them
- `scenarios/` — example scenarios

## Licence

MIT

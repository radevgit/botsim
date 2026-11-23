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
Add `--timing` (before or after the file name) to also get the real time the
controller needed per step; that is the only thing that differs between runs.

`scenarios/mixed_fleet.toml` runs two robot types on one layout (a small carrier
and a larger, slower forklift; the layout says which type may use which edge
and at what speed).

`scenarios/lift_jobs.toml` is another example: two floors, one lift and jobs
between the floors (`cargo run --release -p botsim-runner -- scenarios/lift_jobs.toml`).

## Check a layout

```sh
cargo run --release -p botsim-check -- --geometry scenarios/warehouse4.geometry.json scenarios/warehouse4.toml
```

Static checks of the LIF layout for the fleet of a scenario, without running a
simulation. Prints findings as text; add `--json` for JSON. Exit code 0: no
errors, 1: errors found, 2: usage or load error. `--geometry` is optional (the
clearance check does not exist yet).

## Workspace

- `crates/core` — simulation core (`botsim-core`): world, robots, orders, collisions, jobs, metrics
- `crates/baseline` — the bundled baseline controller (`botsim-baseline`)
- `crates/scenario` — the scenario file (TOML): `Scenario`, `load`, `build_world` (`botsim-scenario`); shared by the runner and the checker
- `crates/runner` — CLI and JSON summary (`botsim-runner`); re-exports the scenario crate as `botsim_runner::scenario`
- `crates/check` — static layout checker and CLI (`botsim-check`)
- `docs/rules.md` — the rules the simulator and the baseline follow, with the decisions behind them
- `scenarios/` — example scenarios

## Licence

MIT

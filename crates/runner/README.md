# botsim (runner)

The command line: runs a scenario headless and prints a JSON summary (jobs,
throughput, waiting, collisions, deadlocks, where each robot ended). Without
`--timing` the same scenario and files always give the same output. The scenario
file format is in [botsim-scenario](../scenario/README.md).

Run the commands from the root of the repository.

```sh
cargo run --release -p botsim -- scenarios/ring_jobs.toml
```

```
botsim [--policy baseline|traffic] [--timing]
       [--record <file.jsonl> [--record-interval <seconds>] [--geometry <geometry.json>]]
       <scenario.toml>
```

| Option | |
|---|---|
| `--policy` | who decides which robot may go: the [bundled baseline](../baseline/README.md) (default) or the traffic module (`botsim-traffic`). The traffic module does not answer RELEASE zone requests yet |
| `--timing` | add the real time the controller took per step (differs between runs); not with `--policy traffic` yet |
| `--record <file>` | also write a recording (JSON Lines) for the [viewer](../viewer/README.md) and the [PNG exporter](../export/README.md); the summary is the same with or without it |
| `--record-interval <s>` | simulated seconds between snapshots (default 0.1) |
| `--geometry <file>` | add walls, racks and shelves to the recording |

Exit code: 0 done, 1 error (message on stderr), 2 wrong command line.

## Example scenarios

| Scenario | |
|---|---|
| `scenarios/ring_jobs.toml` | a small first example: a one-way ring, two robots, random jobs; edit it to change robots, jobs and seed |
| `scenarios/mixed_fleet.toml` | two robot types on one layout (a small carrier and a larger, slower forklift; the layout says which type may use which edge, and how fast) |
| `scenarios/lift_jobs.toml` | two floors, one lift, jobs between the floors |

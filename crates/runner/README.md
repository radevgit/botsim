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

## Server and simulator

The controller of the [fleet server](../server/README.md) and the
[simulator](../sim/README.md) do not know each other: the controller is given
data and gives back messages. The module `botsim::connect` of this crate passes
them between the two. The `botsim` command and the tests use it.

```text
connect::controller(&world)                     the rules of botsim-baseline for the world's fleet
connect::with_policy(&world, policy)            with other traffic rules
connect::with_dispatcher(&world, policy, d)     and another dispatcher
connect::with_controller(&world, make)          a controller made from the config
connect::step(&mut controller, &mut world, new_jobs)
                                                one cycle, with World::step
connect::collect(&mut controller, &mut world)   in a step without a cycle
connect::config(&world)                         the site and fleet as the controller gets them
```

The functions give a `Connected` controller: it is used as the controller itself.

`step` takes the world's time, every robot's report and the states of the lifts,
runs one cycle, sends the messages to the world in their order and publishes what
the controller holds back and reserves. The `botsim` command runs a cycle in every
step. A caller may run them less often, with `collect` in each step between, so
that the next cycle gets the reports of all those steps.

The world's time is given in whole milliseconds. **With a time step that is no
whole number of milliseconds** the time between two cycles differs from the step by
up to 1 ms; waiting figures, and the cycle in which a timeout of the rules ends,
can then differ slightly from a count in steps.

Both sides speak [`link`](../link/README.md) and number robots, lifts and doors in
the order they were added to the world: nothing is translated. The site's zone
set, which the controller sends to the robots, is made once from the world's
configuration, as it is (`botsim_vda::zone_set_of_setup`).

The tests that run the controller of the server and the baseline's rules against
the simulator are in this crate (those of the baseline have the prefix
`baseline_`), because it depends on both sides.

## The route through `vda`

For tests: the same run with every message to a robot and every report of a robot
translated by [botsim-vda](../vda/README.md) to VDA 5050 and back (message
structures, no JSON text). The `botsim` command does not take it.

```text
vda_route::controller(&world)                   as connect::controller, on the route
vda_route::with_controller(&world, make)
vda_route::config(&world)                       the site's zone set read as a message's content
controller.receive(&mut world, &vda_message)    a VDA 5050 message from another sender
controller.read_state(&state_message, tick)     the report the controller gets for it
controller.untranslated()                       what could not be translated
```

What the translator refuses of a message to a robot shows in that robot's errors
with the error type and references, as for its own refusals (a zone set's for
10 s), after the message delay; an instant action left out is listed `FAILED`.
Nothing is dropped silently, every case is counted:

| Case | Effect |
|---|---|
| a state message refused whole | as one that has not arrived: while it lasts nothing new goes to that robot (to nobody, if it never had one) |
| a state message translated in part | the controller sees the rest |
| an order or zone set refused | the robot reports the error and does not get the message |
| an instant action left out | listed `FAILED` with its error, in its place; the others go |
| a response left out | the others go |
| a refusal with no VDA 5050 error type | counted only |
| the site's zone set refused | no robot of a site with zones gets a task |

An instant action with an empty `actionId` is refused with its error, but a VDA
5050 state cannot carry an action without an id, so the controller does not see its
`FAILED` entry. Doors and lifts are outside VDA 5050 and go as on the direct route.

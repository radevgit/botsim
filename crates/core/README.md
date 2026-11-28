# botsim-core

The simulation: one layout, a clock with a fixed time step, and robots that
execute VDA 5050 orders. It reads no files and uses no network; a controller and
the file formats live in other crates.

The same inputs and seed give the same run.

## Main types

The layout graph (`Graph`), the robot types (`RobotSpec`) and the zones
(`Zones`) are in [botsim-model](../model/README.md).

| Type | |
|---|---|
| `world::World` | the world: add robots, doors and lifts, send them messages, step, read states and results |
| `jobs::JobGenerator` | seeded random transport jobs (pickup node, drop node) |
| `observer::Observer` | a read-only view of the run, e.g. for a recording or a viewer |

## Using it

1. Build a `Graph`, then `World::new(graph, time_step, start_unix_ms)`.
2. `add_robot` for each robot.
3. Each step: read `state` for each robot, let your controller send `send_order`,
   `send_instant_actions`, `send_responses` or `send_zone_set`, then `step`.
4. Read `collisions`, `deadlocks`, `wait_totals` and `snapshot`.

The controller of the [fleet server](../server/README.md), with the
[baseline's traffic rules](../baseline/README.md), is a ready controller, run
against a world by [botsim-bridge](../bridge/README.md); the
[scenario crate](../scenario/README.md) builds a world from a TOML file.

## What it simulates

- **Orders:** base and horizon, order updates, rejection with the VDA 5050
  errors. Node actions are timed waits with their blocking types.
- **Instant actions:** `cancelOrder`, `startCharging`, `stopCharging`,
  `enableZoneSet`, `deleteZoneSet`; others are answered with an error.
- **Driving:** differential, omnidirectional and three-wheel robots; straight
  edges and LIF trajectories; edge orientation, rotation rules and speed limits;
  driving through nodes where it can; several maps.
- **Safety stop:** a protective field ahead of a moving robot; it brakes for
  another robot in it.
- **Doors and lifts**, with their own command ports; one robot rides a lift at a
  time.
- **RELEASE zones:** robots request them and do not enter before they are
  granted; leases and revocation.
- **Battery and chargers**, **message delay**, and **disturbances** (lower top
  speed, random short stops, varying action times).
- **Observation:** collisions, deadlocks (rings of waiting robots), who waits for
  whom and why.

## What it does not do

- It does not decide anything: no routes, no job assignment, no traffic rules.
- Collisions and deadlocks are reported, not prevented or resolved.
- Only robots are obstacles: no walls or people. A robot standing or turning at a
  node has no protective field.
- Zone types other than RELEASE are loaded and checked, but robots do not react
  to them. One zone set, for one map.
- No edge actions, pause, operating modes, `factsheetRequest` or breakdowns.

The exact rules are in [docs/rules.md](../../docs/rules.md).

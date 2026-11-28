# botsim-core

The simulation: one layout, a clock with a fixed time step, and robots that
execute orders. It reads no files and uses no network; a controller and the file
formats live in other crates.

The same inputs and seed give the same run.

It talks in the messages of [botsim-link](../link/README.md), which for robots
have the meaning of VDA 5050; it reads and writes no VDA 5050 message (that is
[botsim-vda](../vda/README.md)'s). VDA 5050 types are used for setup data only: a
robot's factsheet and the zone set of the site.

## Main types

The layout graph (`Graph`), the robot types (`RobotSpec`) and the zones
(`Zones`) are in [botsim-model](../model/README.md).

| Type | |
|---|---|
| `world::World` | the world: add robots, doors and lifts, send them messages, step, get their reports and the results |
| `robot::Unread` | what was sent to a robot and could not be read, with its error |
| `jobs::JobGenerator` | seeded random transport jobs (pickup node, drop node) |
| `observer::Observer` | a read-only view of the run, e.g. for a recording or a viewer |

## Using it

1. Build a `Graph`, then `World::new(graph, time_step, start_unix_ms)`.
2. `add_robot` for each robot.
3. Each step: take `reports` (once in every step) and `lift_reports` or
   `door_reports`, let your controller send messages with `send`, `command_door`
   and `command_lift`, then `step`.
4. Read `collisions`, `deadlocks`, `wait_totals` and `snapshot`.

Robot `i`, in the order the robots were added, has the number `(i, 0)`; doors and
lifts are numbered the same way. A message that names what is not there is an
error or is refused by the robot, never a panic.

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

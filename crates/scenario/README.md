# botsim-scenario

The scenario file (TOML) that describes one run: loading it, checking it, and
building the simulated world from it. Used by the [runner](../runner/README.md)
and the [layout checker](../check/README.md).

## Using it

| Call | |
|---|---|
| `Scenario::load(path)` | read and check a scenario file (`from_toml` for text) |
| `build_world(base_dir)` | read the layout, factsheets and zone set, relative to `base_dir` (the scenario's folder), and place the robots |
| `job_generator(graph)` | the random jobs of the `[jobs]` section, if there is one |

Unknown keys are errors. Problems with the scenario or its files come back as a
`ScenarioError`.

## File format

Examples are in [scenarios/](../../scenarios).

### Top level

| Key | Default | |
|---|---|---|
| `name`, `seed` | required | the seed fixes all random values |
| `duration`, `time_step` | required | seconds |
| `start_time` | required | UTC, like `2026-11-20T10:00:00Z` |
| `lif` | required | the LIF layout file |
| `zone_set` | none | VDA 5050 zone set (the `zoneSet` object or a whole message) |
| `deadlock_timeout` | 30 | seconds a ring of waiting robots must last |
| `safety_stop` | true | robots brake for robots ahead |
| `drive_through_nodes` | true | no stop at a node where it is not needed |
| `message_delay` | 0 | seconds a message takes, each way |
| `stop_at_first_deadlock` | false | end the run at the first deadlock |
| `charger_power_w` | none | makes stations of type `charger` chargers |
| `charge_low` | 25 | percent below which the controller sends a robot to charge |
| `conflict_reservation` | true | the controller gets the conflict map of the layout |
| `conflict_clearance` | 0.10 | metres: bodies nearer than this conflict |
| `route_load_weight`, `route_wait_weight` | 0 | metres added to a route's cost for busy places (baseline controller) |

### `[[robot]]`: a group of robots of one type (at least one)

| Key | |
|---|---|
| `manufacturer`, `serial_prefix` | serial numbers are `<prefix>-001`, `-002`, … |
| `factsheet` | VDA 5050 factsheet file |
| `start_nodes` | one node per robot; this sets the number of robots |
| `start_heading` | radians (default 0), or `"road"`: along the road leaving the start node |
| `start_headings` | instead: one heading per start node |
| `action_time` | seconds an action takes |
| `[robot.action_times]` | seconds per action type; `pick` and `drop` take 5 s unless listed here |
| `zone_requests` | `"on_release"` (default) or `"on_horizon"`: when a RELEASE zone is requested |
| `battery_wh` | battery capacity; without it no battery model |
| `drive_wh_per_m`, `idle_w`, `work_w` | energy use (default 0); need `battery_wh` |
| `start_charge` | percent, or `[low, high]` for a random value per robot (default 100); needs `battery_wh` |

### `[jobs]`

Without this section no jobs are created and no controller runs.

| Key | |
|---|---|
| `rate_per_hour` | average new jobs per simulated hour |
| `nodes` | jobs between these nodes (at least two), or |
| `stations` | jobs between these LIF stations, or |
| `pickup_stations`, `drop_stations` | jobs from the first to the second list; `both_directions = true` also makes them backwards |

With none of the three, jobs go between all stations of the layout.

### `[disturbances]`

| Key | Default | |
|---|---|---|
| `enabled` | true | false switches the section off |
| `speed_variation` | 0 | each robot's top speed lowered by up to this share |
| `action_variation` | 0 | each action up to this share longer |
| `stops_per_hour` | 0 | random short stops per hour of driving |
| `stop_seconds_min`, `stop_seconds_max` | 5, 15 | length of a stop |

### `[[door]]` and `[[lift]]`

| Door key | |
|---|---|
| `id`, `edges` | the layout edges through the door |
| `passage_width` | metres (optional): wider robots cannot pass |
| `open_seconds`, `close_seconds` | |
| `hold_seconds` | optional: closes by itself after this long open |

| Lift key | |
|---|---|
| `id`, `floors` | map ids, lowest floor first (at least two) |
| `doors` | names of the cabin's doors (default `["front"]`) |
| `stops` | where robots ride: `{ node, door, waiting }` (`door` and `waiting` optional) |
| `start_floor` | default: the first floor |
| `travel_seconds_per_floor`, `door_open_seconds`, `door_close_seconds` | |
| `door_hold_seconds` | optional: the door closes by itself after this long open |

## What it does not do

It runs nothing and chooses no controller. Many controller settings above are
only carried here; the runner passes them on. A scenario read from text has no
file to read the layout from again, so the runner makes no conflict map for it.

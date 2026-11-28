# botsim-server

The fleet server: the control loop that carries out transport jobs, with a slot for
the traffic rules. It talks to the robots, doors and lifts only through the
messages of [botsim-link](../link/README.md), which for robots have the meaning of
VDA 5050; real VDA 5050 messages come from the translator,
[botsim-vda](../vda/README.md).

Library only. It knows no simulator and no scenario file, reads no files and uses
no network: it is given data and gives back messages.

## Using it

```text
Controller::with_policy(&config, rules)        a controller with these traffic rules
Controller::with_dispatcher(&config, rules, d) and another dispatcher
controller.step(&input, new_jobs)              one cycle; returns the messages to send
controller.completed(), rejected(), pending(), metrics(seconds)
```

`config` (`cycle::Config`) holds the layout, the robots with their identity and
type from their factsheets (`FleetRobot::from_factsheet`), the zones and the zone
set to send for them, the lifts and the doors. Robots, lifts and doors are named
in the messages by their places in these lists; the order of the robots also
decides ties.

`input` (`cycle::Input`) holds the time, every robot's state report, in the fleet's
order, and the states of the lifts (who is in them). The result (`cycle::Output`)
holds the messages to robots, doors and lifts in the order to send them, why each
robot is held back, and the reservations if somebody watches.

To run it against the simulator use [botsim-bridge](../bridge/README.md).

A refusal of an order shows only in the robot's state; see the
[rules](../../docs/rules.md), §1a.

Optional: `set_charging`, `set_load_weight`, `set_wait_weight`.

## Parts

- The **controller** (`controller`) carries out the tasks: it builds the
  messages, releases routes, finds another route when asked (with
  [botsim-router](../router/README.md)), runs lifts and doors and keeps track of
  each robot's work.
- The **dispatcher** says which robot gets which job, and which one drives to a
  charger or a parking place. It is the trait `Dispatcher` of
  [botsim-dispatcher](../dispatcher/README.md), which also holds the baseline's,
  used unless another is given.
- The **traffic rules** say who may go now: how far each route is released, and
  the answers to zone requests. Their slot is the trait `policy::TrafficPolicy`;
  whoever makes the controller chooses them. A policy must implement `start`,
  `place`, `release` and `zone_answers`.

## Traffic rules

| Rules | Made with |
|---|---|
| first come, first served, the reference | `botsim_baseline::controller(&config)` of [botsim-baseline](../baseline/README.md) |
| the traffic module, [botsim-traffic](../traffic/README.md) | `traffic_policy::TrafficAdapter::new(&lif, &types, &options)`: the layout, the robot types and what the conflict map counts ([botsim-space](../space/README.md)) |
| any other | `Controller::with_policy(&config, rules)` |

The controller's work and the rules the baseline follows are in
[docs/rules.md](../../docs/rules.md), §9.

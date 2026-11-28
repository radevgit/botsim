# botsim-server

The fleet server: the control loop that carries out transport jobs, with a slot for
the traffic rules. It talks to the robots, doors and lifts only through the
messages of [botsim-link](../link/README.md), which for robots have the meaning of
VDA 5050; real VDA 5050 messages come from the translator,
[botsim-vda](../vda/README.md).

Library only. It knows no simulator and no scenario file, reads no files and no
clock and uses no network: it is given data and gives back messages.

## Using it

```text
Controller::with_policy(&config, rules)        a controller with these traffic rules
Controller::with_dispatcher(&config, rules, d) and another dispatcher
controller.step(now, inbox)                    one cycle; returns the outbox
controller.completed(), rejected(), pending(), metrics(seconds)
```

`config` (`cycle::Config`) holds the layout, the robots with their identity and
type from their factsheets (`FleetRobot::from_factsheet`), the zones and the zone
set to send for them, the lifts, the doors and the time the controller starts.
Robots, lifts and doors are named in the messages by their places in these lists;
the order of the robots also decides ties.

`now` is a time in whole milliseconds since 1970. `inbox` (`cycle::Inbox`) holds
what has arrived since the cycle before: robot state reports and lift states, in
any order, none or several of one robot or lift, and new jobs. The result
(`cycle::Outbox`) holds the messages to robots, doors and lifts in the order to
send them, why each robot is held back, and the reservations if somebody watches.
Cycles may come at any interval.

To run it against the simulator use the [command line crate](../runner/README.md)
`botsim`, which connects the two.

A refusal of an order shows only in the robot's state; see the
[rules](../../docs/rules.md), §1a.

Optional: `set_charging`, `set_load_weight`, `set_wait_weight`.

## Reports, silent robots and time

- Of several reports of a robot the **newest** counts: the latest time in its
  header, of equal times the higher sequence number, of equal headers the later in
  the list. A report older than the one kept is not looked at; the same one again
  counts as arrived. So **a robot whose clock goes back is held back** until its
  time has passed that of the kept report.
- **Until every robot of the fleet has reported once**, nothing new goes to
  anyone; reports and jobs are taken.
- **A robot without a new report** then holds back only itself: nothing is sent or
  released to it, what it holds (nodes, zones, a lift, a door) stays held, and the
  dispatcher gives it no job. This is the strict rule: a robot that reports less
  often than the cycles run is released only in the cycles with its report.
- A time that does not advance or goes back counts no time; a jump counts in
  full. A robot's waiting counts the time since its report before.

The details are in the documentation of the `cycle` module.

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
  `place`, `release` and `zone_answers`, and must give nothing to, and take nothing
  from, a robot without a new report.

## Traffic rules

| Rules | Made with |
|---|---|
| first come, first served, the reference | `botsim_baseline::controller(&config)` of [botsim-baseline](../baseline/README.md) |
| the traffic module, [botsim-traffic](../traffic/README.md) | `traffic_policy::TrafficAdapter::new(&lif, &types, &options)`: the layout, the robot types and what the conflict map counts ([botsim-space](../space/README.md)) |
| any other | `Controller::with_policy(&config, rules)` |

The controller's work and the rules the baseline follows are in
[docs/rules.md](../../docs/rules.md), §9.

# botsim-bridge

Runs the controller of the [fleet server](../server/README.md) against the
simulated world of [botsim-core](../core/README.md). The two do not know each
other: the controller is given data and gives back messages, and this crate passes
them between it and the world. The `botsim` command and the tests both use it.

```text
bridge::controller(&world)                     the rules of botsim-baseline for the world's fleet
bridge::with_policy(&world, policy)            with other traffic rules
bridge::with_dispatcher(&world, policy, d)     and another dispatcher
bridge::with_controller(&world, make)          a controller made from the config
bridge::step(&mut controller, &mut world, new_jobs)
                                               once per simulation step, with World::step
bridge::config(&world)                         the site and fleet as the controller gets them
```

The functions give a `Bridged` controller: it is used as the controller itself.

`step` takes the clock, every robot's report and the states of the lifts, runs one
cycle, sends the messages to the world in their order and publishes what the
controller holds back and reserves.

Both sides speak [`link`](../link/README.md) and number robots, lifts and doors in
the order they were added to the world: nothing is translated. The site's zone
set, which the controller sends to the robots, is made once from the world's
configuration, as it is (`botsim_vda::zone_set_of_setup`).

## The route through `vda`

For tests: the same run with every message to a robot and every report of a robot
translated by [botsim-vda](../vda/README.md) to VDA 5050 and back (message
structures, no JSON text). The `botsim` command does not take it.

```text
bridge::vda_route::controller(&world)          as bridge::controller, on the route
bridge::vda_route::with_controller(&world, make)
bridge::vda_route::config(&world)              the site's zone set read as a message's content
controller.receive(&mut world, &vda_message)   a VDA 5050 message from another sender
controller.read_state(&state_message, tick)    the report the controller gets for it
controller.untranslated()                      what could not be translated
```

What the translator refuses of a message to a robot shows in that robot's errors
with the error type and references, as for its own refusals (a zone set's for
10 s), after the message delay; an instant action left out is listed `FAILED`.
Nothing is dropped silently, every case is counted:

| Case | Effect |
|---|---|
| a state message refused whole | as one that cannot be had: that cycle the controller only takes the new jobs |
| a state message translated in part | the controller sees the rest |
| an order or zone set refused | the robot reports the error and does not get the message |
| an instant action left out | listed `FAILED` with its error, in its place; the others go |
| a response left out | the others go |
| a refusal with no VDA 5050 error type | counted only |
| the site's zone set refused | no robot of a site with zones gets a task |

An instant action with an empty `actionId` is refused with its error, but a VDA
5050 state cannot carry an action without an id, so the controller does not see its
`FAILED` entry. Doors and lifts are outside VDA 5050 and go as on the direct route.

This crate is there only until `botsim` connects the server and the simulator
itself.

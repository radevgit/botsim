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
controller.untranslated()                      what could not be translated
```

The functions give a `Bridged` controller: it is used as the controller itself and
holds what translates for it.

`step` reads the clock, every robot's state and the lifts, runs one cycle, sends
the messages to the world in their order and publishes what the controller holds
back and reserves.

## Translation

The controller speaks [`link`](../link/README.md), the simulated robots VDA 5050.
Robot messages are translated by [botsim-vda](../vda/README.md) and nowhere else.
Doors and lifts are outside VDA 5050: their numbers, maps and cabin doors are
looked up here.

Nothing is dropped silently. Every case is counted (`report`):

| Case | Effect |
|---|---|
| a state message refused whole | as one that cannot be had: that cycle the controller only takes the new jobs |
| a state message translated in part | the controller sees the rest |
| a message to a robot refused | not sent; the controller waits as for a lost one |
| one instant action or response | left out, the others go |
| the site's zone set refused | no robot of a site with zones gets a task |
| a lift state, or a command to a lift or door, that names what is not there | left out |

It is there only until the server and the simulator are connected through
[`link`](../link/README.md).

# botsim-bridge

Runs the [baseline controller](../baseline/README.md) against the simulated world
of [botsim-core](../core/README.md). The two do not know each other: the
controller is given data and gives back messages, and this crate passes them
between it and the world. The `botsim` command and the tests both use it.

```text
bridge::controller(&world)                     the baseline rules for the world's fleet
bridge::with_policy(&world, policy)            with other traffic rules
bridge::with_dispatcher(&world, policy, d)     and another dispatcher
bridge::step(&mut controller, &mut world, new_jobs)
                                               once per simulation step, with World::step
bridge::config(&world)                         the site and fleet as the controller gets them
```

`step` reads the clock, every robot's state and the riders of the lifts, runs one
cycle, sends the messages to the world in their order and publishes what the
controller holds back and reserves.

It is there only until the server and the simulator are connected through
[`link`](../link/README.md).

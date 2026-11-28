# botsim-baseline

First come, first served traffic rules for the [fleet
server](../server/README.md): the first rules behind its slot. They are a simple
reference that other traffic rules are measured against, and are kept as they are:
only bugs are fixed.

## Using it

```text
botsim_baseline::controller(&config)    a controller with these rules for a site and fleet
BaselinePolicy::new(&graph)             the rules alone, for Controller::with_dispatcher
```

How the controller is run is told in [botsim-server](../server/README.md).

Optional: the conflict lists of the layout
(`controller.policy_mut().set_conflicts(..)`, then `set_node_conflicts` and
`set_node_node_conflicts`), which the [runner](../runner/README.md) makes with
[botsim-space](../space/README.md). `policy().table()` says who holds which node
and edge, `policy().give_ways()` how often a robot went first.

## The rules

Traffic:

- A robot is released its node and the next two, as far as nobody else holds
  them.
- With the conflict lists, it is also not released where its body would meet what
  another robot holds on another edge or node.
- "Go-first": a robot blocked for 5 s may go ahead of the robot that holds its
  way, if that robot is stopped and cannot go on because the blocked robot is in
  its way (and further conditions hold).
- A robot blocked for 10 s asks for another route.
- A RELEASE zone is granted to one robot at a time, first come first served.
- A robot without a new report in a cycle (see
  [botsim-server](../server/README.md)) keeps what it holds, is not answered, and
  does not go first nor is gone past.

Jobs and routes are not these rules' business: the server's controller and its
dispatcher (the baseline's unless another is given) do them.

- The oldest job first, to the nearest robot (by route length) that is free or
  on its way to a parking place.
- Shortest route over the edges the robot type may use; optionally longer ways
  round busy places.
- A free robot with a low battery goes to a free charger.
- An idle robot goes to a free station of type `parking`, if the layout has one.
- One robot at a time uses a lift or a door.

## What it does not do

- It does not look ahead beyond the release window and does not back a robot out:
  robots can block each other for good.
- Once go-first lets a robot onto an edge, the other conflict checks of that edge
  are skipped, except for a robot standing on a node too close to its end.
- It does not handle failed robots. A job a robot refuses is not given out again.
- Zone grants have no lease and are not revoked.

The exact rules are in [docs/rules.md](../../docs/rules.md).

# botsim-baseline

The bundled fleet controller. It talks to the robots only through VDA 5050
messages (and to doors and lifts through their command ports). It is a simple
reference that other traffic rules are measured against, and is kept as it is:
only bugs are fixed.

## Using it

```text
Controller::new(&world)                the baseline rules
controller.step(&mut world, new_jobs)  once per simulation step, with World::step
controller.completed(), rejected(), pending(), metrics(seconds)
```

Optional: `set_charging`, `set_load_weight`, `set_wait_weight`, and the conflict
lists of the layout (`set_conflicts`, then `set_node_conflicts` and
`set_node_node_conflicts`), which the [runner](../runner/README.md) makes with
[botsim-sweep](../sweep/README.md).

## Two parts

- The **dispatcher** (`controller`) assigns jobs, plans routes, sends charging
  and parking trips, runs lifts and doors, and builds the messages.
- The **traffic policy** says who may go now: how far each route is released,
  and the answers to zone requests. It is the trait `policy::TrafficPolicy`.
  Another policy can replace the baseline's:
  `Controller::with_policy(&world, policy)`. A policy must implement `start`,
  `place`, `release` and `zone_answers`.

## The rules

Jobs and routes:

- The oldest job first, to the nearest robot (by route length) that is free or
  on its way to a parking place.
- Shortest route over the edges the robot type may use; optionally longer ways
  round busy places.
- A free robot with a low battery goes to a free charger.
- An idle robot goes to a free station of type `parking`, if the layout has one.
- One robot at a time uses a lift or a door.

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

## What it does not do

- It does not look ahead beyond the release window and does not back a robot out:
  robots can block each other for good.
- Once go-first lets a robot onto an edge, the other conflict checks of that edge
  are skipped, except for a robot standing on a node too close to its end.
- It does not handle failed robots. A job a robot refuses is not given out again.
- Zone grants have no lease and are not revoked.

The exact rules are in [docs/rules.md](../../docs/rules.md).

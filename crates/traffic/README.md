# botsim-traffic

Who may go now, in a mixed fleet of VDA 5050 robots on a LIF layout: the module keeps a
passing order at every place that two robots cannot use at the same time, and releases
each robot's route node by node according to it.

It **never plans paths** and never talks to a robot. Routes come from a dispatcher; the
dispatcher sends what is released and, when asked, looks for another way. The crate depends
only on [`botsim-sweep`](../sweep/README.md) and `vdma-lif`, not on the simulator.

## What it decides

- A **route** is node, edge, node, … of one robot type (`manufacturer/seriesName`). The index
  of a piece is its VDA 5050 `sequenceId` in an order that starts at 0.
- Two robots **conflict** if they use the same node, the same edge, a pair the conflict map
  lists for their types (edge and edge, node and edge, node and node), or two pieces of one
  RELEASE zone.
- Every conflict gets an **order**: who passes first. Conflicts that can only be ordered
  together (head-on, following) share one decision. The orders never form a cycle.
- A new route's passes go **after** those already ordered. It goes ahead only where it cannot
  wait, and only if the other robot is not yet released there and the new route does not end
  there.
- A route that cannot be ordered whole is **taken as far as it can be**; later releases take
  more.
- **Release** is first come, first served where that is safe: a robot that asks first may be
  put ahead, for the whole group, if the orders stay valid.
- It asks for **another way** after a new or changed route of which nothing beyond what the
  robot holds can be taken (until one is), while a robot waits for one whose route ends
  there, and when the robot is on a ring of waiting robots (then: keep clear of its next
  node).
- What is released is not taken back, except by a confirmed cancel.

## What it guarantees

- **No collision**, as far as the conflict map lists every pair, the progress reports are
  true and the robots keep to what is released: the module releases no robot into a place
  that conflicts with one another robot holds. It cannot undo robots that already stand in
  conflicting places when the second one is added; it names them.
- **No deadlock guarantee.** An order over whole routes would give one, but routes are taken
  in parts, and robots can come to wait for each other for good (see below).

The same calls give the same answers. Wrong names and impossible reports are answered with
an error or `None`.

## Interface

```rust
use botsim_sweep::{MapOptions, compute};
use botsim_traffic::{Report, Route, Spots, Traffic};

// types: one botsim_sweep::TypeRef per robot type, from its factsheet
let map = compute(&lif, &types, &MapOptions::default()).ok_or("not a graph")?;
let mut traffic = Traffic::new(Spots::new(&lif, &map)?);

// the robot stands on the first node; an error can still mean that it was added
let route = Route::new("ACME/Small", &["A", "B"], &["A-B"]).ok_or("not a route")?;
let robot = traffic.add(route)?;

// every cycle: first everyone's progress (node indices), then one query per robot
traffic.cycle_start(&[Report { robot, reached: 0, released: 0 }]);
if let Some(answer) = traffic.release(robot, 4) {
    // release up to node `answer.through`; look for another way if `answer.reroute`
}
```

| call | when | answer |
|---|---|---|
| `Spots::new`, `Spots::add_zone` | once, before the fleet | an error for names that are unknown or listed twice |
| `add(route)` | a robot enters, standing on the first node | its number. `NotAdded::Standing`: it is kept on that node, with the reason and the robots already released there |
| `new_order(robot, route)` | the next order, from a robot at the end of its route | `NotAdded::Reroute`: it is on the new order, but nothing beyond its node is taken yet. Other errors change nothing |
| `change(robot, route)` | another route beyond what is released | `NotAdded` if nothing more can be taken; the route then ends at the last released node |
| `cycle_start(&[Report])` | first in every cycle | the reports that cannot be; the others are taken |
| `release(robot, limit)` | per robot | `Release { through, reroute }` |
| `reroute_needed(robot)` | any time | `Reroute { avoid }`: what another way should keep clear of |
| `cut_short(robot)` | its order is refused or being cancelled | it still holds what was released |
| `cancelled(robot, Cancel)` | a state report on the cancel | `true` once confirmed: what is beyond where it stands is free |
| `zone_request(robot, zone_set, zone)` | a request to enter a RELEASE zone | `Granted` or `Queued` |
| `holder`, `blockers`, `next_holder`, `refused_by`, `waiting_on_rings` | questions; change nothing | who holds a piece or a robot back |

`give_next` (one node) is the step `release` repeats; `reverse` turns the order of one
conflict's group by hand.

## Running it in botsim

```sh
cargo run --release -p botsim -- --policy traffic scenarios/mixed_fleet.toml
```

The adapter is [`runner/src/traffic_policy.rs`](../runner/src/traffic_policy.rs). It builds
the conflict map of the whole fleet with the scenario's `conflict_clearance`. With the
scenario's safety stop on, the map also counts what the protective field of a standing robot
reaches along an edge (`MapOptions::protective_field`), so that the order covers the wait of
a robot stopped by its field. `--timing` cannot be used with this policy.

## Not solved

- **Deadlocks remain.** In botsim's M1 measurement (12 robots, 90 runs of two hours, at
  commit `dd988c4` with the safety field in the map) there were 19 deadlocks, and in 13 runs
  the whole fleet stopped. No collision. Against a ring the module can only ask for another
  way; the dispatcher may have none, and botsim's gives none to a job with a lift or a door.
- **The way out is not kept free.** A robot that waits next to a station with one way in and
  out can keep the robot there from ever leaving.
- **Two robots already standing in conflicting places** (the same node, one zone) get no new
  route. A cancel helps only if it stops a released robot short of the conflict; otherwise
  an operator has to move one.
- A head-on meeting on a single lane without a passing place is not prevented
  (`scenarios/corridor_deadlock.toml`).
- **Adapter gaps:**
  - RELEASE zones are not given to the module and zone requests are not answered: a scenario
    with such zones stalls (`ring_jobs`: no job done, 29 of 35 without its zones).
  - Refused or cancelled orders are not passed on.
  - The dispatcher can only be told nodes to keep clear of; its holder queries and the
    reservations for a viewer are not answered.

## Tests

```sh
cargo test -p botsim-traffic
# a million random cases per kind; takes minutes
cargo test -p botsim-traffic --release many_cases -- --ignored --nocapture
```

The random cases compare how a route is placed (`add`, `change`, `new_order`) with trying
every order. They use the module's own grouping of conflicts, list edge pairs only, and do
not check that no longer part of a route could have been taken.

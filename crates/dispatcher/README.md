# botsim-dispatcher

The dispatcher of a fleet server: which robot gets which transport job, and which
robot drives to a charger or a parking place.

Library only; of the botsim crates it depends on `botsim-model` and
`botsim-router` only, reads no files and uses no network. It knows no messages, no
traffic rules and nothing of a simulation. `botsim-baseline` uses it.

## The interface

The crate root holds the interface, so that another dispatcher can take the
bundled one's place (`Controller::with_dispatcher` in `botsim-baseline`).

| Trait | Implemented by | |
|---|---|---|
| `Dispatcher` | a dispatcher | takes new jobs, hears how tasks end, and gives out tasks in `dispatch` |
| `Control` | the controller that calls it | who holds a node, what a route costs now; carries out a task |

- The dispatcher gets plain data: what stays the same (`Site`: the layout, what
  each robot type may drive, the lift rides) and every robot as it stands
  (`Robot`).
- A task (`Task`) is a route with its purpose: a job, a trip to a charger, or a
  trip to a parking place.
- Dispatching is serial: a task is carried out before the call returns, so the next
  decision sees its effect (`Answer`: taken, refused, or impossible).

## The bundled dispatcher

`baseline::BaselineDispatcher`, kept as the reference:

- Low robots go to free chargers, the nearest pair first (only if charging is
  set). Below the critical limit a robot takes no jobs.
- The oldest job first, to the nearest robot (by route length) that is free or on
  its way to a parking place; ties go to the lower robot number. A job no such
  robot can do does not block the jobs behind it.
- Free robots that got nothing go to free parking places, the nearest pair first.

A job whose pickup or drop is not a node of the layout stays queued.

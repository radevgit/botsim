# botsim: rules and decisions

This file lists the rules the simulator and the bundled baseline controller
follow, and where a rule is a decision (marked **S&lt;n&gt;**, the numbered question
in the project journal) rather than something the standards prescribe. It is
meant to be kept up to date with the code and to be the source for a later
user document.

Standards: VDA 5050 version 3.0.1 (orders, state, instant actions) and VDMA
LIF 2.0 (layouts).

## 1. Time and the world

- Time is a fixed step (`time_step` in the scenario, in seconds). There is no
  default: the scenario must give it. Time is counted in steps, never read from
  the clock, so a run is repeatable.
- One step: the controller reads states and sends orders, then every robot
  moves (in the order they were added), then collisions are checked.
- Message timestamps are simulated time on top of the scenario's `start_time`
  (UTC). Each robot numbers its state messages 0, 1, 2, ….
- A robot is identified by `manufacturer` + `serialNumber`, as in the VDA 5050
  topic. Serial numbers are `<serial_prefix>-001`, `-002`, ….
- A layout (LIF) can have several maps (levels); every node and every robot
  pose carries a `mapId`. Robots on different maps never collide.
  Lifts and doors are not simulated yet.

## 2. Robot motion

- Only differential-drive robots are supported so far (others are refused).
- The robot's position is its kinematic center. Its body (a rectangle or a
  circle) is taken from the factsheet and turns with the heading.
- Speed and acceleration limits come from the factsheet. If the factsheet has
  no maximum angular speed, 1.0 rad/s is used (**S7**).
- On an edge the robot accelerates, cruises and brakes to stop at the end
  node. It **stops at every node** (no driving through nodes at speed yet).
- At a node it turns on the spot to the heading of the next edge, then drives.
- Heading on an edge (**S9**, **S13**): no orientation given → always drive
  forwards. Orientation 0 → forwards, π → backwards (3.14 is accepted for π,
  tolerance 0.01 rad). Any other value is refused (`ORIENTATION_UNREACHABLE`).
- Edges are straight lines; their length is the distance between the nodes.

## 3. Orders (VDA 5050)

- Sequence numbers are shared by nodes and edges: node 0, edge 1, node 2, ….
  The first node of an order is always released. After an unreleased element
  nothing released may follow. The released part is the *base*, the rest the
  *horizon*.
- The robot drives the base only. At the end of the base it waits; an order
  *update* (same `orderId`, higher `orderUpdateId`, starting at the last base
  node) extends it.
- Acceptance order (Figure 8 of the standard):
  1. the order must be valid, else `VALIDATION_FAILURE` or `NO_ROUTE_TO_TARGET`;
  2. a new `orderId` is accepted only if the robot is idle (else
     `OTHER_ORDER_ACTIVE`), `orderUpdateId` is 0, and the first node is where
     the robot stands within 0.1 m (else `START_NODE_OUT_OF_RANGE`);
  3. the same `orderId` is an update: lower id → `OUTDATED_ORDER_UPDATE`,
     same id → `SAME_ORDER_UPDATE_ID`, after a cancel →
     `ORDER_UPDATE_FOLLOWING_CANCEL`; it must start at the end of the base.
- A refused message changes nothing; its error stays in the state until a new
  order or update is accepted.
- An order is finished only when all its actions are finished.

## 4. Actions

- Node actions run when the robot reaches the node (the node counts as
  reached before its actions). Blocking types follow §6.2.2 of the standard:
  NONE (driving and parallel actions allowed), SOFT (parallel allowed, no
  driving), SINGLE (driving allowed, runs alone), HARD (neither).
- Actions take simulated time. `action_time` in the scenario is the default
  duration, `[robot.action_times]` sets it per action type. `pick` and `drop`
  take 5 s unless the scenario says otherwise (**S30**).
- Not yet: edge actions, failing or paused actions.

## 5. Instant actions

- `cancelOrder`: the robot brakes to the next node and stops; running and
  waiting actions become FAILED; the instant action is RUNNING until all
  movement has stopped, then FINISHED. Order ids are kept. With no active
  order or a wrong `orderId` it fails with `NO_ORDER_TO_CANCEL`.
- Every other instant action fails with `INVALID_INSTANT_ACTION`.
- Simplification: every action can be cancelled.

## 6. State message

- Built from the robot's real state and checked in the tests against the
  official VDA 5050 state schema.
- `nodeStates` / `edgeStates` list everything still ahead (base and horizon);
  the first node of an order is never listed; `lastNodeId` is the last node
  reached. `driving` is true while driving or turning on the spot.
- Placeholders: battery 100 % (permanently powered robot), operating mode
  AUTOMATIC, no emergency stop, no loads.
- Errors use the 28 predefined error types of the standard (Table 9), as an
  enumeration (**S20**).

## 7. Collisions

- After every step each pair of robots on the same map is tested for overlap
  (rectangle or circle, filled; **touching counts as overlap**).
- A collision is recorded when two robots *start* to overlap, once per
  contact; they must separate before a new contact counts again.
- Robots do not react to collisions (no onboard safety stop yet). The count
  must be 0 for a good controller.

## 8. Jobs

- A job moves something from a pickup node to a drop node (**S28**): the
  robot drives to the pickup node, runs `pick`, drives to the drop node, runs
  `drop`.
- Jobs are created at random with a fixed average rate per hour between the
  nodes listed in the scenario's `[jobs]` section. Pickup and drop differ.
  The seed in the scenario fixes the sequence; the job random stream is
  separate from any future random feature.
- At most one job per step is created.

## 9. Baseline controller

The bundled controller talks to the robots only with VDA 5050 messages.

- **Free robot**: no job; it stays where it is (**S31**). Its position is the
  layout node nearest to its reported position, within 0.1 m.
- **Assignment**: jobs are served first come first served; each job goes to
  the nearest free robot (shortest route length to the pickup; a tie goes to
  the robot added first). A job no free robot can reach waits and does not
  block later jobs.
- **Route**: shortest route by edge length (edges are one-way as in the
  layout).
- **Release window** (**S29**, changed to 2 by the user): the robot's node
  and the next **2** nodes are released; each time the robot reaches a node the
  base is extended by an order update.
- **Reservations** (**S34**): a node, and the edge leading to it, are reserved
  for a robot from the moment they are released to it until the robot has
  driven on to the next node; a robot is released only onto nodes and edges
  nobody else holds. A robot standing still holds its node. If the way is
  not free the robot waits at the end of its released route and the
  controller tries again every cycle. A node the route passes again later
  (e.g. a second lap of a ring) stays reserved while it is still ahead.
  Consequence of "stays where it is" (S31): a robot that has finished a job
  keeps its node reserved, so it can block others until it gets a new job.
- Not yet: look-ahead beyond the window, timeouts and re-routing, robot types
  and per-type edge rules.

## 10. Metrics

- **Throughput**: finished jobs per simulated hour.
- **Waiting** (**S32**): the time a robot stands still while it has a job and
  is not busy with an action (`pick`, `drop`, …); per job, reported as average
  and maximum over finished jobs. Jobs still running at the end are not
  counted.
- **Collisions**: see section 7.
- **Deadlocks** (**S35**): the simulator only observes and reports; resolving
  them is the controller's job. Robot A *waits for* robot B when A stands
  still with part of its order still ahead, is not busy with an action, and B
  stands on A's next node (within 0.1 m). A deadlock is a cycle of such waits
  (two robots head-on, or a longer ring) that lasts at least the timeout —
  30 s, or `deadlock_timeout` in the scenario. Each deadlock is reported once
  with its robots, when it formed, when it was reported, when it ended (if it
  did) and its duration. Limits: cycles through a node that is reserved but
  empty are not seen, and a robot waiting for a robot that is not itself
  waiting (e.g. one idle for ever) is not a deadlock by this rule.
- Not yet: controller time per step; a debug option to stop at the first
  deadlock.

## 11. Scenario file and result

- One TOML file: name, seed, duration, `time_step`, `start_time`, the layout
  (`lif`), robot groups (`[[robot]]`) and optional `[jobs]`. Paths are
  relative to the scenario file; unknown keys are errors.
- Optional `deadlock_timeout` (seconds, default 30).
- Result: a JSON summary on standard output (scenario, seed, ticks, jobs,
  deadlocks,
  metrics, collisions, final position of each robot). It has no clock times or
  file paths, so the same scenario gives the same bytes.
- Exit codes: 0 done, 1 error (message on standard error), 2 wrong command line.

## 12. Known simplifications (not modelled yet)

Stops at every node; differential drive only; straight edges only; one map
identified by `mapId` only; rotation restrictions at nodes; node orientation
and deviation; edge actions; other instant actions; operating modes; state
message timing; horizon actions; lifts and doors; disturbances; deadlock
resolution (controller side); the Rerun recording.

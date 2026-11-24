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
  pose carries a `mapId`. Robots on different maps never collide. Lifts are
  simulated (§7f, §7g); doors on layout edges are separate peripherals (§7h).

## 1a. Message delay (**S58**)

- Without a delay (the default) the controller and the robots exchange messages
  within the step: an order is received at once and the robot answers at once;
  the controller reads the robot's state as it is now.
- With `message_delay` (scenario, seconds, rounded to whole steps) every message
  takes that long **in each direction**: orders, instant actions and responses
  arrive at the robot that many steps after they were sent (in the order they
  were sent), and the state messages the controller reads are that many steps
  old. Before the first state has "arrived" the controller sees the first state.
  A round trip (an order and the state that shows it) takes twice the delay.
- **The robot's answer to an order is not returned any more:** the sender gets
  "in transit" and learns the outcome from the state — the `orderId` of the
  state, or an error that names the order (as in VDA 5050).
- Only the VDA 5050 messages are delayed. **Lift and door commands and what the
  controller reads from lifts and doors are instant**, and so is everything the
  simulator itself looks at (collisions, deadlocks, the observer, metrics).
- **Baseline:** it sends a job's order and goes on as if it were accepted; if the
  state later shows an error that names the order, it gives the job up (job
  refused, what was reserved for it is released, a lift or door it was given is
  given back). A refused order *update* is not detected with a delay (the
  baseline only sends updates its robots can follow). Releases, re-routing and the
  end of a job are all decided on the old state, so a robot may reach the end of
  its released route before the next update arrives and wait there.
- Not yet: a different delay per direction or per robot, random delays or lost
  messages, a delay for the lift and door commands.

## 1b. The controller's messages (VDA 5050)

The controller sees the robots only through these messages (S61 lists what is
and is not a message yet):

| Direction | Message | State |
|---|---|---|
| to the robot | `order` | done (§3) |
| to the robot | `instantActions` | done, `cancelOrder` only (§3) |
| to the robot | `responses` (answers to zone requests) | done (§7e) |
| to the robot | `zoneSet` + instant actions `enableZoneSet` / `deleteZoneSet` | done (**S62**, §7c): the scenario's zone set file is the *site's* zone set; the baseline controller sends it to every robot and enables it |
| from the robot | `state` | done (§3); delayed with a message delay (§1a) |
| from the robot | `factsheet` | done (**S61**): `World::factsheet(id)` is the factsheet the robot's spec was made from, with the robot's own header (manufacturer, serial number, protocol version, time now, `headerId` 0); not delayed. The baseline gets each robot's type (`manufacturer`, `seriesName`) from it |
| from the robot | `visualization` | **not planned** (S66 discussion): it is the standard's optional topic for a high-rate position and path feed to outside visualization tools (never used by fleet control for logic). Viewers and recorders use the observer port (§7b), which shows the whole world; the message would only matter once an MQTT adapter feeds an outside tool |

Lift and door commands are not VDA 5050; they have their own ports (§7f, §7h).

## 2. Robot motion

- **Differential**, **omnidirectional** (**S63**) and **three-wheel** (car-like,
  **S70**) robots are supported.
- The robot's position is its kinematic center. Its body (a rectangle or a
  circle) is taken from the factsheet and turns with the heading.
- Speed and acceleration limits come from the factsheet. If the factsheet has
  no maximum angular speed, 1.0 rad/s is used (**S7**).
- On an edge the robot accelerates, cruises and brakes to stop at the end
  node, unless it drives through the node (next point).
- **Driving through nodes (S66).** With `drive_through_nodes` (scenario, default
  `true`; the core world has it off until `World::set_pass_through(true)`) a robot
  does **not** stop at a node when
  - the next edge is part of the **released** route (the end of the base, with
    the horizon behind it, is a stop),
  - nothing is to be done at the node (**no actions**: `pick`, `drop`, … stop it),
  - it goes **straight on** (same direction) and needs no turn (a corner, or an
    orientation it does not have yet, means a stop and a turn),
  - the next edge is not one it must not start yet (a RELEASE zone not granted, a
    door that is not open), and is not a **door's edge** or a **lift ride** (it
    stops in front of those, so the door or lift can be seen ready from a
    standstill),
  - and it is not being stopped (hold, zone release lost, safety stop, a
    cancelled order).
  Its speed at the node is then the lowest of the two edges' speed limits, and low
  enough to stop at the end of the next edge. The node counts as reached when the
  robot gets to it (the state's `lastNodeId` changes then), so the controller's
  release window (2 nodes ahead) is wide enough for it to keep going; with late
  orders it stops at the end of what it has and goes on when the update arrives.
  - Simplifications: the distance left over in the step in which a node is
    reached is dropped (at most one step's travel, 7.5 cm at 1.5 m/s); a
    `cancelOrder` while driving through a node cannot always stop at the next
    node within the deceleration limit (the speed then drops to 0 at that node);
    turning corners without stopping (blending) is not modelled.
- At a node it turns on the spot to the heading of the next edge, then drives.
- Heading on an edge (**S9**, **S13**): no orientation given → always drive
  forwards. Orientation 0 → forwards, π → backwards (3.14 is accepted for π,
  tolerance 0.01 rad). Any other value is refused (`ORIENTATION_UNREACHABLE`).
- **Rotation allowed at nodes (S64).** The LIF can say, per edge and robot type,
  which ways a robot may turn on the spot at the edge's start node
  (`rotationAtStartNodeAllowed`) and end node (`rotationAtEndNodeAllowed`):
  `NONE`, `CCW`, `CW` or `BOTH` (default). At a node where a robot comes in on one
  edge and leaves on another, the two rules are **combined as a boolean and**
  (LIF §8.3.13.1): `BOTH` + `CCW` = `CCW`, `CW` + `CCW` = nothing. For the first
  edge of an order only its start node rule counts (how the robot came is not
  known); an order update continues from the last edge of what was planned.
  - When the robot has to turn (its heading is not the heading the next edge
    needs), it takes the **shorter way if it is allowed**, else the **other way
    round** (up to 270°), and if no direction is allowed the **order (or update)
    is refused** (`NO_ROUTE_TO_TARGET`, the text names the edge and the node) —
    when it is received, not when the robot gets there.
  - Applies to differential robots and to omnidirectional robots that must turn
    (a named orientation); an omnidirectional robot without orientation keeps its
    heading and never needs a turn (§ above). A lift ride resets the rule (the
    robot is in a cabin).
  - **The baseline controller plans around these rules (S73, §9):** it does not
    send a route that needs a turn the robot cannot make.
  - **A planner can ask the layout (S71):** `Graph::can_follow(prev, next, kinematics,
    manufacturer, series)` says whether a robot that has driven edge `prev` can drive
    edge `next` (no orientation given), and `Graph::can_start(heading, end_rotation,
    edge, …)` the same for the first edge from a known heading. They apply the same
    rules as the robot (rotation rules for a differential robot, "no turn on the
    spot" for a three-wheel robot, nothing for an omnidirectional one, §2) and are
    tested against the robot's own acceptance of an order. The baseline uses them
    (S73).
  - Not yet: `maximumRotationSpeed` per edge; rotation while driving
    (`reachOrientationBeforeEntering = false`).
- **Omnidirectional robots (S63).** Heading and travel direction are
  independent: the robot travels along the edge whatever it faces.
  - With **no orientation** in the order it **keeps its heading** (the standard
    lets the robot choose; for a differential robot S13 says "forwards"). A robot
    that never turns is faster than a differential robot on a route with corners.
  - With an orientation: `TANGENTIAL` is relative to the edge direction (any
    value, not only 0 and π); `GLOBAL` is relative to the map (valid for
    omnidirectional robots only). The robot **reaches the orientation before it
    enters the edge** (turning on the spot at the node) and keeps it. The LIF
    option `reachOrientationBeforeEntering = false` (turn while driving) is not
    modelled.
  - The state's `velocity` has `vx` and `vy` in the robot frame (x forward, y
    left): the speed along the edge split by the angle between travel and
    heading, so a robot facing east and driving north has `vy` = speed. The
    observer's `speed` is the speed over the ground for a robot that moves
    sideways (the signed speed along the heading otherwise).
  - The protective field (§7a) looks the way the robot travels, so it also works
    for sideways travel. Not modelled: the width of the body across the travel
    direction for a door (§7h) is taken across the heading; accelerations are
    the same in every direction.
- **Curved edges (S69).** An edge is a straight line unless the LIF gives the
  robot's type a **`trajectory`**: a NURBS curve (`degree`, default 1; `knotVector`,
  default uniform and clamped; `controlPoints` with `weight`, default 1) in the edge's
  properties for that type. So the same edge can be straight for one type and curved
  for another.
  - The layout is checked when it is read: a valid curve (degree ≥ 1, at least
    degree + 1 finite control points, weights above 0, a knot vector of the right
    length that does not decrease) that **starts at the start node and ends at the
    end node** of its edge (5 cm tolerance); otherwise an error naming the edge.
  - The curve is **sampled every 2 cm** into a path with a length; the robot
    follows it, and the trip is as long as the curve. The edge's `length` in the
    graph stays the straight distance between the nodes; the baseline's **route
    costs** use the length of the curve for the robot's type (S74,
    `Graph::edge_travel_length`).
  - **Heading follows the direction of the path** for a differential robot (forwards;
    backwards with orientation π, then the heading is against the direction) and
    for an omnidirectional robot with a tangential orientation (that angle to the
    direction). At a node it turns on the spot to the direction the curve **starts**
    in, as for a straight edge; on the curve it turns with it, and ends with the
    direction the curve **ends** in. A global orientation, or none for an
    omnidirectional robot (§ above), keeps the heading fixed: the robot then drives
    the curve sideways. The state's `omega` is the speed times the curvature.
  - **Driving through nodes** (§ above) works where the end direction of one edge is
    the start direction of the next (the curves join smoothly); a kink is a stop and
    a turn.
  - The protective field (§7a) and the checks that look the way the robot goes use the
    direction of the path where the robot is.
  - **Speed on a curve (S70):** where the heading follows the path, the robot turns at
    speed × curvature, which may not exceed its maximum angular speed. So on a curve
    the speed is at most (maximum angular speed) / (sharpest curvature of the edge),
    one limit for the whole edge, besides the robot's and the edge's own limits.
    This holds for every kinematics (a differential robot slows down in a tight curve
    too). The LIF `corridor` is not read: the robot follows the curve.
  - **Three-wheel (car-like) robots (S70).** The factsheet has no field for a turning
    radius, so these limits are the simulator's own choice:
    - It **cannot turn on the spot**. An edge must start in the direction the robot
      heads at that node (a little less than 1 degree, 0.02 rad, off counts as aligned
      and is set right). If not, the order is refused (`NO_ROUTE_TO_TARGET`, with a
      message that names the edge and the node). Rotation rules at nodes (S64) do not
      matter for it. A robot standing on a node must therefore already face the way
      its first edge starts.
    - It **may drive backwards** (orientation π on the edge), but only if it already
      heads that way: turning around is turning on the spot.
    - A curve is refused if the speed limit above is lower than the **lowest speed
      the robot can drive** (factsheet `minimumSpeed`): the smallest radius is
      `minimumSpeed / maximumAngularSpeed`. With the factsheet's usual values this is
      small (0.1 m/s and 1 rad/s give 0.1 m); a factsheet with a high minimum speed
      and a low angular speed gives a larger radius.
    - **The baseline plans around the turns** (S73, §9). What it does not know is
      how tight a curve may be (the limit above): if the robot refuses an order
      (the first one: the job is listed as rejected and not tried again) or an order
      *update* (the baseline then sends `cancelOrder`, keeps what the robot holds
      until it has stopped, it brakes to the next node, and lists the job as rejected
      with the robot's error, `NO_ROUTE_TO_TARGET`), the job is given up. The same
      happens if a robot refuses a changed route (re-routing, §9). With a message
      delay a refused update is not seen (§1a). A layout for car-like robots needs
      rounded corners (trajectories); where there is no route without a sharp turn,
      the job waits in the queue.
  - **No panic (project rule):** a robot refusing an order is a result of the
    scenario, not a fault of the simulator; the controller must not stop the run
    for it.
  - **Fix found on the way (S70):** an order update that continued a curved edge
    started from the *start* heading of that edge, not from the heading the curve ends
    with, so the first edge of the update could be refused or turned for nothing.
- Edges are straight lines (without a trajectory); their length is the distance
  between the nodes.
- **Robot types in the layout (S55).** The LIF lists, for every node and edge,
  the robot types (`manufacturer` + `seriesName`) that may use it
  (`mobileRobotTypeNodeProperties`, `mobileRobotTypeEdgeProperties`; "a robot
  type without an entry may not use the node/edge"). A robot's type is the one in
  its factsheet. So:
  - A robot **cannot be placed** on a node its type may not use (the scenario
    names the node and the type).
  - An order with a **node or edge the type may not use** is refused
    (`NO_ROUTE_TO_TARGET`, the nearest predefined error; the text names the node
    or edge). An edge may be used only if it lists the type **and** both its end
    nodes do.
  - **Speed limit:** an edge's `maximumSpeed` for the robot's type limits the
    speed on that edge (the robot's own limit still applies; it brakes to stop at
    the end node as always). A `maximumSpeed` of 0 or less is an error when the
    layout is read (a robot could never drive the edge).
  - **Baseline:** routes are planned per robot type, over the edges the type may
    use; a job goes to the nearest free robot that has a route for it; a job no
    robot type can do waits (it is not refused) (§9).
  - The test layouts therefore list the robot type used in the tests (the
    official LIF example 14 names `Manufacturer_A` / `Series_1`; the copies used
    in tests were changed to `ACME` / `X1`).
  - Not yet: the other per-type properties (own trajectory and corridor,
    orientations and rotation rules, `maximumRotationSpeed`, load restrictions,
    required and forbidden actions); travel time (not length) as the route cost.

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
- Robots do not react to a collision itself; they brake *before* one with the
  onboard safety stop (section 7a). The count must be 0 for a good controller.

## 7a. Onboard safety stop (**S54**)

- Every robot has a **protective field** ahead of it, in the direction of
  **travel** (the direction of the edge, not the heading: a robot driving
  backwards looks backwards). If the body of **another robot** is in the field,
  the robot **brakes with its maximum deceleration and stands still** until the
  field is clear; then it goes on with its order. A robot that is turning or
  standing at a node has no field.
- **Field:** a rectangle that starts at the front of the robot's body and is as
  wide as the body plus **0.1 m** on each side. Its length is the braking
  distance at the present speed (`v² / (2 d)`) plus the distance covered in one
  time step plus a margin of **0.3 m**. The margins are our choice, not from a
  standard.
- The robot reports a **protective field violation** in its state while it is
  stopped by the field. Like a random stop (S37), this **does not count as
  waiting** in the waiting metric (the controller sees only the state). The
  world's own deadlock check does count it: a robot stopped by another robot
  waits for that robot, so two robots stopped head-on are a deadlock.
- The check uses the same broad phase as the collision check (§13) and the
  positions after each step, so a robot reacts one step (0.05 s) later; the
  length of one step is part of the field for that reason.
- **Switch:** scenario key `safety_stop` (default `true`); `false` lets robots
  drive into each other, which is what the collision tests need. In the core
  library the world has it off until `set_safety_stop(true)`.
- Only robots are obstacles (no walls or people yet). Observer status
  `Obstructed`.
- **Meaning for the collision count:** with the stop on, a controller that
  releases a robot onto a node another robot stands on makes the robot stop
  (waiting, perhaps a deadlock) instead of colliding; collisions then only show
  modelling gaps, as the brief says (edges that cross without a node, a robot
  placed on another, nodes closer than a robot is long).
- Not yet: the field depending on the load, people, walls, a speed reduction
  before stopping (the robot brakes at full deceleration when the field is
  blocked, and only when the other robot is already inside it).

## 7b. Observer port (for viewers and recorders)

- The simulator has a **read-only observer port**, so a viewer or recording
  file can be added later without tying the simulator to any particular
  library (**U5**: no Rerun for now). The core never depends on a viewer.
- When an observer is attached it gets the **layout** once (maps, nodes, edges)
  and **snapshots**, after every step or, like a frame rate, only every chosen
  interval of simulated time (rounded to whole steps; the snapshot is only
  built when due; what happens between two snapshots is not seen by the
  observer, but the simulator still counts it): the time; for every robot its id, map,
  pose, body outline, status (idle, driving, turning, acting, waiting for
  release, held by a disturbance), speed, order id, last node, and the nodes and
  edges still ahead split into **released** and **planned** (horizon); the
  controller's **reservations**; **who blocks whom** (the wait-for rule of
  section 10); robot pairs that **overlap**; and the **deadlocks** that last.
- The data is plain (no file format). Observing changes nothing: a run with and
  without an observer gives the same result, and the same run gives the same
  snapshots. Without an observer no snapshot is built.
- The baseline controller publishes its reservations to the observer; any
  controller can do the same.
- Lifts and doors are in the snapshot (state, queue, edges). Not in it yet:
  stations, zones (they are not part of the layout the observer gets), RELEASE
  zone requests. A recording file or a viewer will be an adapter later.

## 7c. Zones (VDA 5050 zone set)

- A scenario can name a **zone set** file (`zone_set`): a JSON file with the
  VDA 5050 `zoneSet` object (`mapId`, `zoneSetId`, `zones`), or a whole
  `zoneSet` message (its header is ignored). All VDA 5050 zone types are read.
- It is **loaded and checked** when the world is built. Only RELEASE zones have
  behaviour so far (section 7e); the other types are loaded and listed only. The
  result lists the zones and their types.
- Checks (**S40**): the map is a map of the layout; zone ids are unique; every
  polygon has at least 3 finite vertices, no repeated vertex in a row, no
  crossing or touching sides, and an area; `SPEED_LIMIT` speed > 0,
  `PRIORITY` and `PENALTY` factors from 0 to 1, directions finite.
- A zone is a filled shape; a point on its border is inside. Concave polygons
  are fine.
- One zone set is for one map; several maps would need one zone set each (not
  supported yet).
- **Zone sets are messages (S62).** The scenario's `zone_set` is the zone set of
  the **site**: the world holds it and the controller reads it; the **robots do
  not have it** until the controller gives it to them, as in VDA 5050 §6.4.2:
  - the **`zoneSet` message** (to one robot): the robot checks it against the
    layout (§7c checks) and stores it **disabled**. A zone set with a
    `zoneSetId` the robot already has is **not taken over**: error
    `DUPLICATE_ZONE_SET` (WARNING, reference `zoneSetId`), reported in the state
    for **10 s** ("a reasonable amount of time", our choice). A zone set that does
    not fit is refused: unknown map → `UNKNOWN_MAP_ID`, any other fault →
    `VALIDATION_FAILURE` (reference `zoneSetId`), also for 10 s;
  - the **instant action `enableZoneSet`** (parameter `zoneSetId`): the zone set is
    enabled and every other one disabled; from then on the robot states RELEASE
    zone requests (§7e) for its zones, also for an order that is under way.
    `deleteZoneSet` removes a stored zone set that is not enabled. Both finish at
    once; they fail (`INVALID_INSTANT_ACTION`, reference `actionId`) if the robot
    has no such zone set (or it is enabled);
  - the robot's **state** has `zoneSets` (`zoneSetId`, `mapId`, `zoneSetStatus`
    ENABLED/DISABLED) while it stores any, and no `zoneRequests` until a zone set
    is enabled.
  - **Simplifications:** at most **one zone set is enabled at a time**, whatever
    the map (the standard: one per map); no `downloadZoneSet`; the content of a zone
    set with an id is never compared.
  - `World::set_zones` still gives a zone set to all robots already enabled (a
    shortcut for tests); the scenario uses `World::set_site_zones`.
  - **Baseline:** for each robot it sends the `zoneSet` message once; when the
    state shows the set (disabled) it sends `enableZoneSet` once; it gives a robot
    **no job until the state shows the zone set enabled** (a robot without its
    zones would drive into RELEASE zones without asking). With a message delay
    that takes a few round trips (§1a). The first jobs therefore start a few
    cycles later than before S62.

## 7e. RELEASE zone requests (robot side) and the baseline's answers

- Before an edge that **enters** a RELEASE zone the robot adds a `zoneRequest`
  to its state: `requestType` ACCESS, `requestStatus` REQUESTED, a `requestId`
  unique per robot (`req-1`, `req-2`, …), the `zoneId` and `zoneSetId`. A request
  is needed even when the edge is released. With zones loaded the state always
  has a `zoneRequests` array (empty if none); without zones it has none.
- An edge **enters** a zone if it starts outside the zone and reaches its
  inside or border (edges are straight). An edge that starts inside the zone, or
  passes beside it, does not enter it. Only RELEASE zones are requested.
  All zone sets count as enabled from the start.
- **When to request** (**S41**; the standard leaves it to the robot, so it is a
  setting per robot group, `zone_requests` in `[[robot]]`; a real fleet contains
  both kinds, so the controller should be tested against both):
  - `"on_release"` (default): when the entering edge is **released**. Only
    robots that are really coming ask. Risk: the robot reaches the zone before
    the answer, and has to stop.
  - `"on_horizon"`: as soon as the entering edge is **in the order**, also as
    horizon. The controller sees demand early (e.g. to call a lift ahead of
    time); but the order may still change, so requests can go stale and hold up
    grants others need.
- The request stays while the robot is inside the zone or an edge ahead still
  enters it (by the timing rule). After the robot has left the zone, or when no
  edge ahead enters it any more, the request is removed. A zone the robot has
  already crossed on the edge it is still driving is not requested again.
- **Answers and waiting** (**S42**): the controller answers in a `responses`
  message. GRANTED → status GRANTED, the robot may enter. QUEUED → status QUEUED,
  the robot still waits (it changes nothing for a request that is granted,
  revoked or expired). REJECTED → the robot stays out and **removes the request**
  (VDA 5050 §6.9); while it still wants the zone it states a new request (new id)
  at once; the standard leaves retry delays to the integration. An answer to a
  request the robot does not have is ignored.
- **Lease, REVOKED and release loss** (**S43**): a GRANTED answer can carry a
  `leaseExpiry` (ISO 8601 UTC; an unreadable one counts as no lease; no lease =
  valid for ever); a later GRANTED answer with a new one extends it. When the
  lease runs out, or the controller answers REVOKED:
  - **before the robot has entered the zone**: it must not enter. It removes the
    request; if it is already on the entering edge it **brakes and stands still**
    outside the zone (with its maximum deceleration; it cannot stop in less
    than its braking distance); it states a new request and goes on when that is
    granted;
  - **inside the zone** the robot has **lost its release**. The request stays in
    the state with status REVOKED or EXPIRED until the zone is left or the release
    is granted again, and the zone's `releaseLossBehavior` applies (STOP if the
    zone has none):
    - `STOP`: the robot stops driving and stands still; error
      `ZONE_RELEASE_LOST` (CRITICAL) with references `zoneId` and `requestId`,
      until it is granted again (it then drives on) or has left the zone;
    - `CONTINUE`: it drives on along its order; error
      `ZONE_RELEASE_LOSS_HANDLING` (WARNING) until it has left the zone or the
      order has ended inside it;
    - `EVACUATE`: **simplification:** our robot has no evacuation behaviour of
      its own, so it does the same as CONTINUE.
  - A robot that brakes after losing the release and rolls out of the zone
    while braking has left it: the loss is over and it drives on.
  - The baseline controller never revokes and gives no lease (it grants for
    good); the behaviour above is tested by sending the answers by hand.
- **The robot does not start an edge that enters a RELEASE zone unless the
  request for it is GRANTED.** It may turn towards the edge first, but then it
  stands still on the **start node of the entering edge** (reported as not
  driving; it counts as waiting in the waiting metric). A robot that never gets
  an answer waits for ever.
- **Baseline controller** (exit before enter, see 7d): a zone is granted to
  **one robot at a time**, first come first served in the order requests were
  first seen. A robot that has to wait is told QUEUED once, and GRANTED when it
  is its turn. A zone is not free while another robot holds a grant for it (until
  its request is gone, i.e. it has left the zone) or stands inside it without a
  grant. A request for another zone is not held back by one that has to wait.
  Grants have no lease and are never revoked.
- **Not yet:** the requests in the observer snapshot, COORDINATED_REPLANNING, a
  retry delay after a rejection, the layout check that a waiting node lies
  outside the approach (7d), a robot-specific evacuation behaviour.

## 7d. Design rule for RELEASE zones: where a robot waits

(Domain rule given by the user; it is about how layouts and zones are made, and
how the controller uses them. Not enforced by the simulator yet.)

- A robot that waits for access must stand **outside the single-lane stretch**
  that leads to the lift door (or to any shared narrow place). A robot coming
  out needs that same stretch, so anything parked on it blocks the exit:
  - waiting on the **last node before the door**: the robot coming out meets the
    waiting one head-on, and both are stuck (a deadlock);
  - waiting on the **node before that** works only if the robot coming out can
    turn off before reaching it; if the approach is one lane for several
    nodes, it is still blocked.
- **Rule:** wait on a **side waiting node** (a lay-by next to the approach) or
  before the **last junction**, so the way out stays free.
- **Mapping to our tools:** treat the cabin **plus its one-lane approach as one
  RELEASE zone**. The waiting robot asks for access and gets it only after the
  robot inside has left the zone ("exit before enter" as a zone rule). The
  layout has to provide the waiting node; the controller decides when to let the
  robot go.
- Consequence for the robot: it requests the zone before the edge that enters
  it, and waits on the node before that edge, so that node must lie **outside**
  the zone.

## 7f. Lifts (peripheral)

- A lift is a **peripheral** of the simulated world: not a robot and not part
  of VDA 5050 (the standard leaves talking to lifts and doors to the fleet
  control). It has its **own small command port** next to the VDA 5050 messages
  (**S44**). Robots ride it as described in §7g.
- **Lift:** a list of floors (the maps it stops at, lowest first, at least two,
  all different), one cabin, one or more doors (see Doors). The cabin passes one floor in
  `travel_seconds_per_floor`; the door goes closed → opening
  (`door_open_seconds`) → open → closing (`door_close_seconds`) → closed. The
  cabin only moves with the door closed. Times are quantised to the simulation
  step.
- **Commands:**
  - `call(floor)`: bring the cabin to that floor and open the door. Calls are
    served **first come first served**, in the order they came (no reordering to
    save travel). A call for the floor the cabin is at with the door open or
    opening, or going to, needs nothing; the same floor twice in a row in the
    queue is queued once.
  - `release()`: "I am done here": an **open door** is closed. Nothing happens if
    the door is not open. **A queued call waits until the open door is released**
    (or closes by itself, see next); a door that nobody releases blocks the lift.
  - Option `door_hold_seconds`: the door closes by itself after being open that
    long. Without it the door stays open until released.
- **Doors (S45):** a cabin has one or more named **doors** (default one, `front`;
  a through-cabin has e.g. `front` and `rear`). Every door can open at every
  floor, all have the same times, and **only one door is open at a time**.
  `call_door(floor, door)` opens a chosen door; `call(floor)` is the first door.
  A call is a (floor, door) pair: the same floor with another door is a **new
  call** that waits until the open door has been released and closed (no
  travel). Dedup rules above apply to the pair.
- **Status** (for the controller and the observer): floor, position in floors
  (1.5 = half way), door state, the door that is open (or used last), the floor
  it moves to, the calls waiting (floor + door).
- **Scenario:** `[[lift]]` sections (`id`, `floors` as `mapId`s, optional
  `doors`, optional `stops` (§7g), optional `start_floor`, the times, optional
  `door_hold_seconds`).
  Wrong lifts (also no door or twice the same door name) are refused with the
  lift named. The result lists each lift at the end of the run (floor map, door
  state, door name, queued calls).
- **Not yet:** doors as separate peripherals, several cabins in one shaft,
  different door sets per floor. Riding: see §7g.

## 7g. Riding a lift (S46)

- **Stops.** A lift lists its **stops**: a layout node at the cabin entrance
  with the floor (the map of that node) and the door that opens there. A node
  belongs to at most one lift. Scenario: `stops = [{ node = "N2" },
  { node = "N102", door = "rear", waiting = "W1" }]` (door default: the first
  door; `waiting` optional, see §9).
- **Ride.** A layout edge whose start and end are stops of one lift on
  different floors (the LIF lift edge, e.g. N2→N102) is a **ride**. An order
  may contain it only if it is one; any other edge to another map is refused as
  before (`NO_ROUTE_TO_TARGET`). A ride edge has no length and no driving.
- **Getting in.** A robot that arrives at the start node of a ride stands there
  (status `WaitingForLift`) until the cabin is **at that floor with the door of
  that stop fully open and nobody in the cabin**. Then it is in the cabin at once
  (`RidingLift`). Robots waiting for the same lift get in in the order they were
  added.
- **In the cabin.** The robot's reported pose and `lastNodeId` stay at the
  entrance until it gets out. It is **on no floor**: it cannot collide and
  nobody waits for it on its node (collisions, deadlock rule). The cabin holds
  **one robot** (simplification).
- **Moving the cabin is the controller's job** (own command port, §7f): the
  robot does not call, release or send the lift anywhere. A controller that
  forgets to send the cabin on leaves the robot in the cabin for ever.
- **Getting out.** The robot is out at once when the cabin is at the floor of
  the ride's **end** stop with that stop's door fully open: its pose becomes the
  end node (other map), the heading is kept (**simplification**: the robot
  leaves facing as it got in; no turning in the cabin), the actions of the end
  node start, and it goes on with its order.
- **cancelOrder** at the entrance ends the order there; in the cabin the robot
  cannot stop between floors, so it finishes the ride and stops at the end node
  (like braking to the next node while driving).
- **Waiting metric:** the robot does not drive during the whole ride, so by the
  S32 rule (standing still with a job, not doing an action) the **ride counts as
  waiting**.
- **Simplifications:** getting in and out takes no time and needs no space; one
  robot per cabin; no cabin size or door width; the robot is invisible while in
  the cabin (observer: pose unchanged, status `RidingLift`).
- The baseline controller uses lifts in jobs: see §9 (S47).

## 7h. Doors (peripheral)

- A door is a **peripheral** like a lift (§7f): not a robot, not part of VDA
  5050, with its **own command port** (**S50**). This section is the door model
  and what robots do at a door; the baseline controller uses doors as in §9
  (**S51**).
- **Door:** guards one or more **layout edges** (e.g. both directions of a
  passage). States: closed → opening (`open_seconds`) → open → closing
  (`close_seconds`) → closed. Times are quantised to the simulation step.
- **Commands:** `open()` — the controller wants the door open (a closed door
  opens; a closing door finishes closing and then opens again). `release()` —
  the controller is done; the door closes. Without a release it stays open.
  Option `hold_seconds`: the door releases itself after being open that long.
- **Safety:** a door **never closes while a robot is on one of its edges**
  (the robot is driving on it, having left the start node); it stays open until
  the robot has left the edge, then closes. A door that is released while a robot
  waits in front of it (has not started) does close.
- **Robots do not open doors.** A robot does not start an edge of a door unless
  the door is **fully open**: it turns towards the edge and waits on the start
  node (status `WaitingForZone`, the same wait as for an ungranted RELEASE zone;
  it counts as waiting under S32).
- **Passage width:** a robot wider than the door's `passage_width` (its body
  width across the direction of travel) cannot pass: an order with an edge of
  that door is refused (`NO_ROUTE_TO_TARGET`, as for any edge the robot cannot
  drive). No width = any robot.
- **Scenario:** `[[door]]` sections (`id`, `edges` as `edgeId`s, optional
  `passage_width`, `open_seconds`, `close_seconds`, optional `hold_seconds`).
  Wrong doors (unknown edge, an edge another door has, bad numbers, same id
  twice) are refused with the door named. The result lists each door at the end
  of the run (`doors` with `id`, `state`; absent without doors). The observer
  snapshot has the doors with their state and edges.
- **Not yet:** doors that open by themselves for a robot; doors on nodes;
  partly open doors; a lift whose cabin door is one of these doors.

## 8. Jobs

- A job moves something from a pickup node to a drop node (**S28**): the
  robot drives to the pickup node, runs `pick`, drives to the drop node, runs
  `drop`.
- Jobs are created at random with a fixed average rate per hour between the
  nodes listed in the scenario's `[jobs]` section. Pickup and drop differ.
  The seed in the scenario fixes the sequence; the job random stream is
  separate from any future random feature.
- At most one job per step is created.
- **Stations (S60).** The LIF can list stations: logical places where a robot
  interacts with the environment (`stationId`, optional name and type, and the
  `interactionNodeIds` where the interaction takes place). They are read into
  the layout (all layouts of the file; ids must be unique, every interaction
  node must exist, at least one). A job can go **between stations**: the
  scenario's `[jobs]` has `nodes = [...]` or `stations = [...]` (not both); with
  neither, jobs go between **all the stations of the layout**. A job goes from
  one station to a different one, and its pickup and drop are interaction nodes.
  - A station with **several interaction nodes**: the LIF leaves the choice to
    fleet control. Here the node is **drawn at random (seeded) for each job**,
    not chosen by distance (a simplification; stations with one node draw
    nothing, so a run between plain nodes gives the same jobs as before).
  - The station's name, type and height are not forwarded in the order (the
    LIF says they may be), and stations are not in the observer's snapshots or
    the result. Not yet: stations that are busy or hold loads, a choice of the
    interaction node by the controller, edges' load restrictions.

## 9. Baseline controller

The bundled controller talks to the robots only with VDA 5050 messages.

- **Free robot**: no job; it stays where it is (**S31**). Its position is the
  layout node nearest to its reported position, within 0.1 m.
- **Assignment**: jobs are served first come first served; each job goes to
  the nearest free robot (shortest route length to the pickup; a tie goes to
  the robot added first). A job no free robot can reach waits and does not
  block later jobs.
- **Route**: shortest route by edge length (edges are one-way as in the
  layout), **per robot type** (**S55**): only over edges the robot's type may use
  (§2). The nearest free robot is chosen by the route length of its own type.
  The robot type comes from the robot's factsheet message (§1b). An edge's
  speed limit is not part of the route cost (length only). **The length of an
  edge with a curve is the length of the curve** (S74), which can differ between
  types (a type may have a trajectory on the edge, another a straight line).
- **Route with load (S91, brief request of 2026-11-23):** the shortest route is
  the same every time, so an equally long parallel lane was never chosen. With
  the scenario setting `route_load_weight` (metres, default 0 = off, same routes as
  before) the cost of an edge, when a route is planned, is its length plus the
  weight for each **other** robot that holds the edge and for each that holds the
  node it ends at (so 0 to 2 times the weight). It applies when a job is assigned
  and in the timeout re-route (§9, S53), which can now find the empty lane. Load
  is the current reservations. **Waiting average (S93):** the setting
  `route_wait_weight` (metres, default 0) adds to an edge the weight times the
  moving average (time constant 60 s) of how many robots waited on the node the
  edge ends at (1.0 = one robot waiting all the time), so routes also avoid places
  far ahead where robots keep waiting. Both are off by default. Deterministic. Not
  applied to the choice of the nearest robot; the lift and door rules are unchanged.
- **Turns (S73):** for a type that cannot make every turn (a three-wheel robot,
  or a differential robot where the layout bars some rotation, §2) the route is
  searched over edges: an edge may follow another only if `Graph::can_follow` says the
  robot can. The first edge must suit the robot's heading from its state
  (`Graph::can_start`), the second part of a job (pickup to drop) the edge it arrives
  on at the pickup; re-routing (S53) does the same. Other types use the plain search
  by node, as before (no change in speed). The choice of the nearest robot stays by
  the node distance, as before; if that robot has no route, the next one is tried.
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
- **Lifts** (**S47**): a route with a lift ride (§7g) needs the lift. The
  controller moves the cabin with the lift's command port (§7f); the robot only
  drives and waits.
  - **One robot per lift at a time**, first come first served: the lift is
    *owned* by the robot that asks first.
  - **Where a robot waits (S49, design rule §7d):** a lift stop can name a
    **waiting node** (lay-by, scenario key `waiting`, on the stop's map, not the
    stop itself, off the lane to the stop). The route of a job is planned
    **through that node**: to the lay-by, back, to the entrance, the ride. Without
    a waiting node the robot waits on the node **before** the entrance. That
    node (or the lay-by) is the robot's **gate**. A robot that starts on the
    entrance, or on the lay-by, gets no detour.
  - A robot is **never released past its gate unless it owns the lift**. At the
    gate it asks: if the lift is free it owns it from then on and is released
    through the ride; otherwise it waits and asks again every cycle. (S47 asked
    when the window reached the ride, which could release a robot onto the
    entrance two nodes early; the gate rule fixes that.)
  - **A two-way lane without a lay-by deadlocks:** the waiting robot stands on the
    lane the robot coming out must leave by. The baseline does not detect or
    resolve it (tested: robots going opposite ways are stuck without a lay-by,
    both get through with one). The layout must provide the lay-by.
  - **Calling:** the cabin is called to the entrance (floor and door of the
    stop) and called again every cycle until the robot is in the cabin (the
    lift tells the controller who is in the cabin).
  - **Riding:** when the robot is in the cabin the door is released and the cabin
    is sent to the floor and door of the stop at the end of the ride.
  - **Exiting:** when the robot has got out, the door stays open until the robot
    has driven away from the end node (or its job is done); then the door is
    released and the lift is free for the next robot.
  - Not yet: several robots in a cabin, choosing between lifts, taking the lift
    back from a robot that never comes, parking the cabin somewhere.
- **Doors** (**S51**): a route with an edge through a door (§7h) needs the door
  open. The controller opens and releases it with the door's command port.
  - **One robot per door at a time**, first come first served. A robot asks when
    its release window would include a guarded edge: if the door is free it owns
    it and may be released through the edge; if another robot owns it, the robot
    is released only up to the **start node** of the edge and waits there.
  - The claim is **tentative**: if the robot cannot be released onto the edge's
    end node (somebody else holds it), the claim is dropped in the same cycle, so
    a robot that cannot use the door does not keep it shut for others.
  - **Opening:** asked for every cycle while the robot owns the door. **Passing:**
    when the robot has reached the end node of the edge (or its job is over) the
    door is released and is free for the next robot; the door itself waits until
    the robot has left the edge (§7h).
  - A robot wider than the door cannot pass; its order is refused and the job
    ends up in the refused jobs (no route around the door is looked for).
  - **Opposite directions through a door in a one-lane corridor get stuck:** each
    robot waits on the node the other needs (tested; no collision, no
    resolution), like a lift without a lay-by.
  - Not yet: robots sharing an open door, routes around a door that is too narrow.
- **Timeout re-routing** (**S53**; the brief's "timeout re-routing"). A robot that
  stands at the end of its released route, is waiting (not doing an action) and
  wants to go on, but cannot because another robot holds the next node or edge,
  is counted as *held up*. After **10 s** (`REROUTE_AFTER_SECONDS`, my choice,
  well before the 30 s of a reported deadlock) the controller looks for a new
  route for the rest of the job:
  - same places (the pickup and the drop, each with its action) and same actions,
    but **no node that another robot holds** (its node, or released to it);
  - sent as an **order update** from the robot's node that releases nothing new and
    replaces the unreleased part of the route; node and edge numbering and the
    action ids follow the new route (`pick-3`, `drop-4`);
  - not used for a job with a lift ride or a door edge, nor for a robot that owns a
    lift or a door (their rules and claims depend on the route), and the new route
    uses no lift ride or door edge;
  - if there is no way round (or the pickup or drop itself is held), nothing
    changes and the controller tries again after another 10 s.
  - **What it does and does not solve:** a robot held by an idle robot, or by a
    robot that stays long, goes round. In a dense one-way layout it often finds no
    way round, because the nodes around are held too. Measured (one-way grid,
    40 × 40, 600 s): 300 robots 137 → 212 jobs done, deadlocks 11 → 6; 500 robots
    64 → 121 jobs done, deadlocks 15 → 15. A one-way edge cannot be driven back,
    so a robot cannot "back off"; resolving the rest needs more (e.g. reserving
    more of the route ahead, or fewer robots per street).
- **Going first (S77, brief `baseline_deadlocks.md`, item 2).** A deadlock with a
  safety link: robot X stands at the end of its released route and waits for the
  next node, which robot H has released; H is stopped (speed under 0.05 m/s) and
  X is right in front of it: within 4 m and 60° of H's heading, heading the same
  way (within 45°). H cannot move until X does. After `GIVE_WAY_AFTER_SECONDS`
  (5 s, my choice, before the 10 s re-route) X **takes the node and the edge over**
  (`Reservations::take_over_node/edge`): X holds them, its release goes on as
  usual, and H gets them back when X lets go of them (and not if H gave them up
  itself). `Controller::give_ways()` counts it. **Not done head-on** (X heading
  against H): there both would run into each other; that needs a back-off to
  another node (not built). **Lane change / merge (brief update, X-011/O-018):**
  "same way" also holds when X's route after the node is the one H takes after it
  (`same_route_after`), so X on H's curve, heading across it, still goes first
  (warehouse 4 pair `[34,76]`; no unit test for the crossing geometry yet, and the
  other six pairs are not checked). With the same route X may stand up to 100°
  (not 60°) off H's heading: X-008 on node 137 beside O-015's curve is at 77°
  (warehouse 4, second example). The wider limits (100°, 5 m) also apply when X
  stands on a node H has released (`on_h_way`). Checked by hand against the
  4 deadlocks of the final snapshot of `data/warehouse4.jsonl` (the earlier 7 were
  from another run): X-008/O-015, X-011/O-018, O-014/O-001 (X on a node O-001
  holds) and Y-001/O-013 (the cycle behind Y-011; Y-011 itself is behind O-013 and
  correctly does not go first) are covered. **Turning clearance:** if X has to
  turn on the spot to drive off (its heading is more than 15° off the bearing to
  the node), it goes first only if its distance to H is larger than the sum of the
  two bodies' reach plus 0.1 m. **(Replaced in S86: the circle was too coarse;
  now `turn_is_clear` places both bodies like the world does, turns X in 5°
  steps by the shortest way to the start of the edge, and H is grown by 0.05 m;
  any overlap means no take-over. `Shape::reach` is gone.)** Reason: warehouse 4, 309 s, forklift Y-020 (2.4 × 1.4 m)
  turned 1.7 m from the stopped O-015 and hit it (the recording shape is
  `[cx, cy, half_length, half_width]`, so those are half sizes; there was no
  simulator bug). Where there is no clearance the stand-off stays (needs the
  back-off, not built). **H must be on its way (S84):** the node is in H's released
  route and H does not stand on it; a robot parked on a node (an idle robot holds
  its node) is never gone around, there X would drive into it. The turn is judged
  by the edge's start direction (`Graph::edge_directions`), not the bearing to
  the node, because a curve starts in another direction. **Heading (S86):** if X
  stands on a node H holds, it may head any way as long as it does not face H
  (90° or more off the bearing to H); the 46° case Y-015/Y-005 (limit 45°) was
  stuck on that. "X stands on H's way" means X's node is in H's released route,
  not that H holds it (X may have taken it over earlier; then H no longer held it
  and Y-015/Y-005 stayed stuck). Still unresolved, by design: Y-024/O-009
  (warehouse 4, 380 s on): O-009 must turn toward the edge, and the turn sweeps
  into the forklift's body 1.9 m away (the exact test says overlap); only H
  backing off would help (not built). **A node taken over is
  not released again** when X's route passed it earlier
  (`Reservations::is_taken_over_node/edge`); before, the release of the old visit
  gave the node back every tick and X took it over again every tick (170 take-overs
  in 400 s, now 12). Old text continues: a back-off to
  another node (not built). Test `give_way.rs`: O has a long edge released past the
  node where X stands and waits; both jobs are done, no deadlock, no collision. The
  rule is for the case of the brief; whether it frees the warehouse recordings is
  not measured yet.
- **Known limit: a cycle around a block (S67).** In a dense one-way grid (300
  robots on 40 × 40 nodes, 600 s) the baseline's deadlocks are all **cycles of four
  robots around one block** (two of eight): each robot stands on a corner and waits
  for the next robot's corner. Timeout re-routing is active (hundreds of
  re-routes) but cannot break such a cycle when the other exits of the corners are
  held too. Driving through nodes (§2) does not cause them; it only gets the
  system there sooner and more often (14 against 5 deadlocks, 224 against 192 jobs
  done). The baseline is "today's practice": it does not avoid them (an avoidance
  rule would have to keep a free place in every cycle); they are what a better
  controller is measured against. The result lists the re-routes (`jobs.rerouted`).
- **Known limit: a swap on a dead-end spur (found by the viewer session).** A
  robot in a bay on a two-way spur and a robot on the ring node in front of it, each
  needing the other's place, **deadlock head-on** (seen with only 6 robots). Timeout
  re-routing cannot help: there is no other way into a bay. Layouts avoid it with
  **one-way sidings** (as with the lay-by rule for lifts, §7d). Resolving it in the
  controller needs a **yield rule** (a robot steps aside to a free node and comes
  back); not built.
- Not yet: look-ahead beyond the window, robot types
  and per-type edge rules.

## 10. Metrics

- **Throughput**: finished jobs per simulated hour.
- **Waiting** (**S32**): the time a robot stands still while it has a job and
  is not busy with an action (`pick`, `drop`, …); per job, reported as average
  and maximum over finished jobs. Jobs still running at the end are not
  counted.
- **Collisions**: see section 7.
- **Deadlocks** (**S35**): the simulator only observes and reports; resolving
  them is the controller's job. Robot A *waits for* robot B when
  - (**reservation wait**) A stands still with part of its order still ahead, is
    not busy with an action, and B stands on A's next node (within 0.1 m) **or has
    that node released ahead of it** (reserved, though B is not there yet, **S76**);
  - (**safety wait**, **S76**) A is stopped by its protective field and B is the
    robot in the field (the lowest index if several).

  A deadlock is a cycle of such waits (two robots head-on, a longer ring, with
  either kind of link) that lasts at least the timeout — 30 s, or
  `deadlock_timeout` in the scenario. Each deadlock is reported once
  with its robots, when it formed, when it was reported, when it ended (if it
  did) and its duration. Before S76 only "B stands on the next node" was seen,
  and the freezes of the big layouts (a robot on a node waiting for nodes another
  robot has reserved, that robot stopped by the first one's body) were not
  reported (`deadlock_count` 0 with 80–105 of 112 robots frozen). Limits: one wait
  per robot (the first next node, or the robot in the field), and a robot waiting
  for a robot that is not itself waiting (e.g. one idle for ever) is not a
  deadlock by this rule.
- **Controller time per step** (**S39**): the real (wall-clock) time of every
  controller step: number of steps, total, mean and maximum. It differs from
  run to run, so it is reported **only with `--timing`**; without the flag the
  output of a scenario stays byte-identical. Only the controller is timed, not
  the simulation.
- Not yet: a debug option to stop at the first deadlock.

## 10b. Disturbances

- Optional `[disturbances]` section, seeded by the scenario seed, with an
  on/off switch (`enabled`, default true; `false` keeps the values in the file
  but turns them off).
- **Speed variation** (**S36**): each robot's top speed is lowered by a random
  share between 0 and `speed_variation` (0.05 = up to 5 %), fixed for the whole
  run. Acceleration and turning speed are unchanged. The result reports every
  robot's top speed for the run (`max_speed`).
- The random numbers have their own stream, so switching disturbances on or off
  never changes the jobs of a seed. One number is drawn per robot in the order
  of the scenario file.
- **Random short stops** (**S37**): a robot that is driving or turning is
  stopped for a random time between `stop_seconds_min` and `stop_seconds_max`
  (defaults 5 and 15 s), as if something stood in its way. `stops_per_hour`
  is the average number of stops per hour **of driving or turning** (a robot
  that stands still, or is stopped already, cannot be stopped); 0 = none.
  Each robot has its own random stream, so its stops do not depend on the other
  robots. While driving the robot brakes with its maximum deceleration; while
  turning it stops at once; actions go on running. It then continues with its
  order.
- A stopped robot reports a **protective field violation** in its state
  (`safetyState.fieldViolation`) and `driving` false once it stands still. It
  is **not counted as waiting** (section 10) and therefore not part of a
  deadlock cycle: the controller is not at fault.
- The result has the number of stops (`random_stops`).
- **Varying action durations** (**S38**): with `action_variation` (0.2 = up to
  20 %) every action takes its normal time (section 4) times
  `1 + u × action_variation`, with `u` drawn at random between 0 and 1 for each
  action. Durations only get longer. Each robot has its own random stream and
  each order its own part of it, so the durations of an order do not depend on
  what earlier orders did. 0 = none.
- Not yet: breakdowns.

## 11. Scenario file and result

- One TOML file: name, seed, duration, `time_step`, `start_time`, the layout
  (`lif`), robot groups (`[[robot]]`) and optional `[jobs]`. Paths are
  relative to the scenario file; unknown keys are errors.
- `zone_set` names the site's zone set file (§7c: the controller sends it to the
  robots).
- Optional `deadlock_timeout` (seconds, default 30), `safety_stop` (default
  true, §7a), `drive_through_nodes` (default true, §2) and `message_delay`
  (seconds, default 0, §1a).
- **Start heading (S96):** per robot group `start_heading` is a number (radians, 0 =
  east, the same for all, default 0) or `"road"`: each robot faces the start
  direction of the first edge in the layout that leaves its start node and that its
  type may drive (an edge between maps only if there is no other; no such edge:
  east). `start_headings = [...]` gives one heading for each start node instead
  (the count must match `start_nodes`). `botsim_scenario::road_heading` gives the
  heading for a node, for the checker.
- Optional `route_load_weight` and `route_wait_weight` (metres, default 0, §9
  "Route with load").
- **Debug option `stop_at_first_deadlock`** (**S59**, default `false`): the run
  ends in the step in which the first deadlock is reported (that is after the
  deadlock timeout, §10), so the situation can be looked at (who is in the
  cycle, where the robots stand). The result has `stopped_at_first_deadlock:
  true`, `ticks` is the number of steps actually run (it is the scenario's full
  number otherwise), the jobs and metrics are those of the shortened run, and the
  deadlock is listed as still lasting (`ended_time` null). Without a deadlock the
  run goes on to its end.
- Result: a JSON summary on standard output (scenario, seed, ticks, jobs,
  deadlocks,
  metrics, collisions, and for each robot its type (`manufacturer`,
  `series_name`, **S57**), the number of jobs it finished (`jobs_done`, 0 without
  jobs) and its final position). It has no clock times or
  file paths, so the same scenario gives the same bytes.
- Exit codes: 0 done, 1 error (message on standard error), 2 wrong command line.
- Command line: `botsim-runner [--timing] <scenario.toml>`.

## 12. Known simplifications (not modelled yet)

Three-wheel limits are the simulator's own choice (no turning radius in the
factsheet, §2), and the baseline does not know how tight a curve may be; a job
that no route without a forbidden turn can serve waits for ever (and is searched
for again every cycle); the nearest robot is chosen by route length by node, not
counting turns; no blending through corners; route costs are distances, not
travel times (a curve may be driven slower, §2); the LIF corridor; per-robot-type
edge properties of the LIF (paths, speeds, corridors); rotation restrictions at
nodes; node orientation and deviation; edge actions; other instant actions;
operating modes; state message timing (the state is sent when the controller asks,
not every 0.5 s, and a delay is the same for all messages, §1a); horizon actions;
breakdowns; walls and people as obstacles of the safety stop (§7a); deadlock
resolution beyond the baseline's re-routing (§9);
station properties beyond the interaction nodes (§8); the other per-type LIF edge properties (§2, rotation rules done, not planned around by the baseline); the viewer (the
Rerun recording was dropped, U5). Lifts hold one robot and doors are tied to
edges (§7g, §7h).

## 13. Speed and scale (**S52**)

The brief's target is about 1,000 robots. Measured with
`cargo run --release -p botsim-runner --example scale -- <robots> <side> <seconds> <jobs per robot per hour>`
(a one-way Manhattan grid with 6 m between nodes, 0.05 s steps, one thread):

| Robots | Grid | Before | After |
|---|---|---|---|
| 1000, idle | 50 × 50 | 65 ms per step | 3 ms |
| 1000, busy | 50 × 50 | 102 ms per step (0.5 × real time) | 5–17 ms (3 – 10 × real time) |
| 1000, busy | 100 × 100 | not measured | 11 ms (4.5 × real time) |
| 100 | 30 × 30 | 1.4 ms | 0.3 ms |

What was slow, and what was done:
- **Collisions:** every pair of robots was tested (500,000 pairs at 1,000 robots,
  18 ms). Now a **broad phase** with the `aabb` index (a bounding box per robot,
  rebuilt every step; robots in a lift cabin are left out; one tree for all
  floors, the floor is compared in the exact test) hands only robots with
  meeting boxes to the exact test. The boxes are padded by 1 µm so that touching
  robots are never missed. The result and its order are the same as before
  (tested against all pairs).
- **Which node a robot stands on** (controller, several times per cycle) scanned
  every node. Now a grid of 1 m cells (`NodeGrid`); same answer (tested).
- **Dispatch:** a path search per free robot per job. Now **one search backwards
  from the pickup** gives the route length from every node; only the chosen robot
  gets a plan. The nearest robot is chosen as before (equal distance, to a
  nanometre: the robot added first). A job whose drop cannot be reached from its
  pickup is recognised once and waits (before, it was retried against every free
  robot in every cycle). Note: the distance used to choose the robot is the plain
  shortest route; a detour through a lift waiting node (§9) is not counted.
- **Deadlock check:** "who stands on my next node" looked at every robot; now a
  grid of the robots' positions.

What is still the biggest cost at 1,000 robots (about 6 + 5 ms per step): the
VDA 5050 state message is built for every robot twice per step (once by the
controller, once by the deadlock check).

**Observer cost** (the `aabb` change does not touch the observer; measured
separately, 1,000 robots on 100 × 100, an observer that only counts): no
observer 5.4 ms per step; a snapshot every step (0.05 s) 7.1 ms (+1.7 ms);
every 0.5 s 5.7 ms; every 2 s about 5.7–6.3 ms (within the noise of the
measurement, ±0.5 ms). So a viewer should not take a snapshot every step at this
size; the snapshot interval (§7b) exists for that.

**Result worth knowing:** with 1,000 robots on 2,500 nodes (40 % of the nodes)
the baseline gridlocks: in 100 s of simulated time 6 of 1,659 jobs were done and
9 deadlocks were reported, with 0 collisions. The baseline reserves nodes first
come first served and cannot resolve a deadlock. At 1,000 robots on 10,000 nodes
no deadlock was reported in 120 s.

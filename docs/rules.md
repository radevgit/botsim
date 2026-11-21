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
- Optional `deadlock_timeout` (seconds, default 30).
- Result: a JSON summary on standard output (scenario, seed, ticks, jobs,
  deadlocks,
  metrics, collisions, final position of each robot). It has no clock times or
  file paths, so the same scenario gives the same bytes.
- Exit codes: 0 done, 1 error (message on standard error), 2 wrong command line.
- Command line: `botsim-runner [--timing] <scenario.toml>`.

## 12. Known simplifications (not modelled yet)

Stops at every node; differential drive only; straight edges only; per-robot-type
edge properties of the LIF (paths, speeds, corridors); rotation restrictions at
nodes; node orientation and deviation; edge actions; other instant actions;
operating modes; state message timing and message delay; horizon actions;
breakdowns; the onboard safety stop (robots never stop for each other, so
controller mistakes show as collisions); deadlock resolution (controller side);
a debug stop at the first deadlock; stations in the layout; the viewer (the
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

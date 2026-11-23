# Graph structural problems found by simulation

Every problem in this list was found by running a big warehouse layout (the "warehouse 2"
example, 240 m × 140 m, up to 112 robots of five types) and looking at why robots
did not move. Each one can be found from the layout (LIF), the robot sizes (factsheets)
and the rules of the baseline **without running the simulation**. The list is the
starting point for a layout **validator**: a program that reads a LIF and the factsheets,
reports these problems with their places, and suggests a fix.

How to read an entry: *symptom* (what the simulation showed), *cause* (what is wrong in the
graph), *check* (how to find it from the layout alone), *fix* (what to change in the layout),
*seen* (a real example, with numbers).

Rules of the baseline that these problems depend on (see `rules.md`):

- A robot gets nodes and edges **one at a time** (released), and waits at the last node it
  has. The reservations cover **nodes and edges only**; an edge belongs to one robot at a time.
- A robot that cannot turn on the spot (three-wheel, S70) can only follow an edge that starts
  in the direction the last one ended (0.02 rad).
- A free robot stays where it is (S31); there is no parking behaviour.
- A robot is stopped by its protective field when another robot is in front of it
  (status `Obstructed`), whether or not that robot holds a node.

Sizes used below: forklift 2.4 m × 1.4 m, three-wheel 1.8 m × 1.0 m, carrier 1.2 m × 0.8 m,
omnidirectional 1.0 m × 0.7 m, small 0.5 m × 0.5 m.

---

## Problems of the whole layout

### G1. Corner where the next road does not start in the direction of the last one

- **Symptom:** a three-wheel robot reached 0 of 70 stations (a single robot alone, no traffic).
- **Cause:** every siding (pocket, bay, stall) left the road and rejoined it at about 48°. The
  edge directions at the node differ, so the robot cannot follow (`Graph::can_follow` is false).
  The same holds for every road corner without an arc.
- **Check:** for each robot type that cannot turn on the spot, walk the graph with
  `Graph::can_start` / `Graph::can_follow` from every node and report the nodes and stations
  that cannot be reached, and the nodes where a robot gets stuck (no way on). This is exact
  (it is the rule of the robot itself) and needs no simulation. **Reaching is not enough: a robot
  that enters a place it cannot leave is lost.** Build the graph of edges (an edge leads to
  another if `can_follow`), take its strongly connected components, and report the stations that
  are not in the biggest one (the generator example does this for the three-wheel type:
  `three_wheel_report`).
- **Fix:** let a siding leave and rejoin the road tangentially (an S-shaped curve, ends in the
  direction of the road); put a turn arc at every corner.
- **Seen:** three-wheel robots reached 0 of 70 stations; after the fix 55 of 55 that their roads
  allow. Dead-end nodes for them: 21 → 5 after the corners were improved.

### G2. A turn arc does not fit

- **Symptom:** a junction stays a sharp corner, so again robots that cannot turn on the spot
  get stuck there (G1), and the other robots turn on the spot.
- **Cause:** an arc of radius *r* for a turn of angle θ needs a tangent length *t = r·tan(θ/2)* on
  both roads (3.5 m for 90° and r = 3.5 m). A road piece between two junctions, or between a
  junction and the end of a siding, must be longer than the tangents at both ends plus a margin
  (about 0.5 m). Bays and pockets that begin 3 m from a junction leave no room.
- **Check:** for each node with a turn between 25° and 100°, compute *t*, and test the lengths
  of the roads in and out against *t* + (the tangent at the far end, if there is a turn there)
  + 0.5 m. Report junctions that get no arc.
- **Fix:** move the siding ends at least *t* + 0.5 m from the junction; on narrow roads that only
  small robots use, let the arc shrink (radius down to 1.8 m).
- **Seen:** bays 3 m from the row junctions on the ring road (moved to ≥ 4 m); the 6 m stubs from
  the ring road to the lanes along the walls (arc shrunk to fit).

### G3. Nodes closer together than a robot is long

- **Symptom:** two robots overlap in a stop pocket (collision, magenta ring in the viewer).
- **Cause:** the baseline lets a robot stand on every node it has been given, so two robots can
  stand on **neighbouring nodes**. If the nodes are closer together than the length of the
  robot (plus a margin), the bodies overlap. The S-curve of a pocket was two arcs with a node in
  the middle: the pocket nodes were 2 m apart, a forklift is 2.4 m long.
- **Check:** for every pair of nodes joined by an edge (and for every pair of nodes that are
  both stopping places and closer than the largest robot that may use both), test the distance
  against the length of the longest robot type allowed on the edges. Also nodes that are
  neighbours through an arc. Report the pairs.
- **Fix:** no node in the middle of a short curve (one edge, e.g. a cubic curve), or move the
  nodes apart; the minimum is the longest robot length plus a margin for the protective field
  (about 3 m for a forklift).
- **Seen:** pocket nodes at x = 37, 39, 41, 43, 45 → after one cubic curve per S-link the nodes
  are 4 m apart.

### G4. Edges that are too long

- **Symptom:** the whole hall drives slowly; robots queue behind one robot on a long road.
- **Cause:** an edge is reserved by one robot at a time. A 50 m road is one robot long: a
  convoy cannot form. (In a recording no edge was in the reservations of two robots.)
- **Check:** list the straight edges longer than about 2.5 times the longest robot (6 m for a
  forklift); report the total length of road that is covered by long edges.
- **Fix:** put nodes in between (every 6 m).
- **Seen:** 111 of 589 straight edges were longer than 8 m (46% of the straight road),
  12 longer than 30 m, the longest 51 m. With nodes every 6 m: 43 instead of 30 robots driving
  at 400 s; more robots close together, which showed more of the problems below.

### G5. A branch or junction inside a siding

- **Symptom:** two robots block each other for ever at the top left of the hall.
- **Cause:** the lane along the north wall branches off the ring road at x = 40, exactly in the
  middle of a stop pocket (x = 37 to 45). A robot that leaves the pocket meets a robot that
  enters the lane.
- **Check:** for each siding (start, apex, end) test that no other node or edge touches the road
  between its ends or within 4 m of them.
- **Fix:** leave out the pocket (or move the branch) so that the siding has the road to itself.
- **Seen:** four such places (x = 40 and x = 200, top and bottom).

---

## Problems at junctions

These came from a robot **waiting on the corner node** of a junction next to the arc of
another flow. They freeze every robot behind them.

### G6b. A node 1 to 2 m from the corner of an arc, made by something else

- **Symptom:** the same stand-off as G6, at a place where the corner node had been taken out
  or where an arc of the layout (not a turn arc) joins a road.
- **Cause:** a node made later (e.g. when long edges are cut into pieces of 6 m) lands near the
  corner: (100, 62.25) was 0.25 m from the removed corner (100, 62); (210.7, 130) was 1.3 m from
  the corner of the arc where the parking lane joins the top ring road.
- **Check:** no node (any) within about 3 m of the corner of an arc, except the nodes the arc
  itself starts and ends in.
- **Fix:** do not place such nodes (skip them when cutting edges).

### G6. Corner node beside the arcs of a junction (uncovered conflict)

- **Symptom:** two robots stand next to each other and neither moves: one is
  `WaitingForRelease` (on the corner node, it needs the next node), the other `Obstructed`
  (on the turn arc, its protective field sees the first one). About 100 robots queue behind
  such a pair. The detector shows 0 deadlocks: the cycle is made of a node reservation and a
  safety stop, and the detector only counts reservation cycles.
- **Cause:** the arc bypasses the corner node, so the corner node is free and is released to
  the robot that goes straight; that robot drives to the corner and stops there when the next
  node (where the arc ends) is held by the robot on the arc. The corner is only
  *r*·(√2 − 1) = 1.45 m from the arc (r = 3.5 m, 90°), and the arc of a 45° branch starts only
  1.45 m from the corner. The bodies and the protective fields overlap, but the two robots
  share no node until the arc ends.
- **General rule (the check):** **two edges that come closer than the half widths of the robots
  plus the protective field (about 2 m), but have no node in common, are an uncovered
  conflict.** Compute this for all pairs of edges (and for waiting positions: every node is a
  place where a robot can stop, so test the body at each node against the edges nearby).
  This finds G6 and G5 and parts of G3.
- **Fix (works for a T-junction or a corner):** take out the corner node when at most one road goes
  straight on and every turn has an arc: the through road becomes one edge past the corner, so
  the robot waits before the junction, outside the arcs.
- **Fix (a crossing of two roads):** the corner node must stay (it is where the two roads exclude
  each other). Let only the robots that cannot turn on the spot use the arcs there; the others
  turn at the corner node, so they hold it and nobody waits on it. A conflict is left: a
  three-wheel robot on an arc and a robot at the corner node (seen: T-004 / Y-014 at (52, 98),
  overlapping from 556 s to the end of a 6000 s run; X-001 / T-003 at (70, 98)); it needs the
  baseline to reserve the zone. Two other things were tried and were worse:
  - a bigger radius (6 m, 2.5 m clearance): fewer arcs fit (G2);
  - **no arcs at crossings at all** (every robot turns at the corner node): the three-wheel robots
    can still *reach* 53 of 70 stations, but can **come back from only 331 of 680 edges and 18
    stations** (with the arcs: 799 of 855 edges, all 55 stations). They enter pockets and cannot
    get out to the rest of the network; they stand there idle for ever and block the pocket
    (the waiting chains ended at idle three-wheel robots in pockets), and they did 2 jobs in
    1200 s instead of 14. **The check for a robot type that cannot turn on the spot is not
    "which stations can it reach" but "which stations are in its biggest strongly connected
    set of edges"** (G1).
- **Seen:** (100, 74) Y-021 / X-011; (30, 46); (30, 98); (86, 98); (160, 10); (140, 64);
  (100, 10); (140, 46); (160, 74). After the two fixes above: 4 stand-offs → 2 → 0 at 400 s,
  and at 1200 s two are left: (100, 62) Y-022 / Y-025 and (206, 46) X-026 / T-013 (a three-wheel
  robot on an arc at a crossing, as expected).
- **Baseline:** it should reserve the junction zone for the arcs as well, and the deadlock
  detector should count a safety stop as a wait.

### G10. A short closed one-way loop that fills up

- **Symptom:** at 1200 s about 20 robots stand in a ring and none moves. The wait-for graph has a
  real cycle (every robot waits for the next one), made of release waits.
- **Cause:** four one-way roads around one block of the grid (y = 66 east, x = 160 south,
  y = 46 west, x = 140 north) make a directed cycle of about 80 m. It holds about one robot per
  3.8 m (a forklift 2.4 m plus the gap), so about 21 robots; more robots enter than can leave,
  the loop is full, and no robot can move because the next place is taken.
- **Check:** find the directed cycles of the road graph that go through the junction nodes of one
  block (short simple cycles) and compute the capacity: cycle length / (longest robot allowed +
  gap). Report cycles whose capacity is less than the number of robots that can be on them
  (the fleet size is an upper limit), and say which roads feed them.
- **Fix:** a way out of the loop for the robots in it (a bypass or a shortcut inside the block, or
  one road two-way), more than one way to enter and leave each block, or fewer loops (alternate
  the road directions so that the blocks are not all loops). The baseline can also limit the number of
  robots that it releases into a loop (an entry may be released only if there is room for the
  robot and one more in the loop).
- **Seen:** 20 robots (12 forklifts and carriers, three-wheel, omnidirectional, small) around the
  block x 140 to 160, y 46 to 66 at 1200 s; the detector reported 0 deadlocks. Again, after all
  the junction fixes, in the block x 70 to 100, y 80 to 98 (clockwise: y = 98 east, x = 100 south,
  y = 80 west, x = 70 north): 17 robots at 800 s, 18 from 1000 s on, all `WaitingForRelease`
  (one `Obstructed`), frozen to the end; 54 of the 112 robots were in the west grid then. The
  north labyrinth was empty, so its entry was not the cause.
- **Why every cell is a loop:** rows alternate east / west and columns alternate north / south
  (a Manhattan grid), so **every block of the grid is a directed cycle** (clockwise, then
  counter-clockwise, and so on). Any block can fill up. The check should list all of them.
- **A cheaper fix in the layout:** pair the lanes (east, east, west, west, ... and north, north,
  south, south, ...): then only every other block is a loop.

### G11. A siding next to its road (parallel edges with no common node)

- **Symptom:** a collision at a stop pocket: a forklift that leaves a pocket (heading 139°, on
  the S-curve) overlaps a forklift on the road beside it (seen at (195, 46), 165 s).
- **Cause:** the pocket lies 1.8 m beside the road; where it joins the road it comes closer than
  the two robots are wide (1.4 m each). The S-curve edge and the road edge share only the
  end nodes, so a robot on one and a robot on the other can be side by side.
- **Check:** the general check of G6: two edges closer than the sum of the half widths plus the
  protective field over some length, with no common node.
- **Fix:** a bigger offset (2.6 m or more) and longer S-curves make the zone shorter; or
  reserve the zone in the baseline.

### G12. Too few exits: nowhere to go round

- **Symptom:** the baseline's timeout re-routing (S53) finds no new route, so a robot that is
  held up waits for ever (`rerouted` 0 or 2 in runs where 95 of 112 robots waited).
- **Cause:** in a one-way layout most nodes have **one** way out. Re-routing looks for a route
  with no node that another robot holds; with one exit per node there is no such route.
- **Check:** the out-degree of every node (and of every edge end) per robot type; the share of
  nodes with one exit, and for each such node, whether a second way to the same goal exists that
  does not pass the same next node (the "bypass": a path that leaves the road before the
  blocked node and joins it after).
- **Seen:** of 905 nodes, 675 have one exit, 195 two, 35 three.
- **Fix:** add exits where a stand-off is likely (before junctions and at bottlenecks), e.g.
  connectors between the two roads of a pair of lanes that run the same way (lane changes). Each
  new junction is a new place for the conflicts of G6, so the exits need the same care (nodes
  4 m or more from the arcs, G3).

---

## Problems of the roads and the fleet together

### G7. Narrow roads and the robots that may not use them

- **Check:** for each robot type, test that the roads it may use form a strongly connected
  graph and that every station it should serve has an interaction node in it
  (`stations_served`); report the stations a type cannot reach. A job for a station that a type
  cannot reach waits for another robot (the baseline takes the nearest free robot that has a
  route).
- **Seen:** forklift and three-wheel robot may not use the labyrinths and the lanes along the walls
  (55 of 70 stations); small robots all 70.

### G8. Where a robot starts

- **Symptom:** a three-wheel robot never moved; robots that start on a road block it for ever
  (they are idle and stay there, S31).
- **Cause:** a robot starts heading east (0 rad). A three-wheel robot cannot turn on the spot, so
  every road that leaves its start node must run east. And an idle robot on a road node blocks a
  one-way road.
- **Check (scenario against layout):** for every start node test that the robot's heading allows
  the roads that leave it (`Graph::can_start`), that the node is not a station stop on the road,
  and that start nodes are at least 6 m apart.
- **Fix:** start robots in pockets, off the road, and three-wheel robots only where all roads leave
  east. (Parking spaces for idle robots would be better; the baseline does not send them there.)

### G9. Stops with no shelving close

- **Check:** a station must have racking or shelving within reach (2.5 m in the generator) of
  every interaction node.
- **Seen:** stops that were first put in niches cut through the shelving, where the nearest shelf
  was 4.5 m away.

---

## Finding the same problems in a recording (no layout needed)

The wait-for graph of one snapshot: an edge from a robot to every robot that holds a node it
needs (release wait), and an edge from every `Obstructed` robot to the nearest robot ahead of it
(within 4 m and 60°, safety stop). **A cycle in this graph is a deadlock**, also when the detector
shows none. Robots that are on a path into a cycle are the jam behind it. Four pairs of robots
had frozen 100 of 112 robots. Listing the cycles' places gives the junctions to look at.

## Ideas for the validator

1. Read the LIF, the factsheets, (optionally) the geometry.
2. Run the checks G1 to G9 (all are static) and print for each problem: where (node or edge,
   coordinates), the rule that is broken, the numbers (distance, length, angle), and the
   suggested fix from this document.
3. The general check "two edges closer than a limit but no common node" finds a whole class
   (G5, G6, parts of G3); the limit follows from the robot sizes in the factsheets.
4. A fix suggestion can be mechanical for some (insert nodes in a long edge, shrink an arc,
   remove a corner node) and a hint for others (move a siding).
5. Report per robot type (a layout can be fine for a carrier and broken for a three-wheel robot).

## What the baseline has to do (not the layout)

- Reserve the junction zone also for the arcs; release a robot only up to the node before the
  zone if it cannot be released through it.
- Count safety-stop cycles as deadlocks. (The baseline does re-route a robot that is held up for
  10 s, S53, but it needs a route that avoids the nodes other robots hold; in these runs
  `rerouted` was 0 or 2 while 95 of 112 robots waited more than 600 s: **there was no other
  way**, see G12.)
- Send idle robots to a parking place.
- Speed: the route search with the turn rule recomputes the curve directions and compares robot
  types as strings at every edge, once per free robot and queued job and step. Keep the
  directions per edge and robot type, compare types by number, and do not search again for
  jobs that nobody can take.

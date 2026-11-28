# botsim-router

Route finding on a layout graph, for the fleet server.

Library only; of the botsim crates it depends on `botsim-model` only, reads no
files and uses no network. `botsim-dispatcher` and `botsim-server` use it.

- Layout nodes and edges are the numbers of the layout graph. What a search needs
  besides the graph (the edges a robot type may drive, costs, turn rules, rides,
  who holds what) is given to it as plain data.
- Edges are directed. The same input always gives the same route.
- The searches do not check their input: node and edge numbers and the lengths of
  lists must fit the graph (the doc comments say which). Exceptions: `load`
  accepts lists that do not fit, and `lift` a waiting node the graph does not
  have.

## Modules

| Module | |
|---|---|
| `route` | cheapest routes (Dijkstra), with blocked nodes, forbidden edges and the turns a robot type cannot make; the lengths from every node to one target |
| `lift` | a route through the waiting node of every lift entrance it uses |
| `load` | the cost of every edge with penalties for held edges and nodes, and for places where robots wait |
| `types` | what a robot type may drive (`TypeRouting`): its edges, their lengths and the turns it cannot make |

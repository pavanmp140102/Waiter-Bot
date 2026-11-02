# M9: Stretch

**Goal:** optional extras that show more ROS and more initiative. Start only when M0 to M8 are done, and only the items you choose.

**Depends on:** M5.

## Candidate items

| ID | Item | What it shows | Gate |
|---|---|---|---|
| X1 | **Order queue with batching.** A `PlaceOrder` service adds to a queue. While the robot is home, orders arriving within `batch_window_sec` form one trip. Orders arriving during a trip wait for the next one | The "busy day" of the brief: more than one batch, no rejected goals | T tests for batching and queue order, one end-to-end scenario |
| X2 | **Nearest-first ordering.** Tables are visited in the order that shortens the trip, using Nav2's `ComputePathToPose` for the distances | Optimisation, as the assessment asks | A test that the order changes with the layout, and trips get shorter |
| X3 | **Behaviour tree.** Express the state machine as a BehaviorTree.CPP tree and view it in Groot2, behind the same interface | Nav2's own paradigm, and a visual explanation | The same T and E tests pass against the tree |
| X4 | **Lifecycle node.** Make the task manager a managed (lifecycle) node, brought up by Nav2's lifecycle manager style | Another ROS 2 concept | Transitions covered by TN tests |
| X5 | **Battery and charging.** A simulated battery topic. A low level sends the robot home after the current delivery and blocks new goals | Real-world behaviour, and the old dashboard's battery idea made real | T tests, one scenario |
| X6 | **Collision monitor and speed limit near tables.** Nav2 collision monitor and speed filter zones | Safe operation among customers | N-level test |
| X7 | **Web dashboard.** A small web page through `rosbridge` | Presentation | Manual |

## Sub-tasks

- [ ] Pick items with the maintainer. Each picked item gets its own branch and its own test additions.
- [ ] Add each item's tests to [../test_plan.md](../test_plan.md) before writing code.
- [ ] Update [../interfaces.md](../interfaces.md) for any new interface.

## Notes

- Do not let a stretch item change the default behaviour of R1 to R7. Each one is behind a parameter that is off by default.
- X3 competes with the Python state machine. If chosen, keep both and select by parameter, so the T tests remain a reference.

## Result

(filled in when the milestone lands)

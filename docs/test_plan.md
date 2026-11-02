# Test plan

Every test has an ID. Milestone files use these IDs as their gates.

## Levels

| Level | Prefix | Needs | Run with | Typical duration |
|---|---|---|---|---|
| Static checks | S | Nothing running | `colcon build`, `colcon test` | Seconds |
| Task logic | T | Python only, no ROS | `pytest` via `colcon test` | Seconds |
| Task node | TN | ROS, no simulator, a fake Nav2 action server | `pytest` spinning the node in-process | Under a minute |
| Navigation | N | Gazebo (headless) and Nav2 | `launch_testing` | Minutes |
| End to end | E | The whole system | `launch_testing` plus the scenario runner | Minutes each |
| Docker | DK | Docker and Compose | `docker compose` services | Minutes |
| Manual | MAN | Gazebo GUI and RViz | By hand | - |

The behaviour lives in a ROS-free module on purpose, so level T carries most of the coverage and runs in seconds. Randomised tests use a fixed seed so a failure can be reproduced.

## Definition of a successful trip

Measured from Gazebo ground truth after the robot has been stationary for one second: the robot is within **0.30 m** of the named location's position and within **0.35 rad** of its yaw. A scenario passes when its outcome and its visited-locations list (in order) match its `expect` block.

## Static checks (S)

| ID | Check | Pass criterion |
|---|---|---|
| S1 | `colcon build --symlink-install` | Succeeds, no warnings from `waiter_bot_*` packages |
| S2 | `ament_flake8`, `ament_pep257`, `ament_copyright`, `ament_xmllint`, `ament_cmake_lint` on all packages | Clean |
| S3 | `xacro` then `check_urdf` on the robot; `gz sdf -k` on the world and every model | All parse without errors |
| S4 | Locations in `cafe.yaml` against the world and the map | Each location is a free map cell with 0.4 m clearance, and matches the matching entity in `cafe.sdf` within 0.05 m |
| S5 | `grep` for `/home/`, `ros_ws`, `defect_bot`, the author's user name and other absolute or foreign paths in `src/`, `docker/` and the root config files (the docs may mention them when describing the old submission) | No matches |
| S6 | `waiter_bot_core/butler_fsm.py` and `locations.py` import nothing from `rclpy` or ROS packages | Pass |
| S7 | `rosdep install --simulate` against the `package.xml` files | Every dependency resolves |
| S8 | Repo size and tracked binaries | Under 15 MB, no `.blend`, `.blend1` or duplicate formats |

## Task logic (T), `butler_fsm`

All feed events into the state machine and compare the returned commands, state, and result.

| ID | Case | Req | Expected |
|---|---|---|---|
| T1 | One table, confirmations off | R1 | `to_kitchen`, `to_table`, `to_home`; outcome `completed` |
| T2 | One table, confirmations on, all given | R1 | Same path with a timer started at the kitchen and the table, each stopped by its confirmation |
| T3 | Kitchen timeout | R2, R3a | `at_kitchen`, timeout, `to_home` (no kitchen return), `kitchen_timeout` |
| T4 | Table timeout, one table | R2, R3b | `at_table`, timeout, `returning_food`, `to_home`; table `skipped`, `partial` |
| T5 | Cancel all while `to_kitchen` | R4a | `to_home`, `cancelled` |
| T6 | Cancel all while `to_table` | R4b | `returning_food`, `to_home`, `cancelled` |
| T7 | Three tables, all confirmed | R5 | One kitchen visit, tables in request order, then `to_home`; `completed` |
| T8 | Three tables, table1 ignored | R6 | table1 `skipped`, table2 and table3 served, `returning_food`, `to_home`; `partial` |
| T9 | Three tables, table2 cancelled while `to_table` for table1 | R7 | table2 skipped without a visit, table1 and table3 served, `returning_food`, `to_home` |
| T10 | Cancel all while `at_kitchen` | R4a | `to_home` |
| T11 | Cancel all while `at_table` | R4b | `returning_food`, `to_home` |
| T12 | Cancel a table that was already delivered | D | Command rejected, state unchanged |
| T13 | All tables cancelled before the kitchen pickup | D-c | Nothing is loaded, so `to_home` directly, with no kitchen return; outcome `partial` |
| T14 | Navigation failure, retries left | D-e | Same goal sent again, retry counter incremented |
| T15 | Navigation failure, retries used up at a table | D-e | Table `skipped`, next leg continues |
| T16 | Navigation failure, retries used up going home | D-e | `error`, `nav_failed` |
| T17 | Random event sequences (1000, fixed seed) | all | Invariants: never two timers at once, every task ends in `idle` or `error`, home is never skipped while food is not onboard, the kitchen is never skipped when food is onboard |
| T15b | Navigation failure going to the kitchen, retries used up | D-e | Orders skipped, `to_home`, outcome `nav_failed` |
| T18 | Confirmation for the wrong location | D | Rejected, timer untouched |
| T19 | Unknown table, duplicate table, empty list, goal while busy | D-g | Goal rejected with the matching reason |
| T20 | Rapid duplicate events (confirm and timeout together) | all | Exactly one transition, the second event ignored |
| T21 | Food returned to the kitchen | R3b | `food_onboard` is true on the way back and false on the way home. Found by T17 |
| T22 | Kitchen dwell on a return trip | D-f | The robot waits for the dwell timer before going home |

## Task node (TN), `butler_task_manager` with a fake navigator

| ID | Check | Pass criterion |
|---|---|---|
| TN1 | Parameters load from `cafe.yaml`; a missing location pose stops start-up with a clear error | Pass |
| TN2 | Goal accepted, feedback flows, result reaches the client | Feedback has the expected states in order |
| TN3 | `Confirm` and `CancelTable` services reach the state machine | Same results as T2 and T9 |
| TN4 | `/butler/state` is transient local | A subscriber started after the change still receives the latest state |
| TN5 | Fake navigator aborts a goal | Retry behaviour as T14 to T16 |
| TN6 | Action cancel while a navigation goal is active | The navigation goal is cancelled and the return leg starts (R4) |
| TN7 | A 20 s timeout under `use_sim_time` and a paused clock | The timeout does not fire until simulated time passes |

## Navigation (N)

| ID | Check | Pass criterion |
|---|---|---|
| N1 | Simulation starts headless; `/scan` at 10 Hz, `/odom`, `odom -> base_link` TF | Pass within 60 s |
| N2 | Nav2 lifecycle nodes all reach `active` | Pass within 60 s |
| N3 | AMCL pose converges after the initial pose is set | Covariance below a threshold within 15 s |
| N4 | `NavigateToPose` to kitchen, each table, and home, three rounds | Every trip meets the success definition |
| N5 | Cancel a `NavigateToPose` goal in motion | Robot stops within 2 s |

## End to end (E)

Each runs the scenario file named in the table through the whole system in headless Gazebo.

| ID | Scenario file | Req | Outcome and path |
|---|---|---|---|
| E1 | `r1_single_no_confirm` | R1 | `completed`: kitchen, table1, home |
| E2 | `r2_kitchen_ignored` | R2, R3a | `kitchen_timeout`: kitchen, home |
| E3 | `r3b_table_ignored` | R3b | `partial`: kitchen, table1, kitchen, home |
| E4 | `r4a_cancel_to_kitchen` | R4a | `cancelled`: home directly |
| E5 | `r4b_cancel_to_table` | R4b | `cancelled`: kitchen, table, kitchen, home |
| E6 | `r5_multi_all_served` | R5 | `completed`: kitchen, table1, table2, table3, home |
| E7 | `r6_table_ignored` | R6 | `partial`: kitchen, table1, table2, table3, kitchen, home |
| E8 | `r7_table_cancelled` | R7 | `partial`: kitchen, table1, table3, kitchen, home |
| E9 | `new_table_from_yaml` | D4 | A fourth table added only in YAML is served, with no code change |

E1 to E8 are the seven situations of the assessment. E9 proves the "generic" claim.

## Docker (DK)

| ID | Check | Pass criterion |
|---|---|---|
| DK1 | Image builds from a clean checkout with `rosdep` only | Succeeds |
| DK2 | `test` service | S, T and TN pass |
| DK3 | `test-e2e` service | E1 to E9 pass |
| DK4 | `sim` service on software rendering | N1 passes |
| DK5 | `docker/run.sh` picks a graphics mode | Window opens, or falls back with a message |

## Manual (MAN)

| ID | Check |
|---|---|
| MAN1 | Drive with the keyboard in the café, build the map, save it |
| MAN2 | Dashboard: place a three-table order, watch RViz and the dashboard together, press confirm and cancel buttons |
| MAN3 | The README quick start works from a clean machine in under 10 minutes |

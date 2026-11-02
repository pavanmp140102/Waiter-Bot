# Requirements

The assessment lists seven situations. Several are ambiguous in the PDF, so this file fixes one reading of each. Everything else in the project (tests, scenarios, the state machine) refers to these IDs.

## Vocabulary

- **Home**, **kitchen**, **table1..table3**: named locations in the `map` frame ([world_and_robot.md](world_and_robot.md)). The number of tables is configuration, not code.
- **Order**: a request to deliver food to one table.
- **Confirmation**: a signal that someone attended the robot, given by calling the `Confirm` service for the robot's current location. At the kitchen it means "the food is loaded"; at a table it means "the food was taken".
- **Timeout**: no confirmation within `confirmation_timeout_sec` of arriving (default 20 s).
- **Food onboard**: the robot has loaded food at the kitchen and at least one order is not yet delivered.

## The one rule that covers most cases

> The robot goes home only when it is not carrying food. If it is, it returns to the kitchen first.

R3b, R4b, R6 and R7 all follow from this rule, so the state machine implements the rule once instead of each case separately.

## Requirements

| ID | Situation (from the PDF) | Required behaviour | Fixed reading |
|---|---|---|---|
| R1 | Single order, no confirmation needed | home → kitchen → table → home | A parameter turns confirmation off per location kind (`require_kitchen_confirmation`, `require_table_confirmation`). With both off, the robot does not wait anywhere |
| R2 | Nobody attends the robot | Wait for confirmation, then go home after the timeout | At the kitchen, a timeout sends the robot home. A table timeout is covered by R3b and R6 |
| R3a | No confirmation at the kitchen | Home after the timeout | Nothing is onboard, so the robot goes straight home |
| R3b | No confirmation at the table | Kitchen first, then home | The undelivered food is onboard, so by the rule above the robot returns it |
| R4a | Order cancelled while going to the kitchen | Return home | Nothing onboard, so home directly. Applies to the whole task cancelled through the action |
| R4b | Order cancelled while going to the table | Return to the kitchen, then home | Food is onboard. Applies to the whole task cancelled through the action, including while waiting at a table |
| R5 | Several orders | Kitchen once, each table in turn, then home | One kitchen visit loads every order. All delivered means nothing onboard, so home directly |
| R6 | Several orders, one table gives no confirmation | Skip that table's wait, go on to the next, and after the last table go to the kitchen, then home | The timed-out table is marked `skipped`. Its food is still onboard |
| R7 | Several orders, one table's order cancelled | Skip that table, deliver the others, then kitchen, then home | Cancelling one table uses the `CancelTable` service. The cancelled food is onboard if it was already loaded |

## Further decisions (gaps in the PDF)

| ID | Decision | Reason |
|---|---|---|
| D-a | Orders arrive as one action goal with a list of tables | "Multiple orders" then means one batch. Queueing and batching across goals is stretch work (M9) |
| D-b | While a task runs, a new goal is rejected with the reason `busy` | Predictable, and testable. Replaced by queueing in M9 |
| D-c | A table cancelled **before** the kitchen pickup is removed from the load, so it does not count as onboard | Cancelling before pickup costs nothing. If every table is cancelled, the robot goes home |
| D-d | Visiting order is the request order | Simple and predictable. Nearest-first ordering is stretch work (M9) |
| D-e | A navigation failure is retried `nav_max_retries` times (default 2). Then: a table is skipped, the kitchen sends the robot home, and a failed trip home ends the task in the `error` state | The robot must never get stuck waiting silently |
| D-f | On arrival at the kitchen for a return trip, the robot does not wait. It heads home at once, after an optional `kitchen_return_dwell_sec` (default 0) | The PDF only says "kitchen first before going to the home position" |
| D-g | An unknown or duplicate table name rejects the goal | Fails fast with a clear message |

## Non-functional requirements (from "points for evaluation")

| Evaluation point | How the project answers it |
|---|---|
| Runs in a ROS environment | ROS 2 Jazzy, Gazebo Harmonic, Nav2. One command starts everything ([setup.md](setup.md)) |
| Code on GitHub, a push per milestone | One branch and PR per milestone ([plan/overview.md](plan/overview.md)) |
| Use as many ROS concepts as possible | Actions (server and client), services, parameters, QoS (transient local), callback groups and a multi-threaded executor, TF, launch, Nav2, `ros_gz` bridge, `launch_testing`, lifecycle node (stretch) |
| Generic, not hardcoded | Locations, tables, timeouts, and the number of tables are YAML parameters. The UI builds itself from them |
| Clean and lean | Python with `rclpy`, one purpose per package, ROS-free logic modules, no dead files ([migration.md](migration.md)) |
| Documentation | This folder, plus a **Result** section per milestone |
| Optimised | Event-driven, with no polling loops or sleeps: each event costs O(1) work in the state machine, and Nav2 does the planning and control |
| Innovation and presentation | A scenario runner with scripted "people" (R1 to R7 replayable by name), a live dashboard, RViz state markers |

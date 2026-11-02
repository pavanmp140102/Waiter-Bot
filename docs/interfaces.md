# Interfaces

Exact definitions and defaults. Message and parameter names are fixed here so the task manager, the dashboard, the scenario runner and the tests can be built in parallel.

## Package `waiter_bot_interfaces`

### `action/DeliverOrders.action`

```text
# Goal: tables to serve, in visiting order. Names must exist in the `tables` parameter.
string[] tables
---
# Result
string outcome        # see "Outcomes" below
string[] delivered
string[] skipped      # no confirmation at the table, or navigation failed
string[] cancelled    # cancelled with CancelTable
---
# Feedback
string state          # a state from architecture.md section 5
string target         # location being approached or waited at, empty when idle
string[] pending
string[] delivered
float32 timer_remaining   # seconds left on the confirmation timer, 0 when none
```

Outcomes:

| Outcome | Meaning | Action result |
|---|---|---|
| `completed` | Every order delivered, robot home | succeeded |
| `partial` | Robot home, but at least one order skipped or cancelled | succeeded |
| `kitchen_timeout` | Nobody loaded the food (R2, R3a), robot home | succeeded |
| `cancelled` | The whole task was cancelled (R4a, R4b), robot home | canceled |
| `nav_failed` | A trip home failed after all retries, state `error` | aborted |

The goal is **rejected** (not aborted) with a reason in the goal response for: unknown or duplicate table, empty list, `busy`, `nav2_unavailable`.

### `srv/Confirm.srv`

```text
# Someone attended the robot at `location` ("kitchen" or a table name).
string location
---
bool accepted
string message        # reason when not accepted
```

Accepted only when the robot is waiting at exactly that location.

### `srv/CancelTable.srv`

```text
# Cancel one table's order (R7).
string table
---
bool accepted
string message
```

Accepted for a table that is pending or being served. Rejected for an unknown or already delivered table.

### `msg/ButlerState.msg`

Published on `/butler/state`, QoS reliable and **transient local**, depth 1.

```text
builtin_interfaces/Time stamp
string state
string target
string[] pending
string[] delivered
string[] skipped
string[] cancelled
bool food_onboard
float32 timer_remaining
```

## Node `butler_task_manager` (package `waiter_bot_core`)

| Kind | Name | Type | Notes |
|---|---|---|---|
| Action server | `/butler/deliver_orders` | `DeliverOrders` | Reentrant callback group |
| Service server | `/butler/confirm` | `Confirm` | |
| Service server | `/butler/cancel_table` | `CancelTable` | |
| Publisher | `/butler/state` | `ButlerState` | Transient local. Published on every state change and once a second while a timer runs |
| Publisher | `/butler/markers` | `visualization_msgs/MarkerArray` | Location names and the state text in RViz (M6) |
| Action client | `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | The only way the robot moves |

### Parameters (`config/cafe.yaml`)

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `home` | string | `home` | Name of the home location |
| `kitchen` | string | `kitchen` | Name of the kitchen location |
| `tables` | string[] | `[table1, table2, table3]` | Names of the servable tables |
| `locations.<name>.x`, `.y`, `.yaw` | double | see [world_and_robot.md](world_and_robot.md) | Pose in the `map` frame, one set per name above |
| `confirmation_timeout_sec` | double | `20.0` | Wait time at the kitchen and at each table |
| `require_kitchen_confirmation` | bool | `true` | When false the robot loads without waiting (R1) |
| `require_table_confirmation` | bool | `true` | When false the robot delivers without waiting (R1) |
| `kitchen_return_dwell_sec` | double | `0.0` | Pause at the kitchen when returning food |
| `nav_max_retries` | int | `2` | Retries for a failed navigation goal |
| `map_frame` | string | `map` | Frame for goal poses |

A start-up check fails fast if a name in `tables`, `home` or `kitchen` has no `locations.<name>` entry.

## Package `waiter_bot_ui`

Node `dashboard`. Reads the same parameters from `butler_task_manager` with the parameter client, so a new table in YAML appears as a new button. Uses the three interfaces above and nothing else.

## Package `waiter_bot_bringup`

Node `scenario_runner`. Plays a scenario file (M5): a list of timed or event-triggered "person" actions.

```yaml
# scenarios/r6_table_ignored.yaml
name: r6_table_ignored
goal: {tables: [table1, table2, table3]}
people:
  kitchen: {confirm_after_sec: 2.0}
  table1:  {ignore: true}
  table2:  {confirm_after_sec: 2.0}
  table3:  {confirm_after_sec: 2.0}
expect:
  outcome: partial
  visited: [kitchen, table1, table2, table3, kitchen, home]
```

Supported person behaviours: `confirm_after_sec`, `ignore`, `cancel_after_sec` (calls `CancelTable`). A top-level `cancel_task_after_sec` or `cancel_task_when_state` cancels the whole goal (R4).

## Topics from the simulation

| Topic | Type | From |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 to Gazebo |
| `/odom` | `nav_msgs/Odometry` | Gazebo `DiffDrive` |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo `gpu_lidar` |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `odom -> base_link` from Gazebo, the rest from `robot_state_publisher`, `map -> odom` from AMCL |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo, bridged |
| `/ground_truth/pose` | `geometry_msgs/PoseStamped` | Gazebo, bridged **for tests only** |

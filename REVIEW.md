# Waiter-Bot — Code Review

**Reviewed:** commit `d81d18c` (`main`), 2026-11-02
**Against:** *Goat Robotics — Assessment for the Position of ROS Developer* (Butler robot for "French Door Café")

---

## 1. Summary

The repo is a **robot description and simulation package**, not a solution to the assessment.
The assessment is about **task logic**: taking orders, going home → kitchen → table(s) → home, waiting for confirmations with timeouts, and handling cancellations. None of that logic exists in the code.

The most serious problems, in order:

1. **No order/task logic.** No node accepts an order, tracks state, waits for a confirmation, applies a timeout, or handles a cancellation. **0 of the 7 milestones** are implemented.
2. **The package does not launch.** Every launch file has missing imports or calls executables that don't exist in ROS 2. Several files are ROS 1 code (`move_base`, `move_base_msgs`, `DWAPlannerROS`, ROS 1 controller YAML) with ROS 2 names swapped in.
3. **The GUI publishes to a topic no node listens to.** `/waiter_bot/command` carries free-text strings, and no node subscribes to it.
4. **Repo hygiene works against you.** There's one commit, about 125 MB of Blender files and `.blend1` backups, duplicate scripts and maps, and placeholder metadata. A reference to `defect_bot_description` shows the files were copied from another project.
5. **The README describes things that aren't there.** It names launch files that don't exist and a placeholder clone URL. It has no milestone write-up, design rationale or demo, and the brief asks for all three.

What's good: the custom café world, the robot model with meshes and sensors, and the idea of a control-panel GUI. These are solid pieces of "presentation". They just need the actual robot logic underneath them.

---

## 2. Requirements coverage

| # | Milestone | Status | What's in the code |
|---|-----------|--------|--------------------|
| 1 | Single order: home → kitchen → table → home, no confirmations | ❌ | `movebase_client()` in [src/control_panel.py](ros_ws/src/waiter_bot_description/src/control_panel.py#L55) can send one goal, but it uses the ROS 1 `move_base_msgs` API, and nothing calls it in sequence. |
| 2 | Wait for confirmation; return home on timeout | ❌ | No confirmation input and no timer. |
| 3a | No kitchen confirmation → home after timeout | ❌ | — |
| 3b | No table confirmation → kitchen → home | ❌ | — |
| 4 | Cancel en route to the table → kitchen → home; cancel en route to the kitchen → home | ❌ | The only "cancel" is an E-stop that shells out to `ros2 action send_goal /move_base/cancel ...` ([src/control_panel.py:116](ros_ws/src/waiter_bot_description/src/control_panel.py#L116)), which isn't a valid command. |
| 5 | Multiple orders: kitchen once, deliver to all tables, go home | ❌ | — |
| 6 | Multiple orders, one table doesn't confirm → skip it, finish the rest, kitchen → home | ❌ | — |
| 7 | Multiple orders, one table cancelled → skip it, kitchen → home | ❌ | — |

Evaluation criteria:

| Criterion | Status | Notes |
|-----------|--------|-------|
| Runs in a ROS environment | ❌ | Launch files fail at import time (see §4.2). |
| Pushed to GitHub with a "proper git push … for each milestone" | ❌ | One commit for the whole workspace, so there's no milestone history. |
| "As many ROS concepts as possible" | ⚠️ | Only topics. No actions, services, parameters, lifecycle nodes, custom interfaces or a behavior tree. |
| Generic, not hardcoded | ❌ | Table coordinates are hardcoded globals (`t1`…`t9`). There are 9 tables where the brief asks for 3, and the GUI loops over `range(1, 10)`. |
| Clean and lean | ❌ | Two diverging control panels plus a generated copy, dead launch files, ROS 1 leftovers, and 125 MB of assets. |
| Documentation per milestone | ❌ | The README is generic setup text, partly inaccurate. |

---

## 3. Architecture and ROS concepts

### 3.1 What's there

```
[PyQt GUI] --String "Navigate to Table 3"--> /waiter_bot/command --> (nobody)
[PyQt GUI] <--Float32-- /waiter_bot/battery_level <-- (nobody publishes)
```

- Everything lives in one package, `waiter_bot_description`. By ROS convention, a `*_description` package holds only URDF and meshes.
- The GUI is the only "logic". It's coupled to UI widgets, so it can't be tested or reused headless.
- Commands are free-text strings (`"Navigate to Table 3"`), so every consumer would have to parse English. That's the opposite of a typed ROS interface.

### 3.2 Issues

- **No state machine.** The whole problem (milestones 2–7) is a state machine with timeouts and preemption. You need to model it explicitly, as a hand-written FSM, a SMACH/FlexBE graph, or a **BehaviorTree.CPP** tree. Nav2 already uses behavior trees, which makes that the strongest ROS-native choice.
- **No navigation abstraction.** You should call Nav2's `NavigateToPose` action (or `FollowWaypoints` / `NavigateThroughPoses`) with an `ActionClient` and real feedback and cancellation. Wrapping the ROS 1 `move_base` action won't work.
- **Blocking calls in callbacks.** `rclpy.spin_until_future_complete()` inside a GUI handler ([src/control_panel.py:69](ros_ws/src/waiter_bot_description/src/control_panel.py#L69)) deadlocks if the node is already spinning. It also freezes the Qt event loop.
- **No parameters.** Locations, timeouts and table count should be ROS parameters loaded from YAML.
- **ROS 1 / ROS 2 mix.** `move_base`, `DWAPlannerROS`, `NavfnROS`, `controller_patience`, `effort_controllers/JointPositionController` with `type:` keys, and `libgazebo_ros2_skid_steer_drive.so` are all ROS 1 concepts. Renaming them doesn't make them ROS 2: these plugins and parameter names don't exist in Humble.

---

## 4. Code-level issues

### 4.1 GUI / control panel

There are three files: [src/control_panel.py](ros_ws/src/waiter_bot_description/src/control_panel.py), [UI/control_panel.py](ros_ws/src/waiter_bot_description/UI/control_panel.py) and [UI/control_panel_1.py](ros_ws/src/waiter_bot_description/UI/control_panel_1.py). They have diverged, and it's unclear which one is real.

| Where | Problem | Fix |
|-------|---------|-----|
| [src/control_panel.py:18](ros_ws/src/waiter_bot_description/src/control_panel.py#L18) | `move_base_msgs` doesn't exist in ROS 2, so the script crashes at import. | Use `nav2_msgs.action.NavigateToPose`. |
| [src/control_panel.py:127](ros_ws/src/waiter_bot_description/src/control_panel.py#L127) | Calls `ui.setupUi()`, but that `Ui_waiter_bot` has no `setupUi` method (the comment says "Continue with similar updates…"). | Finish it, or delete this file. |
| [src/control_panel.py:130-133](ros_ws/src/waiter_bot_description/src/control_panel.py#L130-L133) | `rclpy.spin(node)` blocks forever, so `app.exec_()` is never reached and the window never responds. | Spin ROS in a `MultiThreadedExecutor` on a background thread, or call `spin_once` from a `QTimer`. |
| [UI/control_panel.py:196-202](ros_ws/src/waiter_bot_description/UI/control_panel.py#L196-L202) | The node is never spun, so `battery_callback` never fires. | Same fix as above. |
| [src/control_panel.py:30-42](ros_ws/src/waiter_bot_description/src/control_panel.py#L30-L42) | Hardcoded table coordinates as module globals, 9 tables where the brief has 3. | Load them from a `locations.yaml` parameter file. |
| [src/control_panel.py:116](ros_ws/src/waiter_bot_description/src/control_panel.py#L116) | `Popen('ros2 action send_goal ...', shell=True)` from inside a node. | Use `goal_handle.cancel_goal_async()`. Never shell out to the CLI from a node. |
| [src/control_panel.py:29](ros_ws/src/waiter_bot_description/src/control_panel.py#L29), `global move_cmd` | One mutable global `Twist` shared by every handler. | Build a local message per publish. |
| Both | Fixed orientation quaternion `(0,0,0.701,0.712)` for every goal. | Store a yaw per location. |
| [UI/control_panel.py:178](ros_ws/src/waiter_bot_description/UI/control_panel.py#L178) | Publishes free-text commands. | Send typed goals or requests (see §6). |
| — | `control.ui` exists but is hand-converted into Python and then edited ("WARNING! All changes made in this file will be lost!"). | Load the `.ui` at runtime with `uic.loadUi()`, or regenerate the Python and never hand-edit it. |

### 4.2 Launch files

None of these will run as written.

| File | Problems |
|------|----------|
| [launch/gazebo_launch.py](ros_ws/src/waiter_bot_description/launch/gazebo_launch.py) | Missing imports: `get_package_share_directory`, `LaunchConfiguration`, `IfCondition`. `spawn_model` and `gzserver` aren't `gazebo_ros` node executables; use `spawn_entity.py` and include `gazebo.launch.py`. The `world_name` argument is never used. `robot_state_publisher` is never started, so `robot_description` doesn't exist to spawn from. `control_panel.py` isn't installed as an executable. |
| [launch/display_launch.py](ros_ws/src/waiter_bot_description/launch/display_launch.py) | Missing `LaunchConfiguration` import. `$(find …)` is ROS 1 syntax. Running `xacro` as a node doesn't publish `robot_description`; pass `Command(['xacro ', path])` as a parameter to `robot_state_publisher`. |
| [launch/controller_launch.py](ros_ws/src/waiter_bot_description/launch/controller_launch.py) | Missing `get_package_share_directory`. `use_sim_time: 'true'` is a string, not a bool. [controller.yaml](ros_ws/src/waiter_bot_description/launch/controller.yaml) uses ROS 1 `ros_control` format. Position controllers on drive wheels are also wrong; a mobile base needs `diff_drive_controller`. |
| [navigation/movebase.launch.py](ros_ws/src/waiter_bot_description/navigation/movebase.launch.py) | Runs `navigation_launch.py` as a node `executable`; it must be included with `IncludeLaunchDescription`. Loads params from **`defect_bot_description`**, another project ([L27-31](ros_ws/src/waiter_bot_description/navigation/movebase.launch.py#L27-L31)). `LaunchConfiguration(...) == 'true'` compares a substitution object to a string at import time, so it's always `False`. `FindPackageShare` is imported from the wrong module (`launch_ros.substitutions`) and has no `.find()`. |
| [navigation/move_base_mapless.launch.py](ros_ws/src/waiter_bot_description/navigation/move_base_mapless.launch.py) | Same issues, plus it starts Nav2 three times with the same node names. |
| [navigation/navigation.launch.py](ros_ws/src/waiter_bot_description/navigation/navigation.launch.py) | `IncludeLaunchDescription` imported from `launch_ros.actions` (wrong), `PythonLaunchDescriptionSource` not imported, `map_server` given a ROS 1 `--map-file` argument, references a nonexistent `rviz_config/nav2.rviz`, and no lifecycle manager, so `map_server`/`amcl` never activate. |
| [navigation/gmapping.launch.py](ros_ws/src/waiter_bot_description/navigation/gmapping.launch.py) | Passes the boolean `gmap` argument as the RViz config *path*. `maxUrange`/`maxRange` are GMapping params that slam_toolbox ignores. Near-duplicate of `gmapping_auto.launch.py`. |
| [navigation/bot_amcl.launch.py](ros_ws/src/waiter_bot_description/navigation/bot_amcl.launch.py) | ROS 1 AMCL param names (`kld_err`, `gui_publish_rate`, `initial_pose_a`). Nav2 AMCL uses `pf_err`, `set_initial_pose`, `initial_pose.yaw`, etc. |

### 4.3 Config, URDF, map

- [map/map1.yaml](ros_ws/src/waiter_bot_description/map/map1.yaml): `image: home/hppmp/ros_ws/my_bot/map/map1.pgm` is an absolute path to your machine, with the leading `/` missing. Use `image: map1.pgm`.
- `config/` and `param/` hold two parallel, ROS 1 style parameter sets (`DWAPlannerROS`, `NavfnROS`, `recovery_behaviour_enabled`). Replace both with one Nav2 `nav2_params.yaml` (controller_server / planner_server / bt_navigator / costmaps).
- [urdf/waiter_bot.gazebo:115](ros_ws/src/waiter_bot_description/urdf/waiter_bot.gazebo#L115): `libgazebo_ros2_skid_steer_drive.so`, `libgazebo_ros2_gpu_laser.so` and `libgazebo_ros2_openni_kinect.so` don't exist. ROS 2 uses `libgazebo_ros_diff_drive.so`, `libgazebo_ros_ray_sensor.so` and `libgazebo_ros_camera.so`, with `<ros><remapping>` blocks instead of `<topicName>`/`<robotNamespace>`. The file also loads `gazebo_ros2_control` *and* a drive plugin, so two systems would fight over the wheels.
- Joint name `RB_w` vs `RB_W` elsewhere is a case typo waiting to break something.
- Four cameras and a lidar for a 2D waypoint task is fine for looks, but they cost simulation time. Keep the lidar and drop the rest unless you use them.

### 4.4 Build files

- [CMakeLists.txt](ros_ws/src/waiter_bot_description/CMakeLists.txt) installs `UI/` as data but never installs `control_panel.py` as an executable, so `ros2 run` can't find it. Use `ament_python` or `install(PROGRAMS ...)`.
- [package.xml](ros_ws/src/waiter_bot_description/package.xml): placeholder maintainer (`Your Name`, `your_email@example.com`). It depends on `rclcpp` (no C++ in the package) and is missing the real runtime deps: `rclpy`, `nav2_bringup`, `gazebo_ros`, `slam_toolbox`, `robot_state_publisher`, `python3-pyqt5`.

---

## 5. Repo, git and documentation

### Git

- **Single commit** for everything ("initial commit ros_ws"). The brief explicitly asks for "proper git push and documented approach for each milestone". Reviewers read commit history as evidence of how you work.
- **~125 MB of binary assets** in `waiter/`: `.blend`, `.blend1` autosave backups, both `.obj` *and* `.dae` *and* `.stl` of the same models, and filenames with spaces (`environment v10.obj`). Keep only what the world loads, use Git LFS for large meshes, and add a `.gitignore` (`*.blend1`, `build/`, `install/`, `log/`, `.vscode/`).
- Duplicate files: `UI/map1.pgm` = `map/map1.pgm`, `UI/control_panel.py` vs `src/control_panel.py`, `gmapping.launch.py` ≈ `gmapping_auto.launch.py`, `launch/urdf.rviz` vs `rviz_config/`.
- `.vscode/settings.json` has an absolute path to your machine. Don't commit it.

### README

- References `navigation.py`, `gmapping.py`, `bot_amcl.py` etc. The real files are `*.launch.py` under `navigation/`, not `launch/`.
- The clone URL is `https://github.com/yourusername/ros_ws.git`, and the workspace tree is broken markdown.
- It mentions `config/costmap_global.yaml`, which doesn't exist.
- Missing everything the evaluators asked for: the problem restated, architecture, design decisions, how each milestone is handled, how to run each scenario, and results (GIF/video).
- "Feel free to submit issues or pull requests" is boilerplate. Use the space to show your work.

---

## 6. How to make it better

### 6.1 Target architecture

Split into focused packages:

```
ros_ws/src/
├── waiter_bot_description/   URDF, meshes only
├── waiter_bot_gazebo/        world, spawn launch
├── waiter_bot_navigation/    nav2_params.yaml, map, bringup launch
├── waiter_bot_interfaces/    custom .action / .srv / .msg
├── waiter_bot_core/          butler task manager (the actual assessment)
└── waiter_bot_ui/            Qt / web dashboard (optional)
```

Data flow:

```
 Host UI / CLI                      waiter_bot_core                       Nav2
 ───────────────                    ─────────────────                     ─────
 DeliverOrders.action ───goal────▶  ButlerTaskManager  ──NavigateToPose─▶ bt_navigator
   tables: [1,2,3]   ◀─feedback───   (state machine /   ◀──feedback──────
                                      behavior tree)
 CancelTable.srv ─────────────────▶      │
 Confirm.srv (kitchen/table) ─────▶      │  params: locations.yaml, timeouts
                                         ▼
                                  /butler/state (latched status topic) ─▶ UI / RViz markers
```

### 6.2 Interfaces (`waiter_bot_interfaces`)

```text
# action/DeliverOrders.action
string[] tables            # e.g. ["table1", "table3"]
---
string[] delivered
string[] skipped           # no-confirm or cancelled
---
string state               # GOING_TO_KITCHEN, WAITING_KITCHEN, GOING_TO_TABLE, ...
string current_target
```

```text
# srv/Confirm.srv
string location            # "kitchen" or "table2"
---
bool accepted
```

```text
# srv/CancelTable.srv
string table
---
bool accepted
```

Cancelling the *whole* task is just the action's built-in cancel. That gives milestone 4 for free, with proper semantics.

### 6.3 Config, not code (genericness)

```yaml
# waiter_bot_core/config/cafe.yaml
butler_task_manager:
  ros__parameters:
    confirmation_timeout_sec: 15.0
    require_kitchen_confirmation: true
    require_table_confirmation: true
    locations:
      names: [home, kitchen, table1, table2, table3]
      home:    [0.0,  0.0, 0.0]     # x, y, yaw
      kitchen: [0.10, -0.36, 1.57]
      table1:  [1.47, 5.98, 1.57]
      table2:  [-2.28, 6.02, 1.57]
      table3:  [-4.86, 6.15, 1.57]
```

Adding table 4 then means editing YAML, not code. Even better, add a small tool that saves the robot's current pose as a named location (an RViz "2D Goal Pose" click → `/butler/save_location`).

### 6.4 One state machine covers all 7 milestones

Milestones 2–7 are all special cases of one rule set:

```
IDLE(home)
  └─ goal(tables) ─▶ GO_KITCHEN
GO_KITCHEN
  ├─ cancel ─────────▶ GO_HOME                          (M4)
  └─ arrived ────────▶ WAIT_KITCHEN
WAIT_KITCHEN
  ├─ confirm / no confirmation required ─▶ next table   (M1, M5)
  └─ timeout ────────▶ GO_HOME                          (M2, M3a)
GO_TABLE(t)
  ├─ t cancelled ────▶ skip t → next table              (M7)
  ├─ whole task cancelled ─▶ RETURN_KITCHEN             (M4)
  └─ arrived ────────▶ WAIT_TABLE(t)
WAIT_TABLE(t)
  ├─ confirm ────────▶ delivered += t → next table      (M1, M5)
  └─ timeout ────────▶ undelivered += t → next table    (M3b, M6)
next table:
  ├─ more tables ────▶ GO_TABLE(next)
  ├─ none, any undelivered/skipped ─▶ RETURN_KITCHEN    (M3b, M6, M7)
  └─ none, all delivered ─▶ GO_HOME                     (M1, M5)
RETURN_KITCHEN ──▶ GO_HOME ──▶ IDLE
```

Implementation options, best first for this brief:

1. **BehaviorTree.CPP v4 + Groot2.** It's the language Nav2 itself speaks, you get a visual tree to show off ("presentation matters"), and timeouts and preemption are built-in decorators.
2. **SMACH / YASMIN (ROS 2).** An explicit FSM with a visual viewer.
3. **A hand-written FSM** (an `Enum` plus a transition table) in an `rclpy` node with a `MultiThreadedExecutor` and `ReentrantCallbackGroup`, so waiting for confirmations never blocks action callbacks.

Whichever you pick, follow these rules:

- Use the Nav2 `NavigateToPose` action client asynchronously. Never use `spin_until_future_complete` inside a callback.
- Use a ROS timer (`create_timer`) for confirmation timeouts, not `time.sleep`.
- Publish `/butler/state` with `TRANSIENT_LOCAL` QoS so late-joining UIs see the current state.
- Order tables by nearest-neighbour or by Nav2 path length (`ComputePathToPose`) instead of request order. That's a cheap "optimized" point.
- Optionally make the node a **lifecycle node** to show off another ROS 2 concept.

### 6.5 Testing and demonstrating

- **Make the logic testable without Gazebo.** Inject a fake navigator, a stub that "arrives" after N ms, and write `pytest` / `launch_testing` cases, one per milestone. That's a strong signal to reviewers.
- Provide a **scenario runner**, e.g. `ros2 launch waiter_bot_core scenario.launch.py scenario:=m6`, that sends the goal and scripts the confirms and cancels.
- Record a short GIF per milestone (Gazebo plus RViz plus the state topic) and embed it in the README.
- Add CI with GitHub Actions: `colcon build && colcon test` in a `ros:humble` container.

### 6.6 Documentation outline

```
README.md
├── Demo GIF up front
├── Problem → architecture diagram
├── Quick start (3 commands: build, sim, send order)
├── Interfaces (action/srv tables)
├── Milestones 1–7: scenario, state path, command to reproduce, GIF
├── Design decisions & trade-offs (why BT vs FSM, why actions, etc.)
└── Known limitations / future work
```

### 6.7 Priority order (if you redo it in 48 h again)

1. **First 25%:** get one ROS 2 sim launch working (`robot_state_publisher` + Gazebo + diff drive + lidar), then Nav2 bringup with a saved map. Commit.
2. **Next 35%:** interfaces package plus the task-manager node with the state machine, using a fake navigator so you can unit-test all 7 milestones before touching Gazebo. Commit per milestone.
3. **Next 20%:** wire it to real Nav2, write scenario launch files, record GIFs.
4. **Last 20%:** README, a minimal UI that calls the action and services, and cleanup (`.gitignore`, remove dead files).

The main lesson from this attempt is that time went into what you can *see* (the world, meshes, GUI, many launch variants) while the *graded* part, the order-handling logic, never got built. In a time-boxed assessment, build the smallest working end-to-end slice first (even with turtlebot3 and its stock world), then polish.

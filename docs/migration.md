# Migration from the first submission

The first submission (commit `d81d18c`, tagged `legacy-submission` in M0) was written in 48 hours for ROS 1 style tooling with ROS 2 names. An internal review of it found what was wrong; the findings are summarised below. This file says what happens to every part. Nothing is deleted from history: the tag keeps the old tree reachable.

| Old path (under `ros_ws/src/waiter_bot_description/`) | Fate | Where it goes |
|---|---|---|
| `urdf/waiter_bot.xacro` | Rewritten | `waiter_bot_description/urdf/waiter_bot.urdf.xacro` (M1). Link and joint geometry reused, physics rebuilt |
| `urdf/waiter_bot.gazebo`, `urdf/waiter_bot.trans` | Dropped | Gazebo Classic plugins and ROS 1 transmissions. Replaced by `gz-sim` systems in the Xacro |
| `urdf/materials.xacro` | Reused | Merged into the new Xacro |
| `meshes/*.stl` (4.4 MB) | Reused for visuals | `waiter_bot_description/meshes/`. Scale and axes checked in M1 |
| `world/cafe.world`, `cafe3.world`, `cafe2/` | Dropped | Gazebo Classic SDF 1.6 with absolute `/home/hppmp/...` paths. Replaced by `cafe.sdf` (M2) |
| `waiter/` (125 MB: `.blend`, `.blend1`, `.obj`, `.dae`, `.stl`) | Dropped | Only meshes the new world loads are converted and kept (M2). Sources not committed |
| `map/map1.*`, `UI/map1.*` | Dropped | Duplicate maps with a broken, absolute `image:` path. New map in M2 and M3 |
| `config/*.yaml`, `param/*.yaml` | Dropped | ROS 1 `move_base`, `DWAPlannerROS`, `NavfnROS`. Replaced by one `nav2_params.yaml` (M3) |
| `launch/*.py`, `navigation/*.launch.py` | Rewritten | `waiter_bot_gazebo`, `waiter_bot_navigation` and `waiter_bot_bringup` launch files. None of the old ones imported correctly |
| `launch/controller.yaml`, `controller_launch.py` | Dropped | `ros_control` position controllers on drive wheels. Gazebo's `DiffDrive` replaces them |
| `src/control_panel.py`, `UI/control_panel.py`, `UI/control_panel_1.py`, `UI/control.ui` | Rewritten | `waiter_bot_ui` dashboard (M6). Icons reused if they fit. One file, built from parameters, no generated Qt code edited by hand |
| `rviz_config/*.rviz`, `launch/urdf.rviz` | Rewritten | RViz 2 configurations (M3, M6) |
| `package.xml`, `CMakeLists.txt` | Rewritten | One manifest per new package, real maintainer data, correct dependencies |
| Hardcoded table list `t1..t9`, `kitchen`, `charge` | Replaced | `locations.<name>` parameters in `cafe.yaml`; the old coordinates are not valid in the new world |
| `README.md` | Rewritten | M7. The README stays the single README of the repository |
| `.vscode/` | Ignored | Added to `.gitignore` |

Repository layout change: the `ros_ws/` wrapper folder is removed and `src/` moves to the repository root, so the repository root is the colcon workspace.

## What the first submission did well, and keeps

- The idea of a café world with a kitchen, tables and counter.
- The robot's look: meshes, proportions, and the sensor layout.
- A control panel for operators, now driven by the real interfaces.

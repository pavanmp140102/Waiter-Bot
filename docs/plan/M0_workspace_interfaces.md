# M0: Workspace and interfaces

**Goal:** a clean colcon workspace that builds, with every package present as a skeleton and the interfaces defined. Later milestones only add files to packages that already exist.

**Depends on:** nothing.

## Sub-tasks

- [ ] Tag the first submission: `git tag legacy-submission d81d18c`.
- [ ] Move `ros_ws/src/*` to `src/` and remove the `ros_ws/` wrapper, so the repository root is the workspace. Use `git mv`.
- [ ] Remove everything marked **Dropped** in [../migration.md](../migration.md). Keep the files marked **Reused** in a staging folder until M1 and M2 take them.
- [ ] `.gitignore`: `build/`, `install/`, `log/`, `.vscode/`, `__pycache__/`, `*.blend1`, `.claude/`, `CLAUDE.md`.
- [ ] `LICENSE` (MIT), `<license>MIT</license>` and real maintainer data in every manifest. Every Python and CMake file starts with the short MIT header that `ament_copyright` expects.
- [ ] `waiter_bot_interfaces` (`ament_cmake`): `ButlerState.msg`, `DeliverOrders.action`, `Confirm.srv`, `CancelTable.srv`, exactly as in [../interfaces.md](../interfaces.md).
- [ ] `waiter_bot_description`, `waiter_bot_gazebo`, `waiter_bot_navigation` (`ament_cmake`, data only): manifest and `CMakeLists.txt` installing their folders.
- [ ] `waiter_bot_core`, `waiter_bot_bringup`, `waiter_bot_ui` (`ament_python`): `package.xml`, `setup.py`, `setup.cfg`, `resource/`, an empty module with a test.
- [ ] Declare dependencies in each `package.xml` (`rclpy`, `rclpy` action and service types, `nav2_msgs`, `geometry_msgs`, `visualization_msgs`, `ros_gz_sim`, `ros_gz_bridge`, `nav2_bringup`, `xacro`, `robot_state_publisher`, `python3-pyqt5`, test deps).
- [ ] Lint with `ament_flake8`, `ament_pep257`, `ament_copyright`, `ament_xmllint` through each package's tests.
- [ ] Local, untracked `CLAUDE.md` with the workflow and conventions of this repository.
- [ ] Record the confirmed build and test commands in [../setup.md](../setup.md).

## Files created

```
.gitignore  LICENSE
src/waiter_bot_interfaces/{package.xml,CMakeLists.txt,msg/,srv/,action/}
src/waiter_bot_description/{package.xml,CMakeLists.txt}
src/waiter_bot_gazebo/{package.xml,CMakeLists.txt}
src/waiter_bot_navigation/{package.xml,CMakeLists.txt}
src/waiter_bot_core/{package.xml,setup.py,setup.cfg,resource/,waiter_bot_core/,test/}
src/waiter_bot_bringup/{package.xml,setup.py,setup.cfg,resource/,waiter_bot_bringup/,test/}
src/waiter_bot_ui/{package.xml,setup.py,setup.cfg,resource/,waiter_bot_ui/,test/}
```

## Gate

- S1: `colcon build --symlink-install` succeeds.
- S2: lint tests pass on the skeleton packages.
- S5 and S8: no absolute or foreign paths, and the tracked size is small (the heavy assets are gone).
- `ros2 interface show waiter_bot_interfaces/action/DeliverOrders` prints the definition.

## Notes

- Keep field and parameter names exactly as in [../interfaces.md](../interfaces.md). M4 and M6 are built against them.
- Do not rewrite history. The old tree stays reachable through the tag.
- The single README of the repository is the root `README.md`. It is rewritten in M7; until then it carries a short "rebuild in progress" note.

## Result

(filled in when the milestone lands)

# M2: World and map

**Goal:** a café world with a kitchen, three tables and a home spot, a saved map of it, and a single location table that the world, the map and the parameters all agree on.

**Depends on:** M1.

## Sub-tasks

- [x] Decide the layout: final `home`, `kitchen`, `table1..3` poses, written into [../world_and_robot.md](../world_and_robot.md).
- [x] `worlds/cafe.sdf` (SDF 1.9): floor, walls, kitchen counter, three tables with chairs, a few obstacles, lighting, the `Sensors` and `UserCommands` and `SceneBroadcaster` systems.
- [x] Model folders under `models/` (`model.config`, `model.sdf`). Reuse old meshes only if converted, decimated, and under 1 MB each; otherwise use primitive shapes.
- [x] Floor markers for `home` and `kitchen` so the demo shows the targets.
- [x] `sim.launch.py` extends `GZ_SIM_RESOURCE_PATH` from the installed share directory. No absolute paths.
- [x] Drive the robot with `teleop_twist_keyboard` while `slam_toolbox` runs (MAN1). Save with `map_saver_cli`. Clean up the map in a pixel editor if needed.
- [x] `waiter_bot_navigation/maps/cafe.{yaml,pgm}` with a relative `image:` entry, resolution 0.05 m.
- [x] `waiter_bot_core/config/cafe.yaml` with the locations (the parameters of [../interfaces.md](../interfaces.md)).
- [x] Test S4: every location is free with 0.4 m clearance in the map, and matches the world entity within 0.05 m.
- [x] Test S8: the repository stays under 15 MB.

## Files created

```
src/waiter_bot_gazebo/{worlds/cafe.sdf,models/*/,test/test_world.py}
src/waiter_bot_navigation/maps/{cafe.yaml,cafe.pgm}
src/waiter_bot_core/config/cafe.yaml
```

## Gate

- S3 on the world and every model (`gz sdf -k`).
- S4: locations against the map and the world.
- S8: size and tracked binaries.
- `sim.launch.py world:=cafe` starts, the robot spawns at `home`, and the lidar sees the walls (N1 in the café).

## Notes

- The map is generated once and committed. The test checks it; it is not regenerated on every build.
- Keep chairs and tables as static models. They are obstacles, not interactive objects.
- Time-box the visuals. A clean, simple café that works beats an elaborate one that does not load.

## Result

The café world (`cafe.sdf`) loads with five floor markers, a kitchen counter, three tables with chairs and two columns, built from primitive-shape models under `models/`. The saved map is `cafe.{yaml,pgm}` at 0.05 m, about 157 KB. `sim.launch.py world:=cafe` starts headless and the lidar returns ranges.

- The lidar minimum range rose from 0.12 m to 0.45 m, because the lidar renders the robot's own body meshes (returns up to 0.42 m). [../world_and_robot.md](../world_and_robot.md) records it.
- S3 covers the world and each model. S4 (`test_locations_map.py`) checks 0.4 m of free map around every location and a world marker within 0.05 m. S8 checks the repository is under 15 MB with no `.blend` files.
- `cafe.yaml` locations are unchanged from the provisional layout, so the M4 task logic needed no edit.

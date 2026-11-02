# M2: World and map

**Goal:** a café world with a kitchen, three tables and a home spot, a saved map of it, and a single location table that the world, the map and the parameters all agree on.

**Depends on:** M1.

## Sub-tasks

- [ ] Decide the layout: final `home`, `kitchen`, `table1..3` poses, written into [../world_and_robot.md](../world_and_robot.md).
- [ ] `worlds/cafe.sdf` (SDF 1.9): floor, walls, kitchen counter, three tables with chairs, a few obstacles, lighting, the `Sensors` and `UserCommands` and `SceneBroadcaster` systems.
- [ ] Model folders under `models/` (`model.config`, `model.sdf`). Reuse old meshes only if converted, decimated, and under 1 MB each; otherwise use primitive shapes.
- [ ] Floor markers for `home` and `kitchen` so the demo shows the targets.
- [ ] `sim.launch.py` extends `GZ_SIM_RESOURCE_PATH` from the installed share directory. No absolute paths.
- [ ] Drive the robot with `teleop_twist_keyboard` while `slam_toolbox` runs (MAN1). Save with `map_saver_cli`. Clean up the map in a pixel editor if needed.
- [ ] `waiter_bot_navigation/maps/cafe.{yaml,pgm}` with a relative `image:` entry, resolution 0.05 m.
- [ ] `waiter_bot_core/config/cafe.yaml` with the locations (the parameters of [../interfaces.md](../interfaces.md)).
- [ ] Test S4: every location is free with 0.4 m clearance in the map, and matches the world entity within 0.05 m.
- [ ] Test S8: the repository stays under 15 MB.

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

(filled in when the milestone lands)

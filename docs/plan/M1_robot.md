# M1: Robot

**Goal:** the `waiter_bot` robot spawns in an empty Gazebo Harmonic world, drives from `/cmd_vel`, and publishes odometry, TF and a lidar scan, all bridged to ROS 2.

**Depends on:** M0.

## Sub-tasks

- [ ] Audit the old Xacro: bounding box, wheel radius and separation, mesh scale and axes, inertias. Write the findings in the **Result** section.
- [ ] `urdf/waiter_bot.urdf.xacro` with properties for every dimension: `base_footprint`, `base_link`, four wheels, `lidar_link`, `camera_link`. Visual meshes from the old STLs, collision with primitives, masses and inertias from `xacro` macros (box and cylinder inertia).
- [ ] Rename joints to a consistent scheme (`rf_wheel_joint`, ...).
- [ ] `gz-sim` systems in the Xacro: `DiffDrive` (two joints per side, `odom`, `base_footprint` frame, TF on), `JointStatePublisher`, `Sensors` with `ogre2`, a `gpu_lidar` sensor.
- [ ] Wheel friction: low `mu2` so the four-wheel base can turn. Casters as frictionless spheres, or removed (decide from the turn test).
- [ ] `waiter_bot_gazebo/config/bridge.yaml` for `ros_gz_bridge` as in [../world_and_robot.md](../world_and_robot.md).
- [ ] `waiter_bot_gazebo/launch/sim.launch.py`: Gazebo server (and GUI by argument), `robot_state_publisher` with the Xacro output as `robot_description`, `ros_gz_sim create`, `parameter_bridge`. Arguments: `world`, `headless`, `x`, `y`, `yaw`, `use_sim_time`.
- [ ] `waiter_bot_description/launch/display.launch.py` to view the model in RViz alone.
- [ ] RViz 2 configuration for the robot.
- [ ] Static test for S3: `xacro` then `check_urdf`.
- [ ] Launch test N1 on an empty world.
- [ ] Turn and drive test: command 1 rad/s for 6.28 s, and 0.3 m/s for 3 s; record the real turn and distance error.

## Files created

```
src/waiter_bot_description/{urdf/waiter_bot.urdf.xacro,meshes/,rviz/robot.rviz,launch/display.launch.py,test/}
src/waiter_bot_gazebo/{config/bridge.yaml,worlds/empty.sdf,launch/sim.launch.py,test/}
```

## Gate

- S3: Xacro and URDF check pass.
- N1: with an empty world, `/scan` at about 10 Hz, `/odom` present, TF `odom -> base_footprint -> ... -> lidar_link` present, within 60 s of launch.
- A velocity command moves the robot: the turn is within 10 percent of the command and the distance within 5 percent.

## Notes

- The odometry frame comes from `DiffDrive`. Do not also publish it from a second node.
- If the real turn is worse than 10 percent, switch to two driven wheels (see the risk in [overview.md](overview.md)) and record why.
- `use_sim_time:=true` for every node, set from the launch arguments.

## Result

(filled in when the milestone lands)

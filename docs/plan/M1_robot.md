# M1: Robot

**Goal:** the `waiter_bot` robot spawns in an empty Gazebo Harmonic world, drives from `/cmd_vel`, and publishes odometry, TF and a lidar scan, all bridged to ROS 2.

**Depends on:** M0.

## Sub-tasks

- [x] Audit the old Xacro: bounding box, wheel radius and separation, mesh scale and axes, inertias. Write the findings in the **Result** section.
- [x] `urdf/waiter_bot.urdf.xacro` with properties for every dimension: `base_footprint`, `base_link`, four wheels, `lidar_link`, `camera_link`. Visual meshes from the old STLs, collision with primitives, masses and inertias from `xacro` macros (box and cylinder inertia).
- [x] Rename joints to a consistent scheme (`rf_wheel_joint`, ...).
- [x] `gz-sim` systems in the Xacro: `DiffDrive` (two joints per side, `odom`, `base_footprint` frame, TF on), `JointStatePublisher`, `Sensors` with `ogre2`, a `gpu_lidar` sensor.
- [x] Wheel friction: low `mu2` so the four-wheel base can turn. Casters as frictionless spheres, or removed (decide from the turn test).
- [x] `waiter_bot_gazebo/config/bridge.yaml` for `ros_gz_bridge` as in [../world_and_robot.md](../world_and_robot.md).
- [x] `waiter_bot_gazebo/launch/sim.launch.py`: Gazebo server (and GUI by argument), `robot_state_publisher` with the Xacro output as `robot_description`, `ros_gz_sim create`, `parameter_bridge`. Arguments: `world`, `headless`, `camera`, `ground_truth`, `x`, `y`, `z`, `yaw`. Simulation time is always on.
- [x] `waiter_bot_description/launch/display.launch.py` to view the model in RViz alone.
- [x] RViz 2 configuration for the robot.
- [x] Static test for S3: `xacro` then `check_urdf`.
- [x] Launch test N1 on an empty world.
- [x] Turn and drive test: command 1 rad/s for 6.28 s, and 0.3 m/s for 3 s; record the real turn and distance error.

## Files created

```
src/waiter_bot_description/{urdf/waiter_bot.urdf.xacro,meshes/,rviz/robot.rviz,launch/display.launch.py,test/}
src/waiter_bot_gazebo/{config/bridge.yaml,worlds/empty.sdf,launch/sim.launch.py,test/}
```

## Gate

- S3: Xacro and URDF check pass.
- N1: with an empty world, `/scan` at about 10 Hz, `/odom` present, TF `odom -> base_footprint -> ... -> lidar_link` present, within 60 s of launch.
- N1b and N1c: with the robot driven by `/cmd_vel`, the true distance is within 5 percent of the odometry's, and the true turn within 10 percent of the odometry's.

## Notes

- The odometry frame comes from `DiffDrive`. Do not also publish it from a second node.
- The odometry is the reference because it follows the wheels, not the command. The gap between it and the ground truth is the slip of the four-wheel base. A fall-back to two driven wheels was planned for a slip above 10 percent; it was not needed.
- `use_sim_time:=true` for every node, set from the launch arguments.

## Result

Done. The robot spawns in an empty Gazebo Harmonic world, drives from `/cmd_vel`, and publishes odometry, TF, a lidar scan, joint states and a camera image through one bridge file. N1, N1b and N1c pass on every run (seven runs, about 18 s each), and the description and gazebo packages pass all their tests, lint included.

Audit of the original model (it was wrong in my first plan):

- Units are millimetres, scale 0.001. The body is **0.567 by 0.537 m and 1.29 m tall**, not the 0.48 by 0.36 m I assumed. Wheels: 127 mm diameter, 25.4 mm wide, at x = ±0.125 m and y = ±0.165 m (track 0.33 m).
- The body is a full-footprint tower with the lidar mount inside it, so the lidar would see its own body. The physics model therefore leaves a gap between a low chassis and the tower collision for the lidar plane.
- The original mass (83 kg) and inertias came from the CAD export, and the castors were tiny decoration. They are replaced as described in [../world_and_robot.md](../world_and_robot.md).

Measured (headless, this machine, real-time factor about 0.9), final configuration:

- Drive 0.3 m/s for 3 s: true distance 0.859 m, odometry 0.856 m, heading change 0.000 rad. Reverse 0.3 m/s for 2 s: 0.649 m against 0.653 m.
- Turn 1 rad/s for 3 s: true 2.61 rad, odometry 2.80 rad, so the four-wheel base slips about 7 percent, inside the 10 percent gate. The turn test uses 2 s (about 1.8 rad) to stay well clear of the wrap at pi.
- Lidar 10 Hz in simulated time, frame `lidar_link`, ranges all `inf` in an empty world, as expected.
- Spawn to first sensor data: under 25 s.

The friction took four attempts, and the first explanation I wrote down was wrong:

| Setup | Straight, true against odometry | Turn, true against odometry |
|---|---|---|
| `mu2` 0.2, no `fdir1` | good | 1.42 rad against 3.0 commanded (47 percent) |
| `fdir1` along the link x axis, `mu2` 0.05 | good | 2.59 rad against 2.77 odometry |
| `fdir1` `0 1 0`, as a first review fix | 0.909 m against 0.864 m | **0.000 rad**: no turn at all |
| `fdir1` `0 0 1`, axle, final | 0.859 m against 0.856 m | 2.61 rad against 2.80 odometry |

- The second setup worked, but only by luck: `fdir1` is expressed in the collision frame and the link x axis turns with the wheel, so the high-friction direction did not stay pointing along the roll. I had also changed `mu2` at the same time, so crediting `fdir1` for the improvement was not justified.
- The review fix of `0 1 0` assumed the collision frame equals the link frame. The wheel cylinder is turned 90 degrees about x, so in that frame `0 1 0` points up. The ground then gets no usable friction direction, friction becomes equal in every direction, and a skid-steer base cannot turn. The axle is `0 0 1` in the collision frame.

What the tests and runs found:

- **Meshes did not load** ("Unable to find file model://waiter_bot_description/..."): the resource path I set pointed at the wrong package folder. It now points at the folder that contains `waiter_bot_description`.
- **Every test run leaked a `gz sim` server.** `gz_sim.launch.py` starts the simulator through a shell, so stopping the launch killed the shell and left the server running. The launch file now runs `gz sim` directly, with a longer grace period, and leaves no process behind. This would have put extra `/clock` publishers on the graph in later milestones.

Code review (`/code-review high` on the PR, findings posted as comments there) found ten issues, all addressed:

- **Physics:** the friction direction rotated with the wheel (see above).
- **Test robustness:** a random ROS domain above 101 can collide with Linux ephemeral ports (now derived from the process id within 1 to 100, in the gazebo test and the node tests of `waiter_bot_core`); the drive helper could spin until the 240 s timeout (now has a wall-clock deadline and fails with a message); tests 2 and 3 depended on test 1 (the spawn wait is now in `setUpClass`); the turn test sat near the wrap at pi (shorter turn); the resting-height tolerance was loose (now 0.01 m).
- **Production model:** the test-only pose publisher and its bridge entry shipped in every spawn. They are now behind `ground_truth:=true` (default false), with their own bridge file; only the simulation test turns it on (tests added to `test_urdf.py`).
- **Launch:** `headless:=True` or `1` silently started the GUI (now normalised); the spawn height was hard-coded (now the `z` argument).
- **Housekeeping:** a missing docstring on a nested helper, and two undeclared dependencies in `waiter_bot_description`.

Differences from the plan:

- The Xacro is one file, with the physics in the same file and the plugins in `<gazebo>` blocks, instead of separate `.gazebo` and `.trans` files.
- `sim.launch.py` has no `use_sim_time` argument: everything it starts uses simulation time. It has `camera` (default true) so tests can run lighter.
- The N tests carry the ctest label `sim`, so the fast Docker `test` service leaves them out with `-LE sim` and the later `test-e2e` service runs them.
- Not exercised: `display.launch.py` and `rviz/robot.rviz` (they need a display; the RViz configuration is written but has not been opened), and the Gazebo GUI path of `sim.launch.py`.


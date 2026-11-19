# M3: Navigation

**Goal:** Nav2 on Jazzy localises the robot on the saved map and drives it to any named location through `NavigateToPose`, in simulation, reliably.

**Depends on:** M2.

## Sub-tasks

- [x] Confirm the type of Nav2's `cmd_vel` on this install (`Twist` expected, see [../architecture.md](../architecture.md) section 1) and set the bridge to match.
- [x] `waiter_bot_navigation/config/nav2_params.yaml`: start from the installed `nav2_bringup` parameters. Set the robot radius, the footprint, the lidar topic, the goal tolerances, the controller (Regulated Pure Pursuit or DWB), the planner (`NavfnPlanner` or `SmacPlanner2D`), AMCL (`set_initial_pose: true` with the home pose), `use_sim_time`.
- [x] `launch/navigation.launch.py`: include `nav2_bringup`'s `bringup_launch.py` (or its localization and navigation launch files) with `IncludeLaunchDescription`, our map and parameter file, `use_sim_time`. Nothing is started as a plain `Node` that is really a launch file.
- [x] RViz 2 configuration with the map, costmaps, plan, particle cloud, and robot.
- [x] A wait helper in the test harness: Nav2 lifecycle nodes active, `/navigate_to_pose` server available, AMCL pose published.
- [x] Tests N2 to N5: lifecycle active, AMCL converged, trips to every location, cancel works.
- [x] Tune: speeds, inflation radius, and goal tolerance so every trip meets the success definition and none takes more than 60 s.

## Files created

```
src/waiter_bot_navigation/{config/nav2_params.yaml,launch/navigation.launch.py,rviz/nav.rviz,test/}
```

## Gate

N1 to N5, from [../test_plan.md](../test_plan.md): the simulation starts, Nav2 reaches `active`, AMCL converges, three rounds of trips to kitchen, each table, and home all meet the 0.30 m and 0.35 rad success definition, and a cancel stops the robot within 2 s.

## Notes

- Use only `NavigateToPose`. The task manager never talks to the planner or controller servers.
- Tune on the real map and world of M2, not on a TurtleBot3 setup. A good starting point is not a finished one.
- If the success rate is under 100 percent, record the failures and their causes in the **Result** section before tuning further.

## Result

Nav2 on Jazzy localises on the café map and drives to every named location. `navigation.launch.py` includes `nav2_bringup`'s `bringup_launch.py` with our map and parameters (`rviz:=true` opens RViz). All ten lifecycle nodes reach `active`, AMCL starts at the home pose with a small covariance, three rounds of kitchen, table1 to 3 and home all succeed (trips take 11 to 22 s, limit 60 s), and a cancel stops the robot within 2 s. The test is `test/test_navigation.py` (label `sim`, about 4 minutes).

- `cmd_vel` is `geometry_msgs/Twist` end to end (controller, velocity smoother, collision monitor, bridge), confirmed by the robot driving.
- Controller: Regulated Pure Pursuit, which suits the skid-steer base, at 0.4 m/s. Footprint polygon 0.58 by 0.54 m, inflation 0.55 m, lidar range 0.45 to 6 m to match the sensor minimum from M2.
- First run: 14 of 15 trips met the success definition. The failure was the kitchen at 0.32 m from the true pose, against a 0.30 m limit, with a goal tolerance of 0.15 m. The robot ends 0.2 to 0.25 m short of the kitchen pose in true position (localisation offset next to the flat counter wall). Tightening the goal tolerance to 0.08 m and 0.12 rad cleared it: the worst final error was 0.25 m, kitchen, and every heading error was under 0.14 rad.
- The kitchen is the weakest stop. If M5 shows trouble there, check the AMCL offset near the counter.

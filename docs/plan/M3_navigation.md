# M3: Navigation

**Goal:** Nav2 on Jazzy localises the robot on the saved map and drives it to any named location through `NavigateToPose`, in simulation, reliably.

**Depends on:** M2.

## Sub-tasks

- [ ] Confirm the type of Nav2's `cmd_vel` on this install (`Twist` expected, see [../architecture.md](../architecture.md) section 1) and set the bridge to match.
- [ ] `waiter_bot_navigation/config/nav2_params.yaml`: start from the installed `nav2_bringup` parameters. Set the robot radius, the footprint, the lidar topic, the goal tolerances, the controller (Regulated Pure Pursuit or DWB), the planner (`NavfnPlanner` or `SmacPlanner2D`), AMCL (`set_initial_pose: true` with the home pose), `use_sim_time`.
- [ ] `launch/navigation.launch.py`: include `nav2_bringup`'s `bringup_launch.py` (or its localization and navigation launch files) with `IncludeLaunchDescription`, our map and parameter file, `use_sim_time`. Nothing is started as a plain `Node` that is really a launch file.
- [ ] RViz 2 configuration with the map, costmaps, plan, particle cloud, and robot.
- [ ] A wait helper in the test harness: Nav2 lifecycle nodes active, `/navigate_to_pose` server available, AMCL pose published.
- [ ] Tests N2 to N5: lifecycle active, AMCL converged, trips to every location, cancel works.
- [ ] Tune: speeds, inflation radius, and goal tolerance so every trip meets the success definition and none takes more than 60 s.

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

(filled in when the milestone lands)

# M6: Dashboard and visuals

**Goal:** operators and viewers can see and drive the system without a terminal: a dashboard to place, confirm and cancel orders, and RViz markers that show where the robot is going and why.

**Depends on:** M5.

## Sub-tasks

- [ ] `waiter_bot_ui` node `dashboard` (`PyQt5`), built from the parameters of `butler_task_manager` read with an async parameter client: one order button per table, one confirm button per location, a cancel-table button per table, a cancel-all button.
- [ ] Order basket: select several tables, press one button to send one `DeliverOrders` goal (R5 to R7 by hand).
- [ ] State panel: current state, target, pending, delivered, skipped, cancelled, a countdown for the confirmation timer, and a short history of states. Driven by `/butler/state`.
- [ ] ROS and Qt together without blocking: the ROS executor runs on a background thread, and updates reach the Qt thread through signals. No `rclpy.spin` on the main thread.
- [ ] Buttons are enabled only when the matching service or action would accept the call, using the state message.
- [ ] `butler_task_manager` publishes `/butler/markers`: one text and one arrow per location, the current target highlighted, the state text above the robot.
- [ ] RViz configuration showing map, robot, plan, location markers and state text.
- [ ] `dashboard` option in `butler.launch.py`.
- [ ] A headless test: start the dashboard with `QT_QPA_PLATFORM=offscreen`, press the buttons through Qt test calls, and check the goals and services received by a fake task manager.

## Files created

```
src/waiter_bot_ui/waiter_bot_ui/{dashboard.py,widgets.py}
src/waiter_bot_ui/test/test_dashboard.py
src/waiter_bot_navigation/rviz/butler.rviz
```

## Gate

- The offscreen dashboard test passes and lint stays clean.
- MAN2: place a three-table order, watch the dashboard and RViz together, and use the confirm and cancel buttons to reproduce R3b, R6 and R7 by hand.

## Notes

- The dashboard calls only the three public interfaces. It never publishes `cmd_vel` or talks to Nav2. The first submission's manual-drive pad is not carried over.
- Adding a table to `cafe.yaml` must add a button, with no UI code change (checked with E9 data).
- Keep the styling plain and readable: colour for state, large text for the countdown.

## Result

(filled in when the milestone lands)

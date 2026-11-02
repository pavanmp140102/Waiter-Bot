# M4: Task logic

**Goal:** the butler's behaviour, complete and proven: a ROS-free state machine that implements R1 to R7, and the `rclpy` node that connects it to ROS and Nav2. No simulator is needed to finish this milestone.

**Depends on:** M0.

## Sub-tasks

### `butler_fsm` (ROS-free)

- [ ] Data types with `dataclasses` and `enum`: `State`, `Event` variants, `Command` variants, `Order` status (`pending`, `delivered`, `skipped`, `cancelled`), `Config` (timeouts, switches, retries).
- [ ] `ButlerFsm.handle(event) -> list[Command]`, one method per event: order received, arrived, nav failed, confirmed, timeout, cancel all, cancel table. Event handlers are small and return commands, never raise on a stale event.
- [ ] `next_leg()` as specified in [../architecture.md](../architecture.md) section 5, the single place that chooses the next table, the kitchen return, or home.
- [ ] `food_onboard` as a property of the order list and the `picked_up` flag.
- [ ] Goal validation (unknown table, duplicate, empty list, busy), returning a reason string.
- [ ] Retry bookkeeping for navigation failures (D-e).
- [ ] Stale-event rule: a timeout or confirmation that no longer matches the current state is ignored (T20).
- [ ] A `snapshot()` method for the state message and the action feedback.

### `locations` (ROS-free)

- [ ] `Pose2D` dataclass and a `LocationRegistry` built from a plain dict, with validation: every name has `x`, `y`, `yaw`; home, kitchen and tables exist.

### `butler_task_manager` node (`rclpy`)

- [ ] Parameters declared from [../interfaces.md](../interfaces.md) and loaded from `config/cafe.yaml`. Fail fast on a missing location.
- [ ] Action server `/butler/deliver_orders` with goal, cancel and execute callbacks (a `ReentrantCallbackGroup`). The execute callback waits on a `threading.Event`, no polling.
- [ ] Service servers `/butler/confirm` and `/butler/cancel_table` in their own callback groups.
- [ ] `NavigateToPose` action client; goal, feedback, result and cancel done-callbacks turn into `Arrived` or `NavFailed` events. No blocking waits.
- [ ] One-shot timers on the node clock for confirmation timeouts, cancelled when the guarded event arrives.
- [ ] `/butler/state` publisher, transient local, on every change and once a second while a timer runs.
- [ ] A command executor: maps `Navigate`, `StartTimer`, `StopTimer`, `PublishState`, `Finish` to ROS calls. This is the only place that touches ROS.
- [ ] `MultiThreadedExecutor` in `main()`, with a clean shutdown that cancels any active Nav2 goal.
- [ ] Console script entry point `butler_task_manager`, and `launch/task_manager.launch.py` that loads the YAML.

### Tests

- [ ] T1 to T20 as `pytest` files in `waiter_bot_core/test/`, one test per row of [../test_plan.md](../test_plan.md).
- [ ] T17: a seeded random-sequence test that checks the invariants after every event.
- [ ] A fake `NavigateToPose` action server for TN tests, with controllable delay, success and failure.
- [ ] TN1 to TN7, spinning the node and the fake server in one process.
- [ ] S6: a static test that the two ROS-free modules import no ROS package.

## Files created

```
src/waiter_bot_core/waiter_bot_core/{butler_fsm.py,locations.py,task_manager.py}
src/waiter_bot_core/{config/cafe.yaml,launch/task_manager.launch.py}
src/waiter_bot_core/test/{test_fsm_*.py,test_locations.py,test_node_*.py,fake_nav.py,test_no_ros_in_logic.py}
```

## Gate

T1 to T20, TN1 to TN7 and S6 pass, and S2 lint stays clean. Run `colcon test --packages-select waiter_bot_core`; it takes seconds, with no simulator.

## Notes

- The FSM never reads a clock and never creates a thread. Time is an event (`Timeout`) that the node produces. This is what keeps T1 to T20 deterministic.
- Keep `butler_fsm.py` short and readable: a reviewer should be able to read it in one sitting and check R1 to R7 against it. The target is under 300 lines.
- Prefer a small table or a few well-named methods over deep nesting. Do not special-case any requirement ID in code.
- Never call `spin_until_future_complete` inside a callback, and never use `time.sleep` for timeouts.

## Result

(filled in when the milestone lands)

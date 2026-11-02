# M4: Task logic

**Goal:** the butler's behaviour, complete and proven: a ROS-free state machine that implements R1 to R7, and the `rclpy` node that connects it to ROS and Nav2. No simulator is needed to finish this milestone.

**Depends on:** M0.

## Sub-tasks

### `butler_fsm` (ROS-free)

- [x] Data types with `dataclasses` and `enum`: `State`, `Event` variants, `Command` variants, `Order` status (`pending`, `delivered`, `skipped`, `cancelled`), `Config` (timeouts, switches, retries).
- [x] `ButlerFsm.handle(event) -> list[Command]`, one method per event: order received, arrived, nav failed, confirmed, timeout, cancel all, cancel table. Event handlers are small and return commands, never raise on a stale event.
- [x] `next_leg()` as specified in [../architecture.md](../architecture.md) section 5, the single place that chooses the next table, the kitchen return, or home.
- [x] `food_onboard` as a property of the order list and the `picked_up` flag.
- [x] Goal validation (unknown table, duplicate, empty list, busy), returning a reason string.
- [x] Retry bookkeeping for navigation failures (D-e).
- [x] Stale-event rule: a timeout or confirmation that no longer matches the current state is ignored (T20).
- [x] A `snapshot()` method for the state message and the action feedback.

### `locations` (ROS-free)

- [x] `Pose2D` dataclass and a `LocationRegistry` built from a plain dict, with validation: every name has `x`, `y`, `yaw`; home, kitchen and tables exist.

### `butler_task_manager` node (`rclpy`)

- [x] Parameters declared from [../interfaces.md](../interfaces.md) and loaded from `config/cafe.yaml`. Fail fast on a missing location.
- [x] Action server `/butler/deliver_orders` with goal, cancel and execute callbacks (a `ReentrantCallbackGroup`). The execute callback waits on a `threading.Event`, no polling.
- [x] Service servers `/butler/confirm` and `/butler/cancel_table` in their own callback groups.
- [x] `NavigateToPose` action client; goal, feedback, result and cancel done-callbacks turn into `Arrived` or `NavFailed` events. No blocking waits.
- [x] One-shot timers on the node clock for confirmation timeouts, cancelled when the guarded event arrives.
- [x] `/butler/state` publisher, transient local, on every change and once a second while a timer runs.
- [x] A command executor: maps `Navigate`, `StartTimer`, `StopTimer`, `PublishState`, `Finish` to ROS calls. This is the only place that touches ROS.
- [x] `MultiThreadedExecutor` in `main()`, with a clean shutdown that cancels any active Nav2 goal.
- [x] Console script entry point `butler_task_manager`, and `launch/task_manager.launch.py` that loads the YAML.

### Tests

- [x] T1 to T20 as `pytest` files in `waiter_bot_core/test/`, one test per row of [../test_plan.md](../test_plan.md).
- [x] T17: a seeded random-sequence test that checks the invariants after every event.
- [x] A fake `NavigateToPose` action server for TN tests, with controllable delay, success and failure.
- [x] TN1 to TN7, spinning the node and the fake server in one process.
- [x] S6: a static test that the two ROS-free modules import no ROS package.

## Files created

```
src/waiter_bot_core/waiter_bot_core/{butler_fsm.py,locations.py,task_manager.py}
src/waiter_bot_core/{config/cafe.yaml,launch/task_manager.launch.py}
src/waiter_bot_core/test/{test_fsm_*.py,test_locations.py,test_node_*.py,fake_nav.py,test_no_ros_in_logic.py}
```

## Gate

T1 to T22, TN1 to TN7 and S6 pass, and S2 lint stays clean. Run `colcon test --packages-select waiter_bot_core`; it takes seconds, with no simulator.

## Notes

- The FSM never reads a clock and never creates a thread. Time is an event (`Timeout`) that the node produces. This is what keeps T1 to T20 deterministic.
- Keep `butler_fsm.py` short and readable: a reviewer should be able to read it in one sitting and check R1 to R7 against it. The target is under 300 lines.
- Prefer a small table or a few well-named methods over deep nesting. Do not special-case any requirement ID in code.
- Never call `spin_until_future_complete` inside a callback, and never use `time.sleep` for timeouts.

## Result

Done. The state machine, the location registry and the node are built, and every behaviour in R1 to R7 is replayed by a test.

Measured:

- 27 logic tests (T1 to T22, including T15b) run in about 0.1 s with no ROS. T17 replays 2000 random tasks (1000 with navigation failures, 1000 without) from a fixed seed and checks the invariants after every event.
- TN1 to TN7 pass in about 3.7 s against a fake Nav2 action server, six runs in a row with no failure.
- S6 passes: `butler_fsm.py` and `locations.py` import only the standard library.
- The whole package: 39 tests, 0 failures, with `ament_flake8`, `ament_pep257`, `ament_copyright` and `ament_xmllint` clean.
- Smoke run of `ros2 launch waiter_bot_core task_manager.launch.py`: the YAML parameters load (including the nested `locations`), `/butler/state` is latched and shows `idle`, and with no Nav2 an order is rejected with `nav2_unavailable` in the log. The node shuts down cleanly.

What the tests found:

- **A real design bug, caught by T17:** after the food was handed back at the kitchen, `food_onboard` stayed true, so the snapshot on the way home said food was still onboard. The machine now unloads on arrival at the kitchen (test T21).
- **Lint:** this project's `ament_pep257` wants multi-line docstring summaries on the second line (D213) and imperative first lines (D401). The code follows that.

Differences from the plan:

- The commands are `Navigate`, `StartTimer`, `StopTimer` and `Finish`. A separate `PublishState` command was dropped: the node publishes after every call, which is simpler and cannot be forgotten.
- The ROS action protocol has no reason text for a rejected goal, so the reason is logged (see [../interfaces.md](../interfaces.md)).
- An unreachable kitchen now ends with the outcome `nav_failed` after going home, and a new goal is accepted from the `error` state, so the robot can recover.
- Added T21 and T22, and made the location parameters accept integers, via a dynamically typed descriptor.
- `butler_fsm.py` is 332 lines, 311 without the licence header: slightly over the 300-line target. Most of the excess is the per-function docstrings the project requires, not logic.
- The `fork()` warning on stderr comes from flake8's multiprocessing in a process that also hosts the node tests. It is harmless.


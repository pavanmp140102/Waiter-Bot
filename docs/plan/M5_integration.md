# M5: Integration

**Goal:** one command starts the whole system, and each of the seven situations of the assessment can be replayed by name and passes as an end-to-end test.

**Depends on:** M3 and M4.

## Sub-tasks

- [ ] `waiter_bot_bringup/launch/butler.launch.py`: includes the simulation (M1, M2), navigation (M3), and the task manager (M4); arguments `world`, `headless`, `rviz`, `dashboard`, `params_file`. `use_sim_time` passed everywhere.
- [ ] Start-up ordering: the task manager starts at once and waits for the Nav2 server on its own, publishing `waiting_for_nav` until it is up. No fixed sleeps in the launch file.
- [ ] `scenario_runner` node (Python): loads a scenario file ([../interfaces.md](../interfaces.md)), waits for the system, sends the goal, plays the "people" (confirm, ignore, cancel), watches `/butler/state`, records the visited locations in order, and compares them with the `expect` block. Exit code 0 on pass.
- [ ] Scenario files in `scenarios/`: one for each row E1 to E9 of [../test_plan.md](../test_plan.md).
- [ ] `scenario.launch.py` that runs `butler.launch.py` headless plus the runner.
- [ ] Ground-truth bridge (`/ground_truth/pose`), used by the harness only, to check every arrival against the success definition.
- [ ] `launch_testing` tests E1 to E9 in `waiter_bot_bringup/test/`, each running a scenario file.
- [ ] A wait-for-ready helper shared with the N tests.
- [ ] A fourth table (`table4`) added to a copy of `cafe.yaml` and the world for E9, to prove no code change is needed.
- [ ] Run the suite three times in a row and record the pass rate.

## Files created

```
src/waiter_bot_bringup/{launch/butler.launch.py,launch/scenario.launch.py,scenarios/*.yaml}
src/waiter_bot_bringup/waiter_bot_bringup/scenario_runner.py
src/waiter_bot_bringup/test/{test_e2e_*.py,helpers.py}
```

## Gate

E1 to E9 pass, and earlier gates (S, T, TN, N) still pass. The three-run check passes with no flaky failures.

## Notes

- A scenario passes on the **path and the outcome**, not on timing. Nothing in a test asserts "after exactly N seconds".
- The runner is deliberately the same tool for tests and for demos, so what is shown in the README is what is tested.
- If a scenario is flaky, find the cause (readiness, tolerance, recovery behaviour) and fix it in the system. Do not add sleeps.

## Result

(filled in when the milestone lands)

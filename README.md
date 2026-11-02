# Waiter-Bot

A café butler robot for ROS 2 Jazzy and Gazebo Harmonic. It takes orders for tables, collects the food from the kitchen, delivers it, and returns home, handling missing confirmations and cancellations on the way.

**Status: rebuild in progress.** The first submission was written in 48 hours for an assessment and did not run on ROS 2; it is kept under the git tag `legacy-submission`. The rebuild is planned in [docs/](docs/index.md) and built milestone by milestone, see [docs/plan/overview.md](docs/plan/overview.md).

The code review of the first submission is in [REVIEW.md](REVIEW.md).

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Licensed under the MIT License, see [LICENSE](LICENSE).

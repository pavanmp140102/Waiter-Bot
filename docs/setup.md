# Setup

Written for Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic.

## Required

```bash
sudo apt install \
  ros-dev-tools python3-colcon-common-extensions python3-pytest \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-launch-testing ros-jazzy-launch-testing-ros \
  python3-pyqt5 python3-yaml liburdfdom-tools
```

The authoritative list is the `package.xml` files. To install exactly what they declare:

```bash
rosdep install --from-paths src --ignore-src -y
```

The Docker image does this, which is how undeclared dependencies get caught (M8).

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run (target commands, available once the milestones land)

```bash
# whole system: Gazebo, Nav2, RViz, the task manager, the dashboard
ros2 launch waiter_bot_bringup butler.launch.py

# place an order for three tables from a second terminal
ros2 action send_goal /butler/deliver_orders waiter_bot_interfaces/action/DeliverOrders \
  "{tables: [table1, table2, table3]}" --feedback

# act as the people
ros2 service call /butler/confirm waiter_bot_interfaces/srv/Confirm "{location: kitchen}"
ros2 service call /butler/cancel_table waiter_bot_interfaces/srv/CancelTable "{table: table2}"

# replay one of the assessment situations by name
ros2 launch waiter_bot_bringup scenario.launch.py scenario:=r6_table_ignored
```

## Tests

```bash
colcon test --packages-select waiter_bot_core        # T and TN, seconds
colcon test --packages-select waiter_bot_bringup     # N and E, slow, headless Gazebo
colcon test-result --verbose
```

Run packages one at a time when `waiter_bot_bringup` is involved: it starts a simulator on the ROS domain, which can disturb other node tests running at the same time. Before a simulation session check that `ros2 topic info /clock` shows one publisher.

## Machine notes

- Headless runs use `gz sim -s -r --headless-rendering`. If the camera or lidar does not render, try `LIBGL_ALWAYS_SOFTWARE=1`.
- If Gazebo never finds its own server, the discovery interface may be wrong on a Wi-Fi-only machine. Setting `GZ_IP=127.0.0.1` pins it to loopback. Do not hardcode this in the launch files.
- Machine-specific notes (GPU, drivers) stay out of the repository.

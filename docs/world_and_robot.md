# World and robot

## Robot `waiter_bot`

The original model is a SolidWorks export in millimetres. Its meshes are kept for looks and its physics is rebuilt. Measured from the original meshes in M1:

| Property | Original | Built in M1 |
|---|---|---|
| Body size | 0.567 m long by 0.537 m wide by 1.29 m tall, in a lower chassis and a tray tower | Same look. Collision is two boxes: a chassis (z 0.02 to 0.19 m) and a tower (z 0.25 to 1.30 m, 0.50 by 0.50 m) |
| Wheels | 127 mm diameter, 25.4 mm wide, at x = ±0.125 m, y = ±0.165 m | Same. Radius 0.0635 m, track 0.33 m |
| Mass | `base_link` 83 kg, wheels 1.4 kg each | Body 20 kg (chassis 12, tower 8), wheels 0.8 kg each, 23.2 kg in total. Inertias are computed in the Xacro from the dimensions |
| Drive | Gazebo Classic skid-steer plugin | Gazebo `DiffDrive` system with two joints per side, limits 1.0 m/s and 1.5 rad/s |
| Friction | none set | `mu1` 1.0 along the wheel, `mu2` 0.05 across it, `fdir1` along the wheel. Without the direction the turn reached only 47 percent of the commanded rate |
| Casters | Two fixed castor links | Removed. Four driven wheels carry the robot, and the body sits 20 mm off the floor |
| Sensors | 4 cameras and a lidar | A lidar (`gpu_lidar`, 360 samples, 10 Hz, 0.12 to 8 m) and one front camera (640 by 480, 15 Hz). `camera:=false` leaves the camera out |
| Joint names | `RF_W`, `LF_W`, `LB_W`, `RB_w` (inconsistent case) | `lf_wheel_joint`, `rf_wheel_joint`, `lb_wheel_joint`, `rb_wheel_joint` |
| Plugins | `libgazebo_ros2_*` (do not exist) | `gz-sim` systems: `DiffDrive`, `JointStatePublisher`, `PosePublisher` (tests only), `Sensors` with `ogre2` (in the world) |

The footprint is large for a café robot: 0.567 by 0.537 m, so a circumscribed radius of about 0.39 m. Aisles in the world are therefore at least 1.2 m wide, and Nav2 uses a polygon footprint, not a circle (M3).

The lidar plane is at 0.211 m. The chassis collision ends at 0.19 m and the tower collision starts at 0.25 m, so the scan sees the room and not the robot's own body. The original meshes, which have the lidar inside the body, are only for looks.

Frames: `base_footprint` (on the floor, REP 120 root), `base_link` (at wheel-axle height, 0.0635 m above the floor), the four wheels, `lidar_link`, `camera_link`. Odometry frame `odom`, published by `DiffDrive`.

Bridge (`waiter_bot_gazebo/config/bridge.yaml`), one YAML for `ros_gz_bridge parameter_bridge`:

| ROS topic | Direction | Gazebo type |
|---|---|---|
| `/clock` | gz to ROS | `gz.msgs.Clock` |
| `/cmd_vel` | ROS to gz | `gz.msgs.Twist` (ROS side `geometry_msgs/Twist`) |
| `/odom` | gz to ROS | `gz.msgs.Odometry` |
| `/tf` (odom to base) | gz to ROS | `gz.msgs.Pose_V` |
| `/scan` | gz to ROS | `gz.msgs.LaserScan` |
| `/joint_states` | gz to ROS | `gz.msgs.Model` |
| `/camera/image_raw`, `/camera/camera_info` | gz to ROS | `gz.msgs.Image`, `gz.msgs.CameraInfo` |
| `/ground_truth/pose` | gz to ROS | `gz.msgs.Pose` (tests only) |

## Café world `cafe.sdf` (SDF 1.9, Gazebo Harmonic)

Built from scratch for this project. Provisional layout, final coordinates are fixed in M2 and then copied into `cafe.yaml` and this table:

| Location | Role | Provisional pose (x, y, yaw) in `map` |
|---|---|---|
| `home` | Start and end position, a floor marker | (0.0, 0.0, 0.0) |
| `kitchen` | Pick-up point at the counter | (4.0, 0.0, 0.0) |
| `table1` | Servable table | (1.5, 4.0, 1.57) |
| `table2` | Servable table | (-1.5, 4.0, 1.57) |
| `table3` | Servable table | (-4.5, 4.0, 1.57) |

Rules for the layout:

- Every location is a **free** cell of the saved map, with at least 0.4 m clearance (test S4).
- Table poses are the **stopping point beside the table**, not the table centre.
- The route from kitchen to each table is at least 1.2 m wide, so Nav2 never needs to squeeze through.
- Walls, a counter, three tables with chairs, and a few columns as obstacles. Names, sizes and poses in the world file are generated from, or checked against, the same location table (S4).

### Assets

The first submission carried 125 MB: `.blend` files with their `.blend1` autosave backups, and the same model as `.obj`, `.dae` and `.stl`. For the rebuild:

- Only meshes the world actually loads are kept, as `.dae` or `.obj` under `waiter_bot_gazebo/models/<name>/meshes/`, each decimated if it exceeds 1 MB. Target total for the whole repo: under 15 MB.
- Models are Gazebo model folders (`model.config`, `model.sdf`) found through `GZ_SIM_RESOURCE_PATH`, which the launch file extends. **No absolute paths anywhere** (S5).
- Blender sources are not committed. If they must be kept, they go to Git LFS under `assets_src/`, outside the installed packages.

### Map

`waiter_bot_navigation/maps/cafe.yaml` and `cafe.pgm`, made once with `slam_toolbox` while driving the robot with the keyboard (MAN1), then trimmed and committed. The map's `image:` entry is a relative path. Resolution 0.05 m.

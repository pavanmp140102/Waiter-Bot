# World and robot

## Robot `waiter_bot`

The existing model is a SolidWorks export. Its geometry is kept for looks, and its physics is rebuilt. The audit below is done in M1; the numbers in the "Target" column are what M1 builds, and they are adjusted there if the audit shows a reason.

| Property | First submission | Target |
|---|---|---|
| Footprint | about 0.48 m long (castors at x ±0.24) by 0.36 m wide | Keep |
| Height | top camera at z = 1.29 m | Keep the body visual. Lidar at about 0.21 m (existing `lidar_j`) |
| Mass | `base_link` 83 kg, wheels 1.4 kg each | About 15 kg total, with inertias computed from simple shapes |
| Collision | Full STL meshes (1.5 MB `base_link.stl`) | Boxes and cylinders; STL meshes for visuals only |
| Drive | Skid steer plugin from Gazebo Classic | Gazebo `gz-sim-diff-drive-system` with two joints per side, wheel diameter 0.127 m, wheel separation 0.355 m (checked in M1) |
| Casters | Two fixed castor links | Frictionless spheres, or removed if the four wheels carry the robot |
| Sensors | 4 cameras and a lidar | Lidar (`gpu_lidar`, 360 samples, 10 Hz, range 0.1 to 8 m). One front camera kept for the demo view. The other cameras are dropped |
| Joint names | `RF_W`, `LF_W`, `LB_W`, `RB_w` (inconsistent case) | `rf_wheel_joint`, `lf_wheel_joint`, `lb_wheel_joint`, `rb_wheel_joint` |
| Plugins | `libgazebo_ros2_*` (do not exist) | `gz-sim` systems: `DiffDrive`, `JointStatePublisher`, `Sensors` with `ogre2` |

Frames: `base_footprint` (REP 120 root), `base_link`, wheels, `lidar_link`, `camera_link`. Odometry frame `odom`, published by `DiffDrive`.

Bridge (`waiter_bot_gazebo/config/bridge.yaml`), one YAML for `ros_gz_bridge parameter_bridge`:

| ROS topic | Direction | Gazebo type |
|---|---|---|
| `/clock` | gz to ROS | `gz.msgs.Clock` |
| `/cmd_vel` | ROS to gz | `gz.msgs.Twist` |
| `/odom` | gz to ROS | `gz.msgs.Odometry` |
| `/tf` (odom to base) | gz to ROS | `gz.msgs.Pose_V` |
| `/scan` | gz to ROS | `gz.msgs.LaserScan` |
| `/joint_states` | gz to ROS | `gz.msgs.Model` |
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

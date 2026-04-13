# Real Robot Launch Matrix

All commands assume:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 1) Base bringup + teleop

Use when you only need drivetrain/sensors/TF and drive manually.

```bash
ros2 launch mecanum_stack_bringup real_robot_teleop.launch.py \
  arduino_serial_port:=/dev/ttyACM0 \
  lidar_serial_port:=/dev/ttyUSB0 \
  lidar_scan_mode:=Standard \
  use_imu_ekf:=true
```

Run keyboard teleop in a separate terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 2) Mapping (SLAM)

Use when creating/updating maps.

```bash
ros2 launch mecanum_stack_bringup real_robot_mapping.launch.py \
  arduino_serial_port:=/dev/ttyACM0 \
  lidar_serial_port:=/dev/ttyUSB0 \
  lidar_scan_mode:=Standard \
  use_imu_ekf:=true
```

## 3) Navigation on saved map (AMCL + Nav2)

Use when running autonomous navigation with a known map.

```bash
ros2 launch mecanum_stack_bringup real_robot_navigation.launch.py \
  arduino_serial_port:=/dev/ttyACM0 \
  lidar_serial_port:=/dev/ttyUSB0 \
  lidar_scan_mode:=Standard \
  use_imu_ekf:=true \
  map_yaml:=/home/vlad/ros2_ws/aux/maps/my_lab.yaml
```

**Localization pose (AMCL)** — the stack does **not** assume you start at map \((0,0)\). After launch, set the robot pose once in RViz with **2D Pose Estimate** (or publish to `/initialpose`). Forcing a wrong initial pose makes scan matching fight odometry and looks like violent jumping on `/amcl_pose` while stationary.

**RViz fixed frame** — if **Fixed Frame** is `base_footprint`, the map appears to spin when the robot rotates (you are viewing the world in the robot frame). Use **Fixed Frame: `map`** when you want the map fixed and the robot to move on the map.

To inspect pose while stopped:

```bash
ros2 topic echo /amcl_pose
```

If heading still jitters with IMU fusion, try `use_imu_ekf:=false` (wheel odometry only for `odom` → `base_link`).

If Nav2 races startup on slower boots, increase:

```bash
navigation_startup_delay:=4.0
```

Disable IMU+EKF quickly in any profile:

```bash
use_imu_ekf:=false
```

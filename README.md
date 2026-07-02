# Collision Avoidance

## Prerequisites
This workspace depends on the **Universal Robots ROS 2 driver** (`ur_robot_driver`,
`ur_description`) and the ZED wrapper. The UR driver must be built/installed and
sourced *before* this workspace. If you built it from source (e.g. in `~/workspace/ros_ur_driver`),
source it first so `$(find ur_robot_driver)` resolves correctly:
```bash
source /opt/ros/humble/setup.bash
source ~/workspace/ros_ur_driver/install/setup.bash   # adjust to your UR driver workspace
```

## Setup
```bash
cd bimanual_ur10e_motion_planner
colcon build --symlink-install       # builds all three packages in dependency order
source install/setup.bash
```

## Usage

### Offline / no robot — load the scene in RViz (recommended first run)
Uses mock hardware (`use_fake_hardware:=true`), so the full MoveIt stack + RViz come
up without any robot on the network. You can plan and execute trajectories in RViz.
```bash
ros2 launch dual_arm_moveit_config dual_arm_fake.launch.py
```

### Real robots — connect to hardware
Brings up the real UR hardware interface and starts the collision-avoidance script.
```bash
ros2 launch dual_arm_moveit_config dual_arm_real_v2.launch.py
```
> Use **Plan and Execute** for normal trajectory (no collision).  
> For collision avoidance: drag arm to collide position and release — the real robot will automatically stop before collision.

### Collision Avoidance Script
```bash
python3 src/dual_arm_pkg/scripts/move_until_collision.py
```

## Notes
- Robot IPs are set in `src/dual_arm_moveit_config/config/dual_arm_real_v2.urdf.xacro`
  (left `192.168.1.10`, right `192.168.1.20`) — edit to match your network.
- Make sure the robot is in **Remote Control mode** before launching the real robot.

# Hand-Eye Calibration
Calibration files are located at:

`~/.ros2/easy_handeye2/calibrations/zed_right_arm_calib.calib`

`~/.ros2/easy_handeye2/calibrations/zed_left_arm_calib.calib`

> **Important:** Always use these files as the primary reference for hand-eye calibration.

# How to run Easy Hand-eye

### Terminal 1 — Robot
```bash
source install/setup.bash
ros2 launch dual_arm_moveit_config dual_arm_real_v2.launch.py
```

### Terminal 2 — Camera
```bash
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm
```

### Terminal 3 — ArUco
```bash
source install/setup.bash
ros2 run aruco_ros single \
  --ros-args \
  -r /image:=/zed/zed_node/rgb/color/rect/image \
  -r /camera_info:=/zed/zed_node/rgb/color/rect/camera_info \
  -p marker_id:=24 \
  -p marker_size:=0.10 \
  -p reference_frame:=zed_left_camera_frame \
  -p camera_frame:=zed_left_camera_frame \
  -p marker_frame:=aruco_marker_frame
```

### Terminal 4 — easy_handeye2
```bash
source /install/setup.bash
ros2 launch dual_arm_pkg handeye_calibrate.launch.py
```

### Terminal 5 
```bash
ros2 topic echo /aruco_single/pose
```


> **Note:** If some files are missing, everything is backed up here: https://drive.google.com/drive/folders/1dW5_k8s-45HRqxhzScIcTyH5_zjxkKVJ?usp=sharing

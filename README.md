# Collision Avoidance

## Setup
```bash
cd ~/ws_dual_arm_final
colcon build --packages-select dual_arm_moveit_config
source install/setup.bash
```

## Usage

### MoveIt + RViz launch file
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
- Make sure the robot is in **Remote Control mode** before launching

# Hand-Eye Calibration
Calibration files are located at:

`~/.ros2/easy_handeye2/calibrations/zed_right_arm_calib.calib`

`~/.ros2/easy_handeye2/calibrations/zed_left_arm_calib.calib`

> **Important:** Always use these files as the primary reference for hand-eye calibration.

# How to run Easy Hand-eye

### Terminal 1 — Robot
```bash
source ~/ws_dual_arm/install/setup.bash
ros2 launch dual_arm_moveit_config dual_arm_real_v2.launch.py
```

### Terminal 2 — Camera
```bash
source ~/ws_dual_arm/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm
```

### Terminal 3 — ArUco
```bash
source ~/ws_dual_arm/install/setup.bash
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
source ~/ws_dual_arm/install/setup.bash
ros2 launch dual_arm_pkg handeye_calibrate.launch.py
```

### Terminal 5 
```bash
ros2 topic echo /aruco_single/pose
```


> **Note:** If some files are missing, everything is backed up here: https://drive.google.com/drive/folders/1dW5_k8s-45HRqxhzScIcTyH5_zjxkKVJ?usp=sharing

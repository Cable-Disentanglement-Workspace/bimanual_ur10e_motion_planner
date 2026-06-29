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


# Collision Avoidance

## Setup
```bash
cd ~/ws_dual_arm_final && source install/setup.bash
```

## Usage

### MoveIt + RViz 
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

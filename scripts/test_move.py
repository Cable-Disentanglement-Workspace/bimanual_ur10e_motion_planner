import rclpy
import time
from aruco_moveit_planner.moveit_planner import MoveItPlanOnlyClient
from geometry_msgs.msg import PoseStamped


def move_with_retry(planner, target, arm="left", max_retries=5, execute=False):
    for attempt in range(max_retries):
        if arm == "left":
            success = planner.plan_to_pose(target)
        else:
            success = planner.plan_to_pose2(target)
        
        if success:
            print(f"Plan succeeded on attempt {attempt + 1}")
            return True
        
        print(f"Attempt {attempt + 1}/{max_retries} failed, retrying...")
    
    print(f"Failed after {max_retries} attempts!")
    return False

def main():
    rclpy.init()

    # Create the planner
    planner = MoveItPlanOnlyClient()

    # # Right arm - use right_base
    # right_target = PoseStamped()
    # right_target.header.frame_id = "right_base"
    # right_target.pose.position.x = -0.293   # small offset
    # right_target.pose.position.y = -0.734
    # right_target.pose.position.z = 0.848
    # right_target.pose.orientation.x = -0.530
    # right_target.pose.orientation.y = 0.548
    # right_target.pose.orientation.z = -0.435
    # right_target.pose.orientation.w = -0.479

        # Right arm - use right_base
    right_target = PoseStamped()
    right_target.header.frame_id = "right_base"
    right_target.pose.position.x = -0.337 #-0.158   # small offset
    right_target.pose.position.y = -0.364 #-0.681
    right_target.pose.position.z = 0.501 #0.449
    right_target.pose.orientation.x = -0.062 #0.083
    right_target.pose.orientation.y = 0.749 #-0.709
    right_target.pose.orientation.z = -0.659 #-0.056
    right_target.pose.orientation.w = -0.035 #0.698

    # Right arm - use right_base
    right_home = PoseStamped()
    right_home.header.frame_id = "right_base"
    right_home.pose.position.x = -0.337   # small offset
    right_home.pose.position.y = -0.314
    right_home.pose.position.z = 0.501
    right_home.pose.orientation.x = -0.062
    right_home.pose.orientation.y = 0.749
    right_home.pose.orientation.z = -0.659
    right_home.pose.orientation.w = -0.035
    planner._remove_obstacle("wall")
    # planner._add_cylinder_obstacle("wall", -0.3, -0.6, 0.5, 1.0, 0.1, frame="right_base")
    # Give time for planning scene to update
    
    time.sleep(1.0)
    if move_with_retry(planner, right_target, arm="right", execute=True):
        print("Target reached, now moving to home...")
        # 
        move_with_retry(planner, right_home, arm="right", execute=True)

    # Remove wall when done
    
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# # Left arm - use left_base
# left_target = PoseStamped()
# left_target.header.frame_id = "left_base"
# left_target.pose.position.x = -0.309
# left_target.pose.position.y = 0.631
# left_target.pose.position.z = 0.442
# left_target.pose.orientation.x = -0.800
# left_target.pose.orientation.y = -0.011
# left_target.pose.orientation.z = 0.072
# left_target.pose.orientation.w = 0.595

# success = planner.plan_to_pose(left_target)   # uses left_arm config
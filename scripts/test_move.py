import rclpy
import time
import glob
import yaml
from aruco_moveit_planner.moveit_planner import MoveItPlanOnlyClient
from geometry_msgs.msg import PoseStamped, Point
from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPositionFK
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# Where trajectories are saved (one YAML per planner label).
SAVE_DIR = "/home/rosi/ZD/zed_Motion/src/scripts/trajectories"
 
# EEF link + base frame used for FK when drawing the Cartesian path.
EEF_LINK = "right_tcp"
BASE_FRAME = "right_base"
LABEL    = "OMPL" 
 
def move_with_retry(planner, target, arm="left", max_retries=5,
                    execute=False, constrain_joints=True,
                    pipeline_id="ompl", planner_id="RRTstarkConfigDefault"):
    for attempt in range(max_retries):
        if arm == "left":
            success = planner.plan_to_pose(target)
        else:
            success = planner.plan_to_pose2(target, constrain_joints=constrain_joints)

        if success:
            print(f"Plan succeeded on attempt {attempt + 1}")

            if execute:
                answer = input("Trajectory shown in RViz. Execute? [y/N] ").strip().lower()
                if answer == "y":
                    if arm == "left":
                        planner.execute_stored_trajectory()   # or your left-arm executor
                    else:
                        planner.execute_stored_trajectory()
                else:
                    print("Discarded plan.")
                    planner._stored_trajectory = None
                    continue  
            return True

        print(f"Attempt {attempt + 1}/{max_retries} failed, retrying...")

    print(f"Failed after {max_retries} attempts!")
    return False

def plan_with(planner, target, pipeline_id, planner_id, arm="right"):
    """Run one plan with a specific pipeline/planner. Returns RobotTrajectory or None."""
    if arm == "left":
        ok = planner.plan_to_pose(target, pipeline_id=pipeline_id, planner_id=planner_id)
    else:
        ok = planner.plan_to_pose2(target, pipeline_id=pipeline_id, planner_id=planner_id)
    return planner._stored_trajectory if ok else None
 
def trajectory_metrics(traj):
    pts = traj.joint_trajectory.points
    names = traj.joint_trajectory.joint_names
    if not pts:
        return {}
    last = pts[-1].time_from_start
    duration = last.sec + last.nanosec * 1e-9

    # joint-space path length (rad) — total motion summed over all joints
    joint_len = 0.0
    # per-joint total travel (rad) — how much each joint turned along the path
    n = len(pts[0].positions)
    per_joint_travel = [0.0] * n
    for a, b in zip(pts[:-1], pts[1:]):
        joint_len += sum((y - x) ** 2 for x, y in zip(a.positions, b.positions)) ** 0.5
        for j in range(n):
            per_joint_travel[j] += abs(b.positions[j] - a.positions[j])

    # net change start->end per joint (rad)
    net_change = [pts[-1].positions[j] - pts[0].positions[j] for j in range(n)]

    return {
        "num_points": len(pts),
        "duration_sec": round(duration, 4),
        "joint_path_len_rad": round(joint_len, 4),
        "max_joint_travel_rad": round(max(per_joint_travel), 4),
        "max_joint_travel_deg": round(max(per_joint_travel) * 57.2958, 1),
        "total_joint_travel_deg": round(sum(per_joint_travel) * 57.2958, 1),
    }

def save_trajectory(traj, path, label=""):
    """Write a trajectory + its metrics to a YAML file."""
    data = {
        "label": label,
        "metrics": trajectory_metrics(traj),
        "trajectory": message_to_ordereddict(traj),
    }
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"[save] {label} -> {path}")
 
def load_trajectory(path):
    """Load one saved YAML trajectory."""
    with open(path) as f:
        data = yaml.unsafe_load(f)
    traj = RobotTrajectory()
    set_message_fields(traj, data["trajectory"])
    return traj, data.get("metrics", {})

def tcp_points(planner, traj):
    """FK every waypoint -> list of TCP Points (for drawing the Cartesian path)."""
    jt = traj.joint_trajectory
    pts = []
    for point in jt.points:
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_FRAME
        req.fk_link_names = [EEF_LINK]
        req.robot_state.joint_state.name = jt.joint_names
        req.robot_state.joint_state.position = list(point.positions)
        fut = planner._fk_client.call_async(req)
        rclpy.spin_until_future_complete(planner, fut)
        res = fut.result()
        if res and res.error_code.val == 1 and res.pose_stamped:
            p = res.pose_stamped[0].pose.position
            pts.append(Point(x=p.x, y=p.y, z=p.z))
    return pts
 
def compare_and_show(planner):
    """Load all *_1.._5 files, print metrics, draw each in a color."""
    # (filename_prefix, color) — one color per planner
    groups = [
        ("RRTstarkConfigDefault",   (1.0, 0.0, 0.0, 1.0)),  # red
        ("RRTConnectkConfigDefault",(0.0, 1.0, 0.0, 1.0)),  # green
        ("PRMstarkConfigDefault",   (0.0, 0.0, 1.0, 1.0)),  # blue
        ("PTP",                     (1.0, 1.0, 0.0, 1.0)),  # yellow
        ("LIN",                     (1.0, 0.0, 1.0, 1.0)),  # magenta
    ]
    named = []
    print("\n=== Comparison ===")
    for prefix, color in groups:
        for path in sorted(glob.glob(f"{SAVE_DIR}/{prefix}_*.yaml")):
            traj, metrics = load_trajectory(path)
            name = path.split("/")[-1].replace(".yaml", "")
            print(f"{name}: {metrics}")
            named.append((name, traj, color))
    if named:
        visualize(planner, named)
        rclpy.spin(planner)

def visualize(planner, named_trajs):
    """Publish each trajectory as a colored LINE_STRIP marker in RViz.
 
    named_trajs: list of (label, RobotTrajectory, (r,g,b,a)).
    """
    array = MarkerArray()
    for i, (label, traj, color) in enumerate(named_trajs):
        line = Marker()
        line.header.frame_id = BASE_FRAME
        line.header.stamp = planner.get_clock().now().to_msg()
        line.ns = "trajectories"
        line.id = i
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.005  # line width (m)
        line.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        line.pose.orientation.w = 1.0
        line.points = tcp_points(planner, traj)
        array.markers.append(line)
 
        # Text label at the end of the path, same color.
        if line.points:
            txt = Marker()
            txt.header.frame_id = BASE_FRAME
            txt.ns = "labels"
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.scale.z = 0.03
            txt.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            txt.pose.position = line.points[-1]
            txt.pose.orientation.w = 1.0
            txt.text = label
            array.markers.append(txt)
 
    planner._marker_pub.publish(array)
    print(f"[viz] Published {len(named_trajs)} trajectories to /trajectory_comparison")
 
def main():
    rclpy.init()

    # Create the planner
    planner = MoveItPlanOnlyClient()

    # Publisher + FK client used by the viz helpers (created here, not in the class).
    from visualization_msgs.msg import MarkerArray as _MA
    planner._marker_pub = planner.create_publisher(_MA, "/trajectory_comparison", 1)
    planner._fk_client = planner.create_client(GetPositionFK, "/compute_fk")
    planner._fk_client.wait_for_service(timeout_sec=5.0)

    # Right arm - staight line on x and y axis   
    right_target = PoseStamped()
    right_target.header.frame_id = "right_base"
    right_target.pose.position.x = -0.369 #-0.158   # small offset
    right_target.pose.position.y = -0.704 #-0.681
    right_target.pose.position.z = 0.331  #0.449                            
    right_target.pose.orientation.x =  0.090
    right_target.pose.orientation.y =  0.674
    right_target.pose.orientation.z = -0.699
    right_target.pose.orientation.w = -0.220

    #0.007, -0.673, 0.728, 0.129
    # Right arm - staight line on x and y axis   
    right_trainsit = PoseStamped()
    right_trainsit.header.frame_id = "right_base"
    right_trainsit.pose.position.x = right_target.pose.position.x + 0.2 #-0.158   # small offset
    right_trainsit.pose.position.y = right_target.pose.position.y + 0.15#-0.681
    right_trainsit.pose.position.z = right_target.pose.position.z #0.449
    right_trainsit.pose.orientation.x =  0.090
    right_trainsit.pose.orientation.y =  0.674
    right_trainsit.pose.orientation.z = -0.699
    right_trainsit.pose.orientation.w = -0.220


    planner._add_box_obstacle("wall1", -0.4, right_target.pose.position.y + 0.1, 0.5, 0.2, 0.01, 0.8, frame="right_base")
    # planner._add_box_obstacle("wall2", -0.25, -0.581, 0.6, 0.5, 0.01, 0.3, frame="right_base")
    # planner._add_box_obstacle("box", 0.112, 0.212, 1.28, 0.05, 0.05, 0.05, frame="world")
    # Give time for planning scene to update
    # planner._remove_obstacle("wall1", "wall2")
    time.sleep(1.0)

    ok = move_with_retry(planner, right_trainsit, arm="right", execute=True)
    
    traj= planner._stored_trajectory
    m = trajectory_metrics(traj)
    # save_trajectory(traj, f"{SAVE_DIR}/RRTstarkConfigDefault_constraints_5.yaml", LABEL)

    # keep node spinning so markers persist
    if ok:
        # print("Target reached, now moving to home...")
        planner._remove_obstacle("wall1")
        # planner._remove_obstacle("box")

        ok = move_with_retry(planner, right_target, arm="right", execute=True)

        # planner._remove_obstacle("wall2")
    
        # time.sleep(1.0)
        # move_with_retry(planner, right_home, arm="right", execute=True, constrain_joints=False)
 
    # ###########Load the path###############
    # planner._marker_pub = planner.create_publisher(MarkerArray, "/trajectory_comparison", 1)
    # planner._fk_client = planner.create_client(GetPositionFK, "/compute_fk")
    # planner._fk_client.wait_for_service(timeout_sec=5.0)
    # compare_and_show(planner)

    planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

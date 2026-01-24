#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler, quaternion_from_matrix
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Transform
import tf_transformations
import os
import yaml

# Carica waypoints
waypoints_yaml_path = os.path.join(get_package_share_directory('ros2_fra2mo'), "config", "waypoints.yaml")
with open(waypoints_yaml_path, 'r') as f:
    yaml_content = yaml.safe_load(f)

def create_pose(transform):
    name = transform["name"]
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    
    # POSIZIONE dal YAML
    x_yaml = transform["position"]["x"]
    y_yaml = transform["position"]["y"]
    z_yaml = transform["position"]["z"]
    
    # QUATERNIONI direttamente dal YAML (come nel tuo esempio commentato)
    rotx_yaml = transform["orientation"]["x"]
    roty_yaml = transform["orientation"]["y"]
    rotz_yaml = transform["orientation"]["z"]
    rotw_yaml = transform["orientation"]["w"]
    
    print(f"{name}: x={x_yaml:.2f}, y={y_yaml:.2f}, quat={rotw_yaml:.3f}")

    # Trasformata 1 (waypoint)
    transform1 = Transform()
    transform1.translation.x = x_yaml
    transform1.translation.y = y_yaml
    transform1.translation.z = z_yaml
    transform1.rotation.x = rotx_yaml
    transform1.rotation.y = roty_yaml
    transform1.rotation.z = rotz_yaml
    transform1.rotation.w = rotw_yaml

    t1_matrix = tf_transformations.translation_matrix([x_yaml, y_yaml, z_yaml])
    t1_matrix = tf_transformations.concatenate_matrices(
        t1_matrix,
        tf_transformations.quaternion_matrix([rotx_yaml, roty_yaml, rotz_yaml, rotw_yaml])
    )

    # Trasformata 2 (offset fisso)
    roll_2, pitch_2, yaw_2 = 0.0, 0.0, -1.57
    quat2 = quaternion_from_euler(roll_2, pitch_2, yaw_2)
    transform2 = Transform()
    transform2.translation.x, transform2.translation.y, transform2.translation.z = -3.0, 3.5, 0.0
    transform2.rotation.x, transform2.rotation.y = quat2[0], quat2[1]
    transform2.rotation.z, transform2.rotation.w = quat2[2], quat2[3]

    t2_matrix = tf_transformations.translation_matrix([-3.0, 3.5, 0.0])
    t2_matrix = tf_transformations.concatenate_matrices(
        t2_matrix,
        tf_transformations.quaternion_matrix(quat2)
    )

    # Composizione: T_inv * T1
    composed_matrix = tf_transformations.concatenate_matrices(
        tf_transformations.inverse_matrix(t2_matrix), t1_matrix
    )

    # Estrai translation e rotation
    translation = composed_matrix[:3, 3]
    quaternion_way = quaternion_from_matrix(composed_matrix)

    # Assegna alla pose
    pose.pose.position.x = float(translation[0])
    pose.pose.position.y = float(translation[1])
    pose.pose.position.z = float(translation[2])
    pose.pose.orientation.x = float(quaternion_way[0])
    pose.pose.orientation.y = float(quaternion_way[1])
    pose.pose.orientation.z = float(quaternion_way[2])
    pose.pose.orientation.w = float(quaternion_way[3])
    
    return pose, name

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    print("Caricamento waypoints...")
    goals = list(map(create_pose, yaml_content["waypoints"]))
    goal_poses = []
    
    strategy = yaml_content["mode"][0]["strategy"]
    print(f"Strategia: {strategy}")
    
    if strategy == "path":
        targets = ["Goal_3", "Goal_4", "Goal_2", "Goal_1"]
        for target in targets:
            for pose, name in goals:
                if name == target:
                    goal_poses.append(pose)
                    print(f"Aggiunto {target}")
                    break
    else:
        for pose, name in goals:
            if name == "explore":
                goal_poses.append(pose)
                print("Aggiunto explore")
    
    if not goal_poses:
        print("Nessun goal trovato!")
        return
    
    print(f"{len(goal_poses)} waypoints pronti")
    
    navigator.waitUntilNav2Active()
    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)
    
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f"Waypoint {feedback.current_waypoint + 1}/{len(goal_poses)}")
        
        if navigator.get_clock().now() - nav_start > Duration(seconds=6000):
            navigator.cancelTask()
            break
    
    result = navigator.getResult()
    status = {TaskResult.SUCCEEDED: " SUCCESS", TaskResult.CANCELED: "CANCELED", 
              TaskResult.FAILED: "FAILED"}
    print(f"Risultato: {status.get(result, 'UNKNOWN')}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

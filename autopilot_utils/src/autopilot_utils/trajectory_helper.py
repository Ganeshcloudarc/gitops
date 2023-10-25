from nav_msgs.msg import Path
from autopilot_msgs.msg import Trajectory, TrajectoryPoint
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped


def trajectory_to_path(trajectory_msg):
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp = rospy.Time.now()
    for i in range(len(trajectory_msg.points)):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose = trajectory_msg.points[i].pose
        path_msg.poses.append(pose_stamped)

    return path_msg


def trajectory_to_marker(trajectory_msg, max_forward_speed):
    marker_arr_msg = MarkerArray()
    for i in range(len(trajectory_msg.points)):
        traj_point = trajectory_msg.points[i]
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.text = str(round(traj_point.longitudinal_velocity_mps, 2))
        marker.id = i
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        if traj_point.longitudinal_velocity_mps == max_forward_speed: 
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        # marker.lifetime = rospy.Duration(duration)
        marker.pose = traj_point.pose
        marker_arr_msg.markers.append(marker)
    return marker_arr_msg



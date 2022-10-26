#!/usr/bin/env python3
from autopilot_msgs.msg import Trajectory, TrajectoryPoint
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from autopilot_utils.pose_helper import *
import sys
# from nav_msgs.msg import Path


class TrajectoryManager:
    """
        This is a class for Trajectory related common operations
        Attributes:
            traj(autopilot_msgs/Trajectory)
        """

    def __init__(self, traj=None):
        """
               The constructor for TrajectoryManager class.

               Parameters:
                  traj(autopilot_msgs/Trajectory)
               """
        if traj:
            self._traj_len = traj
            self._traj_in = len(self._traj_in)
        else:
            self._traj_len = -1
            self._traj_in = None

    def update(self, traj):
        """
        updates the trajectory
        Parameters:
                  traj(autopilot_msgs/Trajectory)
        """
        self._traj_in = traj
        self._traj_len = len(self._traj_in.points)

    def get_len(self):
            return self._traj_len

    def get_traj_point(self, ind):
        # return 1
        if self._traj_len > 0 and ind < self._traj_len:
            return self._traj_in.points[ind]
        else:
            raise Exception("invalid index or out of bound")

    def next_point_within_dist(self, idx, dist_thr):
        # Find the next index which is dist_thr far from idx point.
        if self._traj_len > 0 and idx < self._traj_len:
            close_dis = self._traj_in.points[idx].accumulated_distance_m
            if abs(self._traj_in.points[idx].accumulated_distance_m - \
                   self._traj_in.points[-1].accumulated_distance_m) <= dist_thr:
                return self._traj_len - 1
            else:
                for i in range(idx, self._traj_len):
                    path_acc_distance = abs(self._traj_in.points[i].accumulated_distance_m - close_dis)
                    if path_acc_distance > dist_thr:
                        return i
        else:
            raise Exception("invalid index or out of bound")

    def find_closest_idx_with_dist_ang_thr(self, curr_pose, dist_thr, angle_thr):
        """
        Finds the closest point on trajectory within a distance threshold and angle threshold, returns true and
        index if one exists.
        Params:
            curr_pose : pose to which want to find close point
            dist_thr :
            angle_thr :
        Returns:
            Found, index (true/false, Int)
        """
        dist_min = sys.maxsize
        idx_min = -1
        curr_yaw = get_yaw(curr_pose.orientation)
        if self._traj_in is None:
            return False, idx_min
        for i in range(0, self._traj_len):
            traj_point = self._traj_in.points[i]
            dis = distance_btw_poses(curr_pose, traj_point.pose)
            if dis > dist_thr:
                continue
            yaw_diff = normalize_angle(curr_yaw - get_yaw(traj_point.pose.orientation))
            # print(yaw_diff)
            if abs(yaw_diff) > angle_thr:
                continue
            if dis < dist_min:
                dist_min = dis
                idx_min = i
        if idx_min >= 0:
            var = True, idx_min
        else:
            var = False, idx_min
        return var

    def find_close_pose_after_index(self, curr_pose, prev_idx, search_distance=10):
        """
        This function returns the closest index, from prev_idx
        Params:
            curr_pose(pose) :
            prev_idx : search starts from this index
            search_distance : distance to search from prev_idx
        Returns:
            close_index: closest pose index to the curr_pose, from prev_idx.
        """

        # check for 10 meters from prev_idx
        dist_min = sys.maxsize
        idx_min = prev_idx
        if prev_idx >= self._traj_len - 1:
            return self._traj_len - 1

        for i in range(prev_idx, self._traj_len):
            if abs(self._traj_in.points[prev_idx].accumulated_distance_m -
                   self._traj_in.points[i].accumulated_distance_m) > search_distance:
                break
            traj_point = self._traj_in.points[i]
            dis = distance_btw_poses(curr_pose, traj_point.pose)
            if dis < dist_min:
                dist_min = dis
                idx_min = i
        return idx_min

    def to_path(self):
        """
        Returns path(Path) of Trajectory.
        """
        path_msg = Path()
        path_msg.header.frame_id = self._traj_len.header.frame_id
        for i in range(self._traj_len):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self._traj_len.header.frame_id
            pose_stamped.pose = self._traj_in.points[i].pose
            path_msg.poses.append(pose_stamped)
        return path_msg

    def to_marker(self, trajectory_msg):
        """
        Returns marker(MarkerArray) of trajectory_points and  velocities.

        """
        marker_arr_msg = MarkerArray()
        marker_arr_msg.header.frame_id = self._traj_in.header.frame_id
        for i in range(self._traj_len):
            traj_point = self._traj_in.points[i]
            marker = Marker()
            marker.header.frame_id = trajectory_msg.header.frame_id
            marker.type = marker.TEXT_VIEW_FACING
            marker.text = str(round(traj_point.longitudinal_velocity_mps, 2))
            marker.id = i
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            # marker.lifetime = rospy.Duration(duration)
            marker.pose = traj_point.pose
            marker_arr_msg.markers.append(marker)
        return marker_arr_msg


if __name__ == "__main__":
    print("library for trajectory")
    tm = TrajectoryManager()
    print(tm.get_len())






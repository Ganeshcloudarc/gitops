#!bin/usr/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Path, Odometry
from zed_interfaces.msg import ObjectsStamped, Object
from geometry_msgs.msg import Point, PoseArray, Pose
from tf_helper import *
import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
import copy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np
from numpy import argmin, zeros
from autopilot_utils.occ_grid_helper import OccupancyGridManager
from autopilot_utils.tf_helper import current_robot_pose
from vehicle_common.vehicle_config import vehicle_data


# from autopilot_boson.nodes.vehicle_config import VehicleConfig

def min_distance_to_object(pose, corners):
    dist_list = []
    for corner in corners:
        dis = math.hypot(pose.position.x - corner[0], pose.position.y - corner[1])
        dis.append(dist_list)
    return min(dist_list)


def distance_btw_poses(pose1, pose2):
    """2d distance between two poses"""
    return math.hypot(pose1.position.x - pose2.position.x, pose1.position.y - pose2.position.y)


def distance_to_robot(pose):
    robot_pose = current_robot_pose()
    return distance_btw_poses(robot_pose, pose)


def get_yaw(orientation):
    _, _, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    return yaw


class CollisionEstimator:
    def __init__(self):

        self.path_end_index = None
        self.obstacle_aware_vel = None
        self.stop_index = None
        self.global_vel = None
        self.close_index = None
        self.revived_path = None
        self.process_data = None
        self.target_frame = "map"
        self.index_old = None
        self.path = None
        self.modified_obj_data = ObjectsStamped()
        self.modified_obj_data.header.frame_id = self.target_frame
        cam_name = rospy.get_param("camera_name", "zed2")
        rospy.Subscriber(f"{cam_name}/zed_node/obj_det/objects", ObjectsStamped, self.objects_cb)
        self.obj_map_frame_pub = rospy.Publisher("/zed_objects_map_frame", ObjectsStamped, queue_size=1)
        self.vel_update_pub = rospy.Publisher("/vel_updates", Float32MultiArray, queue_size=1)

        rospy.Subscriber('/odom_path', Path, self.path_callback)
        rospy.Subscriber('/curvature_velocity_profile', Float32MultiArray, self.global_vel_cb)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # parameters

        self.obstacle_threshold_radius = rospy.get_param("obstacle_threshold_radius", 1.5)
        self.max_cte = rospy.get_param("max_cte_to_interpolate", 3)
        self.forward_collision_check_dis = rospy.get_param("forward_collision_check_dis", 15)  # 15 meters
        self.stop_distance = rospy.get_param("spot_distance_before_collision_ind", 3)
        self.collision_index_dis  = 2

        local_cost_map_topic = rospy.get_param("local_cost_map_topic", "/semantics/costmap_generator/occupancy_grid")
        self.base_frame = rospy.get_param("/patrol/base_frame", "base_link")

        self.ogm = OccupancyGridManager('/semantics/costmap_generator/occupancy_grid',
                                   subscribe_to_updates=False, base_frame=self.base_frame)
        self.left = -1.0 * vehicle_data.dimensions.rear_overhang
        self.right = vehicle_data.dimensions.overall_length
        self.top = vehicle_data.dimensions.overall_width / 2
        self.bottom = - vehicle_data.dimensions.overall_width / 2
        self.resolution = self.ogm.resolution

        # pose array pub
        self.rotated_poses_pub = rospy.Publisher("/rotated_poses", PoseArray, queue_size=10)
        self.pose_arr_msg = PoseArray()
        self.pose_arr_msg.header.frame_id = 'map'
        self.main_loop()

        # try:
        #     transform = self.tf_buffer.lookup_transform(
        #         self.parent_frame, self.child_frame, rospy.Time())
        #     rate.sleep()
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
        #         tf2_ros.ExtrapolationException):
        #     rate.sleep()
        #     continue

    def objects_cb(self, data):
        print("data.header.frame_id", data.header.frame_id)
        object_corner_list = []
        if data.header.frame_id == self.target_frame:
            # no Transform needed
            self.modified_obj_data = data
        else:
            # need to transform from object frame to target frame
            self.modified_obj_data = copy.deepcopy(data)
            self.modified_obj_data.header.frame_id = self.target_frame
            self.modified_obj_data.header.stamp = rospy.Time.now()
            for i, obj_data in enumerate(data.objects):
                # https://www.stereolabs.com/docs/ros/object-detection/

                pos = convert_point(obj_data.position, data.header.frame_id, self.target_frame)
                self.modified_obj_data.objects[i].position = pos

                for j, corner in enumerate(obj_data.bounding_box_3d.corners):
                    print('c', corner.kp)
                    print(data.header.frame_id)
                    point = convert_point(corner.kp, data.header.frame_id, self.target_frame)
                    print('corner', point, corner.kp)
                    self.modified_obj_data.objects[i].bounding_box_3d.corners[j].kp = point

            self.to_visualization_marker_array(self.modified_obj_data)
            # print(data.objects[0].position, modified_obj_data.objects[0].position)

    def path_callback(self, data):
        rospy.logdebug("path data data received of length %s", str(len(data.poses)))
        self.path = data.poses
        self.path_end_index = len(self.path) - 1

    def global_vel_cb(self, data):

        self.global_vel = data.data
        self.obstacle_aware_vel = list(copy.deepcopy(self.global_vel))

    def find_close_point(self, prev_close_ind):

        close_dis = distance_to_robot(self.revived_path[prev_close_ind])
        for ind in range(prev_close_ind + 1, len(self.revived_path)):
            dis = distance_to_robot(self.revived_path[ind])
            if close_dis >= dis:
                close_dis = dis
            else:
                return ind - 1, close_dis

    def find_close_point(self, robot_pose, old_close_index):
        # n = min(100, len(range(index_old, self.path_end_index)))
        # distance_list = [self.calc_distance(robot, ind) for ind in range(index_old, index_old + n)]
        # ind = np.argmin(distance_list)
        # final = ind + index_old
        # dis = distance_list[ind]
        # return final, dis
        close_dis = self.calc_distance(robot_pose, old_close_index)
        for ind in range(old_close_index + 1, self.path_end_index):
            dis = self.calc_distance(robot_pose, ind)
            if close_dis >= dis:
                close_dis = dis
            else:
                # print("find close", index_old, ind)
                return ind - 1, close_dis
        return self.path_end_index, 0

    def calc_nearest_ind(self, robot_pose):
        """
        calc index of the nearest point to current position
        Args:
            robot_pose: pose of robot
        Returns:
            close_index , distance
        """
        distance_list = [self.calc_distance(robot_pose, ind) for ind in range(len(self.path))]
        ind = np.argmin(distance_list)
        self.index_old = ind
        dis = distance_list[ind]
        return ind, dis

    def distance_to_next_index(self, ind):
        return math.hypot(self.path[ind].pose.position.x - self.path[ind + 1].pose.position.x,
                          self.path[ind].pose.position.y - self.path[ind + 1].pose.position.y)

    def calc_distance(self, robot_pose, ind):
        return math.hypot(robot_pose.position.x - self.path[ind].pose.position.x, robot_pose.position.y -
                          self.path[ind].pose.position.y)

    def calc_distance_idx(self, close, next_id):
        return math.hypot(self.path[close][0] - self.path[next_id][0], self.path[close][1] - self.path[next_id][1])

    def check_collision_on_path(self, robot_pose, index_old):
        cost_list = []

        for i in range(index_old, index_old + 30):

            path_pose = self.path[i].pose
            base_x, base_y = self.ogm.get_costmap_x_y(path_pose.position.x, path_pose.position.y)
            print("base_x: %s, base_y: %s", base_x, base_y)
            origin = self.ogm.origin
            angle = math.atan2(origin.position.y, origin.position.x)  # * (180 / Math.PI)

            cos_theta = math.cos(get_yaw(path_pose.orientation))
            sin_theta = math.sin(get_yaw(path_pose.orientation))
            # cos_theta = math.cos(angle)
            # sin_theta = math.sin(angle)
            # for i in np.arange(0.5, 1.5, 0.2):
            # prev_b = path_pose.position.x
            # self.rotated_poses_pub

            '''
            data.pose.pose.position.x - fcu_offset_vehicle * math.cos(yaw)
            '''
            poses_list = []
            min_cost = 0
            for x in np.arange(self.left, self.right, self.resolution):
                for y in np.arange(self.top, self.bottom, -self.resolution):
                    pose = Pose()
                    world_x = path_pose.position.x + (x * cos_theta - y * sin_theta)
                    world_y = path_pose.position.y + (x * sin_theta + y * cos_theta)
                    pose.position.x = world_x
                    pose.position.y = world_y
                    poses_list.append(pose)
                    try:
                        cost = self.ogm.get_cost_from_world_x_y(world_x, world_y)
                    except:
                        cost = 0
                        pass
                    min_cost = max(cost, min_cost)
                    if min_cost >= 100:
                        return True, i

            self.pose_arr_msg.poses = poses_list
            self.rotated_poses_pub.publish(self.pose_arr_msg)
            rospy.logwarn("published")
            cost_list.append(min_cost)
        return False, i

    def main_loop(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            robot_pose = current_robot_pose("map", self.base_frame)
            # print("robot_pose",robot_pose)
            if robot_pose is None:
                rospy.logerr("waiting for tf between map and %s", self.base_frame)
                rate.sleep()
                continue

            if self.index_old is None:
                self.index_old, cross_track_dis = self.calc_nearest_ind(robot_pose)
            else:
                self.index_old, cross_track_dis = self.find_close_point(robot_pose, self.index_old)

            collision_status, collision_index = self.check_collision_on_path(robot_pose, self.index_old+10)
            if collision_status:
                self.velocity_smoother(robot_pose, collision_index, self.index_old)
            else:
                self.obstacle_aware_vel = list(copy.deepcopy(self.global_vel))
                msg = Float32MultiArray()
                msg.data = self.obstacle_aware_vel

                self.vel_update_pub.publish(msg)



    def velocity_smoother(self, robot_pose, ind, close_idx):
        """ Depending upon distance between the robot and the collision point, will set the speed and publish into
        velocity profile for path tracker """
        # dis = distance_btw_poses(robot_pose, self.path[ind].pose)
        # acc_dis = 0
        # for i, k in enumerate(range(ind, 0, -1)):
        #     acc_dis += distance_btw_poses(self.path[i].pose, self.path[i - 1].pose)
        #     if acc_dis > 2:
        #         stop_index = i
        #         break

        self.obstacle_aware_vel = list(copy.deepcopy(self.global_vel))

        object_dis = distance_btw_poses(robot_pose, self.path[ind].pose)
        if object_dis <= self.stop_distance:
            self.obstacle_aware_vel[ind:] = np.zeros(len(self.obstacle_aware_vel[ind:])).tolist()
            msg = Float32MultiArray()
            msg.data = self.obstacle_aware_vel

            self.vel_update_pub.publish(msg)
        else:
            acc_dis = 0
            vel_list = []
            for i in range(ind, close_idx, -1):
                acc_dis += distance_btw_poses(self.path[i].pose, self.path[i-1].pose)
                if acc_dis > self.collision_index_dis:
                    # self.stop_index = j
                    acc_dis = 0
                    for j in range(i, close_idx, -1):
                        acc_dis += distance_btw_poses(self.path[j].pose, self.path[j - 1].pose)
                        if acc_dis > 1:
                            diff_ind = j - i
                            for n, x in enumerate(range(j, i)):
                                vel = min((self.global_vel[x]/diff_ind) * (diff_ind-n), self.global_vel[x])
                                vel_list.append(vel)
                            self.obstacle_aware_vel[j:i] = vel_list
                            self.obstacle_aware_vel[i:] = np.zeros(len(self.obstacle_aware_vel[i:])).tolist()
                            msg = Float32MultiArray()
                            msg.data = self.obstacle_aware_vel

                            self.vel_update_pub.publish(msg)

                            break
                    break

        # print("global vel", len(self.obstacle_aware_vel))
        # print(type(self.global_vel))
        # # print(a)
        # self.obstacle_aware_vel = list(copy.deepcopy(self.global_vel))
        # for j in range(ind - 20, ind):
        #     self.obstacle_aware_vel[j] = 0.0
        #
        # # self.obstacle_aware_vel[ind-20:100] = tuple(np.zeros(120).tolist())
        # msg = Float32MultiArray()
        # msg.data = self.obstacle_aware_vel
        # # self.vel_update_pub.publish(msg)
        # print('data', self.obstacle_aware_vel[ind - 30:ind + 30])
        # print("ind", ind)
        #
        # rospy.loginfo("updated vel  published")
        # print(a)

        # if dis < self.spot_distance_before_collision_ind:
        #     pass
        # else:
        #     acc_dis = 0
        #     for j in range(ind, 0, -1):
        #         acc_dis += distance_btw_poses(self.revived_path[j], self.revived_path[j - 1])
        #         if acc_dis > self.spot_distance_before_collision_ind:
        #             self.stop_index = j
        #             break
        #     for i, k in enumerate(range(self.stop_index, 0, -1)):
        #         pass

    def velocity_marker(self, start_index, vel_list):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.text = str(round(vel, 2))
        marker.id = i
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.3
        marker.color.g = 1.0
        marker.color.b = 1.0
        # marker.lifetime = rospy.Duration(duration)
        marker.pose.orientation = Quaternion(odom_orientation['x'], odom_orientation['y'], odom_orientation['z'],
                                             odom_orientation['w'])
        marker.pose.position.x, marker.pose.position.y = odom_position['x'], odom_position['y']
        marker_arr_msg.markers.append(marker)

    def in_collision_marker_at_index(self, i):
        """ A Marker with connected points, with read colour indication collision will happen will be published"""
        pass

    def no_collision_markers_at_index(self, i):
        """ No collision with green colour box """
        pass

    def to_visualization_marker_array(self, modified_obj_data):
        marker_array = MarkerArray()
        self.obj_map_frame_pub.publish(modified_obj_data)
        print("punlished")

        # // create
        # markers
        # for bounding boxes
        #     Marker
        #     marker
        #     {};
        # marker.header = bboxes.header;
        # marker.ns = "bounding_box";
        # marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        # marker.action = Marker::ADD;
        # marker.lifetime = time_utils::to_message(std::chrono::nanoseconds(100000));
        # for o in object_corner_list:


if __name__ == "__main__":
    rospy.init_node('collsion_estimator_node')
    obj = CollisionEstimator()
    rospy.spin()

'''

    self.parent_frame = parent_frame
    self.child_frame = child_frame
    self.bagfile = bagfile
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    self.lookup_frequency = lookup_frequency
    self.output_topic = output_topic
    self.append = append


    def run(self):
        msg_count = 0
        try:
            bag = rosbag.Bag(self.bagfile, mode='a' if self.append else 'w')
            rate = rospy.Rate(self.lookup_frequency)
            last_stamp = rospy.Time()
            while not rospy.is_shutdown():
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.parent_frame, self.child_frame, rospy.Time())
                    rate.sleep()
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    rate.sleep()
                    continue
                if last_stamp == transform.header.stamp:
                    continue
                pose = transformstamped_to_posestamped(transform)
                bag.write(self.output_topic, pose, t=pose.header.stamp)
                msg_count += 1
                last_stamp = transform.header.stamp
                rospy.loginfo_throttle(
                    10, "Recorded {} PoseStamped messages.".format(msg_count))

        except rospy.ROSInterruptException:
            pass
        finally:
            bag.close()
            rospy.loginfo("Finished recording.")


'''

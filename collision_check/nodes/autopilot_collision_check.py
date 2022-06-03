#!bin/usr/env python3
import rospy
import tf2_ros
from nav_msgs.msg import Path, Odometry
from zed_interfaces.msg import ObjectsStamped, Object
from geometry_msgs.msg import Point
from tf_helper import *
import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
import copy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from numpy import argmin, zeros
from autopilot_utils.occ_grid_helper import OccupancyGridManager
from autopilot_utils.tf_helper import current_robot_pose

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


class CollisionEstimator:
    def __init__(self):

        self.obstacle_aware_vel = None
        self.stop_index = None
        self.global_vel = None
        self.close_index = None
        self.revived_path = None
        self.process_data = None
        self.target_frame = "map"
        self.modified_obj_data = ObjectsStamped()
        self.modified_obj_data.header.frame_id = self.target_frame
        cam_name = rospy.get_param("camera_name", "zed2")
        rospy.Subscriber(f"{cam_name}/zed_node/obj_det/objects", ObjectsStamped, self.objects_cb)
        self.obj_map_frame_pub = rospy.Publisher("/zed_objects_map_frame", ObjectsStamped, queue_size=1)

        rospy.Subscriber('/odom_path', Path, self.path_cb)
        rospy.Subscriber('/velocity_profiler', Float32MultiArray, self.global_vel_cb)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # parameters

        self.obstacle_threshold_radius = rospy.get_param("obstacle_threshold_radius", 1.5)
        self.max_cte = self.get_param("max_cte_to_interpolate", 3)
        self.forward_collision_check_dis = self.get_param("forward_collision_check_dis", 15)  # 15 meters
        self.stop_distance = self.get_param("spot_distance_before_collision_ind", 3)

        local_cost_map_topic =self.get_param("local_cost_map_topic", "/move_base/local_costmap/costmap")

        self.main_loop(self,local_cost_map_topic)



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
        self.revived_path = data.poses

    def global_vel_cb(self, data):
        self.global_vel = data.data
        self.obstacle_aware_vel = copy.deepcopy(self.global_vel)

    def find_close_point(self, prev_close_ind):

        close_dis = distance_to_robot(self.revived_path[prev_close_ind])
        for ind in range(prev_close_ind + 1, len(self.revived_path)):
            dis = distance_to_robot(self.revived_path[ind])
            if close_dis >= dis:
                close_dis = dis
            else:
                return ind - 1, close_dis

    def calc_nearest_ind(self):
        """ return close path point index and distance"""
        robot_pose = current_robot_pose()
        dis_list = []
        for pose in self.revived_path:
            dis = distance_btw_poses(robot_pose, pose)
            dis_list.append(dis)
        ind = argmin(dis_list)
        dis = dis_list[ind]
        return ind, dis

    def main_loop(self, local_cost_map_topic):
        rate = rospy.Rate(10)
        ogm = OccupancyGridManager(local_cost_map_topic, ubscribe_to_updates=True)

        while not rospy.is_shutdown():
            if self.close_index is None:
                self.close_index, dis = self.calc_nearest_ind()
            else:
                self.close_index, dis = self.find_close_point(self.close_index)

            if dis > self.max_cte:
                # do collision check on the interpolated path and activate rectangle collision check
                pass
            else:


                # FOR CAMERA
                # for i in range(self.close_index, len(self.revived_path)):
                #     for ob in self.modified_obj_data.objects:
                #         dis = min_distance_to_object(self.revived_path[i], ob.corners)
                #         if dis <= self.obstacle_threshold_radius:
                #             rospy.loginfo("object detected at index", i)
                #             distance_to_vehicle = distance_to_robot(self.revived_path[i])
                #             self.in_collision_marker_at_index(i)
                #             rospy.loginfo("distance_to_vehicle", distance_to_vehicle)
                #         else:
                #             self.velocity_smoother(i)
                #             self.no_collision_markers_at_index(i)
                #
                #     if distance_to_robot(self.revived_path[i]) > self.forward_collision_check_dis:
                #         break

                ## FOR COSTMAP

    def velocity_smoother(self, ind):
        """ Depending upon distance between the robot and the collision point, will set the speed and publish into
        velocity profile for path tracker """
        dis = distance_to_robot(self.revived_path[ind])
        if dis < self.spot_distance_before_collision_ind:
            pass
        else:
            acc_dis = 0
            for j in range(ind, 0, -1):
                acc_dis += distance_btw_poses(self.revived_path[j], self.revived_path[j - 1])
                if acc_dis > self.spot_distance_before_collision_ind:
                    self.stop_index = j
                    break
            for i, k in enumerate(range(self.stop_index, 0, -1)):

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

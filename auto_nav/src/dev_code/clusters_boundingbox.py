'''implement this https://frc.ri.cmu.edu/~zhangji/publications/IROS_2013.pdf research paper'''

import rospy
from sensor_msgs.msg import PointCloud2
from sklearn import linear_model
from sklearn.cluster import DBSCAN
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
import rospy
import math
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from pykalman import KalmanFilter
import time
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose , PoseStamped
from scipy import interpolate

# autopilot related imports
from autopilot_utils.tf_helper import current_robot_pose, convert_point, transform_cloud
from autopilot_utils.pose_helper import distance_btw_poses, get_yaw
from autopilot_utils.trajectory_smoother import TrajectorySmoother
from vehicle_common.vehicle_config import vehicle_data
from autopilot_utils.trajectory_common import TrajectoryManager
from autopilot_msgs.msg import Trajectory, TrajectoryPoint





def distance_btw_poses(pose1, pose2):
    """
    Calculates distance between poses.
        Parameters:
            pose1(geometry_msgs/Pose.msg): pose one.
            pose2(geometry_msgs/Pose.msg): pose two.
        Returns:
            distance(float): distance between the poses
    """
    distance = math.hypot(pose1.position.x - pose2.position.x, pose1.position.y - pose2.position.y)
    return distance
class RansacLineFittingRows:
    def __init__(self):
        self.point_cloud_sub = rospy.Subscriber("/rslidar_points", PointCloud2, self.point_cloud_callback)
        rospy.Subscriber("vehicle/odom", Odometry, self.odom_callback)
        rospy.Subscriber('/global_gps_trajectory', Trajectory, self.global_traj_callback)
        self.marker_pub = rospy.Publisher("/markers", MarkerArray, queue_size=1)
        self.filtered_point_cloud_pub = rospy.Publisher("/filtered_point_cloud", PointCloud2, queue_size=1)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        self.trajectory_pub = rospy.Publisher("/local_gps_trajectory", Trajectory, queue_size=1)
        self.robot_pose = None
        self.robot_head_pose = Pose()
        self._close_idx = None
        self._traj_in = None
        self._traj_end_index = None
        self._traj_manager = TrajectoryManager()
        self._min_look_ahead_dis = rospy.get_param("/pure_pursuit/min_look_ahead_dis", 3)
        self._max_look_ahead_dis = rospy.get_param("/pure_pursuit/max_look_ahead_dis", 6)
        self._base_to_front = vehicle_data.dimensions.wheel_base + vehicle_data.dimensions.front_overhang
        radial_off_set_to_vehicle_width = rospy.get_param("obstacle_stop_planner/radial_off_set_to_vehicle_width", 0.5)
        self._radius_to_search = vehicle_data.dimensions.overall_width / 2 + radial_off_set_to_vehicle_width


    def point_cloud_callback(self, point_cloud_msg):
        Trajectory_path = []
        rate = rospy.Rate(1)
        # convert point cloud to numpy array
        point_cloud = pc2.read_points(point_cloud_msg, skip_nans=True, field_names=("x", "y", "z"))
        #filter trees from point cloud
        point_cloud = self.filter_point_cloud(point_cloud)
        #find clusters of points in point cloud
        clusters = self.find_clusters_sklearn(point_cloud)
        #min and max size of clusters
        clusters = self.filter_clusters_size(clusters, 50, 5000)  
        # KalmanFilter_clusters = self.kalman_filter_median(clusters)
        #publish filtered point cloud
        # print(KalmanFilter_clusters)
        header = point_cloud_msg.header
        filtered_point_cloud = pc2.create_cloud_xyz32(header, point_cloud)
        # filtered_point_cloud = self.kalman_filter_median(filtered_point_cloud)
        self.filtered_point_cloud_pub.publish(filtered_point_cloud)
        rospy.loginfo("filtered points published")

        header = point_cloud_msg.header
        filtered_point_cloud = pc2.create_cloud_xyz32(header, point_cloud)
        # filtered_point_cloud = self.kalman_filter_median(filtered_point_cloud)
        self.filtered_point_cloud_pub.publish(filtered_point_cloud)

        #find the center of each cluster
        cluster_centers = []
        for cluster in clusters:
            cluster_centers.append(np.mean(cluster, axis=0))
            #create timestamp for markers
        print(cluster_centers)
        # return 0
        timestamp  = time.time()
        kf=self.kalman_filter_position(cluster_centers,timestamp)
        # print(kf)

        #apply kalman filter to the cluster centers to smooth the data
        # print(cluster_centers)
        #find left and right right side of the rows
        left_side = []
        right_side = []
        for cluster_center in cluster_centers:
            if cluster_center[1] < 0:
                left_side.append(cluster_center)
            else:
                right_side.append(cluster_center)
        #ignore nearby points in the left and right side of the rows
        left_side = self.filter_nearby_points(left_side, 3)
        right_side = self.filter_nearby_points(right_side, 3)
        
        #fit a line to the left and right side of the rows
        left_line = self.line_fitting_linear_regression(left_side)
        right_line = self.line_fitting_linear_regression(right_side)
        # check for the close index on the trajectory
        if self._close_idx is None:
            angle_th = 90
            found, index = self._traj_manager.find_closest_idx_with_dist_ang_thr(self.robot_pose,
                                                                                    self._max_look_ahead_dis, angle_th)
            if found:
                self._close_idx = index
            else:
                rospy.logwarn(f"No close point found dist_thr: {self._max_look_ahead_dis}, angle_thr: {angle_th}")
                rate.sleep()
        else:
            self._close_idx = self._traj_manager.find_close_pose_after_index(self.robot_pose, self._close_idx, 10)
        
        front_tip_idx = self._traj_manager.next_point_within_dist(self._close_idx, 3)
        path_percent = (self._traj_in.points[self._close_idx].accumulated_distance_m /
                            self._traj_in.points[-1].accumulated_distance_m) * 100
        rospy.loginfo(f"Closest point index: {self._close_idx}, front tip index: {front_tip_idx}, path percent: {path_percent}")
        if path_percent > 95.0 and distance_btw_poses(self.robot_pose, self._traj_in.points[-1].pose) <= self._min_look_ahead_dis:
            self.count_mission_repeat += 1
            rospy.loginfo(' Mission count %s ', self.count_mission_repeat)
            if self.mission_continue: 
                self._close_idx = 1
                time.sleep(self._time_to_wait_at_ends)
                rate.sleep()
            else:
                # self.send_ack_msg(0, 0, 0)
                rate.sleep()
        prev_processed_ind = self._close_idx
        obstacle_found = False
        center_line1,center_line2 = self.center_line(left_line, right_line)
        # center_line = self.interpolate(center_line)
        cordinates = np.linspace(center_line1,center_line2,50)
        

        for i,ind in enumerate(range(self._close_idx, self._close_idx + 50)):
            path_pose = self._traj_in.points[ind].pose.position
            h=cordinates[i]
            path_pose = np.array([path_pose.x, path_pose.y+(h/2)])
            Trajectory_path.append(path_pose)
        
        #publish path in topic 
        center_path = Path()
        center_path.header.frame_id = "map"
        center_path.header.stamp = rospy.Time.now()
        center_path.poses = []
        for i in Trajectory_path:
            pose = PoseStamped()
            pose.pose.position.x = i[0]
            pose.pose.position.y = i[1]
            center_path.poses.append(pose)
        self.path_pub.publish(center_path)

        # print("Trajectory_path", center_line[0],center_line[-1], i[0], i[-1])
        #publish markers for the left and right side of the rows
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "rslidar"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = []
        for i in range(0, 20):
            point = Point()
            point.x = i
            point.y = left_line[i]
            point.z = 0
            marker.points.append(point)
        marker_array.markers.append(marker)
        
        marker = Marker()
        marker.header.frame_id = "rslidar"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = 1
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points = []
        for i in range(0, 20):
            point = Point()
            point.x = i
            point.y = right_line[i]
            point.z = 0
            marker.points.append(point)
        marker_array.markers.append(marker)
        #find the center line between the left and right line
        
        # #publish markers for the center line
        # marker = Marker()
        # marker.header.frame_id = "rslidar"
        # marker.header.stamp = rospy.Time.now()
        # marker.ns = "my_namespace"
        # marker.id = 2
        # marker.type = marker.LINE_STRIP
        # marker.action = marker.ADD
        # marker.scale.x = 0.1
        # marker.color.a = 1.0
        # marker.color.r = 0.0
        # marker.color.g = 0.0
        # marker.color.b = 1.0
        # marker.points = []
        # for i in range(0, 10):
        #     point = Point()
        #     point.x = i
        #     point.y = center_line[i]
        #     point.z = 0
        #     marker.points.append(point)
        # marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


    def odom_callback(self, data):
        self.robot_pose = data.pose.pose
        self.robot_speed = math.hypot(data.twist.twist.linear.x ** 2, data.twist.twist.linear.y ** 2)
        # print("self.robot_speed", self.robot_speed)
        self.odom_data_in_time = time.time()
        pose_heading = get_yaw(data.pose.pose.orientation)
        self.robot_head_pose.position.x = data.pose.pose.position.x + self._base_to_front * np.cos(pose_heading)
        self.robot_head_pose.position.y = data.pose.pose.position.y + self._base_to_front * np.sin(pose_heading)
        self.robot_head_pose.position.z = data.pose.pose.position.z
        self.robot_head_pose.orientation = data.pose.pose.orientation
    
    def global_traj_callback(self, data):
        self._traj_in = data
        self._traj_manager.update(data)
        self._traj_end_index = self._traj_manager.get_len()

    
    def find_close_point(self, robot_pose, old_close_index):
        close_dis = distance_btw_poses(robot_pose, self.traj_in.points[old_close_index].pose)
        for ind in range(old_close_index + 1, self.traj_end_index):
            dis = distance_btw_poses(robot_pose, self.traj_in.points[ind].pose)
            if close_dis >= dis:
                close_dis = dis
            else:
                robot_heading = get_yaw(robot_pose.orientation)
                path_heading = get_yaw(self.traj_in.points[ind].pose.orientation)
                # print("robot_heading", robot_heading)
                # print("path_heading", path_heading)
                # print("heading err:", abs(abs(robot_heading) - abs(path_heading)))
                # print("math.radians(90)", math.radians(90))
                if abs(abs(robot_heading) - abs(path_heading)) > math.radians(90):
                    rospy.logwarn("Headings are %s apart ", str(abs(robot_heading - path_heading)))
                    heading_ok = False
                else:
                    heading_ok = True
                return ind - 1, close_dis, heading_ok
        return self.traj_end_index, 0, True


    def filter_nearby_points(self, points, distance):
        #filter nearby points
        filtered_points = []
        for point in points:
            for filtered_point in filtered_points:
                if self.distance(point, filtered_point) < distance:
                    break
            else:
                filtered_points.append(point)
        return filtered_points


    def fit_line(self, points):
        #fit a line to the points using ransac
        points = np.array(points)
        x = points[:,0]
        y = points[:,1]
        model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression())
        model_ransac.fit(x[:, np.newaxis], y)
        inlier_mask = model_ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        line_x = np.arange(0, 20, 1)
        line_y_ransac = model_ransac.predict(line_x[:, np.newaxis])
        return line_y_ransac
    
    def center_line(self, left_line, right_line):
        #find the center line between the left and right line
        
        
        a = (left_line[0] + right_line[0]) / 2
        b = (left_line[-1] + right_line[-1]) / 2
        
        return a, b
    
    
    
 
    def stanlee_path_tracking(self, robot_pose, robot_speed, center_line, heading_ok):
        #stanlee path tracking
        #find the closest point on the center line to the robot
        close_point = self.find_close_point(robot_pose, self.old_close_index)
        self.old_close_index = close_point[0]
        #find the distance between the robot and the closest point
        distance = close_point[1]
        #find the heading error
        heading_error = self.heading_error(robot_pose, self.traj_in.points[close_point[0]].pose)
        #find the cross track error
        cross_track_error = self.cross_track_error(robot_pose, self.traj_in.points[close_point[0]].pose, center_line)
        #find the desired speed
        desired_speed = self.desired_speed(robot_speed, distance, heading_error, cross_track_error, heading_ok)
        #find the desired steering angle
        desired_steering_angle = self.desired_steering_angle(robot_pose, self.traj_in.points[close_point[0]].pose, center_line)
        
        return desired_speed, desired_steering_angle
    
    def heading_error(self, robot_pose, path_pose):
        #find the heading error
        robot_heading = get_yaw(robot_pose.orientation)
        path_heading = get_yaw(path_pose.orientation)
        heading_error = path_heading - robot_heading
        return heading_error
    
    def cross_track_error(self, robot_pose, path_pose, center_line):
        #find the cross track error
        cross_track_error = center_line[self.old_close_index] - robot_pose.position.y
        return cross_track_error
    
    def desired_speed(self, robot_speed, distance, heading_error, cross_track_error, heading_ok):
        #find the desired speed
        if heading_ok:
            desired_speed = self.k1 * distance + self.k2 * heading_error + self.k3 * cross_track_error + self.k4 * robot_speed
        else:
            desired_speed = 0
        return desired_speed
    
    def desired_steering_angle(self, robot_pose, path_pose, center_line):
        #find the desired steering angle
        robot_heading = get_yaw(robot_pose.orientation)
        path_heading = get_yaw(path_pose.orientation)
        desired_steering_angle = math.atan2((center_line[self.old_close_index] - robot_pose.position.y), self.lookahead_distance)
        return desired_steering_angle
    
    def line_fitting_linear_regression(self, points):
        #fit a line to the points using linear regression
        points = np.array(points)
        x = points[:,0]
        y = points[:,1]
        model = linear_model.LinearRegression()
        model.fit(x[:, np.newaxis], y)
        line_x = np.arange(0, 20, 1)
        line_y = model.predict(line_x[:, np.newaxis])
        return line_y
    #find clusters of points in point cloud with min and max size in sklearn

    def find_clusters_sklearn(self, point_cloud):
        #find clusters of points in point cloud with min and max size
        point_cloud = np.array(point_cloud)
        db = DBSCAN(eps=0.6, min_samples=5).fit(point_cloud)
        labels = db.labels_
        clusters = []
        for i in range(0, max(labels) + 1):
            clusters.append(point_cloud[labels == i])
        return clusters
    
    def find_clusters(self, point_cloud):
        #find clusters of points in point cloud with min and max size
        clusters = []
        for point in point_cloud:
            if len(clusters) == 0:
                clusters.append([point])
            else:
                for cluster in clusters:
                    if self.distance(point, cluster[0]) < 0.7:
                        cluster.append(point)
                        break
                else:
                    clusters.append([point])
        return clusters
    #filter point cloud with numpy
    def filter_clusters_size(self, clusters, min_size, max_size):
        #filter clusters with min and max size
        filtered_clusters = []
        for cluster in clusters:
            if len(cluster) >= min_size and len(cluster) <= max_size:
                filtered_clusters.append(cluster)
        return filtered_clusters
    
    def kalman_filter_mean(self,mean):
        #filter points in point cloud using kalman filter
        kf = KalmanFilter(transition_matrices = [1],
                          observation_matrices = [1],
                          initial_state_mean = 0,
                          initial_state_covariance = 1,
                          observation_covariance=1,
                          transition_covariance=.01)
        state_means, _ = kf.filter(mean)
        state_means = [state_mean[0] for state_mean in state_means]
        return state_means

    def filter_point_cloud(self, point_cloud):
        point_cloud = np.array(list(point_cloud))
        point_cloud = point_cloud[point_cloud[:,2] > 0.5]
        point_cloud = point_cloud[point_cloud[:,2] < 3]
        point_cloud = point_cloud[point_cloud[:,0] > 1]
        point_cloud = point_cloud[point_cloud[:,0] < 20]
        point_cloud = point_cloud[point_cloud[:,1] > -7]
        point_cloud = point_cloud[point_cloud[:,1] < 7]
        return point_cloud
    
    def distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)
    
    def kalman_filter_position(self,positions, time_stamp):
        # Define the transition matrix
        transition_matrix = [[1, time_stamp, 0], [0, 1, time_stamp], [0, 0, 1]]
        # Define the observation matrix
        observation_matrix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        # Define the initial state mean
        initial_state_mean = [0, 0, 0]
        # Define the initial state covariance
        initial_state_covariance = [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]
        # Define the transition covariance
        transition_covariance = [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]
        # Define the observation covariance
        observation_covariance = [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]
        # Create the Kalman filter object
        kf = KalmanFilter(transition_matrices=transition_matrix,
                        observation_matrices=observation_matrix,
                        initial_state_mean=initial_state_mean,
                        initial_state_covariance=initial_state_covariance,
                        transition_covariance=transition_covariance,
                        observation_covariance=observation_covariance)
        # Filter the position data
        filtered_positions, _ = kf.filter(positions)
        return filtered_positions
    
    

        

if __name__ == '__main__':
    rospy.init_node('ransac_line_fitting_rows')
    ransac_line_fitting_rows = RansacLineFittingRows()
    rospy.spin()
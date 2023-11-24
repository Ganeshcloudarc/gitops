import numpy as np
import math
from .auto_nav_utils import *


class RansacParallelLineFit:
    def __init__(self, parallel_offset, max_number_of_iterations, offset):
        self.parallel_offset = parallel_offset
        self.max_number_of_iterations = max_number_of_iterations
        self.offset = offset

    def compute_based_of_target_pose(self, cloud_points, forward_point, row_spacing):
        p1 = np.array([0,0])
        p2 = np.array([forward_point[0], forward_point[1]])
        self.laser_np_2d = cloud_points
        self.ransac_tolerance = self.offset
        self.row_spacing = row_spacing

        if self.laser_np_2d is None:
            rospy.logwarn("No update on laser scan")
            rate.sleep()

        p3 = self.laser_np_2d
        # print("laser scan",self.laser_np_2d)
        dir_perpendicular_dis = np.cross(p2 - p1, p1 - p3) / np.linalg.norm(p2 - p1)
        left_points_ind = np.where(np.logical_and(dir_perpendicular_dis > 0,
                                                  dir_perpendicular_dis < self.row_spacing))  # np.logical_and(res >0, res> 0.5)
        # print("left_points_ind", left_points_ind)
        right_points_ind = np.where(
            np.logical_and(dir_perpendicular_dis < 0, dir_perpendicular_dis > -self.row_spacing))
        # print("right_points_ind", right_points_ind)
        laser_np_2d_path_close_points = np.where(abs(dir_perpendicular_dis) < self.row_spacing)
        try:
            self.laser_np_2d_path_close_points = np.take(self.laser_np_2d, laser_np_2d_path_close_points, axis=0)[0]
            self.left_points = np.take(self.laser_np_2d, left_points_ind, axis=0)[0]
            self.right_points = np.take(self.laser_np_2d, right_points_ind, axis=0)[0]
            # print("self.left_points ", self.left_points)
        except IndexError:
            rospy.logerr("Index error ")
            print("left_points_ind", left_points_ind)
            print("right_points_ind", right_points_ind)
            print("laser scan", self.laser_np_2d)
            return 0

        # return [1,1,1], left_points_ind[0], right_points_ind[0]

        # self.all_points_pub.publish(xy_to_pointcloud2(self.laser_np_2d_path_close_points, "map"))
        # rospy.logdebug(f"all points are published")
        # self.left_points_pub.publish(xy_to_pointcloud2(self.left_points, "map"))
        # self.right_points_pub.publish(xy_to_pointcloud2(self.right_points, "map"))
        # # continue
        # rospy.loginfo("left and right points are published")
        # RANSAC METHOD STARTS
        maximum_inliers_count_l = 0
        maximum_inliers_count_r = 0

        # inlier_threshould =

        # if self.prev_line_fit

        left_points_ind_final = []
        right_points_ind_final = []
        random_final_left_points = []
        random_final_right_points = []
        coefficents = []

        for i in range(0, self.max_number_of_iterations):
            # rospy.logdebug("on left side")
            ind1, ind2 = ganerate_random_pairs(self.left_points.shape[0])
            p1_left = np.array([self.left_points[ind1][0], self.left_points[ind1][1]])
            p2_left = np.array([self.left_points[ind2][0], self.left_points[ind2][1]])
            p3_left = self.left_points  # self.laser_np_2d_path_close_points
            # p3 = self.left_points #self.laser_np_2d_path_close_points

            dir_perpendicular_dis = np.cross(p2_left - p1_left, p1_left - p3_left) / np.linalg.norm(
                p2_left - p1_left)
            left_points_inds = np.where((abs(dir_perpendicular_dis) <= self.ransac_tolerance))[0]
            theta = np.pi / 2
            rot = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
            v1 = p2_left - p1_left
            vect = v1 * (1 / np.linalg.norm(v1)) * self.row_spacing
            dot_pro = np.dot(rot, vect)
            p1_right = p1_left + dot_pro
            p2_right = p2_left + dot_pro
            # rospy.loginfo(f"maximum_inliers_count_l : {maximum_inliers_count_l},len(left_points_ind) : {len(left_points_ind)}, maximum_inliers_count_r : {maximum_inliers_count_r},len(right_points_ind) : {len(right_points_ind)}")

            p3_right = self.right_points  # self.laser_np_2d_path_close_points  # finding inliers on all points

            dir_perpendicular_dis = np.cross(p2_right - p1_right, p1_right - p3_right) / np.linalg.norm(
                p2_right - p1_right)
            right_points_inds = np.where((abs(dir_perpendicular_dis) <= self.ransac_tolerance))[0]

            # print("right_points_ind", right_points_ind, type(right_points_ind))
            # print("left_points_ind",left_points_ind, type(left_points_ind) )
            # print("left_points_ind", type(left_points_ind))
            # print(left_points_ind[0])
            # print("len(left_points_ind)", len(left_points_ind))
            # rospy.loginfo(f"maximum_inliers_count_l : {maximum_inliers_count_l},len(left_points_ind) : {len(left_points_ind)}, maximum_inliers_count_r : {maximum_inliers_count_r},len(right_points_ind) : {len(right_points_ind)}")
            if len(left_points_inds) > maximum_inliers_count_l and \
                    len(right_points_inds) > maximum_inliers_count_r:
                maximum_inliers_count_l = len(left_points_ind)
                maximum_inliers_count_r = len(right_points_ind)
                # point1 = Pose2D(p1[0], p1[1])
                # point2 = Pose2D(p2[0], p2[1])

                # p1_right_ = p1_right
                # p2_right_ = p2_right
                random_final_left_points = [p1_left, p2_left]
                random_final_right_points = [p1_right, p2_right]

                if p2_left[0] - p1_left[0] != 0:
                    m = (p2_left[1] - p1_left[1]) / (p2_left[0] - p1_left[0])
                    # line_heading = math.atan2((point2.y - point1.y), (point2.x - point1.x))
                else:
                    m = 1000000
                    # line_heading = math.atan2((point2.y - point1.y), (point2.x - point1.x))

                c_l = p1_left[1] - m * p1_left[0]  # y - mx
                c_r = p1_right[1] - m * p1_right[0]
                coefficents = [m, c_l, c_r]

                # A = point1.y - point2.y
                # B = point2.x - point1.x
                # C = point1.x * point2.y - point2.x * point1.y
                #
                # C2 = C - self.row_spacing * math.sqrt(A * A + B * B)
                # coefficents = [A, B, C, C2]
                # coefficents_m = [-A / B, (C / B + C2 / B) / 2]
                # coefficents_m = [-A / B, C / B]
                left_points_ind_final = left_points_ind
                right_points_ind_final = right_points_ind

            # on right side
            # rospy.logdebug("on right side")
            ind1, ind2 = ganerate_random_pairs(self.right_points.shape[0])
            p1_right = np.array([self.right_points[ind1][0], self.right_points[ind1][1]])
            p2_right = np.array([self.right_points[ind2][0], self.right_points[ind2][1]])
            p3_right = self.right_points  # self.laser_np_2d_path_close_points
            dir_perpendicular_dis = np.cross(p2_right - p1_right, p1_right - p3_right) / np.linalg.norm(
                p2_right - p1_right)
            right_points_inds = np.where((abs(dir_perpendicular_dis) <= self.ransac_tolerance))[0]
            theta = -np.pi / 2
            rot = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
            v1 = p2_right - p1_right
            vect = v1 * (1 / np.linalg.norm(v1)) * self.row_spacing
            dot_pro = np.dot(rot, vect)
            p1_left = p1_right + dot_pro
            p2_left = p2_right + dot_pro
            p3_left = self.left_points  # self.laser_np_2d_path_close_points
            dir_perpendicular_dis = np.cross(p2_left - p1_left, p1_left - p3_left) / np.linalg.norm(
                p2_left - p1_left)
            left_points_inds = np.where((abs(dir_perpendicular_dis) <= self.ransac_tolerance))[0]
            # rospy.loginfo(f"maximum_inliers_count_l : {maximum_inliers_count_l},len(left_points_ind) : {len(left_points_ind)}, maximum_inliers_count_r : {maximum_inliers_count_r},len(right_points_ind) : {len(right_points_ind)}")

            if len(left_points_inds) > maximum_inliers_count_l and \
                    len(right_points_inds) > maximum_inliers_count_r:
                maximum_inliers_count_l = len(left_points_ind)
                maximum_inliers_count_r = len(right_points_ind)
                random_final_left_points = [p1_left, p2_left]
                random_final_right_points = [p1_right, p2_right]

                if p2_left[0] - p1_left[0] != 0:
                    m = (p2_left[1] - p1_left[1]) / (p2_left[0] - p1_left[0])
                    # line_heading = math.atan2((point2.y - point1.y), (point2.x - point1.x))
                else:
                    m = 1000000
                    # line_heading = math.atan2((point2.y - point1.y), (point2.x - point1.x))

                c_r = p1_right[1] - m * p1_right[0]  # y - mx
                c_l = p1_left[1] - m * p1_left[0]
                coefficents = [m, c_l, c_r]
                # random_final_left_points.append()

                # A = point1.y - point2.y
                # B = point2.x - point1.x
                # C = point1.x * point2.y - point2.x * point1.y
                #
                # C2 = C - self.row_spacing * math.sqrt(A * A + B * B)
                # coefficents = [A, B, C, C2]
                # coefficents_m = [-A / B, (C / B + C2 / B) / 2]
                # coefficents_m = [-A / B, C / B]
                left_points_ind_final = left_points_ind
                right_points_ind_final = right_points_ind

        ## publish inliers

        return coefficents, left_points_ind_final[0], right_points_ind_final[0]
        # return coefficents, left_points_ind, right_points_ind

    def compute_based_robot_pose(self, cloud_points, end_point):
        """
        Args:
            robot_pose_2d robot_pose
        """

        target_point_dir = math.atan2(end_point.y, end_point.x)
        robot_tie = np.array([np.cos(target_point_dir), np.sin(target_point_dir)])
        s = Pose2D(0, 0)
        target_line = two_points_to_line(s, end_point)
        indexs = target_line.inliers_cross_product(cloud_points, 6)
        # print("indexs", indexs)
        cloud_near_target_line = np.take(cloud_points, indexs, axis=0)
        # return cloud_near_target_line,cloud_near_target_line
        cloud_points = cloud_near_target_line
        max_left_inliers = 0
        max_right_inliers = 0
        left_inliers_index = []
        right_inliers_index = []
        for i in range(0, self.max_number_of_iterations):
            while True:
                ind1, ind2 = np.random.randint(cloud_points.shape[0], size=2)
                p1, p2 = cloud_points[ind1], cloud_points[ind2]
                points_heading = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
                print("target_point_dir", target_point_dir)
                print("points_heading", points_heading)
                if abs(target_point_dir - points_heading) > math.radians(30):
                    continue
                if p1[1] > 0 and p2[1] > 0:
                # if (np.cross(robot_tie, p1) > 0) and (np.cross(robot_tie, p2) > 0):
                    # both on left side
                    rot = get_2d_rotation_matrix(theta=-np.pi / 2)
                    points_vect = p2 - p1
                    vect = points_vect * (1 / np.linalg.norm(points_vect)) * self.parallel_offset
                    dot_pro = np.dot(rot, vect)
                    per_point1 = p1 + dot_pro
                    per_point2 = p2 + dot_pro
                    # if (np.cross(robot_tie, per_point1) < 0) and (np.cross(robot_tie, per_point2) < 0):
                    if per_point1[1] < 0 and per_point2[1] < 0:
                        left_line = two_points_to_line(p1, p2)
                        right_line = two_points_to_line(per_point1, per_point2)
                        break
                    else:
                        pass

                elif p1[1] < 0 and p2[1] < 0:
                # if (np.cross(robot_tie, p1) < 0) and (np.cross(robot_tie, p2) < 0):
                    # Both are Right side
                    rot = get_2d_rotation_matrix(theta=np.pi / 2)
                    points_vect = p2 - p1
                    vect = points_vect * (1 / np.linalg.norm(points_vect)) * self.parallel_offset
                    dot_pro = np.dot(rot, vect)
                    per_point1 = p1 + dot_pro
                    per_point2 = p2 + dot_pro
                    # if (np.cross(robot_tie, per_point1) < 0) and (np.cross(robot_tie, per_point2) < 0):
                    if per_point1[1] > 0 and per_point2[1] > 0:
                        right_line = two_points_to_line(p1, p2)
                        left_line = two_points_to_line(per_point1, per_point2)
                        break
                    else:
                        pass
                else:
                    pass
        print("left_line")
        left_line_inl = left_line.inliers_cross_product(cloud_points, self.offset)
        right_line_inl = right_line.inliers_cross_product(cloud_points, self.offset)
        if len(left_line_inl) > max_left_inliers and len(right_line_inl) > max_right_inliers:
            max_left_inliers = len(left_line_inl)
            max_right_inliers = len(right_line_inl)
            left_inliers_index = left_line_inl
            right_inliers_index = right_line_inl
        else:
            pass
        return left_inliers_index, right_inliers_index
        # return np.take(cloud_points, left_inliers_index, axis=0), np.take(cloud_points, right_inliers_index, axis=0)

    def compute_based_on_prev_center_line(self, prev_lise, cloud_points):
        pass


if __name__ == "__main__":
    rospy.init_node("ransac_impl")
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Path
    from sklearn.linear_model import LinearRegression
    import sys
    from sub_handler import SubscriberHandler

    sub_handler = SubscriberHandler()
    ransac_parallel_line = RansacParallelLineFit(6, 100, 2.5)
    left_cloud_pub = rospy.Publisher("left_cloud", PointCloud2, queue_size=1)
    right_cloud_pub = rospy.Publisher("right_cloud", PointCloud2, queue_size=1)
    left_path_pub = rospy.Publisher("left_path", Path, queue_size=1)
    right_path_pub = rospy.Publisher("right_path", Path, queue_size=1)
    rate = rospy.Rate(1)
    rate.sleep()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if sub_handler.laser_scan_data:
            cloud_points = sub_handler.get_scan("base_link")
            if cloud_points is None:
                rate.sleep()
                continue
            left_inliers, right_inliers = ransac_parallel_line.compute_based_robot_pose(cloud_points,
                                                                                        Pose2D(100, 0))
            print("left_inliers", left_inliers)
            print("right_inliers", right_inliers)

            left_cloud = np.take(cloud_points, left_inliers, axis=0)
            right_cloud = np.take(cloud_points, right_inliers, axis=0)
            # left_cloud, right_cloud = left_inliers, right_inliers
            print("left_cloud", left_cloud)
            left_cloud_pub.publish(xy_to_pointcloud2(left_cloud, "base_link"))
            right_cloud_pub.publish(xy_to_pointcloud2(right_cloud, "base_link"))
            model = LinearRegression()
            model.fit(left_cloud[:, 0, np.newaxis],
                      left_cloud[:, 1, np.newaxis])
            right_line_slope_base_link, right_y_intercept_base_link = model.coef_[0][0], model.intercept_[0]
            p1 = [10, right_line_slope_base_link * 10 + right_y_intercept_base_link]
            p2 = [-10, right_line_slope_base_link * -10 + right_y_intercept_base_link]
            path = two_points_to_path(p1, p2, "base_link")
            left_path_pub.publish(path)
            model = LinearRegression()
            model.fit(right_cloud[:, 0, np.newaxis],
                      right_cloud[:, 1, np.newaxis])
            right_line_slope_base_link, right_y_intercept_base_link = model.coef_[0][0], model.intercept_[0]
            p1 = [10, right_line_slope_base_link * 10 + right_y_intercept_base_link]
            p2 = [-10, right_line_slope_base_link * -10 + right_y_intercept_base_link]
            path = two_points_to_path(p1, p2, "base_link")
            right_path_pub.publish(path)
            rate.sleep()

    rospy.spin()

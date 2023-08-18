'''ransac line fitting from point cloud by detecting the tree trunk and branches'''

import rospy
from sensor_msgs.msg import PointCloud2
import skimage
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
from visualization_msgs.msg import MarkerArray, Marker
from sklearn import linear_model
import math
import time
from geometry_msgs.msg import Point

class RansacLineFitting:
    def __init__(self):
        self.pub = rospy.Publisher('/ransac_line_fitting', PointCloud2, queue_size=1)
        self.trunk_marker_pub = rospy.Publisher('/ransac_line_fitting_marker/trunk', MarkerArray, queue_size=1)
        self.branch_marker_pub = rospy.Publisher('/ransac_line_fitting_marker/branch', MarkerArray, queue_size=1)
        self.sub = rospy.Subscriber('/rslidar_points', PointCloud2, self.callback, queue_size=1)
        
    def callback(self, data):
        # get point cloud
        points = []
        for p in pc2.read_points(data, skip_nans=True):
            points.append([p[0], p[1], p[2]])
        points = np.array(points)
        # get the point cloud of the tree trunk
        trunk_points = []
        for p in points:
            if p[2] > 1.5 and p[2] < 2.5:
                trunk_points.append(p)
        trunk_points = np.array(trunk_points)
        # get the point cloud of the branches
        branch_points = []
        for p in points:
            if p[2] > 2.5:
                branch_points.append(p)
        branch_points = np.array(branch_points)
        # ransac line fitting
        trunk_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression())
        trunk_ransac.fit(trunk_points[:, 0:2], trunk_points[:, 2])
        trunk_inlier_mask = trunk_ransac.inlier_mask_
        trunk_outlier_mask = np.logical_not(trunk_inlier_mask)
        trunk_inlier_points = trunk_points[trunk_inlier_mask]
        trunk_outlier_points = trunk_points[trunk_outlier_mask]
        branch_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression())
        branch_ransac.fit(branch_points[:, 0:2], branch_points[:, 2])
        branch_inlier_mask = branch_ransac.inlier_mask_
        branch_outlier_mask = np.logical_not(branch_inlier_mask)
        branch_inlier_points = branch_points[branch_inlier_mask]
        branch_outlier_points = branch_points[branch_outlier_mask]
        # publish the point cloud of the tree trunk
        trunk_inlier_points = np.hstack((trunk_inlier_points, np.zeros((trunk_inlier_points.shape[0], 1))))
        trunk_outlier_points = np.hstack((trunk_outlier_points, np.zeros((trunk_outlier_points.shape[0], 1))))
        trunk_inlier_points = np.hstack((trunk_inlier_points, np.ones((trunk_inlier_points.shape[0], 1))))
        trunk_outlier_points = np.hstack((trunk_outlier_points, np.ones((trunk_outlier_points.shape[0], 1))))

        # publish the point cloud of the branches
        branch_inlier_points = np.hstack((branch_inlier_points, np.zeros((branch_inlier_points.shape[0], 1))))
        branch_outlier_points = np.hstack((branch_outlier_points, np.zeros((branch_outlier_points.shape[0], 1))))
        branch_inlier_points = np.hstack((branch_inlier_points, np.ones((branch_inlier_points.shape[0], 1))))
        branch_outlier_points = np.hstack((branch_outlier_points, np.ones((branch_outlier_points.shape[0], 1))))
        

        # publish the line of the tree trunk
        trunk_line = []
        trunk_line.append([trunk_ransac.estimator_.coef_[0], trunk_ransac.estimator_.coef_[1], trunk_ransac.estimator_.intercept_])
        trunk_line = np.array(trunk_line)
        trunk_line = np.hstack((trunk_line, np.zeros((trunk_line.shape[0], 1))))
        trunk_line = np.hstack((trunk_line, np.ones((trunk_line.shape[0], 1))))


        #filter the points that are not on the line
        trunk_line_points = []
        for p in trunk_inlier_points:
            if abs(trunk_ransac.estimator_.coef_[0] * p[0] + trunk_ransac.estimator_.coef_[1] * p[1] + trunk_ransac.estimator_.intercept_ - p[2]) < 0.1:
                trunk_line_points.append(p)
        trunk_line_points = np.array(trunk_line_points)
        trunk_line_points = np.hstack((trunk_line_points, np.zeros((trunk_line_points.shape[0], 1))))
        trunk_line_points = np.hstack((trunk_line_points, np.ones((trunk_line_points.shape[0], 1))))
        print(trunk_line_points.shape)
        # publish the line of the branches
        branch_line = []
        branch_line.append([branch_ransac.estimator_.coef_[0], branch_ransac.estimator_.coef_[1], branch_ransac.estimator_.intercept_])
        branch_line = np.array(branch_line)
        branch_line = np.hstack((branch_line, np.zeros((branch_line.shape[0], 1))))
        branch_line = np.hstack((branch_line, np.ones((branch_line.shape[0], 1))))
        # publish the markers of the tree trunk
        trunk_marker = MarkerArray()
        trunk_marker.markers.append(Marker())
        trunk_marker.markers[0].header.frame_id = 'rslidar'
        trunk_marker.markers[0].header.stamp = rospy.Time.now()
        trunk_marker.markers[0].ns = 'tree_trunk'
        trunk_marker.markers[0].id = 0
        trunk_marker.markers[0].type = Marker.LINE_STRIP
        trunk_marker.markers[0].action = Marker.ADD
        trunk_marker.markers[0].pose.position.x = 0
        trunk_marker.markers[0].pose.position.y = 0
        trunk_marker.markers[0].pose.position.z = 0
        trunk_marker.markers[0].pose.orientation.x = 0
        trunk_marker.markers[0].pose.orientation.y = 0
        trunk_marker.markers[0].pose.orientation.z = 0
        trunk_marker.markers[0].pose.orientation.w = 1
        trunk_marker.markers[0].scale.x = 0.1
        trunk_marker.markers[0].scale.y = 0.1
        trunk_marker.markers[0].scale.z = 0.1
        trunk_marker.markers[0].color.a = 1
        trunk_marker.markers[0].color.r = 1
        trunk_marker.markers[0].color.g = 0
        trunk_marker.markers[0].color.b = 0
        trunk_marker.markers[0].lifetime = rospy.Duration(0)
        trunk_marker.markers[0].points = []
        for p in trunk_inlier_points:
            trunk_marker.markers[0].points.append(Point(p[0], p[1], p[2]))
        trunk_marker.markers.append(Marker())
        trunk_marker.markers[1].header.frame_id = 'rslidar'
        trunk_marker.markers[1].header.stamp = rospy.Time.now()
        trunk_marker.markers[1].ns = 'tree_trunk'
        trunk_marker.markers[1].id = 1
        trunk_marker.markers[1].type = Marker.LINE_STRIP
        trunk_marker.markers[1].action = Marker.ADD
        trunk_marker.markers[1].pose.position.x = 0
        trunk_marker.markers[1].pose.position.y = 0
        trunk_marker.markers[1].pose.position.z = 0
        trunk_marker.markers[1].pose.orientation.x = 0
        trunk_marker.markers[1].pose.orientation.y = 0
        trunk_marker.markers[1].pose.orientation.z = 0
        trunk_marker.markers[1].pose.orientation.w = 1
        trunk_marker.markers[1].scale.x = 0.1
        trunk_marker.markers[1].scale.y = 0.1
        trunk_marker.markers[1].scale.z = 0.1
        trunk_marker.markers[1].color.a = 1
        trunk_marker.markers[1].color.r = 0
        trunk_marker.markers[1].color.g = 1
        trunk_marker.markers[1].color.b = 0
        trunk_marker.markers[1].lifetime = rospy.Duration(0)
        trunk_marker.markers[1].points = []
        for p in trunk_outlier_points:
            trunk_marker.markers[1].points.append(Point(p[0], p[1], p[2]))

        # publish the markers of the branches
        branch_marker = MarkerArray()
        branch_marker.markers.append(Marker())
        branch_marker.markers[0].header.frame_id = 'rslidar'
        branch_marker.markers[0].header.stamp = rospy.Time.now()
        branch_marker.markers[0].ns = 'tree_branch'
        branch_marker.markers[0].id = 0
        branch_marker.markers[0].type = Marker.LINE_STRIP
        branch_marker.markers[0].action = Marker.ADD
        branch_marker.markers[0].pose.position.x = 0
        branch_marker.markers[0].pose.position.y = 0
        branch_marker.markers[0].pose.position.z = 0
        branch_marker.markers[0].pose.orientation.x = 0
        branch_marker.markers[0].pose.orientation.y = 0
        branch_marker.markers[0].pose.orientation.z = 0
        branch_marker.markers[0].pose.orientation.w = 1
        branch_marker.markers[0].scale.x = 0.1
        branch_marker.markers[0].scale.y = 0.1
        branch_marker.markers[0].scale.z = 0.1
        branch_marker.markers[0].color.a = 1
        branch_marker.markers[0].color.r = 1
        branch_marker.markers[0].color.g = 0
        branch_marker.markers[0].color.b = 0
        branch_marker.markers[0].lifetime = rospy.Duration(0)
        branch_marker.markers[0].points = []
        for p in branch_inlier_points:
            branch_marker.markers[0].points.append(Point(p[0], p[1], p[2]))
        branch_marker.markers.append(Marker())
        branch_marker.markers[1].header.frame_id = 'rslidar'
        branch_marker.markers[1].header.stamp = rospy.Time.now()
        branch_marker.markers[1].ns = 'tree_branch'
        branch_marker.markers[1].id = 1
        branch_marker.markers[1].type = Marker.LINE_STRIP
        branch_marker.markers[1].action = Marker.ADD
        branch_marker.markers[1].pose.position.x = 0
        branch_marker.markers[1].pose.position.y = 0
        branch_marker.markers[1].pose.position.z = 0
        branch_marker.markers[1].pose.orientation.x = 0
        branch_marker.markers[1].pose.orientation.y = 0
        branch_marker.markers[1].pose.orientation.z = 0
        branch_marker.markers[1].pose.orientation.w = 1
        branch_marker.markers[1].scale.x = 0.1
        branch_marker.markers[1].scale.y = 0.1
        branch_marker.markers[1].scale.z = 0.1
        branch_marker.markers[1].color.a = 1
        branch_marker.markers[1].color.r = 0
        branch_marker.markers[1].color.g = 1
        branch_marker.markers[1].color.b = 0
        branch_marker.markers[1].lifetime = rospy.Duration(0)
        branch_marker.markers[1].points = []
        for p in branch_outlier_points:
            branch_marker.markers[1].points.append(Point(p[0], p[1], p[2]))

        self.trunk_marker_pub.publish(trunk_marker)
        self.branch_marker_pub.publish(branch_marker)


if __name__ == '__main__':
    rospy.init_node('tree_detection')
    tree_detection = RansacLineFitting()
    rospy.spin()
        
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseArray, Pose, TransformStamped, PoseStamped, PolygonStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from jsk_recognition_msgs.msg import PolygonArray
from autopilot_msgs.msg import Trajectory
from sensor_msgs.msg import PointCloud2

class PublisherHandler:
    def __init__(self):
        self.route_path_pub = rospy.Publisher("route_path", Path, queue_size=1, latch=True)
        self.snake_path_pub = rospy.Publisher("snake_path", Path, queue_size=1, latch=True)
        self.dubins_path_pub = rospy.Publisher("dubins_path", Path, queue_size=1, latch=True)
        self.dwa_paths_pub = rospy.Publisher("dwa_paths", MarkerArray, queue_size=1, latch=True)
        self.enlarged_bboxes_pub = rospy.Publisher("enlarged_bboxes", PolygonArray, queue_size=1, latch=True)
        self.dwa_collision_free_paths_pub = rospy.Publisher("dwa_collision_free_paths", MarkerArray, queue_size=1,
                                                            latch=True)
        self.dwa_collision_free_and_close_to_line_pub = rospy.Publisher("dwa_collision_free_and_close_to_line", Path,
                                                                        queue_size=1, latch=True)
        self.local_trajectory_pub = rospy.Publisher("local_gps_trajectory", Trajectory, queue_size=1)
        self.row_end_pose_base_link_pub = rospy.Publisher("row_end_pose_base_link", PoseStamped, queue_size=1)
        self.left_inliers_pub = rospy.Publisher("left_liners", PointCloud2, queue_size=1)
        self.right_inliers_pub = rospy.Publisher("right_liners", PointCloud2, queue_size=1)
        self.left_line_pub = rospy.Publisher("left_line", Path, queue_size=1)
        self.right_line_pub = rospy.Publisher("right_line", Path, queue_size=1)
        self.center_line_pub = rospy.Publisher("center_line", Path, queue_size=1)
        self.vanish_pose_pub = rospy.Publisher("vanish_pose", PoseStamped, queue_size=1)

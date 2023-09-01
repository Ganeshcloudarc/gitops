import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from pykalman import KalmanFilter

class PathFilter:
    def init(self):
        self.path_sub = rospy.Subscriber("/path", Path, self.path_callback)
        self.filtered_path_pub = rospy.Publisher("/filtered_path", Path, queue_size=1)
            # Initialize Kalman filter
        self.kf = KalmanFilter(transition_matrices=np.array([[1, 0, 1, 0],
                                                            [0, 1, 0, 1],
                                                            [0, 0, 1, 0],
                                                            [0, 0, 0, 1]]),
                            observation_matrices=np.array([[1, 0, 0, 0],
                                                            [0, 1, 0, 0]]))
        # initialize the mean and covariance
        self.kf.state_mean = np.array([0,0,0,0])
        self.kf.state_covariance = np.eye(4)

        self.path = Path()
        self.filtered_path = Path()
        self.filtered_path.header.frame_id = "rslidar"

    def path_callback(self, path):
        print("Path received",path)
        self.path = path
        self.filtered_path.poses = []
        for i in range(1, len(self.path.poses)):
            current_pose = self.path.poses[i]
            (predicted_state, _) = self.kf.filter_update(filtered_state_mean=self.kf.state_mean, filtered_state_covariance=self.kf.state_covariance, observation=np.array([current_pose.pose.position.x, current_pose.pose.position.y]))
            filtered_pose = PoseStamped()
            filtered_pose.header.frame_id = "rslidar"
            filtered_pose.pose.position.x = predicted_state[0]
            filtered_pose.pose.position.y = predicted_state[1]
            self.filtered_path.poses.append(filtered_pose)
            self.kf.state_mean = predicted_state
        
        self.filtered_path_pub.publish(self.filtered_path)

if __name__ == "__main__":
    rospy.init_node("filtered_path")
    path_filter = PathFilter()
    path_filter.init()
    rospy.spin()

from scipy.signal import butter, lfilter
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy

class PathFilter:
    def __init__(self):
        self.path_sub = rospy.Subscriber("/path", Path, self.path_callback)
        self.filtered_path_pub = rospy.Publisher("/filtered_path", Path, queue_size=1)

            # Define low pass filter
        self.cutoff_frequency = 0.5 # cutoff frequency in Hz
        self.fs = 10.0 # sample rate in Hz
        self.nyquist_frequency = 0.5 * self.fs
        self.low_cutoff_frequency = self.cutoff_frequency / self.nyquist_frequency
        self.b, self.a = butter(1, self.low_cutoff_frequency, btype='low')
        self.filter_y = []
        
        self.path = Path()
        self.filtered_path = Path()
        self.filtered_path.header.frame_id = "rslidar"

    def path_callback(self, path):
        self.path = path

        # Filter x and y coordinates
        self.filter_y = lfilter(self.b, self.a, [pose.pose.position.y for pose in self.path.poses])
        self.filter_x = lfilter(self.b, self.a, [pose.pose.position.x for pose in self.path.poses])

        # Create filtered path
        self.filtered_path.poses = []
        for i in range(1, len(self.path.poses)):
            filtered_pose = PoseStamped()
            filtered_pose.header.frame_id = "rslidar"
            filtered_pose.pose.position.x = self.filter_x[i]
            filtered_pose.pose.position.y = self.filter_y[i]
            self.filtered_path.poses.append(filtered_pose)

        self.filtered_path_pub.publish(self.filtered_path)



if __name__ == "__main__":
    rospy.init_node("filtered_path")
    path_filter = PathFilter()
    rospy.spin()


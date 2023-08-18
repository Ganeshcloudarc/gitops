import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from pykalman import KalmanFilter
import numpy as np

class BoundingBoxFilter:
    def __init__(self):
        self.box_sub = rospy.Subscriber("/filtered_detector/jsk_bboxes", BoundingBoxArray, self.bounding_box_callback)
        self.filtered_box_pub = rospy.Publisher("/filtered_bounding_boxes", BoundingBoxArray, queue_size=1)

        # Initialize Kalman filter
        self.kf = KalmanFilter(transition_matrices=np.array([[1, 1], [0, 1]]),
                              observation_matrices=np.array([[1, 0]]))

        # Initialize state and covariance
        self.state_mean = np.array([0, 0])
        self.state_covariance = np.array([[0, 0], [0, 0]])

    def bounding_box_callback(self, boxes):
        filtered_boxes = BoundingBoxArray()
        filtered_boxes.header = boxes.header
        filtered_boxes = []

        for box in boxes.boxes:
            # Get current y position
            y = box.pose.position.y

            # Run Kalman filter prediction and correction
            (predicted_state, predicted_cov) = self.kf.filter_update(self.state_mean, self.state_covariance, observation=np.array([y]))

            # Update state and covariance
            self.state_mean = predicted_state
            self.state_covariance = predicted_cov

            # append filtered box to filtered_boxes
            filtered_boxes.append(predicted_state)

        # Publish filtered boxes
        print("Filtered boxes:", filtered_boxes,'len:',len(filtered_boxes))
if __name__ == "__main__":
    rospy.init_node("bounding_box_filter")
    box_filter = BoundingBoxFilter()
    rospy.spin()

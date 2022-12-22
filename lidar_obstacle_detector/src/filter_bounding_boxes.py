import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray

class obs_detection:
    def __init__(self):
        self.sub = rospy.Subscriber("/obstacle_detector/jsk_bboxes",BoundingBoxArray,self.Bounding_Cb)
        self.pub = rospy.Publisher("/filtered_detector/jsk_bboxes",BoundingBoxArray,queue_size=50)
        self.box_msg = BoundingBoxArray()
        self.filtered_box_msg = BoundingBoxArray()

    def Bounding_Cb(self,data):
        
        box_list = data.boxes
        size = len(box_list)

        boxes = []

        for i in range(0,size-1):
            self.box_msg = box_list[i]
            pos_x = self.box_msg.pose.position.x
            pos_z = self.box_msg.pose.position.z

            if(pos_z < -0.3):
                pass

            else:
                boxes.append(box_list[i])

        self.filtered_box_msg.boxes = boxes
        self.filtered_box_msg.header = data.header
        
        self.pub.publish(self.filtered_box_msg)
        print("publishing")

        
if __name__=="__main__":
    rospy.init_node("filtered_bounding_box_data")
    while not rospy.is_shutdown():
        obs_detection()
        rospy.spin()
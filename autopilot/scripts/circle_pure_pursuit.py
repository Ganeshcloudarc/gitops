#!/usr/bin/env python3
try:
    import rospy
    import rospkg
    import math
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Vector3, Point, PoseArray
    from visualization_msgs.msg import Marker
    from std_msgs.msg import Header, ColorRGBA, String
    from autopilot_utils.tf_helper import current_robot_pose,convert_point
    from tf.transformations import euler_from_quaternion, quaternion_from_euler


except Exception as e:
    print('No module named :', str(e))
    exit(e)


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


def rotate_point(point, degree):
    rotated_point = Point()
    yaw = math.radians(degree)
    rotated_point.x = math.cos(yaw)* point.x - math.sin(yaw) * point.y
    rotated_point.y = math.sin(yaw)* point.x + math.cos(yaw) * point.y
    return rotated_point

class Pure_Pursuit_Circle:
    def __init__(self):
        self.target_pose = PoseStamped()
        self.base_frame = "ego_vehicle"
        sub = rospy.Subscriber("target_pose", Pose, self.target_pose_cb)
        
        self.pure_pursuit_circle_vis = rospy.Publisher("/circle_pure_purusit_marker", Marker, queue_size=2)
        self.target_point_pub =  rospy.Publisher("/target_frame_base_link", PoseStamped, queue_size=2)
        self.circle_pose_arr = rospy.Publisher("circle_pose_arr", PoseArray, queue_size=1)


        self.main_loop()
    
    def target_pose_cb(self, data):
        self.target_pose = data

    def main_loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            robot_pose = current_robot_pose("map", self.base_frame)
            if robot_pose is None:
                rospy.logwarn("Waiting for tf")
                rate.sleep()
                continue
            # print(robot_pose)
            lhd = distance_btw_poses(robot_pose, self.target_pose.pose)
            dy = robot_pose.position.y - self.target_pose.pose.position.y
            point= PoseStamped()
            position = convert_point(self.target_pose.pose.position, "map", self.base_frame)
            point.header.frame_id = self.base_frame
            point.pose.position = position
            self.target_point_pub.publish(point)
            # print(point)
            rate.sleep()






            radius = lhd**2/(2*(abs(position.y)))
            range_ = 2*math.pi
            inrement = 0.01
            pose_arr = PoseArray()
            pose_arr.header.frame_id = "ego_vehicle"
            i = 0
            while i < range_:
                p = Point()
                p.x = radius* math.cos(i)
                p.y = radius * math.sin(i)
                # tramsform to( radius, 0)
                relative_p = Point()
                relative_p.x = p.x - radius
                relative_p.y = p.y
                # rotate -90
                rotated_point = rotate_point(relative_p, -90)

                pose = Pose()
                pose.position = rotated_point

                pose_arr.poses.append(pose)
                i = i+ inrement

            self.circle_pose_arr.publish(pose_arr)
            # .poses = 
                
            x1,y1= robot_pose.position.x , robot_pose.position.y
            x2,y2 = self.target_pose.pose.position.x,self.target_pose.pose.position.y
            x3,y3 = (x1+x2)/2, (y1+y2)/2
            # print(radius**2 - lhd**2)
            # print(math.sqrt(abs(radius**2 - lhd**2)))
            xc = x3+ math.sqrt(abs(radius**2 - lhd**2))*(y1-y2)/lhd
            yc = y3+math.sqrt(abs(radius**2 - lhd**2))*(x2-x1)/lhd
            



            # const double range = M_PI / 8;
            # const double increment = 0.1;

            # for (double i = 0; i < range; i += increment)
            # {
            # // calc a point of circumference
            # geometry_msgs::Point p;
            # p.x = radius * cos(i);
            # p.y = radius * sin(i);

            # // transform to (radius,0)
            # geometry_msgs::Point relative_p;
            # relative_p.x = p.x - radius;
            # relative_p.y = p.y;

            # // rotate -90°
            # geometry_msgs::Point rotate_p = rotatePoint(relative_p, -90);

            # // transform to vehicle plane
            # geometry_msgs::Point tf_p = calcAbsoluteCoordinate(rotate_p, current_pose);

            # traj_circle_array.push_back(tf_p);
            # }

            print("steering angle", math.degrees(1/radius))
            print("radius",radius )
            d = radius - position.y
            print("d",d) 

            center_pose = Pose()
            center_pose.position.x = xc
            # center_pose.position.y = self.target_pose.pose.position.y + d
            center_pose.position.y = yc

            global vehicle_pose_marker
            marker = Marker(
                type=Marker.SPHERE,
                id=0,
                lifetime=rospy.Duration(1),
                pose=center_pose,
                scale=Vector3(radius, radius, 0.1),
                header=Header(frame_id='map'),
                color=ColorRGBA(1, 1, 0, 0.5))
            rospy.logdebug("vehicle_pose_callback")
            self.pure_pursuit_circle_vis.publish(marker)
            
       
if __name__ == "__main__":
    rospy.init_node("pp_cirlce_vis")

    pps= Pure_Pursuit_Circle()
    rospy.spin()
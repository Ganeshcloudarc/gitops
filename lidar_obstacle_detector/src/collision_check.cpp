#include<ros/ros.h>
#include<math.h>
#include<ackermann_msgs/AckermannDrive.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>
#include<vector>
#include <typeinfo>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher pub;
using namespace std;

vector<geometry_msgs::PoseStamped> poses;
vector<jsk_recognition_msgs::BoundingBox> boxes;

geometry_msgs::Pose curr_position;
void path_cb(nav_msgs::Path data)
{
   cout<<typeid(data.poses).name()<<endl;
   poses = data.poses;  
}

void jsk_boxes_cb(jsk_recognition_msgs::BoundingBoxArray data)
{
    vector<geometry_msgs::PoseStamped>::iterator itr;
/**    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    geometry_msgs::TransformStamped base_link_to_map; // My frames are named "base_link" and "map"
    base_link_to_map = tf_buffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0) );

    auto robot_pose = poses[0];
    tf2::doTransform(robot_pose, robot_pose, base_link_to_map);
    // ROS_INFO_STREAM(poses[j].header.frame_id);
    **/

    if(data.header.frame_id == "map")
        boxes = data.boxes;

    else;
}

void current_pose_cb(nav_msgs::Odometry data)
{
    cout<<"Position = "<<data.pose.pose.position.x<<endl;
    curr_position = data.pose.pose;
}

int distance(float a,float b,float z1,float c,float d,float z2)
{
    return sqrt((a-c)*(a-c) + (b-d)*(b-d) + (z2-z1)*(z2-z1));
}

void target_pose_cb(geometry_msgs::PoseStamped data)
{
    int n = poses.size();
    float pos = data.pose.position.x;
    for(int i=0;i<n;i++)
    {
        auto curr_pos = poses[i].pose.position.x;
        if(curr_pos == pos)
        {
            cout<<"i am here"<<endl;
            for(int j=i-20;j<i+15;j++)
            {
                for(int k=0;k<boxes.size();k++)
                {
                    int x = boxes[k].pose.position.x;
                    int y = boxes[k].pose.position.y;
                    int z = boxes[k].pose.position.z;

                    int a = poses[j].pose.position.x;
                    int b = poses[j].pose.position.y;
                    int c = poses[j].pose.position.z;

                    int dist = distance(x,y,z,a,b,c);

                    float curr_x = curr_position.position.x;
                    float curr_y = curr_position.position.y;
                    float curr_z = curr_position.position.z;

                    float dist_from_vehicle = distance(curr_x,curr_y,curr_z,a,b,c);

                    cout<<"a: "<<a;
                    cout<<"b: "<<b;
                    cout<<"c: "<<c<<endl;

                    cout<<"x: "<<curr_x;
                    cout<<"y: "<<curr_y;
                    cout<<"z: "<<curr_z<<endl;
                    cout<<"Distance = "<<dist_from_vehicle<<endl;
                }
            }   
        }
        
        else
        {
            ;
        }
    }
}

void vehicle_cmd_drive_cb(ackermann_msgs::AckermannDrive data)
{
    ;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Detecting");
    ros::NodeHandle nh;
    pub = nh.advertise<ackermann_msgs::AckermannDrive>("/vehicle/cmd_drive_safe",10);
    ros::Subscriber sub = nh.subscribe("/filtered_detector/jsk_bboxes",10,jsk_boxes_cb);
    ros::Subscriber sub0 = nh.subscribe("/odom_path",10,path_cb);
    ros::Subscriber sub1 = nh.subscribe("/vehicle/cmd_drive_nosafe",10,vehicle_cmd_drive_cb);
    ros::Subscriber sub2 = nh.subscribe("/target_pose",10,target_pose_cb);
    ros::Subscriber sub3 = nh.subscribe("/mavros/global_position/local",10,current_pose_cb);
    ros::spin();
    return 0;
}
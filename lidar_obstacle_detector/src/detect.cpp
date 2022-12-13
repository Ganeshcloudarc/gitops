#include<ros/ros.h>
#include<math.h>
#include<ackermann_msgs/AckermannDrive.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<vector>

using namespace std;
ros::Publisher pub;

float distance(float x1,float y1,float z1)
{
    return sqrt((x1)*(x1) + (y1)*(y1) + (z1)*(z1));
}

int obs = 0;
void jsk_boxes_cb(jsk_recognition_msgs::BoundingBoxArray data)
{
    vector<jsk_recognition_msgs::BoundingBox> box;
    int n = data.boxes.size();
    
    for(int i=0;i<n;i++)
    {
        int x = data.boxes[i].pose.position.x;
        int y = data.boxes[i].pose.position.y;
        int z = data.boxes[i].pose.position.z;
        int dist = distance(x,y,z);
        cout<<" i am inside callback"<<endl;
        if(dist<3)
        {
            if(dist<2.5 && dist >0.5)
            {
                obs = 1;
            }
        }
    }

    box = data.boxes;
}


void vehicle_cmd_drive_cb(ackermann_msgs::AckermannDrive data)
{
    float speed = data.speed;
    float steering_angle = data.steering_angle;
    float acceleration = data.acceleration;

    ackermann_msgs::AckermannDrive vehicle;

    if(obs == 1)
    {
        vehicle.jerk = 1;
        vehicle.speed = speed;
        vehicle.steering_angle = steering_angle;
        vehicle.acceleration = acceleration;
    }

    else
    {
        vehicle.jerk = 0;
        vehicle.speed = speed;
        vehicle.acceleration = acceleration;
        vehicle.steering_angle = steering_angle;
    }

    pub.publish(vehicle);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Detecting");
    ros::NodeHandle nh;
    pub = nh.advertise<ackermann_msgs::AckermannDrive>("/vehicle/cmd_drive_safe",10);
    ros::Subscriber sub = nh.subscribe("/filtered_detector/jsk_bboxes",10,jsk_boxes_cb);
    ros::Subscriber sub1 = nh.subscribe("/vehicle/cmd_drive_nosafe",10,vehicle_cmd_drive_cb);
    ros::spin();
    return 0;
}
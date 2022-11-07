#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <typeinfo>
#include<vector>

using namespace std;
using namespace ros;

Publisher pub;
const float height_from_ground = 0.4;
void chatterCallback(const jsk_recognition_msgs::BoundingBoxArray& msg)
{
  jsk_recognition_msgs::BoundingBoxArray box_msg;
  int size = msg.boxes.size();
  vector<jsk_recognition_msgs::BoundingBox> box_list;

  for(int i=0;i<size;i++)
  {
    float z = msg.boxes[i].pose.position.z;

    float x = msg.boxes[i].pose.position.x;
    float y = msg.boxes[i].pose.position.y;
    float dist = sqrt(x*x + y*y + z*z);

    // if(dist < 2)
    // {
    //   // cout<<"less dist: "<<dist<<endl;
    // }
    // cout<<"dist: "<<dist<<endl;
    // cout<<"bb ptrint ejfh"<<endl;
    if(z > -height_from_ground && dist > 0.8)
    {
      box_list.push_back(msg.boxes[i]);
    }

    else
    {
       ;
    }
   
  }

    box_msg.boxes = box_list;
    // cout<<"size: "<<box_msg.boxes.size()<<endl;
    box_msg.header = msg.header;
    pub.publish(box_msg);
}

int main(int argc, char **argv)
{
  init(argc, argv, "filter");
  NodeHandle n;
  pub = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/filtered_detector/jsk_bboxes", 10);
  Subscriber sub = n.subscribe("/obstacle_detector/jsk_bboxes", 10, chatterCallback); 
  spin();

}

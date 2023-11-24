#include "autopilot_utils/tf_utils.h"

namespace autopilot_utils
{
    sensor_msgs::PointCloud2 transformCloud(sensor_msgs::PointCloud2 pc2_in, std::string target_frame)
    {
    if (pc2_in.header.frame_id == target_frame)
    {
        return pc2_in;
    }

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped transform;
    // transform = tf_buffer.lookupTransform(pc2_in.header.frame_id,target_frame, ros::Time(0));
 
    try{
         transform = tf_buffer.lookupTransform(target_frame, pc2_in.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        // ros::Duration(1.0).sleep();
        sensor_msgs::PointCloud2 empty_cloudl;
        return empty_cloudl;
    }
    // std::cout <<transform<<"\n";
    sensor_msgs::PointCloud2 pc2_out;
    tf2::doTransform(pc2_in, pc2_out, transform);
    pc2_out.header.frame_id = target_frame;
    // ROS_INFO("Transformed to target frame");

    // std::cout << transform << std::endl;
    return pc2_out;

}


}
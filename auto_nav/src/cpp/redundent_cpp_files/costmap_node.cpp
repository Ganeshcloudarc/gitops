#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "costmap_example_node");
    ros::NodeHandle nh;

    // Create a TransformListener
    // tf::TransformListener tf_listener;
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener(tf2_buffer);

    // Create a Costmap2DROS objectdd
    costmap_2d::Costmap2DROS costmap("/costmap_node/costmap/costmap", tf2_buffer);

    // Initialize the costmap
    costmap.start();

    // Access the underlying costmap
    costmap_2d::Costmap2D* costmap_ptr = costmap.getCostmap();

    // Perform operations on the costmap
    // For example, transform a point from the global frame to the robot frame
    tf::Stamped<tf::Point> global_point;
    global_point.setX(3.0);
    global_point.setY(2.0);
    global_point.setZ(0.0);
    global_point.frame_id_ = "map"; // Source frame
    ROS_INFO_STREAM(costmap.getGlobalFrameID());
    // tf::Stamped<tf::Point> robot_point;
    // try {
    //     tf2_listener.transformPoint(costmap.getGlobalFrameID(), global_point, robot_point);
    //     ROS_INFO("Transformed point from map frame to robot frame: (%.2f, %.2f)", robot_point.x(), robot_point.y());
    // } catch (tf::TransformException& ex) {
    //     ROS_ERROR("Error: %s", ex.what());
    // }

    // Spin to keep the node running
    ros::spin();

    return 0;
}

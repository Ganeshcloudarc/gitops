#include "autopilot_utils/tf_utils.h"

namespace autopilot_utils {
double get_yaw(geometry_msgs::Quaternion orientation) {
  /**
   * Calculates Yaw from quaternion(rotation wrt Z axis).
   * Parameters:
   *      orientation(geometry_msgs::Quaternion): Orientation in quaternion
   * Returns:
   *      float: Yaw in euler.
   */
  // Create a quaternion message
  tf::Quaternion quat(orientation.x, orientation.y, orientation.z,
                      orientation.w);

  // Create a matrix from the quaternion
  tf::Matrix3x3 m(quat);

  // Get the roll, pitch, and yaw from the matrix
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Return the yaw
  return yaw;
}

vector<vector<double>> bbox_to_corners(jsk_recognition_msgs::BoundingBox bbox) {
  /*
      Finds the corner points from jsk_recognition_msgs/BoundingBox.

      Args:
          bbox: jsk_recognition_msgs/BoundingBox object.

      Returns:
          Numpy array of four coordinates representing the corner points.
  */

  vector<vector<double>> corners_list;
  vector<vector<double>> trans_list = {{-1, -1}, {-1, 1}, {1, 1}, {1, -1}};

  // { -1, -1 }: This corresponds to the bottom-left corner of the unit square.
  // { -1, 1 } : This corresponds to the top-left corner of the unit square.
  // { 1, 1 }  : This corresponds to the top-right corner of the unit square.
  // { 1, -1 } : This corresponds to the bottom-right corner of the unit square.

  double yaw = get_yaw(bbox.pose.orientation);
  double c = cos(yaw);
  double s = sin(yaw);
  vector<vector<double>> R = {{c, -s}, {s, c}};

  for (auto trans : trans_list) {
    double x_ = trans[0];
    double y_ = trans[1];
    geometry_msgs::Point point_obj;
    point_obj.x = bbox.pose.position.x + x_ * bbox.dimensions.x;
    point_obj.y = bbox.pose.position.y + y_ * bbox.dimensions.y;

    /*
    Here, bbox.pose.position.x and bbox.pose.position.y represent the position
    of the center of the bounding box in the global coordinate frame. The x_ and
    y_ values come from the trans_list, representing the local coordinates of
    the corners of a unit square. By multiplying x_ and y_ with
    bbox.dimensions.x and bbox.dimensions.y, respectively, the code scales the
    local coordinates to match the actual dimensions of the bounding box. This
    scaling ensures that the corners of the unit square are correctly positioned
    relative to the center of the bounding box.

    */

    vector<double> result = {R[0][0] * point_obj.x + R[0][1] * point_obj.y,
                             R[1][0] * point_obj.x + R[1][1] * point_obj.y};
    corners_list.push_back(result);
  }
  return corners_list;
}

// ----------------------------- The Below Functions are not used--------------------------------------------------

sensor_msgs::PointCloud2 transformCloud(const sensor_msgs::PointCloud2 &pc2_in,
                                        const std::string &target_frame) {
  /*
      Transforms Point Cloud.

      Args:
          pc2_in: Point cloud you want to transform.
          target_frame: Target frame for converting the point cloud (map frame).

      Returns:
          Transformed point cloud in the 'target_frame'.
  */

  if (pc2_in.header.frame_id == target_frame) {
    return pc2_in;
  }

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::TransformStamped transform;

  try {
    // Lookup the transform between the frames
    transform =
        tf_buffer.lookupTransform(target_frame, pc2_in.header.frame_id, ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("The Transform Point Cloud Function Error is: %s", ex.what());
    sensor_msgs::PointCloud2 empty_cloud;
    return empty_cloud;
  }

  sensor_msgs::PointCloud2 pc2_out;
  tf2::doTransform(pc2_in, pc2_out, transform);
  pc2_out.header.frame_id = target_frame;
  return pc2_out;
}

// Function to convert point by transform
geometry_msgs::Point
convert_point_by_transform(const geometry_msgs::Point &point,
                           const geometry_msgs::TransformStamped &trans) {
  geometry_msgs::PointStamped pointStampedIn;
  pointStampedIn.point = point;

  geometry_msgs::PointStamped pointStampedOut;
  tf2::doTransform(pointStampedIn, pointStampedOut, trans);

  return pointStampedOut.point;
}

// Function overload for converting from list or tuple
geometry_msgs::Point
convert_point_by_transform(const std::vector<double> &point,
                           const geometry_msgs::TransformStamped &trans) {
  if (point.size() == 3) {
    geometry_msgs::PointStamped pointStampedIn;
    pointStampedIn.point.x = point[0];
    pointStampedIn.point.y = point[1];
    pointStampedIn.point.z = point[2];

    geometry_msgs::PointStamped pointStampedOut;
    tf2::doTransform(pointStampedIn, pointStampedOut, trans);

    return pointStampedOut.point;
  } else {
    // Handle the case when the input list or tuple does not have exactly three
    // elements You may throw an exception or handle it based on your
    // application logic For simplicity, let's return an empty point in this
    // case
    return geometry_msgs::Point();
  }
}

geometry_msgs::Pose
convert_pose_by_transform(const geometry_msgs::Pose &pose,
                          const geometry_msgs::Transform &trans) {
  // Create a pose stamped object
  geometry_msgs::PoseStamped pose_stamped_obj_in, pose_stamped_obj_out;

  // Set the pose stamped object's pose to the given pose
  pose_stamped_obj_in.pose = pose;

  // Create a transform stamped object
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.transform = trans;

  try {
    // Apply the transform to the pose stamped object and get the resulting pose
    tf2::doTransform(pose_stamped_obj_in, pose_stamped_obj_out,
                     transform_stamped);
  } catch (tf2::TransformException &ex) {
    return pose;
  }

  return pose_stamped_obj_out.pose;
}

zed_interfaces::ObjectsStamped
transform_zed_objects(zed_interfaces::ObjectsStamped object_data,
                      string to_frame) {
  /*
  Transforms ZED objects data to the target frame.

  Args:
      object_data: ZED interfaces ObjectsStamped object data to transform.
      to_frame: Frame to which you want to transform the data.

  Returns:
      Transformed object data in the zed_interfaces/ObjectsStamped format.
  */

  // If object_data is already in the target frame, return it as is
  if (object_data.header.frame_id == to_frame) {
    return object_data;
  }

  // Check if tfBuffer and listener are initialized
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Lookup the transform from object_data frame to target frame and transform
  // the object data
  try {
    geometry_msgs::TransformStamped trans = tf_buffer.lookupTransform(
        to_frame, object_data.header.frame_id, ros::Time(0));

    // Transform the position of each object
    for (int i = 0; i < object_data.objects.size(); i++) {
      // Convert boost::array<float, 3UL> to geometry_msgs::Point
      geometry_msgs::Point position;
      position.x = object_data.objects[i].position[0];
      position.y = object_data.objects[i].position[1];
      position.z = object_data.objects[i].position[2];

      // Update boost::array<float, 3UL> with the converted values
      auto converted_position = convert_point_by_transform(position, trans);
      object_data.objects[i].position = {
          static_cast<float>(converted_position.x),
          static_cast<float>(converted_position.y),
          static_cast<float>(converted_position.z)};
    }

    // Transform the corners of the bounding box of each object
    for (int i = 0; i < object_data.objects.size(); i++) {
      for (int j = 0; j < object_data.objects[i].bounding_box_3d.corners.size();
           j++) {
        // Convert std::vector<double> to geometry_msgs::Point
        geometry_msgs::Point newPoint;
        newPoint.x = object_data.objects[i].bounding_box_3d.corners[j].kp[0];
        newPoint.y = object_data.objects[i].bounding_box_3d.corners[j].kp[1];
        newPoint.z = object_data.objects[i].bounding_box_3d.corners[j].kp[2];

        // Update boost::array<float, 3UL> with the converted values
        auto convertedPoint = convert_point_by_transform(newPoint, trans);
        object_data.objects[i].bounding_box_3d.corners[j].kp = {
            static_cast<float>(convertedPoint.x),
            static_cast<float>(convertedPoint.y),
            static_cast<float>(convertedPoint.z)};
      }
    }

    // Update the frame id and timestamp of the object data
    object_data.header.frame_id = to_frame;
    object_data.header.stamp = ros::Time::now();
    return object_data;
  }

  // If transform lookup fails, return an empty object data
  catch (tf2::LookupException &e) {
    ROS_ERROR("In transform_zed_objects function failed to get transform "
              "from %s to %s: %s",
              to_frame.c_str(), object_data.header.frame_id.c_str(),e.what());
    return zed_interfaces::ObjectsStamped();
  }
  // Add more catch blocks if needed for other exceptions
  catch (std::exception &ex) {
    // Handle other standard exceptions
    ROS_ERROR("Exception at transform_zed_objects: %s", ex.what());
    return zed_interfaces::ObjectsStamped();
  } catch (...) {
    // Handle any other unknown exceptions
    ROS_ERROR("An unknown exception occurred at transform_zed_objects");
    return zed_interfaces::ObjectsStamped();
  }
}

jsk_recognition_msgs::BoundingBoxArray
transform_lidar_objects(jsk_recognition_msgs::BoundingBoxArray bbox_arr_data,
                        std::string to_frame) {
  /*
      Transforms jsk_recognition_msgs/BoundingBoxArray objects data to the
     target frame.

      Args:
          bbox_arr_data: jsk_recognition_msgs/BoundingBoxArray object data to
     transform. to_frame: Frame to which you want to transform the data.

      Returns:
          - Transformed object data (jsk_recognition_msgs/BoundingBoxArray) in
     the target frame.
          - Corners list (list of geometry_msgs/Point).
  */

  // Check if the frame_id is already the target frame
  if (bbox_arr_data.header.frame_id == to_frame) {
    return bbox_arr_data;
  }

  // Initialize tfBuffer and tfListener
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  try {
    // Lookup the transform from "to_frame" to "bbox_arr_data.header.frame_id"
    geometry_msgs::TransformStamped trans = tf_buffer.lookupTransform(
        to_frame, bbox_arr_data.header.frame_id, ros::Time(0));

    // Transform each pose in the bounding box array
    for (int i = 0; i < bbox_arr_data.boxes.size(); i++) {
      // Initialize PoseStamped variables
      geometry_msgs::PoseStamped pose_stamped_in, pose_stamped_out;

      pose_stamped_in.pose = bbox_arr_data.boxes[i].pose;
      pose_stamped_in.header = bbox_arr_data.boxes[i].header;

      // Apply the transform to the pose
      tf2::doTransform(pose_stamped_in, pose_stamped_out, trans);

      // Update the pose in the bounding box array
      bbox_arr_data.boxes[i].pose = pose_stamped_out.pose;

      // Update the frame ID of each bounding box
      bbox_arr_data.boxes[i].header.frame_id = to_frame;
    }

    // Update the frame ID and timestamp of the bounding box array
    bbox_arr_data.header.frame_id = to_frame;
    bbox_arr_data.header.stamp = ros::Time::now();

    // Return the transformed bounding box array
    return bbox_arr_data;
  } catch (tf2::TransformException &e) {
    // If the transform lookup fails, log an error and optionally propagate the
    // exception
    ROS_ERROR(
        "In transform_lidar_objects failed to get transform from %s to %s: %s",
        to_frame.c_str(), bbox_arr_data.header.frame_id.c_str(), e.what());
  }

  // Return an empty bounding box array if there's an error
  return jsk_recognition_msgs::BoundingBoxArray();
}



// Function to convert a quaternion to a list
vector<double> quaternion_to_list(geometry_msgs::Quaternion quaternion) {
  // Create an empty list to store the quaternion values
  vector<double> q;

  // Append the x, y, z, and w values of the quaternion to the list
  q.push_back(quaternion.x);
  q.push_back(quaternion.y);
  q.push_back(quaternion.z);
  q.push_back(quaternion.w);

  // Return the list
  return q;
}

// Function to convert a list to a quaternion
geometry_msgs::Quaternion list_to_quaternion(const vector<double> &l) {
  // Create a quaternion object
  geometry_msgs::Quaternion q;

  // Assign the x, y, z, and w values of the list to the quaternion object
  q.x = l[0];
  q.y = l[1];
  q.z = l[2];
  q.w = l[3];

  // Return the quaternion
  return q;
}

geometry_msgs::Pose convert_pose(const geometry_msgs::Pose &pose,
                                 const std::string &from_frame,
                                 const std::string &to_frame) {
  // Check if tfBuffer and listener are initialized
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Declare PoseStamped variable outside the try block
  geometry_msgs::PoseStamped spose;

  try {
    // Lookup the transform between the frames
    geometry_msgs::TransformStamped trans =
        tf_buffer.lookupTransform(to_frame, from_frame, ros::Time(0));

    // Set the values for the PoseStamped variable
    spose.pose = pose;
    spose.header.stamp = ros::Time::now();
    spose.header.frame_id = from_frame;

    // Transform the pose using tf2
    geometry_msgs::PoseStamped p2;
    tf2::doTransform(spose, p2, trans);

    // Return the transformed pose
    return p2.pose;

  } catch (tf2::TransformException &e) {
    ROS_ERROR(
        "In convert_pose Function failed to get transform from %s to %s: %s",
        to_frame.c_str(), from_frame.c_str(), e.what());
    // Handle the error in an appropriate way (e.g., rethrow or return a default
    // pose)
    return geometry_msgs::Pose();
  }
}

geometry_msgs::Point convert_point(geometry_msgs::Point point,
                                   string from_frame, string to_frame) {
  /*
      Args:
          point: List or geometry_msgs/Point; returns the same format.
          from_frame: Source frame.
          to_frame: Target frame.

      Returns:
          Transformed values in the same format as point.
  */

  // Check if tfBuffer and listener are initialized
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  try {
    geometry_msgs::TransformStamped trans = tf_buffer.lookupTransform(
        to_frame, from_frame, ros::Time::now(), ros::Duration(2.0));

    // Initialize PointStamped variables
    geometry_msgs::PointStamped p;
    p.point = point;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = from_frame;

    geometry_msgs::PointStamped p2;
    tf2::doTransform(p, p2, trans);

    return p2.point;
  } catch (tf2::TransformException &ex) {
    // Handle transform exception
    ROS_ERROR("TransformException - Point convert_point: %s", ex.what());
  } catch (...) {
    // Handle any other unknown exceptions
    ROS_ERROR("An unknown exception occurred in Point convert_point");
  }

  return geometry_msgs::Point();
}

geometry_msgs::Pose current_robot_pose(std::string reference_frame,
                                       std::string base_frame) {
  // Create Pose
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  // Transforms Robots Current Pose to the Base Reference Frame
  return convert_pose(pose, base_frame, reference_frame);
}

geometry_msgs::Pose
align_pose_orientation_to_frame(geometry_msgs::Pose from_pose,
                                std::string from_reference_frame,
                                std::string to_reference_frame) {
  geometry_msgs::Pose p;
  p.orientation.w = 1.0;
  geometry_msgs::Pose pose_orientation =
      convert_pose(p, to_reference_frame, from_reference_frame);
  from_pose.orientation = pose_orientation.orientation;
  return from_pose;
}

geometry_msgs::PoseArray convert_path(const geometry_msgs::PoseArray &path,
                                      const std::string &to_frame) {
  /*
      Args:
          path: List or geometry_msgs/poses; returns the same format.
          to_frame: Target frame.

      Returns:
          Transformed path in the 'to_frame'.
  */

  if (path.header.frame_id == to_frame) {
    return path;
  }

  // Check if tfBuffer and listener are initialized
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  try {
    // Lookup the transform
    geometry_msgs::TransformStamped trans =
        tf_buffer.lookupTransform(to_frame, path.header.frame_id, ros::Time(0));

    // Apply the transform to each pose in the path
    geometry_msgs::PoseStamped pose_stamped_in, pose_stamped_out;
    geometry_msgs::PoseArray transformed_path;
    transformed_path.header.frame_id = to_frame;

    for (int i = 0; i < path.poses.size(); i++) {
      pose_stamped_in.pose = path.poses[i];
      pose_stamped_in.header = path.header;

      tf2::doTransform(pose_stamped_in, pose_stamped_out, trans);
      transformed_path.poses.push_back(pose_stamped_out.pose);
    }

    // Return the transformed path
    return transformed_path;
  } catch (tf2::LookupException &ex) {
    // Handle lookup exception
    ROS_ERROR("LookupException - convert_path: %s", ex.what());
    // Return an empty pose array in case of an exception
    return geometry_msgs::PoseArray();
  } catch (tf2::TransformException &ex) {
    // Handle transform exception
    ROS_ERROR("TransformException - convert_path: %s", ex.what());
    // Return an empty pose array in case of an exception
    return geometry_msgs::PoseArray();
  } catch (std::exception &ex) {
    // Handle other standard exceptions
    ROS_ERROR("Exception - convert_path: %s", ex.what());
    // Return an empty pose array in case of an exception
    return geometry_msgs::PoseArray();
  } catch (...) {
    // Handle any other unknown exceptions
    ROS_ERROR("An unknown exception occurred convert_path");
    // Return an empty pose array in case of an exception
    return geometry_msgs::PoseArray();
  }
}

} // End of namespace autopilot_utils

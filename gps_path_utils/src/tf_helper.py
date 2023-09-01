# Helper Functions for interfacing with TF2
# Stolen from the ACRV 2017 Amazon Robotics Challenge
import rospy
import geometry_msgs.msg as gmsg
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

# Lazy create on use (convert_pose) to avoid errors.
tfBuffer = None
listener = None


def _init_tf():
    # Create buffer and listener
    # Something has changed in tf that means this must happen after init_node
    global tfBuffer, listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


def quaternion_to_list(quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def list_to_quaternion(l):
    q = gmsg.Quaternion()
    q.x = l[0]
    q.y = l[1]
    q.z = l[2]
    q.w = l[3]
    return q


def convert_pose(pose, from_frame, to_frame):
    """
    Convert a pose or transform between frames using tf.
        pose            -> A geometry_msgs.msg/Pose that defines the robots position and orientation in a reference_frame
        from_frame      -> A string that defines the original reference_frame of the robot
        to_frame        -> A string that defines the desired reference_frame of the robot to convert to
    """
    global tfBuffer, listener

    if tfBuffer is None or listener is None:
        _init_tf()

    try:
        trans = tfBuffer.lookup_transform(to_frame, from_frame, rospy.Time.now(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
        return None
    spose = gmsg.PoseStamped()
    spose.pose = pose
    spose.header.stamp = rospy.Time().now
    spose.header.frame_id = from_frame

    p2 = tf2_geometry_msgs.do_transform_pose(spose, trans)

    return p2.pose


def convert_point(point, from_frame, to_frame):
    """
    :param point: list or geometry_msgs/Point returns the same format
    :param from_frame:
    :param to_frame:
    :return: transformed values same format as point
    """
    global tfBuffer, listener

    if tfBuffer is None or listener is None:
        _init_tf()
    try:
        trans = tfBuffer.lookup_transform(to_frame, from_frame, rospy.Time.now(), rospy.Duration(2.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
        rospy.logerr(str(e))
        return None
    # final_point = gmsg.Point()
    if type(point) == type(gmsg.Point()):
        p2 = tf2_geometry_msgs.do_transform_point(gmsg.PointStamped(point=point), trans).point
        return p2
    elif (type(point) == type([]) or type(point) == type(())) and len(point) == 3:
        point = gmsg.Point(point[0], point[1], point[2])
        p2 = tf2_geometry_msgs.do_transform_point(gmsg.PointStamped(point=point), trans).point
        return [p2.x, p2.y, p2.z]


def current_robot_pose(reference_frame="map", base_frame="base_link"):
    """
    Get the current pose of the robot in the given reference frame
        reference_frame         -> A string that defines the reference_frame that the robots current pose will be defined in
    """
    # Create Pose
    p = gmsg.Pose()
    p.orientation.w = 1.0

    # Transforms robots current pose to the base reference frame
    return convert_pose(p, base_frame, reference_frame)


def publish_stamped_transform(stamped_transform, secpointonds=1):
    """
    Publish a stamped transform for debugging purposes.
        stamped_transform       -> A geometry_msgs/TransformStamped to be published
        seconds                 -> An int32 that defines the duration the transform will be broadcast for
    """
    # Create broadcast node
    br = tf2_ros.TransformBroadcaster()

    # Publish once first.
    stamped_transform.header.stamp = rospy.Time.now()
    br.sendTransform(stamped_transform)

    # Publish transform for set time.
    i = 0
    iterations = seconds / 0.05
    while not rospy.is_shutdown() and i < iterations:
        stamped_transform.header.stamp = rospy.Time.now()
        br.sendTransform(stamped_transform)
        rospy.sleep(0.05)
        i += 1


def publish_transform(transform, reference_frame, name, seconds=1):
    """
    Publish a Transform for debugging purposes.
        transform           -> A geometry_msgs/Transform to be published
        reference_frame     -> A string defining the reference frame of the transform
        seconds             -> An int32 that defines the duration the transform will be broadcast for
    """
    # Create a stamped_transform and store the transform in it
    st = gmsg.TransformStamped()
    st.transform = transform
    st.header.frame_id = reference_frame
    st.child_frame_id = name

    # Call the publish_stamped_transform function
    publish_stamped_transform(st, seconds)


def publish_pose_as_transform(pose, reference_frame, name, seconds=1):
    """
    Publish a pose as a transform so that it is visualised in rviz.
    pose                -> A geometry_msgs.msg/Pose to be converted into a transform and published
    reference_frame     -> A string defining the reference_frame of the pose
    name                -> A string defining the child frame of the transform
    seconds             -> An int32 that defines the duration the transform will be broadcast for
    """

    # Create a broadcast node and a stamped transform to broadcast
    t = gmsg.TransformStamped()

    # Prepare broadcast message
    t.header.frame_id = reference_frame
    t.child_frame_id = name

    # Copy in pose values to transform
    t.transform.translation = pose.position
    t.transform.rotation = pose.orientation

    # Call the publish_stamped_transform function
    publish_stamped_transform(t, seconds)


def publish_tf_quaterion_as_transform(translation, quaternion, reference_frame, name, seconds=1):
    qm = gmsg.Transform()
    qm.translation.x = translation[0]
    qm.translation.y = translation[1]
    qm.translation.z = translation[2]
    qm.rotation.x = quaternion[0]
    qm.rotation.y = quaternion[1]
    qm.rotation.z = quaternion[2]
    qm.rotation.w = quaternion[3]
    publish_transform(qm, reference_frame, name, seconds)


def align_pose_orientation_to_frame(from_pose, from_reference_frame, to_reference_frame):
    """
    Update the orientation of from_pose so that it matches the orientation of to_reference_frame
    Useful for aligning a desired position pose with a gripper, for example.
        from_pose                   -> A geometry_msgs.msg/Pose to allign
        from_reference_frame        -> A string defining the reference_frame of the pose
        to_reference_frame          -> A string defining the reference_frame to allign to
    """
    # Create transform
    p = gmsg.Pose()
    p.orientation.w = 1.0

    # Convert reference frame orientation from -> to
    pose_orientation = convert_pose(p, to_reference_frame, from_reference_frame)

    # Copy orientation to pose.
    from_pose.orientation = pose_orientation.orientation

    return from_pose


def get_yaw(orientation):
    _, _, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    return yaw


def transform_cloud(cloud, from_frame, to_frame):
    """
    Transforms Point cloud
    :param cloud: Pointcloud you want to transform
    :param from_frame: from which frame you want to convert (sensor frame/ base frame )
    :param to_frame: to which frame you want to convert the point cloud (map frame )
    :return: transformed point cloud in to_frame
    """
    global tfBuffer, listener
    if tfBuffer is None or listener is None:
        _init_tf()
    try:
        trans = tfBuffer.lookup_transform(to_frame, from_frame, rospy.Time.now(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
        return None
    cloud_out = do_transform_cloud(cloud, trans)
    return cloud_out






# def add_poses(pose1, pose2):
#
#     tf =gmsg.TransformStamped()
#     tf.transform.translation = gmsg.Vector3(pose1.position.x, pose1.position.y, pose1.position.z)
#     tf.transform.rotation = gmsg.Quaternion(pose1.orinetati.x, pose1.position.y, pose1.position.z)


# if __name__ == "__main__":
#     # initialize ros node
#     rospy.init_node('tf2_ros_example', anonymous=True)
#
#     # define source and target frame
#     source_frame = 'base_link'
#     target_frame = 'map'
#
#     point_wrt_source = gmsg.Point(34, 2, 3)
#     p = [34, 2, 3]

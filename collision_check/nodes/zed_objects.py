# FOR CAMERA
'''
for i in range(self.close_index, len(self.revived_path)):
    for ob in self.modified_obj_data.objects:
        dis = min_distance_to_object(self.revived_path[i], ob.corners)
        if dis <= self.obstacle_threshold_radius:
            rospy.loginfo("object detected at index", i)
            distance_to_vehicle = distance_to_robot(self.revived_path[i])
            self.in_collision_marker_at_index(i)
            rospy.loginfo("distance_to_vehicle", distance_to_vehicle)
        else:
            self.velocity_smoother(i)
            self.no_collision_markers_at_index(i)

    if distance_to_robot(self.revived_path[i]) > self.forward_collision_check_dis:
        break
'''

## FOR COSTMAP
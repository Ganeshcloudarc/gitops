Obstalce stop planner {#Obstalce stop planner}
=============

# Purpose / Use cases

Boson autopilot reqires a basic collision checking mechanism on GPS path. Collision checking on the path is important so that vehicle not hit with obstacles. 

Obstacle stop planner takes a reference trajectory, current vehicle position and a laser scan. the main output is local_trajectory with its Longitudinal_vel_mps assigned to zeros if there exits a collision.

This obstacle stop planner is a basic collision checking method inspired from [Tier4](https://tier4.github.io/autoware.iv/tree/main/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/). it will check the possible collision points with laser scan data on the global_trajectoty from vehicle position.

# Design

The process of obstacle stop planner algoritham is described below.

1. subscribe to the global_trajectory and laser scan.
2. find the closest point to the vehicle pose on the global_trajectory.
3. for pose_xyz in [close_point to lookforward_dis_point](10 meters ahead from the close point):
   - convert the input laser scan to KDTree.
   - radius_to_search = vehicle_width + off_set_dis
   - KDTree.query_radius(pose_xyz, r=radius_to_search) , this method returns the indices of laser points which in range of  radius_to_search to a pose_xyz(circular range).
   - if the above method returns any valid indices, it means that we will have a collision at that particular close_point point,  so we set the longituninal_velocity_mps to zero to its previous points till stop_margin meters back the pose_xyz(stop_margin = 3 meters).
   - update the local_trajectory.
4. then will publish this local_trajectory(starts from vehicle's close point on the )

This local_trajectory subscibed by pure pursuit motion controller, which controls the vehicle to follow local_trajectory.



## Updated things from the base implemtation/paper

- Sensor source is Laser scan not Pointcloud2.


## Assumptions / Known limits

- No velocity coontrol smoothing.
- No slow_down_margin, we are not slow downing the vehicle, just applying break it there exist a obatacle in stop_margin.

The following limitations are present:

- This implementation assumes that the path tracking is working accurately, because we are checking a small region arond the path (search radius). if the tracking is bad, cross track error might be more than this search radius, it might cause collision.

## Inputs / Outputs / API

Inputs:
- `autopilot_msgs/Trajectory.msg` is a sequence of trajectory points with the 2D position and pose and velocities from global velocity profiler.
- `TF` tranform between map to robot_base_frame.
- `LaserScan` Laser scan data 
 
Outputs:
- `autopilot_msgs/Trajectory.msg` sequence of trajectory points starting from  vehicle pose and longituninal_velocity_mps is updated with with collision occurance.


## Configuration state

The following defines the configuration state:
- `stop_margin` (float, meters): margin at which vehicle should stop, if there exists a obstacle. if a obstalce lies in this region from the vehicle, the vehicle shoud stop.
- `slow_down_margin` (float, meters) : if obstacle lies in this region vehicle should slow down. (not implimented)
- `radial_off_set_to_vehicle_width` (float, meters): search radius offset to vehicle width. search_radius = vehicle_width + radial_off_set_to_vehicle_width.

- `trajectory_resolution` (float, meters): the minimun distance between the two consicutive trajectory points
- `lookup_collision_distance` (float, meters): how far to we need to check for collision from the close trajectory point to vehicle.
- `traj_in`(string, topic_name): topic name of global trajectory, which is input to obstacle stop planner
- `scan_in` (string, topic_name): topic name of laser scan data
- `traj_out` (string, topic_name): topic name of local trajectory, which is output if obstacle stop planner.
- `max_slow_down_velocity` (float, meter/sec): maximum speed to apply in slow_down_margin.
- `min_slow_down_velocity` (float, meter/sec): minimum speed to appluy in slow_down_margin


## Performance characterization

### Time


- NA

### Space

- NA

# Security considerations

TBD by a security specialist.


# References / External links

- [Original implimentation concept](https://tier4.github.io/autoware.iv/tree/main/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/)
- [Original github of implimentation](https://github.com/tier4/obstacle_stop_planner_refine)
- [autoware trajectory point msg](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_planning_msgs/msg/TrajectoryPoint.idl)
- [autoware trajectory msg](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_planning_msgs/msg/Trajectory.idl)



# Future extensions / Unimplemented parts
- veliocity smoothing.
- slow_down margin .
- support multi sensor input, example zed_camera, 2D lidar, 3D lidar.
- CPP version of obstacle stop planner.
- improved visulizations, margin circle.




# Related issues

- [57](https://github.com/bosonrobotics/autopilot_boson/issues/57) ControllerDiagnose message for pure pursuit controller.
- [56](https://github.com/bosonrobotics/autopilot_boson/issues/56) Trajectory msg for patrolling. 
- [52](https://github.com/bosonrobotics/autopilot_boson/issues/52)  Collision check with zed laser scaner.
- [33](https://github.com/bosonrobotics/autopilot_boson/issues/33) Obstacle stop planner. 


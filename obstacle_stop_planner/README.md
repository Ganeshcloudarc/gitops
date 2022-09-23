# Obstacle stop planner
This is implementation of obstacle stop planner inspired from [Tier4](https://tier4.github.io/autoware.iv/tree/main/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/)

This package helps the vehicle to stop if there exists a obstacle in its GPS path.

to know more implemtation level details refer [obstalce_stop_planner-design](design/obstalce_stop_planner-design.md)

## Installation

catkin_make in catkin_workspace would install this package.

## Running
### with carla simulator
`roslaunch obstacle_stop_planner obstacle_stop_planner.launch carla:=true`
### on real vehicle
`roslaunch obstacle_stop_planner obstacle_stop_planner.launch`


## Parameters

- `stop_margin` (float, default: 7 meters): margin at which vehicle should stop, if there exists a obstacle. if a obstalce lies in this region from the vehicle, the vehicle shoud stop.
- `slow_down_margin` (float, default: 10 meters) : if obstacle lies in this region vehicle should slow down. (not implimented)
- `radial_off_set_to_vehicle_width` (float, default: 0.3 meters): search radius offset to vehicle width. search_radius = vehicle_width + radial_off_set_to_vehicle_width.

- `trajectory_resolution` (float, default: 0.5 meters): the minimun distance between the two consicutive trajectory points
- `lookup_collision_distance` (float, default: 20 meters): how far to we need to check for collision from the close trajectory point to vehicle.
- `traj_in`(string, default: global_gps_trajectory): topic name of global trajectory, which is input to obstacle stop planner
- `scan_in` (string, default: zed/laser_scan): topic name of laser scan data
- `traj_out` (string, default: local_gps_trajectory): topic name of local trajectory, which is output if obstacle stop planner.
- `max_slow_down_velocity` (float, default: 1.5 meter/sec): maximum speed to apply in slow_down_margin.
- `min_slow_down_velocity` (float, default : 0.8meter/sec): minimum speed to appluy in slow_down_margin

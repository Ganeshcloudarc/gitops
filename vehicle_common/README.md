# vehicle_common package
This package containis Vehicle description config file, URDF file, map to base_link tf broadcaster, Python api to access vehicle related data(dimentions and sensor locations).
##  Vehicle config file
This a YAML file, having information about vehicle dimentions, sensor related information(sensor position and frame).

vehicle dimentions ref:
![](images/vehicle_params_naming.png)
- You can chage the [YAML](params/vehicle_config.yaml) file fields under `dimentions` depending upon the vehicle dimentions following above vehicle naming convention.
-  sensor info and locations from front_wheel axle center need tp filled under `sensors` in the [YAML](params/vehicle_config.yaml) file.

<details>
<summary>View vehicle_config.yaml</summary>

```
dimensions: # for yellow porter ref https://trucks.cardekho.com/en/trucks/piaggio/porter-700/specifications
  overall_length    : 3.544 # all the mesurments in MKS(meters, kgs, seconds)
  overall_width     : 1.460
  overall_height     : 1.75
  wheel_base        : 1.82
  track_width       : 1.40
  front_overhang    : 1.0
  rear_overhang     : 0.7
  ground_clearance  : 0.2
  payload           : 750 # kgs
  tyre_radius       : 0.3
  tyre_section_width :  0.145

speed:
  max_forward_speed      : 3  # m/s
  min_forward_speed      : 0.3 # m/s
  max_backward_speed : -3
  min_backward_speed : -0.3
  max_acceleration : 0.5
  min_acceleration : -0.5
  max_steering_angle : 30 # degrees
  min_steering_angle : -30 # degrees

## center of gravity also a point
COG_position:
  - 0.4
  - 1
  - 0.5   # [X, Y, Z] distances from rear wheel axle center

robot_origin_frame: "base_link" # name of link at rear wheel

sensors: 
  # sensor info and locations from front_wheel axle center
  fcu:
    model: "pixhawk4"
    sensor_frame: "fcu_link"
    position :
      - 0.2
      - 0
      - 0    # [X, Y, Z] distances from front wheel axle center to fcu
    orientation:
      - 0
      - 0
      - 0 # [row, pitch, yaw] of fcu in radians wrt front wheel axle center

  zed_camera:
    model : "zed2i"
    sensor_frame : "zed2i_base_link"
    position:
        - 0.4
        - 0
        - 0.4    # [X, Y, Z] distances from front wheel axle center to camera_center
    orientation:
        - 0
        - 0
        - 0 # [row, pitch, yaw] of zed camera in radians wrt front wheel axle center

  lidar:
    model: "rs_lidar"
    sensor_frame : "rs_lidar"
    position:
      - 0.4
      - 0
      - 0.2    # [X, Y, Z] distances from front wheel axle center to lidar
    orientation:
      - 0
      - 0
      - 0 # [row, pitch, yaw] of lidar in radians wrt front wheel axle center

```
</details>


## URDF file
ROS version of desciption of vehicle is called URDF(universal robot desciption file).

- All the sensor locations data and their tf frames and locations are kept in this file. which is very usefull when transforming frames from one another.

## map to base_link tf broadcaster
A node subscribes to odometry from mavros, publisher a proper tf (base_link as rear_axle of vehicle), Odometry (with child_frame base_link (rear_axle center)), and a Footprint of vehicle.

### Parameters:

* `base_frame` (string, default: base_link) - base frame of vehicle
* `odometry_in`(string, default: /mavros/local_position/odom) -  odometry topic name from mavros 
* `send_odom` (boolean, default: true) -true to publish odometry 
* `send_footprint`(boolean, default: true) - to publish vehicle footprint
* `odometry_out`(string, default: /vehicle/odom) - name of odometry topic with child frame as base_link(rear_axle)
* `foorprint_out`(string, default: /vehicle/foot_print) - name of footprint topic which is of vehicle size.

## Python API for vehicle info
You can able to access all the feilds in [YAML](params/vehicle_config.yaml) with DOT operator.

### Example

```
from vehicle_common.vehicle_config import vehicle_data
print(vehicle_data.dimensions.wheel_base)
print(vehicle_data.sensors.fcu.model)
```

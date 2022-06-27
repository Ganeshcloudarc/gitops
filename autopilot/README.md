installation:
```
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
sudo apt install ros-noetic-rosbridge-server
sudo apt install ros-noetic-rviz-satellite*
pip install geojson
pip install rospy-message-converter
pip install numpy
```


## Launch mavros node
```
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600 
change it according to port
```


## Save path
```
roslaunch autopilot_boson save_path.launch mission_file:=name_of_file.json
```
#### Arguments
* `mission_file` (str, default: default.json) - name of json file you want to save to.
* `min_dis` (float, default: 0.1) - minimum distance between the two consecutive gps points

## Launch patrolling
```
roslaunch autopilot_boson patrol.launch mission_file:=name_of_file.json
roslaunch pilot all_pilot.launch
```
#### Arguments
* `mission_file`  (string, default: default.json) - mission file name (should be present in mission_files directory)
* `continue` (boolean, default: false)  - continue to reach first after reaching last point in a loop.
* `mission_trips` (Integer, default: 0) - Integer - number of times you want to continue mission (forward direction), zero to keep forever
* `zed_od_enable`   (boolean, default: false) - true to start zed camera object detection and stop the vehicle if any object in range.
* `gps_failsafe` (boolean, default: false) - true, to stop vehicle when lost GPS RTK.
False, to warn and vehicle continues to move even without RTK.
* `ctc_failsafe` (boolean, default: false) - true -> to stop vehicle when cross-track-error is more than a threshold (3m)
* `rc_control`    (boolean, default: false) - true to enable RC control (both automode and manual mode and stop).
* `rviz`         (boolean, default: true) - to open rviz.
* `record`        (boolean, default: false) - to record bag file.
* `carla`          (boolean, default: false) - to run the entire patrol on carla simulator.

#### Parameters
* `max_speed` (float, default: 1.5) - The max speed vehicle can go in patrol mode in meters/sec
* `wheel_base` (float, default: 2) - distance between front and rear wheels in meters.
* `look_ahead_distance` (float, default: 3) - The minimum look ahead distance.
* `speed_gain` (float, default: 0.1) - speed gain for pure pursuit
* `speed_curvature_gain` (float, default: 0.1) - curvature gain used to calculate speed at given point on the path.
* `path_topic` (str, default: "odom_path") - we have two paths which we can follow 'odom_path' and 'gps_path'. 
* `wait_time_at_ends` (int, default: 5) - once reached end, how much time to wait in seconds.
* `map/to_center` (boolean, true) - in Google Maps center vehicle always

#### example launch commands
```
1 - to just launch
roslaunch autopilot_boson patrol.launch mission_file:=name_of_file.json
2- to enable rc control
roslaunch autopilot_boson patrol.launch mission_file:=name_of_file.json rc_control:=true
```

## Visualize live tracking of vehicle on Google Maps.
```
roslaunch rosbridge_server rosbridge_websocket.launch
roscd autopilot_boson
python3 -m http.server
http://0.0.0.0:8000/ (open it on web browser)
Then navigate as below 
boson_console_ui->boson_map.html (open it)
```

TODO
* velocity and curvature based speed control
* move reverse direction( backward tracking )
* 
* indepth code review by Dinesh

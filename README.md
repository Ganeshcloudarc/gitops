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
* `min_dis` (float, default: 0.1) - minimum distance between the two consecutive gps points

## Launch patrolling
```
roslaunch autopilot_boson patrol.launch mission_file:=name_of_file.json
roslaunch pilot all_pilot.launch
```
#### Arguments
* `mission_file`  (str, default: default.json) - mission file name (should be present in mission_files directory)
* `mission_mode`  (int, default: 0) - mission mode 0,1 or 2. mode type [0, 1, 2]. **0**-> go and stop
                                                                   **1**-> go and return to the path starting point
                                                                   **2-**> Keep on doing (go and return) .
* `mission_trips` (int , default: 0) - if the mission_mode == 2 then how meny times you want to loop the path.
* `od_enable`     (boolean, default: false) - true when obstacle detection node running.
* `from_start`    (boolean, default: false) - true if you want to start patrol from first recorded path, else it will start to follow the path from its present location.
* `rc_control`    (boolean, default: false) - true to enable RC control (both automode and manual mode).
* `rviz`         (boolean, default: true) -- to open rviz.


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
2 - launch with mission_mode 1 (go to B and come back to A)
roslaunch autopilot_boson patrol.launch mission_file:=name_of_file.json mission_mode:=1
3 - launch with mission_mode 1 (go to B and come back to A and repeat for ever)
roslaunch autopilot_boson patrol.launch mission_file:=name_of_file.json mission_mode:=2
4 - launch with mission_mode 1 (go to B and come back to A and repeat for given times)
roslaunch autopilot_boson patrol.launch mission_file:=name_of_file.json mission_mode:=2 mission_trips:=3
5 - launch with rc_control enable
roslaunch autopilot_boson patrol.launch mission_file:=name_of_file.json mission_mode:=2 mission_trips:=3 rc_control:=true
6 - to disable rviz
roslaunch autopilot_boson patrol.launch mission_file:=name_of_file.json rviz:=false
7 - to enable to obstacle detection
roslaunch autopilot_boson patrol.launch mission_file:=name_of_file.json ob_enable:=true
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

installation:
* sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
* wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
* sudo bash ./install_geographiclib_datasets.sh
* sudo apt install ros-noetic-rosbridge-server
* pip install geojson
* pip install rospy-message-converter



launch mavros 
```
roslaunch mavros px4.launch
```


to save path
```
roslaunch autopilot_boson save_path.launch mission_file:=name_of_file.json
```

to launch patrolling
```
roslaunch autopilot_boson patrol.launch mission_file:=name_of_file.json
roslaunch pilot all_pilot.launch
```

to visualize live tracking of vehicle on google maps.
```
roslaunch rosbridge_server rosbridge_websocket.launch
roscd autopilot_boson
python3 -m http.server
http://0.0.0.0:8000/ (open it on web browser)
Then navigate as below 
boson_console_ui->boson_map.html (open it)
```
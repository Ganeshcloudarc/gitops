## GPS path utils
- Here we will have few methods and implementations of gps path smoothing techiniques.



## KML TO Mission_file converter
- This node is responsible for convering KML file to mission_file (geojson) format, so that the it can be used for our patroling applications.

### Node params
`kml_file` : (string, default.kml) name of kml file as an input at gps_path_utils/kml_files directory

`linear_sampling_dis` : (float, 10) linear sampling distance, incresing this value increses smoothess.

`rviz` : (bool, false) 

### steps 

1. save Line string type kml file to autopilot_boson/gps_path_utils/kml_files directory.



### Run
- with default kml file

``` roslaunch gps_path_utils kml_to_mission_converter.launch```

- with another kml file

``` roslaunch gps_path_utils kml_to_mission_converter.launch kml_file:=kml_file.kml```

### Note
- The produced mission file (json file) saved in autopilot's mission_files directory with same name as kml file.
- You can use the above mission file for patrol, by setting mission_file arg in patrol.launch file


### known issue
- the provided method gives you smoother version of gps path, but cannot guarenties the travesrabliity of vehicle.


byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3
byte level
string name
time stamp
string message
float64 look_ahead # meters
float64 cte # meters
float64 longitudinal_velocity_mps
float64 steering_angle # degrees
float64 lateral_velocity_dps # degrees per second
float64 acceleration_mps2
geometry_msgs/Pose target_pose
geographic_msgs/GeoPose target_gps_pose
geometry_msgs/Pose vehicle_pose
sensor_msgs/NavSatFix vehicle_gps_pose
FloatKeyValue[] values


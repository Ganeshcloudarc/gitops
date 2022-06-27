byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3
byte level
string name
string message
float64 look_ahead # meters
float64 cte # meters
float64 speed # in m/s
float64 steering_angle # degrees
float64 linear_acceleration
float64 angular_acceleration
float64 current_point_heading # degrees
float64 target_point_heading # degrees
float64 current_point_curvature
float64 target_point_curvature
FloatKeyValue[] values


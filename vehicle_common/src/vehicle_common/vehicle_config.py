import yaml
import rospy
import rospkg
import sys
from dotmap import DotMap

try:
    ros_pack = rospkg.RosPack()
    config_file = ros_pack.get_path('vehicle_common') + "/params/vehicle_config.yaml"
    rospy.loginfo("config_file", config_file)
    with open(config_file) as f:
        vehicle_d = yaml.safe_load(f)
        vehicle_data = DotMap(vehicle_d)
except Exception as e:
    rospy.logerr("Please source 'vehicle_common' package" + str(e))
    sys.exit()

if __name__ == "__main__":
    print(vehicle_data.dimentions)

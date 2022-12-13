import yaml
import rospy
import rospkg
import sys
from dotmap import DotMap

try:
    ros_pack = rospkg.RosPack()
    try:
        vehice_type = rospy.get_param("vehicle_type", "porter")
    except:
        vehice_type = "porter"

    config_file = ros_pack.get_path('vehicle_common') + "/params/" + str(vehice_type) + "_config.yaml"
    rospy.loginfo("config_file", config_file)
    try:
        with open(config_file) as f:
            vehicle_d = yaml.safe_load(f)
    except:
        rospy.logerr(f"could not find {vehice_type}_config.file")
        sys.exit(f"could not find {vehice_type}_config.file")
except Exception as e:
    rospy.logerr("Please source 'vehicle_common' package" + str(e))
    sys.exit()

vehicle_data = DotMap(vehicle_d)

if __name__ == "__main__":
    print(vehicle_data.dimensions.wheel_base)

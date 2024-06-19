#include "vehicle_common/config_vehicle.h"

DataVehicle::DataVehicle() {
    ros::NodeHandle nh;
    nh.param("dimensions/overall_length", overall_length, 3.544);
    nh.param("dimensions/overall_width", overall_width, 1.460);
    nh.param("dimensions/overall_height", overall_height, 1.75);
    nh.param("dimensions/wheel_base", wheel_base, 1.82);
    nh.param("dimensions/track_width", track_width, 1.40);
    nh.param("dimensions/front_overhang", front_overhang, 1.0);
    nh.param("dimensions/rear_overhang", rear_overhang, 0.7);
    nh.param("dimensions/ground_clearance", ground_clearance, 0.2);
    nh.param("dimensions/tyre_radius", tyre_radius, 0.3);
    nh.param("dimensions/tyre_section_width", tyre_section_width, 0.145);
    nh.param("dimensions/gear_ratio", gear_ratio, 26.67);
}

int main() {
    DataVehicle vehicle_data_obj;
    return 0;
}
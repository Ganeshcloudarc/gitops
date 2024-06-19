#ifndef CONFIG_VEHICLE_H
#define CONFIG_VEHICLE_H

// Basic Imports
#include <iostream>

// Ros Related Imports
#include <ros/ros.h>

class DataVehicle{
public:
    double overall_length;
    double overall_width;
    double overall_height;
    double wheel_base;
    double track_width;
    double front_overhang;
    double rear_overhang;
    double ground_clearance;
    double tyre_radius;
    double tyre_section_width;
    double gear_ratio;

    DataVehicle();

};

#endif
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
#include <stdbool.h>
#include <time.h>

#include <boost/assign/std/vector.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/geometries/adapted/boost_polygon/point.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/dsv/write.hpp>
#include <boost/geometry/io/io.hpp>
#include <chrono>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <list>

#include "autopilot_msgs/ControllerDiagnose.h"
#include "mavros_msgs/GPSRAW.h"
#include "pilot_msgs/VehicleInfo.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"  //type /
// #include <Eigen/Dense>

#include <XmlRpcValue.h>

#include <typeinfo>
namespace bg = boost::geometry;

using namespace boost::geometry;
using boost::geometry::append;
using boost::geometry::correct;
using boost::geometry::make;
using namespace boost::assign;
using namespace std;
using namespace Eigen;

int gps_fix_type;

// Below global variables accessing in class methods maybe a bad practice. fix
// this. @iam-vishnu constexpr int OK = diagnostic_msgs::DiagnosticStatus::OK;
// constexpr int ERROR = diagnostic_msgs::DiagnosticStatus::ERROR;
// constexpr int WARN = diagnostic_msgs::DiagnosticStatus::WARN;
// constexpr int STALE = diagnostic_msgs::DiagnosticStatus::STALE;

// ToDo @ Vishnu

/*
- Show thresholds for GPS(DONE) and CTE(DONE)
- Dynamic GPS Threshold as python version (DONE)
- verify heading angle calculation - useful for debugging (DONE)
- dynamic cte at turnings.(DONE)
- Fix parameters such that, no long lengthy names will be used.
- add battery safety
- add steering safety

*/

class VehicleSafety {
  ros::NodeHandle nh;
  static VehicleSafety self;

 private:
  int counter, gps_curr_lat, gps_curr_long;
  int prev_compass;
  int curr_compass;
  int error_counter_for_tracking_controller{0};
  int error_counter_for_tracking_controller_th{2};
  int CTE_THR;
  // Todo : Keep default values if param not available
  int battery_soc{-1};
  int store_batt_level{-1};
  int motor_rpm{0};  // motor rpm will be zero when no can data
  int demand_steering_angle{0};
  int current_steering_angle{0};
  bool is_steering_stuck;
  int BATT_SOC_TH = nh.param("/vehicle_safety/BATT_TH", BATT_SOC_TH);
  int CTE_THR_AT_STRAIGHT =
      nh.param("/vehicle_safety/CTE_THR", CTE_THR_AT_STRAIGHT);
  int CTE_THR_AT_CURVE =
      nh.param("/vehicle_safety/CTE_THR_AT_CURVE", CTE_THR_AT_CURVE);
  bool emergency_stop = false;
  int TRACKING_CONTROLLER_TIMEOUT = 1;
  int GPS_FIX_CB_TIMEOUT = 1; //timout in sec to check gps callback
  float prev_coordinates_lat, prev_coordinates_long;
  float curr_coordinates_lat = 0, curr_coordinates_long = 0;
  double gps_accuracy = -1, sensor_accuracy_value, GPS_ACC_IDEAL, GPS_ACC_THR;
  int calc_heading = 0;
  int gps1_fix, gps2_fix;
  float position_covariance;
  long start_time_gps, start_time;
  long gps_lost_time, gps_start_time;
  std::time_t steering_error_start_time{0};
  int gps_lost_count = 0;

  int OK = diagnostic_msgs::DiagnosticStatus::OK;
  int ERROR = diagnostic_msgs::DiagnosticStatus::ERROR;
  int WARN = diagnostic_msgs::DiagnosticStatus::WARN;
  int STALE = diagnostic_msgs::DiagnosticStatus::STALE;

  float HEAD_THR = nh.param("/vehicle_safety/HEAD_THR", HEAD_THR);
  float GPS_FIX_THR = nh.param("/vehicle_safety/GPS_FIX_THR", GPS_FIX_THR);
  bool use_geo_fence = nh.param("/vehicle_safety/use_geo_fence", use_geo_fence);
  bool use_inner_geo_fence =
      nh.param("/vehicle_safety/use_inner_geo_fence", use_inner_geo_fence);
  float steering_diff_th = nh.param("/vehicle_safety/STEER_DIFF_TH", steering_diff_th);
  float steering_stuck_time_th = nh.param("/vehicle_safety/STEER_STUCK_TIME_TH", steering_stuck_time_th);
  bool is_inside_geo_fence;
  bool is_with_in_no_go_zone;
  double time_to_launch;
  ros::Time time_on_tracking_cb;
  ros::Time time_on_gps_fix_cb;

  int mavros_head;
  vector<int> ignore_heading = {0, 45, 90, 135, 180, 225, 270, 315, 360};

  autopilot_msgs::ControllerDiagnose pp_diagnose_data;
  pilot_msgs::VehicleInfo can_data;
  int flag1 = 0, reset_gps = 0, reset_head = 0, reset_geo_fence = 0;

  Matrix<double, 3, 3> m_prev;

  boost::array<double, 9> cov_value;
  ros::Publisher gps_diagnostic_publisher, sensor_accuracy_publisher;
  ros::Subscriber number_subscriber, emergency_stop_subscriber, gps_subscriber,
      heading_failsafe_subscriber, global_pos_subscriber,
      control_diag_subscriber, sensor_msg_subscriber, is_curve_sub,
      can_data_sub;
  ros::ServiceServer reset_service;
  vector<int> co_ordinates;
  vector<int> li;
  XmlRpc::XmlRpcValue no_go_zone_coords_xmlrpc;
  std::vector<std::vector<std::vector<double>>> no_go_zone_coordinates;

 public:
  // int vs;

  VehicleSafety() {
    number_subscriber = nh.subscribe("/mavros/global_position/raw/fix", 1,
                                     &VehicleSafety::callback_number, this);
    gps_subscriber = nh.subscribe("/mavros/gpsstatus/gps2/raw", 1,
                                  &VehicleSafety::gps_failsafe, this);
    heading_failsafe_subscriber =
        nh.subscribe("/mavros/gpsstatus/gps1/raw", 1,
                     &VehicleSafety::heading_failsafe, this);
    global_pos_subscriber =
        nh.subscribe("/mavros/global_position/compass_hdg", 1,
                     &VehicleSafety::mavros_heading, this);
    control_diag_subscriber =
        nh.subscribe("/pure_pursuit_diagnose", 1,
                     &VehicleSafety::path_track_diagnose_callback, this);
    sensor_msg_subscriber =
        nh.subscribe("/mavros/global_position/global", 1,
                     &VehicleSafety ::global_gps_callback, this);
    is_curve_sub = nh.subscribe("/global_gps_path/is_curve", 1,
                                &VehicleSafety::is_curve_cb, this);

    can_data_sub =
        nh.subscribe("vehicle_info", 1, &VehicleSafety::can_data_cb, this);

    // nh.getParam("/vehicle_safety/no_go_zone_coordinates",
    // no_go_zone_coords_xmlrpc);
  }

  void is_curve_cb(const std_msgs::Bool &msg) {
    bool is_curve = msg.data;
    // ROS_INFO_STREAM("STRAIGHT " << CTE_THR_AT_STRAIGHT << ", " <<
    // CTE_THR_AT_CURVE);
    if (is_curve) {
      CTE_THR = CTE_THR_AT_CURVE;
      // ROS_INFO("CURVE");
    } else {
      CTE_THR = CTE_THR_AT_STRAIGHT;
      // ROS_INFO("STRAIGHT");
    }
  }

  void callback_number(const sensor_msgs::NavSatFix &msg) {
    cov_value = msg.position_covariance;
  }

  double sensor_accuracy(boost::array<double, 9> cov) {
    vector<vector<double>> lat_cov = {{cov.at(0), cov.at(1), cov.at(2)},
                                      {cov.at(3), cov.at(4), cov.at(5)},
                                      {cov.at(6), cov.at(7), cov.at(8)}};
    Matrix<double, 3, 3> m;

    for (int i = 0; i < lat_cov.size(); i++) {
      for (int j = 0; j < lat_cov[0].size(); j++) {
        m(i, j) = lat_cov[i][j];
      }
    }

    m_prev = m;
    Matrix<double, 3, 3> trans = m.transpose();
    Matrix<double, 3, 3> inv = m.inverse();

    auto acc_mat = trans * inv * m_prev;
    auto acc = 100 - acc_mat.determinant();
    reset_gps = 1;

    return acc;
  }

  void emergency_callback(const std_msgs::Bool &msg) {
    emergency_stop = msg.data;
  }

  /*
  void heading_failsafe(const mavros_msgs::GPSRAW &msg)
  {
      reset_head = 1;
      gps1_fix = msg.fix_type;
      prev_compass = curr_compass;
      float prev_coordinate_lat = curr_coordinates_lat;
      float prev_coordinate_long = curr_coordinates_long;

      auto gps_curr_lat = msg.lat;
      auto gps_curr_long = msg.lon;

      curr_coordinates_lat = gps_curr_lat;
      curr_coordinates_long = gps_curr_long;

      calc_heading = atan2(curr_coordinates_lat - prev_coordinate_lat,
  curr_coordinates_long - prev_coordinate_long);

      calc_heading *= 180 / 3.14159;
      calc_heading = (450 - int(calc_heading)) % 360;
      co_ordinates.push_back(calc_heading);
      int co_size = co_ordinates.size();

      if (co_size >= 5)
      {
          int x = abs(co_ordinates[co_size - 2] - co_ordinates[co_size - 1]);
          if (x == 0 && (co_ordinates[co_size - 2] || co_ordinates[co_size - 1]
  || co_ordinates[co_size - 3] || (co_ordinates[co_size - 4]) == 90) || x == 90
  || x == 45 || x == 180 || x == 135 || x == 270 || x == 315 || x == 225)
          {
              calc_heading = 0;
          }
          else
          {
              ;
          }
      }
  }
  */

  void heading_failsafe(const mavros_msgs::GPSRAW &data) {
    reset_head = 1;
    gps1_fix = data.fix_type;
    time_on_gps_fix_cb = ros::Time::now();
    // start_time_ros = ros::Time::now();
    // ROS_ERROR_STREAM("callbakck ros time : " << start_time_ros);
    // Calculating the heading using the new and previous coordinates
    double delta_lat = data.lat - curr_coordinates_lat;
    double delta_long = data.lon - curr_coordinates_long;
    curr_coordinates_lat = data.lat;
    curr_coordinates_long = data.lon;
    calc_heading =
        fmod((450 - atan2(delta_lat, delta_long) * 180.0 / 3.14159265), 360.0);
    // Add current heading to the list
    li.push_back(calc_heading);
    if (li.size() >= 5) {
      int x = abs(li[li.size() - 2] - li[li.size() - 1]);
      if ((x == 0 && (li[li.size() - 2] == 90 || li[li.size() - 1] == 90 ||
                      li[li.size() - 3] == 90 || li[li.size() - 4] == 90)) ||
          x == 90 || x == 45 || x == 180 || x == 135 || x == 270 || x == 315 ||
          x == 225) {
        calc_heading = 0;
      }
      li.erase(li.begin());
    }
  }

  void gps_failsafe(const mavros_msgs ::GPSRAW &msg) {
    gps2_fix = msg.fix_type;
  }

  void mavros_heading(const std_msgs::Float64 &msg) {
    mavros_head = (int)msg.data;
  }

  void path_track_diagnose_callback(
      const autopilot_msgs::ControllerDiagnose &msg) {
    auto start = std::chrono ::system_clock ::now();
    time_on_tracking_cb = ros::Time::now();
    pp_diagnose_data = msg;
    flag1 = 1;
  }

  void can_data_cb(const pilot_msgs::VehicleInfo &msg) {
    can_data = msg;
    battery_soc = can_data.battery_soc.data;
    motor_rpm = can_data.motor_rpm.data;
    demand_steering_angle = can_data.demand_steering_angle.data;
    current_steering_angle = can_data.steering_angle.data;
    // ROS_INFO_STREAM(can_data.battery_soc.data);
  }

  void global_gps_callback(const sensor_msgs::NavSatFix &msg) {
    nh.getParam("/vehicle_safety/no_go_zone_coordinates",
                no_go_zone_coords_xmlrpc);
    cov_value = msg.position_covariance;
    double lat = msg.latitude;
    double lon = msg.longitude;
    is_inside_geo_fence = isWithInGeoFence(lat, lon);
    is_with_in_no_go_zone =
        isWithinNoGoZone(no_go_zone_coords_xmlrpc, lat, lon);
    // ROS_WARN_STREAM(" NoGoZone: " << is_with_in_no_go_zone );

    ROS_WARN_STREAM_THROTTLE(10,"Geofence: " << is_inside_geo_fence
                                 << " NoGoZone: " << is_with_in_no_go_zone);
    reset_geo_fence = 1;
  }

  void gps_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    std::vector<int> default_value = {6};
    std::vector<int> gps1_fix_list, gps2_fix_list;
    ros::param::param<std::vector<int>>("/vehicle_safety/GPS1_FIX_TH",
                                        gps1_fix_list, default_value);
    ros::param::param<std::vector<int>>("/vehicle_safety/GPS2_FIX_TH",
                                        gps2_fix_list, default_value);

    std::string gps1_fix_list_str;
    for (int i : gps1_fix_list) gps1_fix_list_str += std::to_string(i) + ' ';

    std::string gps2_fix_list_str;
    for (int i : gps2_fix_list) gps2_fix_list_str += std::to_string(i) + ' ';

    int val;
    auto start = std::chrono::system_clock::now();

    gps_accuracy = sensor_accuracy(cov_value);

    if (time_on_gps_fix_cb != ros::Time(0)) {
      if (gps_accuracy != -1) {
        if (gps_accuracy < 0 || gps_accuracy > 100) {
          stat.add("GPS ACCURACY LOW", gps_accuracy);
        } else {
          if (abs(GPS_ACC_IDEAL - gps_accuracy) < GPS_ACC_THR) {
            stat.add("GPS ACCURACY HIGH", gps_accuracy);
          } else {
            stat.add("GPS ACCURACY LOW", gps_accuracy);
          }
        }
      } else {
        stat.add("GPS ACCURACY NONE", gps_accuracy);
      }
       
      ros::Duration elapsed  = ros::Time::now() - time_on_gps_fix_cb;
      double elapsed_sec = elapsed.toSec(); //local variable

      // ROS_WARN_STREAM("ROS TIME : " << time_on_gps_fix_cb << " ELAPSED : " << elapsed_sec << " Elapsed " << elapsed);
      
      if (elapsed_sec < GPS_FIX_CB_TIMEOUT) {
        if (std::find(gps1_fix_list.begin(), gps1_fix_list.end(), gps1_fix) !=
                gps1_fix_list.end() &&
            std::find(gps2_fix_list.begin(), gps2_fix_list.end(), gps2_fix) !=
                gps2_fix_list.end()) {
          stat.summary(OK, "GPS FIX OK");
          // stat.add("BOTH GPS Fix Type", 6);
          stat.add("GPS1 Fix Type", gps1_fix);
          stat.add("GPS2 Fix Type", gps2_fix);
          stat.add("GPS1_FIX_THRESHOLD", gps1_fix_list_str);
          stat.add("GPS2_FIX_THRESHOLD", gps2_fix_list_str);
          start_time = 0;
        } else if (gps1_fix == 5 && gps2_fix == 6) {
          if (start_time == 0) {
            start_time = time(0);
          }
          if ((time(0) - start_time) > GPS_FIX_THR) {
            stat.summary(ERROR, "ERROR: GPS FIX LOST TIMEOUT");
            stat.add("GPS1 Fix Type", gps1_fix);
            stat.add("GPS2 Fix Type", gps2_fix);
            stat.add("GPS1_FIX_THRESHOLD", gps1_fix_list_str);
            stat.add("GPS2_FIX_THRESHOLD", gps2_fix_list_str);
          } else {
            stat.summary(WARN, "WARN: GPS FIX LOST");
            stat.add("GPS1 Fix Type", gps1_fix);
            stat.add("GPS2 Fix Type", gps2_fix);
            stat.add("GPS1_FIX_THRESHOLD", gps1_fix_list_str);
            stat.add("GPS2_FIX_THRESHOLD", gps2_fix_list_str);
          }
        } else {
          stat.summary(ERROR, "ERROR: GPS FIX LOST");
          stat.add("GPS1 Fix Type", gps1_fix);
          stat.add("GPS2 Fix Type", gps2_fix);
          stat.add("GPS1_FIX_THRESHOLD", gps1_fix_list_str);
          stat.add("GPS2_FIX_THRESHOLD", gps2_fix_list_str);
        }
      } else {
      stat.summaryf(ERROR, "No update on GPS Tracking from last: %f secs",
                    elapsed_sec);
    }
  } else {
      stat.summary(ERROR, "Waiting for GPS Callback Data");
    }
  }

  void cte_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    // auto start = std::chrono::system_clock::now();
    ros::Duration elapsed = ros::Time::now() - time_on_tracking_cb;
    double elapsed_sec = elapsed.toSec();
    if (time_on_tracking_cb != ros::Time(0)) {
      if (elapsed_sec < TRACKING_CONTROLLER_TIMEOUT) {
        // if (flag1 == 1) {
          if (pp_diagnose_data.level == ERROR) {
            stat.summary(ERROR, pp_diagnose_data.message);
          } else if (pp_diagnose_data.level == WARN) {
            stat.summary(WARN, pp_diagnose_data.message);
            stat.add("CTE Value", pp_diagnose_data.cte);
            stat.add("CTE_THR", CTE_THR);
          }

          else {
            if (abs(pp_diagnose_data.cte) > CTE_THR) {
              stat.summary(ERROR, "HIGH CTE");
            } else {
              stat.summary(OK, "CTE OK");
            }
            stat.add("CTE Value", abs(pp_diagnose_data.cte));
            stat.add("CTE_THR", CTE_THR);
          }
        //   error_counter_for_tracking_controller = 0;
        // }

        // else {
        //   // eliminate false detections due to VS high freq.
        //   if (error_counter_for_tracking_controller >
        //       error_counter_for_tracking_controller_th) {
        //     stat.summary(ERROR, "No update on Tracking Controller");
        //     ROS_ERROR_THROTTLE(1,"No update on Tracking Controller");
        //   } else {
        //     error_counter_for_tracking_controller += 1;
        //     stat.summary(WARN, "No update on Tracking Controller");
        //     ROS_WARN_THROTTLE(1,"No update on Tracking Controller");
        //   }
        // }
        stat.add("Motor RPM", motor_rpm);
      }

      else {
        stat.summaryf(
            ERROR, "No update on Tracking Controller from last : %f secs", elapsed_sec);
      }
    } else {
      stat.summary(ERROR, "Waiting for Tracking Controller diagnose");
    }

    // flag1 = 0;
  }

  void heading_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    if (reset_head == 1) {
      if (mavros_head == -1) {
        stat.summary(ERROR, "Heading None");
        stat.add("Heading Value", mavros_head);
      } else {
        int found = 0;
        for (int i = 0; i < ignore_heading.size(); i++) {
          if (calc_heading == ignore_heading[i]) {
            found = 1;
          }
        }

        if (found == 1) {
          stat.summary(OK, "Heading OK");
          stat.add("mavros heading", mavros_head);
          stat.add("Calculated heading", calc_heading);
        }

        else {
          auto val1 = calc_heading - HEAD_THR;
          auto val2 = calc_heading + HEAD_THR;
          if (mavros_head > val1 && mavros_head < val2) {
            stat.summary(OK, "Heading OK");
            stat.add("mavros heading", mavros_head);
            stat.add("Calculated heading", calc_heading);
          } else {
            stat.summary(ERROR, "Heading ERROR");
            stat.add("mavros heading", mavros_head);
            stat.add("Calculated heading", calc_heading);
          }
        }
        int mavros_head = 0;
        found = 0;
      }
    } else {
      stat.summary(ERROR, "Heading ERROR");
      stat.add("mavros heading", mavros_head);
      stat.add("Calculated heading", calc_heading);
    }
    reset_head = 0;
    mavros_head = -1;
    calc_heading = -1;
  }

  void geo_fence_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat) {
    if (!use_inner_geo_fence) {
      is_with_in_no_go_zone = false;
    }
    if (reset_geo_fence == 1) {
      if (use_geo_fence) {
        if (is_inside_geo_fence && !is_with_in_no_go_zone) {
          stat.summary(OK, "OK: Vehicle is inside Go Zone");
          stat.add("Status", "OK");
        }

        else {
          stat.summary(ERROR, "ERROR: Vehicle is in No Go Zone");
          stat.add("Status", "STOP");
        }
      }

      else {
        stat.summary(WARN, "WARN: Geofence is not in Use");
        stat.add("Status", "WARN");
      }
    }

    else {
      stat.summary(WARN, "WARN: Geofence is not in Use");
      stat.add("Status", "WARN");
    }

    reset_geo_fence = 0;
  }

  bool isWithInGeoFence(double lat, double lon) {
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    typedef boost::geometry::model::polygon<point_type> polygon_type;
    typedef boost::geometry::model::d2::point_xy<double> point_xy;
    vector<vector<double>> pt = geoFence();
    model::d2::point_xy<double> pt2;
    polygon_type poly;
    vector<point_xy> points;

    // TODO @iam-vishnu. Do the below process outside this function. Don't
    // repeat always.
    for (int i = 0; i < pt.size(); i++) {
      boost::geometry::assign_values(pt2, pt[i][0], pt[i][1]);
      // ROS_INFO_STREAM("INFO STREAM " << pt[i][0] << "," << pt[i][1]);
      points += pt2;
    }

    boost::geometry::assign_points(poly, points);

    point_type p(lat, lon);
    if (boost::geometry::within(p, poly)) {
      return true;
    }

    return false;
  }

  bool isWithinNoGoZone(const XmlRpc::XmlRpcValue no_go_zone_coords_xmlrpc,
                        double lat, double lon) {
    for (int i = 0; i < no_go_zone_coords_xmlrpc.size(); ++i) {
      XmlRpc::XmlRpcValue polygon_coords_xmlrpc = no_go_zone_coords_xmlrpc[i];
      std::vector<std::vector<double>> polygon_coords;
      for (int j = 0; j < polygon_coords_xmlrpc.size(); ++j) {
        XmlRpc::XmlRpcValue point_coords_xmlrpc = polygon_coords_xmlrpc[j];
        std::vector<double> point_coords{
            static_cast<double>(point_coords_xmlrpc[0]),
            static_cast<double>(point_coords_xmlrpc[1])};
        polygon_coords.push_back(point_coords);
      }
      no_go_zone_coordinates.push_back(polygon_coords);
    }

    // Convert no_go_zone_coordinates to Boost MultiPolygon
    typedef bg::model::d2::point_xy<double> point_t;
    typedef bg::model::polygon<point_t> polygon_t;
    typedef bg::model::multi_polygon<polygon_t> multipolygon_t;
    multipolygon_t no_go_zone;

    for (auto &polygon_coords : no_go_zone_coordinates) {
      polygon_t polygon;
      for (auto &point_coords : polygon_coords) {
        point_t point(point_coords[0], point_coords[1]);
        bg::append(polygon.outer(), point);
      }
      boost::geometry::correct(polygon);
      // boost::geometry::append(no_go_zone, polygon); // Add the polygon to
      // no_go_zone
      no_go_zone.push_back(polygon);
    }

    // Check if the point is within the no-go zone
    point_t point(lat, lon);
    if (boost::geometry::within(point, no_go_zone)) {
      return true;
      // ROS_ERROR_STREAM("---"<< true);
    }
    // ROS_ERROR_STREAM("---"<< false);
    return false;
  }
  // return bg::within(point, no_go_zone);

  vector<vector<double>> geoFence() {
    XmlRpc::XmlRpcValue trajectory;
    nh.getParam("/vehicle_safety/geo_fence_coordinates", trajectory);

    XmlRpc::XmlRpcValue ::iterator itr;

    vector<vector<double>> geo_fence;
    if (trajectory.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < trajectory.size(); i++) {
        XmlRpc::XmlRpcValue trajectoryObject = trajectory[i];

        /*Individual coordinate points in trajectory*/

        double xCoordinate = trajectoryObject[0];
        double yCoordinate = trajectoryObject[1];
        // cout<<" X coo"<<xCoordinate<<endl;
        // cout<<" Y coo"<<yCoordinate<<endl;

        vector<double> gps;
        gps.push_back(xCoordinate);
        gps.push_back(yCoordinate);

        geo_fence.push_back(gps);
      }
    }

    return geo_fence;
  }

  void emergency_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat) {
    if (emergency_stop) {
      stat.summary(WARN, "Emergency Stop Status");
      stat.add("Status", "STOP");
    } else {
      stat.summary(OK, "Emergency Stop Status");
      stat.add("Status", "RUN");
    }
  }

  void can_steering_diagnostics(
        diagnostic_updater::DiagnosticStatusWrapper &stat) {
      
      if (current_steering_angle == 0) {
        stat.summary(STALE, "No Steering Data");
      }
      // In below logic, If demanded angle is zero, there is something from tracking algorithm but not the steering issue.
      if (abs(demand_steering_angle - current_steering_angle) >
              steering_diff_th &&
          demand_steering_angle != 0) {
        if (steering_error_start_time == 0) {
          steering_error_start_time = std::time(nullptr);
        }
        if (std::time(nullptr) - steering_error_start_time >
            steering_stuck_time_th) {
          
          is_steering_stuck = true;
        } else {
          is_steering_stuck = false;
        }
      } else {
        steering_error_start_time = 0;
        is_steering_stuck = false;
      }
      if (is_steering_stuck) {
        stat.summary(ERROR, "Steering Stuck");
      } else {
        stat.summary(OK, "Steering OK");
      }
      stat.add("Demand Steering Angle", demand_steering_angle);
      stat.add("Current Steering Angle", current_steering_angle);

      //TODO @iam-vishnu: Continuosly monitor for timeout of steering data.
    }
    
  void can_batt_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    std::string log_msg, stat_summ_msg;
    int stat_summ_stat;
    if (battery_soc < 0) {
      stat.summary(STALE, "No Data from CAN Battery");
      stat.add("Status", "STALE");
    }
    // calculate batt only when there is change in value. To save CPU

    // if (battery_soc != store_batt_level) {
    //   store_batt_level = battery_soc;

    if ((battery_soc > 0) && (battery_soc < BATT_SOC_TH)) {
      // stop the vehicle
      log_msg = "Low Battery";
      stat_summ_stat = ERROR;
      stat_summ_msg = "Low Battery";
      stat.summary(ERROR, stat_summ_msg);
      ROS_ERROR_STREAM_THROTTLE(100, stat_summ_msg);
    } else if (battery_soc > BATT_SOC_TH) {
      log_msg = "Battery OK";
      stat_summ_stat = OK;
      stat_summ_msg = "Battery % OK";
      stat.summary(OK, stat_summ_msg);
      ROS_INFO_STREAM_THROTTLE(100, stat_summ_msg);
    } else {
      ;
    }
    // }
    ROS_INFO_STREAM_THROTTLE(
        100, "SOC: " << battery_soc << ", Batt Th: " << BATT_SOC_TH);
    stat.add("Battery", battery_soc);
    stat.add("Battery_TH", BATT_SOC_TH);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "Vehicle_Safety_cpp");
  ros::NodeHandle nh;

  VehicleSafety vs = VehicleSafety();

  diagnostic_updater::Updater updater;
  updater.setHardwareID("Zekrom-v1");
  updater.add("GPS", &vs, &VehicleSafety::gps_diagnostics);
  updater.add("TrackingController", &vs, &VehicleSafety ::cte_diagnostics);
  updater.add("Heading", &vs, &VehicleSafety ::heading_diagnostics);
  updater.add("Emergency", &vs, &VehicleSafety::emergency_diagnostics);
  updater.add("GeoFence", &vs, &VehicleSafety::geo_fence_diagnostics);
  updater.add("CAN_Batt_Diagnostics", &vs,
              &VehicleSafety::can_batt_diagnostics);
  updater.add("CAN_Steering_Diagnostics", &vs,
              &VehicleSafety::can_steering_diagnostics);
  ros::Rate r(10);  // 10 hz
  while (nh.ok()) {
    ros::spinOnce();
    updater.update();
    r.sleep();  // To reduce cpu load
  }
}

#ifndef COSTMAP_MANAGER_H
#define COSTMAP_MANAGER_H
#include <cmath>
#include <costmap_2d/costmap_2d.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

using namespace std;
namespace autopilot_utils {
bool raytraceLineLookGap(costmap_2d::Costmap2D &costmap, double x1, double y1,
                         double x2, double y2, double &x_free, double &y_free,
                         double gap_len);
std::pair<bool, unsigned char> raytraceLineCost(costmap_2d::Costmap2D &costmap,
                                                double x1, double y1, double x2,
                                                double y2);
std::pair<bool, unsigned char> raytraceLineCost(costmap_2d::Costmap2D &costmap,
                                                double x1, double y1, double x2,
                                                double y2,
                                                unsigned char cost_th);

class OccupencyGridManager {
public:
  OccupencyGridManager(ros::NodeHandle, const string,
                       bool subscribe_to_updates);
  OccupencyGridManager();
  ~OccupencyGridManager();
  // nav_msgs::OccupancyGrid current_costmap;
  costmap_2d::Costmap2D costmap_2d;
  bool checkData();
  std::pair<bool, unsigned char> get_line_cost_world(double x1, double y1,
                                                     double x2, double y2);

private:
  void OccupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &);
  void
  OccupancyGridUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr &);
  ros::Subscriber occupency_sub, occupency_updaes_sub;
  bool occupency_map_received;
  unsigned char get_line_cost_map(unsigned int x1, unsigned int y1,
                                  unsigned int x2, unsigned int y2);
};
} // namespace autopilot_utils

#endif
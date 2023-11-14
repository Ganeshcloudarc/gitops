#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d.h>
#include<map_msgs/OccupancyGridUpdate.h>
#include<cmath>
using namespace std;
namespace auto_nav{
        std::pair<bool,unsigned char> raytraceLineCost(costmap_2d::Costmap2D &costmap,double x1, double y1, double x2, double y2)
        {
            unsigned int mx1,my1,mx2,my2;
            if (costmap.worldToMap(x1, y1, mx1, my1) and costmap.worldToMap(x2, y2, mx2, my2))
            {
                // unsigned char cost=  get_line_cost_map(mx1,my1,mx2,my2);

                 unsigned char cost;
                // Calculate differences and absolute differences between coordinates
                int dx_ = mx2 - mx1;
                int dy_ = my2 - my1;
                int dx = abs(dx_);
                int dy = abs(dy_);
                //  calculate the direction of the line
                // int sx = 1 if x1 < x2 else -1
                // int sy = 1 if y1 < y2 else -1
                int sx = (mx1<mx2) ? 1 : -1;
                int sy = (my1<my2) ? 1 :-1;
                int error = dx-dy;

                int x,y;
                x = mx1;
                y = my1;
                bool collision_found;
                // cout<<x1<<" "<<y1<<" "<<endl;
                // cout<<x2<<" "<<y2<<" "<<endl;
                while (x != mx2 or y !=my2 )
                {   //cout<<"x : " <<x <<" y : "<<y1;
                    int double_error = 2 * error;
                    if (double_error > -dy){
                        error -= dy;
                        x+=sx;
                    }
                    if (double_error < dx){
                            error += dx;
                            y += sy;
                    }
                
                    cost = costmap.getCost(x, y);
                    // cout<<"x :" <<x<<" y:"<<y<<endl;
                    // cout<<"cost :"<<static_cast<unsigned int>(cost)<<endl;
                    if (cost != 0)
                        {   //cout<<"COST is not ZERO"<<endl;
                            return std::make_pair(true, cost);
                        }
                
                }

                //  cout<<"REACHED END OF LOOP"<<endl;
                unsigned char c = 0;
                std::make_pair(true, c);
            }
            else
            {
                return std::make_pair(false, 0);
            }
        
        }


    


class OccupencyGridManager
{   public:
        OccupencyGridManager(ros::NodeHandle , const string , bool subscribe_to_updates);
        OccupencyGridManager();
        ~OccupencyGridManager();
        // nav_msgs::OccupancyGrid current_costmap;
        costmap_2d::Costmap2D costmap_2d;
        bool checkData();
        std::pair<bool,unsigned char> get_line_cost_world(double x1, double y1, double x2, double y2);


    private:
        void OccupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr&);
        void OccupancyGridUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr&);
        ros::Subscriber occupency_sub, occupency_updaes_sub;
        bool occupency_map_received;
        unsigned char get_line_cost_map(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);

    



};

OccupencyGridManager::OccupencyGridManager(ros::NodeHandle nh, const string topic, bool subscribe_to_updates= false):
occupency_map_received(false)
{
    occupency_sub = nh.subscribe<nav_msgs::OccupancyGrid>(topic, 1,  &OccupencyGridManager::OccupancyGridCallback, this);
    if (subscribe_to_updates)
    {

    occupency_updaes_sub = nh.subscribe<map_msgs::OccupancyGridUpdate>(topic+"updates", 1,  &OccupencyGridManager::OccupancyGridUpdateCallback, this);

    }
}

void OccupencyGridManager::OccupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy_grid_msg)
{
    // costmap_2d.setDefaultValue(0);
    costmap_2d.resizeMap(occupancy_grid_msg->info.width, occupancy_grid_msg->info.height, occupancy_grid_msg->info.resolution,
    occupancy_grid_msg->info.origin.position.x, occupancy_grid_msg->info.origin.position.y);
    // Copy the data from the OccupancyGrid to the Costmap2D
    for (unsigned int y = 0; y < occupancy_grid_msg->info.height; ++y) {
        for (unsigned int x = 0; x < occupancy_grid_msg->info.width; ++x) {
            unsigned int index = x + y * occupancy_grid_msg->info.width;
            costmap_2d.setCost(x, y, occupancy_grid_msg->data[index]);
        }
    }
}
void OccupencyGridManager::OccupancyGridUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& update_msg)
{
    for (unsigned int y = update_msg->y; y < update_msg->height; y++) {
        for (unsigned int x = update_msg->x; x <  update_msg->width; x++) {
            unsigned int index = x + y * update_msg->width;
            costmap_2d.setCost(x, y, update_msg->data[index]);
        }
    }
}

unsigned char OccupencyGridManager::get_line_cost_map(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
{   unsigned char cost;
     // Calculate differences and absolute differences between coordinates
    int dx_ = x2 - x1;
    int dy_ = y2 - y1;
    int dx = abs(dx_);
    int dy = abs(dy_);
    //  calculate the direction of the line
    // int sx = 1 if x1 < x2 else -1
    // int sy = 1 if y1 < y2 else -1
    int sx = (x1<x2) ? 1 : -1;
    int sy = (y1<y2) ? 1 :-1;
    int error = dx-dy;

    int x,y;
    x = x1;
    y = y1;
    bool collision_found;
    // cout<<x1<<" "<<y1<<" "<<endl;
    // cout<<x2<<" "<<y2<<" "<<endl;
    while (x != x2 or y !=y2 )
    {   //cout<<"x : " <<x <<" y : "<<y1;
        int double_error = 2 * error;
        if (double_error > -dy){
            error -= dy;
            x+=sx;
        }
        if (double_error < dx){
                error += dx;
                y += sy;
        }
       
        cost = costmap_2d.getCost(x, y);
        // cout<<"cost :"<<static_cast<unsigned int>(cost)<<endl;
        if (cost != 0)
            {
                return cost;
            }
       
    }
    
    return cost;
}

std::pair<bool,unsigned char> OccupencyGridManager::get_line_cost_world(double x1, double y1, double x2, double y2)
{
    unsigned int mx1,my1,mx2,my2;
    if (costmap_2d.worldToMap(x1, y1, mx1, my1) and costmap_2d.worldToMap(x2, y2, mx2, my2))
    {
        unsigned char cost=  get_line_cost_map(mx1,my1,mx2,my2);
        std::make_pair(true, cost);
    }
    else
    {
        return std::make_pair(false, 0);
    }

}








}



/*
def get_line_cost_map(self, x1, y1, x2, y2):
        cost = 0
        # Calculate differences and absolute differences between coordinates
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)

        # Calculate the direction of the line
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1

        # Initialize error
        error = dx - dy

        # Initialize coordinates
        x, y = x1, y1

        try:
            cost = self.get_cost_from_costmap_x_y(x, y)
            if cost != 0:
                return cost, self.get_world_x_y(x, y)
        except:
            pass

        # Main loop to iterate through the line
        while x != x2 or y != y2:
            # Add the current point to the list
            # Calculate the next coordinates
            double_error = 2 * error
            if double_error > -dy:
                error -= dy
                x += sx
            if double_error < dx:
                error += dx
                y += sy
            try:
                cost = self.get_cost_from_costmap_x_y(x, y)
                if cost != 0:
                    return cost, self.get_world_x_y(x, y)
            except:
                pass

        return cost, (None, None)



    def get_line_cost_world(self, x1, y1, x2, y2):
        x1, y1 = self.get_costmap_x_y(x1, y1)
        x2, y2 = self.get_costmap_x_y(x2, y2)
        print("val", self.get_line_cost_map(x1, y1, x2, y2))
        cost, obs_xy = self.get_line_cost_map(x1, y1, x2, y2)
        return cost, obs_xy*/

// void occupancyGridToCostmap2D(const nav_msgs::OccupancyGrid& occupancy_grid_msg, costmap_2d::Costmap2D& costmap_2d) {
//     // Assuming that the resolution, width, and height of the OccupancyGrid match the Costmap2D
//     costmap_2d.setDefaultValue(0);
//     costmap_2d.resizeMap(occupancy_grid_msg.info.width, occupancy_grid_msg.info.height, occupancy_grid_msg.info.resolution,
//     occupancy_grid_msg.info.origin.position.x, occupancy_grid_msg.info.origin.position.y);
//     // Copy the data from the OccupancyGrid to the Costmap2D
//     for (unsigned int y = 0; y < occupancy_grid_msg.info.height; ++y) {
//         for (unsigned int x = 0; x < occupancy_grid_msg.info.width; ++x) {
//             unsigned int index = x + y * occupancy_grid_msg.info.width;
//             costmap_2d.setCost(x, y, occupancy_grid_msg.data[index]);
//         }
//     }
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "occupancygrid_to_costmap2d_example");
//     ros::NodeHandle nh;

//     // Create a subscriber for the OccupancyGrid message
//     ros::Subscriber occupancy_grid_subscriber = nh.subscribe("/your_occupancy_grid_topic", 1, occupancyGridCallback);

//     // Initialize a Costmap2D object
//     costmap_2d::Costmap2D costmap;

//     // Spin to keep the program running and handle messages
//     ros::spin();

//     return 0;
// }

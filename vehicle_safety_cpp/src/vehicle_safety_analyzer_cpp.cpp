#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>
#include <diagnostic_updater/publisher.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <pilot/vehicle_stop_command.h>
#include <ctime>
#include <map> 
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
using namespace std;
class FailSafeAutoPilot
{
    ros::NodeHandle fs;
    private:
        int status = {};
        bool fail_status;
        // vector<string> whitelist = {"vehicle_safety_cpp_diagnostics: GPS","vehicle_safety_cpp_diagnostics: TrackingController","vehicle_safety_cpp_diagnostics: Emergency","vehicle_safety_cpp_diagnostics: GeoFence"};
        vector<string> whitelist = fs.param("/vehicle_safety/WHITE_LIST",whitelist);
        //  float GPS_FIX_THR=nh.param("/vehicle_safety/GPS_FIX_THR",GPS_FIX_THR);
        vector<string> blacklist = fs.param("/vehicle_safety/BLACK_LIST",blacklist);
        // vector<string> blacklist = {"vehicle_safety_cpp_diagnostics: Heading"};
        bool use_vehicle_safety =true;
        ros::Publisher stop_cmd_publisher;
        ros::Subscriber vehicle_safety_diagnose; 
    public:
        
    FailSafeAutoPilot()
    {
        stop_cmd_publisher = fs.advertise<pilot::vehicle_stop_command>("/vehicle/stop_command", 1,this);
        pilot::vehicle_stop_command vehicle_stop_command_msg;
        if (use_vehicle_safety)
        {
            vehicle_safety_diagnose = fs.subscribe("/vehicle_safety_diagnostics", 1, &FailSafeAutoPilot::vehicle_safety_diagnose_cb, this);
        }

    }
    
    void vehicle_safety_diagnose_cb(const diagnostic_msgs::DiagnosticArray& data)
    {
        map<string,bool> dicto;
        vector<bool> val;
        auto OK = diagnostic_msgs::DiagnosticStatus::OK; 
        auto ERROR = diagnostic_msgs::DiagnosticStatus::ERROR;
        auto WARN = diagnostic_msgs::DiagnosticStatus::WARN; 
        vector<string> reason;
        for(auto field=0; field<data.status.size(); field++)
        {
            auto found_white = find(whitelist.begin(),whitelist.end(),data.status[field].name);
            auto found_black = find(blacklist.begin(),blacklist.end(),data.status[field].name);
            if ((found_white != whitelist.end())&&(found_black == blacklist.end())) 
            {
                if(data.status[field].level == ERROR)
                {
                    dicto.insert({data.status[field].name , true});
                    reason.push_back(data.status[field].name);
                }
                else 
                {
                    dicto.insert({data.status[field].name , false});
                }
            }
            else 
            {
                ;
            }            
        }
        for(auto& it : dicto)
        {
            // cout<<it.second<<endl;
            val.push_back(it.second);
        }
        
        bool anyValid = std::any_of(val.begin(), val.end(),[](bool x) { return x; });
        if(anyValid == true)
        {
            fail_status = true;
        }
        else
        {
            fail_status = false;
        }

        pilot::vehicle_stop_command vehicle_stop_command_msg;

        if(fail_status == true)
        {
            ROS_ERROR_THROTTLE(2,"Stopping the Vehicle");
            string str="";
            for(int i=0; i<reason.size(); i++)
            {
               str+= reason[i]+" ";
            }
            
            vehicle_stop_command_msg.node = ros::this_node::getName();
            vehicle_stop_command_msg.message = "ERROR: " + str;
            vehicle_stop_command_msg.status = true;
            ROS_ERROR_STREAM_THROTTLE(2,"" << vehicle_stop_command_msg.message);
            fail_status = false;
        }

        else 
        {
            vehicle_stop_command_msg.node = ros::this_node::getName();
            vehicle_stop_command_msg.message = "OK";
            vehicle_stop_command_msg.status = false;
            ROS_INFO_THROTTLE(60,"OK from Vehicle Safety");
        }
        stop_cmd_publisher.publish(vehicle_stop_command_msg);        
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "Vehicle_Safety_Analyzer_cpp");
    ros::NodeHandle fs;
    FailSafeAutoPilot failsafe;
    // while(ros::ok())
    // {
    //     sleep(0.1);
    //     ros::spinOnce();
    // }
    ros::spin(); // To reduce cpu load. Instead of using while loop. Spin will do the job
    return 0;
}


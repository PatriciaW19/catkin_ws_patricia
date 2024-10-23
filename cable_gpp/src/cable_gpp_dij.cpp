#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <cable_gpp/cable_gpp.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <vector>

PLUGINLIB_EXPORT_CLASS(cable_gpp_ns::cable_gpp, mbf_costmap_core::CostmapPlanner)

// using namespace std;

namespace cable_gpp_ns
{
    cable_gpp::cable_gpp()
    {
    }
    
    cable_gpp::cable_gpp(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        // initialize(name, costmap_ros);
        ROS_INFO_STREAM("Constructor");

    }

    void cable_gpp::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
       
    }

    // void cable_gpp::initialize(std::string name, 
    //                         costmap_2d::Costmap2DROS *costmap_ros,
    //                         std::string global_frame)
    // {
        
    // }

    uint32_t cable_gpp::makePlan(const geometry_msgs::PoseStamped &start,
                        const geometry_msgs::PoseStamped &goal,
                        double tolerance,
                        std::vector<geometry_msgs::PoseStamped> &plan,
                        double &cost,
                        std::string &message)
    {
        plan.clear();
        
        //1-D array representation of the costmap
        const auto &costmap = ...; // Retrieve the c

        ros::Time start_time = ros::Time::now();

        //Calculate the shortest path using Dijkstra's (implent )
        std::vector<geometry_msgs::PoseStamped> path = Dijkstra(star)
       

    }

    bool cable_gpp::cancel()
    {
        return false;
    }
}
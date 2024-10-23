#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <cable_gpp/cable_gpp.h>

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
        ROS_INFO_STREAM("Initialize");
        // initialize the publisher
        
        // Test für publisher Node
            ros::NodeHandle private_nh("~/" + name);
            this->plan_gpp_publisher_ = private_nh.advertise<nav_msgs::Path>("plan_cable_gpp",1);
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
        ROS_INFO_STREAM("Goal before loop" << goal.pose);
        ROS_INFO_STREAM("cost: " << cost);


        
        plan.push_back(start);
        for (int i = 0; i < 20; i++)
        {

            geometry_msgs::PoseStamped new_goal = goal;
            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

            new_goal.pose.position.x = -2.5 + (0.05 * i);
            new_goal.pose.position.y = -3.5 + (0.05 * i);
            new_goal.pose.orientation.x = goal_quat.x();
            ;
            new_goal.pose.orientation.y = goal_quat.y();
            new_goal.pose.orientation.z = goal_quat.z();
            new_goal.pose.orientation.w = goal_quat.w();

            plan.push_back(new_goal);
            // ROS_INFO_STREAM("New Goal: x=" << new_goal.pose.position.x << ", y=" << new_goal.pose.position.y
            //                                << ", z=" << new_goal.pose.position.z
            //                                << ", orientation (x,y,z,w)=" << new_goal.pose.orientation.x << ", "
            //                                << new_goal.pose.orientation.y << ", " << new_goal.pose.orientation.z << ", "
            //                                << new_goal.pose.orientation.w);
        }
        plan.push_back(goal);

        // std::size_t end_point = plan.size();



        for (std::size_t i = 0; i < plan.size(); i++)
        {
            ROS_INFO_STREAM("Loop Through: " << plan[i].pose);
        }


        publishCableGPP(plan); 

        return true;
    }

    void cable_gpp::publishCableGPP(std::vector<geometry_msgs::PoseStamped> &plan){
        nav_msgs::Path path_to_publish;
        path_to_publish.header.stamp = ros::Time::now();
        path_to_publish.header.frame_id = plan[0].header.frame_id;

        path_to_publish.poses = plan;

        this-> plan_gpp_publisher_.publish(path_to_publish);

    }


    bool cable_gpp::cancel()
    {
        return false;
    }
}
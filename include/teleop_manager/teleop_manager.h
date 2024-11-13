#ifndef TELEOP_MANAGER_H
#define TELEOP_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

class TeleopManager
{
    public:
        TeleopManager();
        void process();

    private:
        //function
        void joy_callback(const sensor_msgs::Joy::ConstPtr &msg);
        void emergency_stop_callback(const std_msgs::Bool::ConstPtr &msg);
        void local_path_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);
        void visual_path_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);
        int select_mode(const sensor_msgs::JoyConstPtr &msg, int mode);
        void print_info(geometry_msgs::Twist vel);

        //param
        int hz_;
        int mode_;
        float max_velocity_;
        float max_yawrate_;

        //flags
        bool stop_flag_ = 0;
        bool get_joy_ = 0;
        bool get_local_path_vel_ = 0;
        bool get_visual_path_vel_ = 0;

        //msgs
        geometry_msgs::Twist joy_vel_;
        geometry_msgs::Twist local_vel_;
        geometry_msgs::Twist visual_vel_;

        //ros
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        //subscriber
        ros::Subscriber sub_joy_;
        ros::Subscriber sub_emergency_stop_;
        ros::Subscriber sub_local_path_cmd_vel_;
        ros::Subscriber sub_visual_path_cmd_vel_;
        //publisher
        ros::Publisher pub_cmd_vel_;
};
#endif

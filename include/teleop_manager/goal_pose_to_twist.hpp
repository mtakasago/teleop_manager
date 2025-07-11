#ifndef GOAL_POSE_TO_TWIST_HPP
#define GOAL_POSE_TO_TWIST_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class GoalPoseToTwist : public rclcpp::Node
{
    public:
        GoalPoseToTwist();
        void process();
        int hz_ = 10;

    private:
        //function
        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        double get_yaw(const geometry_msgs::msg::Quaternion &q);
        double set_yaw(double yaw);

        //param
        double goal_x_;
        double goal_y_;
        double goal_yaw_;

        //flags
        bool receive_odom_ = false;
        bool receive_goal_ = false;
        bool reached_goal_point_ = false;
        bool reached_goal_orientation_ = false;

        //msgs
        geometry_msgs::msg::PoseStamped goal_pose_;
        nav_msgs::msg::Odometry old_odom_;
        nav_msgs::msg::Odometry current_odom_;
        rclcpp::TimerBase::SharedPtr timer_;

        //subscriber
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

        //publisher
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
};

#endif

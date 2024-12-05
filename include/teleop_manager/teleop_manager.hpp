#ifndef TELEOP_MANAGER_H
#define TELEOP_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <cmath>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TeleopManager : public rclcpp::Node
{
    public:
        TeleopManager();
        void process();
        int hz_;

    private:
        //function
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void local_path_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void visual_path_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        int select_mode(const sensor_msgs::msg::Joy::SharedPtr msg, int mode);
        void print_info(geometry_msgs::msg::Twist vel);

        //param
        int mode_;
        float max_x_velocity_;
        float max_y_velocity_;
        float max_yawrate_;
        double max_auto_velocity_;

        //flags
        bool stop_flag_ = 0;
        bool get_joy_ = 0;
        bool get_local_path_vel_ = 0;
        bool get_visual_path_vel_ = 0;

        //msgs
        geometry_msgs::msg::Twist joy_vel_;
        geometry_msgs::msg::Twist local_vel_;
        geometry_msgs::msg::Twist visual_vel_;

        //subscriber
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emergency_stop_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_local_path_cmd_vel_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_visual_path_cmd_vel_;
        //publisher
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
};
#endif

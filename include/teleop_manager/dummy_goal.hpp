#ifndef DUMMY_GOAL_HPP
#define DUMMY_GOAL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class DummyGoal : public rclcpp::Node
{
    public:
        DummyGoal();
        void process();

    private:
        //param
        double goal_x_;
        double goal_y_;
        double goal_yaw_;
        //function
        tf2::Quaternion get_quaternion(double yaw);

        //msg
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;

};

#endif

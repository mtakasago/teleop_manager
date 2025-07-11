#include "teleop_manager/dummy_goal.hpp"

DummyGoal::DummyGoal():Node("dummy_goal_node")
{

    goal_x_ = this->declare_parameter<double>("goal_x", 0.0);
    goal_y_ = this->declare_parameter<double>("goal_y", 0.0);
    goal_yaw_ = this->declare_parameter<double>("goal_yaw", 0.0);

    pub_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);
}

tf2::Quaternion DummyGoal::get_quaternion(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw); //roll = 0.0, pitch = 0.0
    return q;
}

void DummyGoal::process()
{
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "base_link";
    goal_pose.header.stamp = get_clock()->now();

    goal_pose.pose.position.x = goal_x_;
    goal_pose.pose.position.y = goal_y_;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = sin(goal_yaw_ / 2.0);
    goal_pose.pose.orientation.w = cos(goal_yaw_ / 2.0);

    pub_goal_->publish(goal_pose);
    RCLCPP_INFO(this->get_logger(),"Send Goal (%f, %f, %f)",goal_x_, goal_y_, goal_yaw_);
}

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    DummyGoal dummy_goal;;
    auto node = std::make_shared<DummyGoal>();
    node->process();
    rclcpp::shutdown();
    return 0;
}

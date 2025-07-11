#include "teleop_manager/goal_pose_to_twist.hpp"

GoalPoseToTwist::GoalPoseToTwist():Node("GoalPoseToTwist_node")
{
	//params

	//sub
	sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal", 10, std::bind(&GoalPoseToTwist::goal_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&GoalPoseToTwist::odom_callback, this, std::placeholders::_1));
	//pub
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/visual_path_cmd_vel", 10);
	timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GoalPoseToTwist::process, this));
}

void GoalPoseToTwist::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	if(!receive_goal_)
	{
		goal_pose_ = *msg;
        goal_x_ = goal_pose_.pose.position.x;
        goal_y_ = goal_pose_.pose.position.y;
        goal_yaw_ = get_yaw(goal_pose_.pose.orientation);
		receive_goal_ = true;
		reached_goal_point_ = false;
		reached_goal_orientation_ = false;
        RCLCPP_INFO(this->get_logger(),"Get New Goal (%f, %f, %f)",goal_x_, goal_y_, goal_yaw_);
	}
}
void GoalPoseToTwist::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	if(!receive_goal_) old_odom_ = *msg;
	current_odom_ = *msg;
	receive_odom_ = true;
}

double GoalPoseToTwist::get_yaw(const geometry_msgs::msg::Quaternion &q)
{
    double roll, pitch, yaw;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    return yaw;
}

double GoalPoseToTwist::set_yaw(double yaw)
{
    if(yaw > M_PI) yaw -= 2*M_PI;
    if(yaw < -M_PI) yaw += 2*M_PI;
    return yaw;
}

void GoalPoseToTwist::process()
{
	if(!receive_odom_)
    {
        RCLCPP_ERROR(this->get_logger(),"No Odometry");
		return;
	}
    if(!receive_goal_)
    {
        RCLCPP_ERROR(this->get_logger(),"Waiting for New Goal...");
        return;
    }

    double robot_x = current_odom_.pose.pose.position.x - old_odom_.pose.pose.position.x;
    double robot_y = current_odom_.pose.pose.position.y - old_odom_.pose.pose.position.y;
    double robot_yaw = set_yaw(get_yaw(current_odom_.pose.pose.orientation) - get_yaw(old_odom_.pose.pose.orientation));

    double dx = goal_x_ - robot_x;
    double dy = goal_y_ - robot_y;
    double goal_direction = atan2(dy, dx);
    double dyaw = set_yaw(goal_direction - robot_yaw);
    double goal_dist = std::hypot(dx, dy);

    geometry_msgs::msg::Twist cmd_vel;

    if (!reached_goal_point_)
    {
        if (fabs(dyaw) > 0.1) cmd_vel.angular.z = 0.5 * dyaw;
        else if (goal_dist > 0.1) cmd_vel.linear.x = 0.2;
        else reached_goal_point_ = true;
    }
    else if (!reached_goal_orientation_)
    {
        double final_dyaw = set_yaw(goal_yaw_ - robot_yaw);
        if (fabs(final_dyaw) > 0.1) cmd_vel.angular.z = 0.5 * final_dyaw;
    }
    else
    {
        reached_goal_orientation_ = true;
        RCLCPP_INFO(this->get_logger(),"I Did it!!! Goal!!!!");
        RCLCPP_INFO(this->get_logger(),"Waiting for New Goal...");
        receive_goal_ = false;
    }
        pub_cmd_vel_->publish(cmd_vel);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    GoalPoseToTwist goal_pose_to_twist;
    auto node = std::make_shared<GoalPoseToTwist>();
    rclcpp::Rate loop_rate(node->hz_);
    while(rclcpp::ok())
    {
        node->process();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}

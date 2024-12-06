#include "teleop_manager/teleop_manager.hpp"

TeleopManager::TeleopManager():Node("teleop_manager_node")
{
    // params
    max_x_velocity_ = this->declare_parameter<float>("max_x_velocity", 1.0); // m/s
    max_y_velocity_ = this->declare_parameter<float>("max_y_velocity", 1.0); // m/s
    max_auto_velocity_ = this->declare_parameter<float>("max_auto_velocity", 1.0); // m/s
    max_yawrate_ = this->declare_parameter<float>("max_yawrate", 1.0); // m/s
    hz_ = this->declare_parameter<int>("hz", 10);

    // publisher
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::QoS(1).reliable());

    // subscriber
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            rclcpp::QoS(1).reliable(),
            std::bind(&TeleopManager::joy_callback,this,std::placeholders::_1));
    sub_emergency_stop_ = this->create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop",
            rclcpp::QoS(1).reliable(),
            std::bind(&TeleopManager::emergency_stop_callback,this,std::placeholders::_1));
    sub_local_path_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/local_path_cmd_vel",
            rclcpp::QoS(1).reliable(),
            std::bind(&TeleopManager::local_path_vel_callback,this,std::placeholders::_1));
    sub_visual_path_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/visual_path_cmd_vel",
            rclcpp::QoS(1).reliable(),
            std::bind(&TeleopManager::visual_path_vel_callback,this,std::placeholders::_1));
}

void TeleopManager::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    mode_ = select_mode(msg, mode_);
    if(msg->buttons[4] == 1) // press L1 button
    {
        joy_vel_.linear.x = msg->axes[1] * max_x_velocity_;
        joy_vel_.linear.y = msg->axes[3] * max_y_velocity_;
        joy_vel_.angular.z = msg->axes[0] * max_yawrate_;
    }
    else joy_vel_ = geometry_msgs::msg::Twist();

    get_joy_ = true;
}

void TeleopManager::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    stop_flag_ = msg->data;
}

void TeleopManager::local_path_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    local_vel_ = *msg;
    local_vel_.linear.x = std::min(float(msg->linear.x), max_auto_velocity_);
    local_vel_.linear.y = std::min(float(msg->linear.y), max_auto_velocity_);
    get_local_path_vel_ = true;
}

void TeleopManager::visual_path_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    visual_vel_ = *msg;
    visual_vel_.linear.x = std::min(float(msg->linear.x), max_auto_velocity_);
    visual_vel_.linear.y = std::min(float(msg->linear.y), max_auto_velocity_);
    get_visual_path_vel_ = true;
}

int TeleopManager::select_mode(const sensor_msgs::msg::Joy::SharedPtr msg, int mode)
{
    // mode description
    // 0: stop
    // 1: manual joycon operation
    // 2: auto1  local_path operation
    // 3: auto2  visual_path operation
    // 4: combi  automaticaly select local or visual

    int new_mode = mode;
    if(msg->buttons[2] == 1) new_mode = 0; // X button
    if(msg->buttons[3] == 1) new_mode = 1; // Y button
    if(msg->buttons[0] == 1) new_mode = 2; // A button
    if(msg->buttons[1] == 1) new_mode = 3; // B button
    if(msg->buttons[8] == 1) new_mode = 4; // Center Circle button
    return new_mode;
}

void TeleopManager::print_info(geometry_msgs::msg::Twist vel)
{
    std::string mode_str = "stop";
    if(mode_ == 1) mode_str = "manual";
    else if(mode_ == 2) mode_str = "auto(lidar)";
    else if(mode_ == 3) mode_str = "auto(visual)";
    else if(mode_ == 4) mode_str = "auto(combi)";

    std::cout<<"===== "<< mode_str <<" ====="<<std::endl;
    std::cout<<"linear_x : "<< vel.linear.x <<std::endl;
    std::cout<<"linear_y : "<< vel.linear.y <<std::endl;
    std::cout<<"angular_z: "<< vel.angular.z <<std::endl;
}
void TeleopManager::process()
{
        auto final_vel = geometry_msgs::msg::Twist();
        if((stop_flag_ && mode_ != 1) || mode_ == 0) final_vel = geometry_msgs::msg::Twist();
        else if(mode_ == 1)
        {
            if(get_joy_) final_vel = joy_vel_;
            else std::cout<<"No joycon command"<<std::endl;
        }
        else if(mode_ == 2)
        {
            if(get_local_path_vel_) final_vel = local_vel_;
            else std::cout<<"No local_path cmd_vel"<<std::endl;
        }
        else if(mode_ == 3)
        {
            if(get_visual_path_vel_) final_vel = visual_vel_;
            else std::cout<<"No visual_path cmd_vel"<<std::endl;
        }
        print_info(final_vel);
        pub_cmd_vel_->publish(final_vel);

        get_joy_ = get_local_path_vel_ = get_visual_path_vel_ = 0;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    TeleopManager teleop_manager;
    auto node = std::make_shared<TeleopManager>();
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


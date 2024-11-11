#include "teleop_manager.h"

TeleopManager::TeleopManager(void): private_nh_("")
{
    // params
    private_nh_.param<float>("max_velocity", max_velocity_, 1.0); // m/s
    private_nh_.param<float>("max_yawrate", max_yawrate_, 1.0); // m/s
    private_nh_.param<int>("hz", hz_, 50);

    // publisher
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmb_vel",1,true);

    // subscriber
    sub_joy_ = nh_.subscribe("/joy",1,&TeleopManager::joy_callback,this);
    sub_emergency_stop_ = nh_.subscribe("/emergency_stop",1,&TeleopManager::emergency_stop_callback,this);
    sub_local_path_cmd_vel = nh_.subscribe("/local_path_cmd_vel",&TeleopManager::local_path_callback,this);
    sub_visual_path_cmd_vel = nh_.subscribe("/visual_path_cmd_vel",&TeleopManager::visual_path_callback,this);

}

void TeleopManager::joy_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
    mode_ = select_mode(&msg, mode_);
    std::cout<<"mode: "<< mode_ <<std::endl;
}

void TeleopManager::local_path_calback(const geometry_msgs::Twist::ConstPtr &msg)
{
}

void TeleopManager::emergency_stop_callback(const std_msgs::Bool::ConstPtr &msg)
{
}

void TeleopManager::visual_path_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
}

int TeleopManeger::select_mode(sensor_msgs::JoyConstPtr &msg)
{
    // mode description
    // 0: stop
    // 1: manual (joycon operation)
    // 2: auto (local_path operation / visual_path operation)
    // 3: ???
    //
    if(msg->button[2] == 1) return 0; // X button
    if(msg->button[0] == 1) return 1; // A button
    if(msg->button[1] == 1) return 2; // B button
}


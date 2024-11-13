#include "teleop_manager/teleop_manager.h"

TeleopManager::TeleopManager(): private_nh_("~")
{
    // params
    private_nh_.param<float>("max_velocity", max_velocity_, 1.0); // m/s
    private_nh_.param<float>("max_yawrate", max_yawrate_, 1.0); // m/s
    private_nh_.param<int>("hz", hz_, 10);

    // publisher
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);

    // subscriber
    sub_joy_ = nh_.subscribe("/joy",10,&TeleopManager::joy_callback,this);
    sub_emergency_stop_ = nh_.subscribe("/emergency_stop",10,&TeleopManager::emergency_stop_callback,this);
    sub_local_path_cmd_vel_ = nh_.subscribe("/local_path_cmd_vel",10,&TeleopManager::local_path_vel_callback,this);
    sub_visual_path_cmd_vel_ = nh_.subscribe("/visual_path_cmd_vel",10,&TeleopManager::visual_path_vel_callback,this);
}

void TeleopManager::joy_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
    mode_ = select_mode(msg, mode_);
    if(msg->buttons[4] == 1) // press L1 button
    {
        joy_vel_.linear.x = msg->axes[1] * max_velocity_;
        joy_vel_.angular.z = msg->axes[0] * max_yawrate_;
    }
    else joy_vel_ = geometry_msgs::Twist();

    get_joy_ = true;
}

void TeleopManager::emergency_stop_callback(const std_msgs::Bool::ConstPtr &msg)
{
    stop_flag_ = msg->data;
}

void TeleopManager::local_path_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    local_vel_ = *msg;
    get_local_path_vel_ = true;
}

void TeleopManager::visual_path_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    visual_vel_ = *msg;
    get_visual_path_vel_ = true;
}

int TeleopManager::select_mode(const sensor_msgs::JoyConstPtr &msg, int mode)
{
    // mode description
    // 0: stop
    // 1: manual joycon operation
    // 2: auto1  local_path operation
    // 3: auto2  visual_path operation
    // 4: combi  automaticaly select local or visual

    int new_mode = mode;
    if(msg->buttons[2] == 1) new_mode = 0; // X button
    if(msg->buttons[0] == 1) new_mode = 1; // A button
    if(msg->buttons[1] == 1) new_mode = 2; // B button
    if(msg->buttons[3] == 1) new_mode = 3; // Y button
    if(msg->buttons[8] == 1) new_mode = 4; // Center Circle button
    return new_mode;
}

void TeleopManager::print_info(geometry_msgs::Twist vel)
{
    std::string mode_str = "stop";
    if(mode_ == 1) mode_str = "manual";
    else if(mode_ == 2) mode_str = "auto(local)";
    else if(mode_ == 3) mode_str = "auto(visual)";
    else if(mode_ == 4) mode_str = "auto(combi)";

    std::cout<<"===== "<< mode_str <<" ====="<<std::endl;
    std::cout<<"linear_x : "<< vel.linear.x <<std::endl;
    std::cout<<"angular_z: "<< vel.angular.z <<std::endl;
}
void TeleopManager::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        geometry_msgs::Twist final_vel;
        if(stop_flag_ || mode_ == 0) final_vel = geometry_msgs::Twist();
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
        pub_cmd_vel_.publish(final_vel);

        get_joy_ = get_local_path_vel_ = get_visual_path_vel_ = 0;

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_manager");
    TeleopManager teleop_manager;
    teleop_manager.process();
    return 0;
}


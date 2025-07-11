cmake_minimum_required(VERSION 3.8)
project(teleop_manager)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies package.xmlから自動で探す
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)

ament_auto_add_executable(teleop_manager_node
    src/teleop_manager.cpp
)
ament_auto_add_executable(goal_pose_to_twist_node
    src/goal_pose_to_twist.cpp
)
ament_auto_add_executable(dummy_goal_node
    src/dummy_goal.cpp
)

ament_auto_package()

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

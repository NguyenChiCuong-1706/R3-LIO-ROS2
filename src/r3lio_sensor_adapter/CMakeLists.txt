cmake_minimum_required(VERSION 3.16)
project(r3lio_sensor_adapter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rigelslam_rot REQUIRED)

# PCL's Eigen CMake module exports this as a bare compiler argument.
remove_definitions(EIGEN_MPL2_ONLY)
add_compile_definitions(EIGEN_MPL2_ONLY)

add_executable(r3lio_sensor_adapter src/sensor_adapter.cpp)
target_compile_features(r3lio_sensor_adapter PRIVATE cxx_std_17)
ament_target_dependencies(r3lio_sensor_adapter rclcpp sensor_msgs livox_ros_driver2 rigelslam_rot)

install(TARGETS r3lio_sensor_adapter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
ament_package()

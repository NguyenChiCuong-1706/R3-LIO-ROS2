cmake_minimum_required(VERSION 3.16)
project(rigelslam_rot)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -Wall -march=native")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
if(CMAKE_BUILD_TYPE STREQUAL Release)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g")
endif()

option(BA_ENABLE "Enable mapping with bundle adjustment" OFF)
option(TEASER_ON "Enable teaser registration" OFF)
option(CERES_ON "Enable ceres optimization" ON)
option(G2O_ON "Enable g2o optimization" OFF)
option(GTSAM_ON "Enable gtsam optimization" OFF)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.7 REQUIRED)
# The preserved initializer uses ceres::LocalParameterization (removed in 2.2).
find_package(Ceres 2.1.0 EXACT REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Torch REQUIRED)
find_package(OpenMP)
find_library(GLOG_LIBRARY glog REQUIRED)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h REQUIRED)
find_library(ONNXRUNTIME_LIBRARY onnxruntime REQUIRED)

# Eigen's UseEigen3.cmake adds this token as a bare compiler argument.
# Retain its intended meaning as a preprocessor definition.
remove_definitions(EIGEN_MPL2_ONLY)
add_compile_definitions(EIGEN_MPL2_ONLY)

include(ProcessorCount)
ProcessorCount(R3LIO_PROCESSOR_COUNT)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  if(R3LIO_PROCESSOR_COUNT GREATER 4)
    add_compile_definitions(MP_EN MP_PROC_NUM=2)
  elseif(R3LIO_PROCESSOR_COUNT GREATER 3)
    add_compile_definitions(MP_EN MP_PROC_NUM=1)
  else()
    add_compile_definitions(MP_PROC_NUM=1)
  endif()
else()
  add_compile_definitions(MP_PROC_NUM=1)
endif()
if(BA_ENABLE)
  add_compile_definitions(BA_ENABLE)
endif()
if(TEASER_ON)
  find_package(teaserpp REQUIRED)
  add_compile_definitions(TEASER_ON)
endif()
if(CERES_ON)
  add_compile_definitions(CERES_ON)
endif()
if(G2O_ON)
  add_compile_definitions(G2O_ON)
endif()
if(GTSAM_ON)
  add_compile_definitions(GTSAM_ON)
endif()
add_compile_definitions(ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/" NOMINMAX)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Motor.msg" DEPENDENCIES std_msgs)

file(GLOB_RECURSE R3LIO_CORE_SOURCES CONFIGURE_DEPENDS "src/*.cpp" "src/*.cc")
list(FILTER R3LIO_CORE_SOURCES EXCLUDE REGEX "/BEVProcess/")
add_library(r3lio_core SHARED ${R3LIO_CORE_SOURCES})
target_include_directories(r3lio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR} ${TORCH_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/HesaiLidar_General_SDK/include)
foreach(R3LIO_SUBDIR DataIO Feature ImageProcess Initializer LoopCloser
    LoopCloser/ContourContext Mapper Matcher Misc Optimizer Registrator
    Registrator/super4pcs Registrator/fast_gicp SensorProcess Viewer xfeat
    MLSD BEVProcess patchwork)
  target_include_directories(r3lio_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${R3LIO_SUBDIR}>)
endforeach()
ament_target_dependencies(r3lio_core rclcpp sensor_msgs std_msgs geometry_msgs
  nav_msgs visualization_msgs tf2 tf2_ros cv_bridge pcl_conversions)
target_link_libraries(r3lio_core ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}
  ${TORCH_LIBRARIES} ${ONNXRUNTIME_LIBRARY}
  ${GLOG_LIBRARY} Ceres::ceres yaml-cpp gtsam)
if(OpenMP_CXX_FOUND)
  target_link_libraries(r3lio_core OpenMP::OpenMP_CXX)
endif()
if(TEASER_ON)
  target_link_libraries(r3lio_core teaserpp::teaser_registration teaserpp::teaser_io)
endif()

add_executable(r3lio_odometry Examples/ZGSlamRos2.cpp)
ament_target_dependencies(r3lio_odometry rclcpp sensor_msgs std_msgs
  geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros cv_bridge pcl_conversions)
target_link_libraries(r3lio_odometry r3lio_core ${GLOG_LIBRARY})
rosidl_get_typesupport_target(R3LIO_TYPESUPPORT ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(r3lio_odometry ${R3LIO_TYPESUPPORT})

install(TARGETS r3lio_core r3lio_odometry
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch weights DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_executable(test_zg_calibration_params test/test_zg_calibration_params.cpp)
  target_include_directories(test_zg_calibration_params PRIVATE include)
  target_link_libraries(test_zg_calibration_params Eigen3::Eigen)
  add_test(NAME zg_calibration_params COMMAND test_zg_calibration_params)
endif()
ament_package()

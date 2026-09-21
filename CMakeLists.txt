cmake_minimum_required(VERSION 3.16)
project(auto_handeye_calibration LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(industrial_calibration REQUIRED COMPONENTS core optimizations target_finders_opencv)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco calib3d imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_core
  src/math.cpp
  src/calibration.cpp)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  Eigen3::Eigen
  rclcpp::rclcpp
  yaml-cpp
  ${OpenCV_LIBS}
  industrial_calibration::industrial_calibration_core
  industrial_calibration::industrial_calibration_optimizations
  industrial_calibration::industrial_calibration_target_finders_opencv)

add_executable(auto_handeye_node src/auto_handeye_node.cpp)
target_compile_features(auto_handeye_node PUBLIC cxx_std_17)
target_compile_options(auto_handeye_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(auto_handeye_node ${PROJECT_NAME}_core)
ament_target_dependencies(auto_handeye_node
  cv_bridge geometry_msgs moveit_ros_planning_interface rclcpp sensor_msgs
  std_msgs std_srvs tf2_eigen tf2_ros)

install(TARGETS ${PROJECT_NAME}_core auto_handeye_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/analyze_session.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_math test/test_math.cpp)
  target_link_libraries(test_math ${PROJECT_NAME}_core)
  ament_add_gtest(test_synthetic_calibration test/test_synthetic_calibration.cpp)
  target_link_libraries(test_synthetic_calibration ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()

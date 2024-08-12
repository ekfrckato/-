cmake_minimum_required(VERSION 3.8)
project(vision_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_executable(run_ddsm src/run_ddsm.cpp src/ddsm_operator_def.cpp)
ament_target_dependencies(run_ddsm rclcpp std_msgs geometry_msgs)

add_executable(run_cm src/run_opencm.cpp src/motor_driver.cpp)
ament_target_dependencies(run_cm rclcpp std_msgs geometry_msgs dynamixel_sdk)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  run_ddsm
  run_cm
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

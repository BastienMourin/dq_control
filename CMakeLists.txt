cmake_minimum_required(VERSION 3.5)
project(dq_control)

find_package(ament_cmake REQUIRED)
find_package(dq_operations REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(controller SHARED src/controller.cpp)
ament_target_dependencies(controller dq_operations rclcpp Eigen3)

add_library(single_arm_position_controller SHARED src/single_arm_position_controller.cpp)
target_include_directories(single_arm_position_controller PUBLIC include)
ament_target_dependencies(single_arm_position_controller dq_operations rclcpp Eigen3)
target_link_libraries(single_arm_position_controller controller)

install(TARGETS controller single_arm_position_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

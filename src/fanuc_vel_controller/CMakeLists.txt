cmake_minimum_required(VERSION 3.8)
project(fanuc_vel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python executables
install(PROGRAMS
  scripts/robot_jacobian_eg.py
  scripts/simple_twist_pub.py
  scripts/aruco_node.py
  fanuc_vel_controller/ee_vel_cmd.py
  fanuc_vel_controller/fanuc_model.py
  fanuc_vel_controller/current_cart.py 
  fanuc_vel_controller/pose_vel_cmd.py 
  fanuc_vel_controller/aruco_tracker.py 
  DESTINATION lib/${PROJECT_NAME}
)

# ament_python_install_package("fanuc_vel_controller")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

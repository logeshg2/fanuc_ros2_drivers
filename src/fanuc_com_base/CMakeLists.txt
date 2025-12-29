cmake_minimum_required(VERSION 3.8)
project(fanuc_com_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package("ComDependencies")
# install(DIRECTORY ComDependencies DESTINATION lib/${PROJECT_NAME})

ament_package()

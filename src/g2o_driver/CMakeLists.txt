cmake_minimum_required(VERSION 3.8)
project(g2o_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(driver src/driver.cpp)
ament_target_dependencies(driver
  rclcpp
)

install(TARGETS driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()

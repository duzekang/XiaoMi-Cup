cmake_minimum_required(VERSION 3.8)
project(control)

# 强制设置C++17标准
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)  # 直接使用系统安装的Eigen
find_package(lcm REQUIRED)
find_package(Threads REQUIRED)  # 添加线程库查找
find_package(cyberdog_msg REQUIRED)
find_package(yaml-cpp REQUIRED)

# 添加可执行文件
add_executable(slope_controller 
  src/slope_controller.cpp
)
add_executable(stand 
  src/stand.cpp
)
add_executable(walk_straight 
  src/walk_straight.cpp
)
add_executable(complete_process 
  src/complete_process.cpp
)

# 链接库
target_link_libraries(slope_controller
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  Threads::Threads
  lcm
  Eigen3::Eigen
)
target_link_libraries(walk_straight
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  lcm
  Eigen3::Eigen
)

target_link_libraries(stand
  ${rclcpp_LIBRARIES}
  Threads::Threads 
  lcm
)

target_link_libraries(complete_process
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  Threads::Threads
  lcm
  Eigen3::Eigen
)

# 包含目录
target_include_directories(slope_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}  # 新增
  ${geometry_msgs_INCLUDE_DIRS}
  include/
)

include_directories(
  ${YAML_CPP_INCLUDE_DIRS}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装YAML配置文件
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装
install(TARGETS slope_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS walk_straight
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS stand
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS complete_process
  DESTINATION lib/${PROJECT_NAME}
)




ament_export_dependencies(
  rclcpp
  sensor_msgs
  eigen3_cmake_module
  lcm
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

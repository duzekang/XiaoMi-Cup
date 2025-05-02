cmake_minimum_required(VERSION 3.8)
project(control)

# 强制设置C++17标准
set(CMAKE_CXX_STANDARD 17
)
set(CMAKE_CXX_STANDARD_REQUIRED ON
)
set(CMAKE_CXX_EXTENSIONS OFF
)

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

# 添加可执行文件
add_executable(slope_controller 
  src/slope_controller.cpp
)

add_executable(stand 
  src/stand.cpp
)

# 链接库
target_link_libraries(slope_controller
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}     # 新增
  ${geometry_msgs_LIBRARIES}   # 新增
  Threads::Threads  # 添加pthread链接
  lcm
  Eigen3::Eigen
)

target_link_libraries(stand
  ${rclcpp_LIBRARIES}
  Threads::Threads  # 添加pthread链接
  lcm
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

# 安装
install(TARGETS slope_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS stand
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

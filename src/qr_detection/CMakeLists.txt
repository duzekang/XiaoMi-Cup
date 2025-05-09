cmake_minimum_required(VERSION 3.8)
project(qr_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBar REQUIRED IMPORTED_TARGET zbar)

# 添加可执行文件
add_executable(qr_detector src/qr_detector.cpp)
target_include_directories(qr_detector PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${ZBar_INCLUDE_DIRS}
)
target_include_directories(qr_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${ZBar_INCLUDE_DIRS}
)
target_link_libraries(qr_detector
  ${rclcpp_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  PkgConfig::ZBar
)

# 安装目标
install(TARGETS qr_detector
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
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

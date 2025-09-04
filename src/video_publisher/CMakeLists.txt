cmake_minimum_required(VERSION 3.8)
project(video_publisher)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 添加可执行文件
add_executable(talker src/publisher_video.cpp)# talker为节点名

# 添加依赖
ament_target_dependencies(talker 
OpenCV  
cv_bridge
rclcpp 
sensor_msgs
)
# 链接库
target_link_libraries(talker ${OpenCV_LIBS} cv_bridge::cv_bridge) # 在target_link_libraries中添加

# 安装目标
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME}
)

# # 安装启动文件
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

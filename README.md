# ArmorDetector_ros2 Demo
## 项目概述
这是一个基于 ROS2 和 OpenCV 的装甲板识别项目。
项目包含两个 ROS2 节点：
1. **视频发布节点** (`video_publisher`): 读取本地视频文件并发布为 ROS2 图像话题
2. **视频处理节点** (`video_subscriber`): 订阅图像话题，进行装甲板检测和处理，并发布处理后的视频，采用rpt可视化视频
## 项目结构
```
ArmorDetector_ros2/
├── src/
│   ├── video_publisher/
│   │   ├── src/
│   │   │   └── publisher_video.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── video_subscriber/
│       ├── src/
│       │   └── subscriber_video.cpp
│       ├── launch/
│       │   └── launch.py
│       ├── CMakeLists.txt
│       └── package.xml
├── build/
├── install/
└── log/
```
## 功能特点
- 从视频文件读取帧并发布到 ROS2 话题
- 使用 HSV 颜色空间识别红色装甲板
- 基于轮廓分析和几何特征匹配灯条对
- 检测并标记装甲板的四个角点
## 安装和配置
### 1. 创建工作空间
```bash
mkdir -p ~/ArmorDetector_ros2/src
cd ~/ArmorDetector_ros2/src
```
### 2. 创建功能包
```bash
# 创建视频发布功能包video_publisher
ros2 pkg create video_publisher --build-type ament_cmake --dependencies rclcpp sensor_msgs cv_bridge opencv4

# 创建视频处理功能包video_subscriber
ros2 pkg create video_subscriber --build-type ament_cmake --dependencies rclcpp sensor_msgs cv_bridge opencv4
```
### 3. 添加可执行文件
将提供的 C++ 源代码分别添加到对应功能包的 `src` 目录中：
- `video_publisher/src/publisher_video.cpp`
- `video_subscriber/src/subscriber_video.cpp`
### 4. 添加启动文件
在 `video_subscriber` 功能包中创建 `launch` 目录，并添加 `launch.py` 文件。
### 5. 配置 CMakeLists.txt
对于每个功能包，确保 `CMakeLists.txt` 包含以下内容：
**video_publisher/CMakeLists.txt**:
```cmake
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(talker src/publisher_video.cpp)
ament_target_dependencies(talker rclcpp sensor_msgs cv_bridge)
target_link_libraries(talker ${OpenCV_LIBS} cv_bridge::cv_bridge)

install(TARGETS talker DESTINATION lib/${PROJECT_NAME})
```
**video_subscriber/CMakeLists.txt**:
```cmake
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(listener src/subscriber_video.cpp)
ament_target_dependencies(listener rclcpp sensor_msgs cv_bridge)
target_link_libraries(listener ${OpenCV_LIBS} cv_bridge::cv_bridge)

install(TARGETS listener DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
```
### 6. 配置 package.xml
确保两个功能包的 `package.xml` 包含以下依赖：
```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>image_transport</depend>
```
对于 `video_subscriber`，还需要添加：
```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```
## 构建项目
```bash
cd ~/ArmorDetector_ros2
# 在构建之前，最好在您的工作空间根目录（ArmorDetector_ros2）中运行``rosdep``来检查是否存在缺失的依赖项：
rosdep install -i --from-path src --rosdistro humble -y
# 在工作空间根目录（ArmorDetector_ros2）中，构建新软件包：
colcon build --packages-select video_publisher
colcon build --packages-select video_subscriber
source install/setup.bash
```
## 运行演示
### 1. 使用 launch 文件同时启动两个节点
```bash
# 进入之前创建的目录ArmorDetector_ros2/src/video_subscriber/launch并运行以下命令：
cd launch
ros2 launch launch.py
```
### 2. 可视化结果
打开新终端，使用 RQT 图像查看工具查看处理后的视频：
```bash
source ~/ArmorDetector_ros2/install/setup.bash
# 查看原始视频
rqt_image_view --topic /video_frames
# 查看处理后的视频
rqt_image_view --topic /processed_video_frames
```
## 检测算法说明
装甲板检测算原理及流程参考ArmorDetector（仅基于opencv）

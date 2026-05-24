# =============================================================================
# 激光雷达循迹ROS2包构建配置文件
# 功能：定义包的构建规则、依赖关系和安装配置
# 作者：zyh
# 版本：1.0
# 日期：2025-08-24
# =============================================================================

# 设置CMake最低版本要求
# 注意：ROS2 Humble需要CMake 3.8或更高版本
cmake_minimum_required(VERSION 3.8)

# 定义项目名称
# 这个名称将用于生成可执行文件和安装路径
project(lidar_tracking)

# =============================================================================
# 查找和配置ROS2依赖包
# =============================================================================

# 查找ament_cmake构建工具包（必需）
# ament_cmake是ROS2的标准构建系统
find_package(ament_cmake REQUIRED)

# 查找geometry_msgs消息包（必需）
# 用于发布/订阅Twist消息（速度控制指令）
find_package(geometry_msgs REQUIRED)

# 查找rclcpp客户端库（必需）
# ROS2的C++客户端库，提供节点创建和通信功能
find_package(rclcpp REQUIRED)

# 查找sensor_msgs消息包（必需）
# 用于订阅LaserScan消息（激光雷达扫描数据）
find_package(sensor_msgs REQUIRED)

# 查找std_msgs消息包（必需）
# 提供标准消息类型支持
find_package(std_msgs REQUIRED)

# =============================================================================
# 配置包含目录
# =============================================================================

# 设置头文件包含路径
include_directories(
  include                                    # 项目自定义头文件目录
  ${ament_INCLUDE_DIRS}                     # ROS2系统头文件目录
)

# =============================================================================
# 创建可执行文件
# =============================================================================

# 定义可执行文件
# 源文件：src/lidar_tracking.cpp
# 输出文件：lidar_tracking
add_executable(lidar_tracking src/lidar_tracking.cpp)

# =============================================================================
# 配置依赖库链接
# =============================================================================

# 链接ROS2依赖库到可执行文件
# 这些库提供消息类型定义和ROS2运行时支持
ament_target_dependencies(lidar_tracking
  geometry_msgs    # 几何消息库（Twist等）
  rclcpp          # ROS2 C++客户端库
  sensor_msgs     # 传感器消息库（LaserScan等）
  std_msgs        # 标准消息库
)

# =============================================================================
# 配置安装规则
# =============================================================================

# 安装可执行文件
# 目标：lidar_tracking可执行文件
# 安装位置：lib/lidar_tracking/（相对于安装前缀）
install(TARGETS
  lidar_tracking
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件目录（如果存在）
# 检查launch目录是否存在，如果存在则安装到share/lidar_tracking/launch/
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch                                    # 源目录：项目根目录下的launch文件夹
    DESTINATION share/${PROJECT_NAME}         # 目标目录：share/lidar_tracking/launch/
    USE_SOURCE_PERMISSIONS
  )
endif()

# 安装config目录（新增：安装参数文件）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
  )
endif()

# =============================================================================
# 导出包配置
# =============================================================================

# 导出ROS2包配置
# 生成package.xml中定义的依赖关系，供其他包使用
ament_package()
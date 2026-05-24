#include <iostream>  // 引入输入输出库
#include <geometry_msgs/msg/pose_stamped.hpp>  // 引入定位数据的消息类型
#include <geometry_msgs/msg/twist.hpp>  // 引入速度命令数据的消息类型
#include <nav_msgs/msg/path.hpp>  // 引入路径数据的消息类型
#include <nav_msgs/msg/odometry.hpp>  // 引入里程计数据的消息类型
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // 引入转换库，用于处理坐标变换
#include <std_msgs/msg/int16.hpp>  // 引入16位整数的消息类型
#include <std_msgs/msg/float64.hpp>  // 引入64位浮点数的消息类型
#include <sensor_msgs/msg/imu.hpp>  // 引入IMU传感器数据的消息类型
#include <tf2_msgs/msg/tf_message.hpp>  // 引入tf消息
#include <tf2_ros/transform_broadcaster.h>  // 引入坐标转换广播器
#include <tf2_ros/transform_listener.h>  // 引入坐标转换监听器
#include <tf2/transform_datatypes.h>  // 引入转换数据类型
#include <tf2/LinearMath/Quaternion.h>  // 引入四元数数学库
#include <tf2/LinearMath/Matrix3x3.h>  // 引入3x3矩阵数学库
#include <message_filters/sync_policies/approximate_time.h>  // 引入近似时间同步策略
#include <message_filters/sync_policies/exact_time.h>  // 引入精确时间同步策略
#include <message_filters/subscriber.h>  // 引入消息过滤订阅者
#include <message_filters/synchronizer.h>  // 引入消息同步器
#include "rclcpp/rclcpp.hpp"  // 引入ROS 2 C++客户端库
#include "nav_msgs/msg/path.hpp"  // 引入路径数据的消息类型
#include "geometry_msgs/msg/pose_stamped.hpp"  // 引入带时间戳的定位数据类型
#include "geometry_msgs/msg/twist.hpp"  // 引入速度指令数据类型
#include "nav_msgs/msg/odometry.hpp"  // 引入里程计数据的消息类型
#include "tf2_ros/transform_listener.h"  // 引入tf监听器
#include "tf2_ros/buffer.h"  // 引入tf缓冲区
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"  // 引入tf2与几何消息的转换支持
#include "visualization_msgs/msg/marker.hpp"  // 引入可视化标记消息
#include "std_msgs/msg/float64.hpp"  // 引入64位浮点数消息类型
#include "std_msgs/msg/string.hpp"  // 引入字符串消息类型
#include <cmath>  // 引入数学库

#define PI 3.14159265358979  // 定义圆周率常量

// 初始化变量
double last_steeringangle = 0;  // 上一个转向角度
double L, Lfw, Lrv, Lfw_, Vcmd, lfw, lrv, steering, u, v;  // 车辆参数及控制变量
double Gas_gain, baseAngle, baseSpeed, Angle_gain_p, Angle_gain_d, goalRadius;  // 控制器参数
int controller_freq;  // 控制器频率
bool foundForwardPt, goal_received, goal_reached;  // 目标点是否找到，目标是否接收到，目标是否到达
double k_rou;  // 路径跟踪控制参数
double vp_max_base, vp_min;  // 最大最小速度参数
double stop_flag = 0.0;  // 停车标志
int mapPathNum;  // 路径点数
double slow_final, fast_final;  // 速度控制相关参数
int stopIdx = 0;  // 停止位置索引
double line_wight = 0.0;  // 车道线宽度
double initbaseSpeed;  // 初始基本速度
double obs_flag = 0.0;  // 障碍物标志
bool traffic_flag = true;  // 红绿灯标志

// L1控制器类定义
class L1Controller : public rclcpp::Node
{
public:
  L1Controller();  // 构造函数
  void initMarker();  // 初始化标记
  bool isWayPtAwayFromLfwDist(const geometry_msgs::msg::Point &wayPt, const geometry_msgs::msg::Point &car_pos);  // 判断目标点是否远离前轴
  bool isForwardWayPt(const geometry_msgs::msg::Point &wayPt, const geometry_msgs::msg::Pose &carPose);  // 判断目标点是否在前方
  double getYawFromPose(const geometry_msgs::msg::Pose &carPose);  // 从定位数据中获取航向角
  double getEta(const geometry_msgs::msg::Pose &carPose);  // 获取与目标点的偏航角
  double getCar2GoalDist();  // 获取车辆与目标点的距离
  double getL1Distance(const double &_Vcmd);  // 获取L1控制距离
  double getSteeringAngle(double eta);  // 获取转向角
  double getGasInput(const float &current_v);  // 获取油门输入
  double isline(double line_wight);  // 判断是否为车道线

  geometry_msgs::msg::Point get_odom_car2WayPtVec(const geometry_msgs::msg::Pose &carPose);  // 获取车辆与路径点的位移向量

private:
  // 订阅者定义
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub, encoder_sub;  // 里程计订阅者和编码器订阅者
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;  // 路径订阅者
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;  // 目标位姿订阅者
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr final_goal_sub, line_sub;  // 终点和车道线宽度订阅者
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traffic_light_sub;  // 红绿灯状态订阅者

  // 发布者定义
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;  // 速度命令发布者
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;  // 可视化标记发布者

  // 定时器定义
  rclcpp::TimerBase::SharedPtr timer1, timer2;  // 定时器1和定时器2

  // tf相关
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  // tf监听器
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  // tf缓冲区

  // 可视化标记
  visualization_msgs::msg::Marker points, line_strip, goal_circle;  // 点，线条和目标圆圈标记

  // 控制命令
  geometry_msgs::msg::Twist cmd_vel;  // 控制命令

  // 其他
  geometry_msgs::msg::Point odom_goal_pos;  // 目标位置
  nav_msgs::msg::Odometry odom, encoder;  // 里程计和编码器数据
  nav_msgs::msg::Path map_path, odom_path;  // 路径数据
  geometry_msgs::msg::PoseStamped goal_pos;  // 目标位姿
  rclcpp::Time current_time;  // 当前时间

  // 回调函数
  void odomCB(const nav_msgs::msg::Odometry::SharedPtr odomMsg);  // 里程计回调
  void pathCB(const nav_msgs::msg::Path::SharedPtr pathMsg);  // 路径回调
  void encoderCB(const nav_msgs::msg::Odometry::SharedPtr encoderMsg);  // 编码器回调
  void goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalMsg);  // 目标回调
  void goalReachingCB();  // 目标到达回调
  void controlLoopCB();  // 控制回路回调
  void stopCB(const std_msgs::msg::Float64::SharedPtr stopMsg);  // 停止回调
  void lineCB(const std_msgs::msg::Float64::SharedPtr lineMsg);  // 车道线回调
  void lightCB(const std_msgs::msg::String::SharedPtr msg);  // 红绿灯回调
};


L1Controller::L1Controller() : Node("art_car_controller")
{
  // 车辆参数
  this->declare_parameter("L", 0.305);  // 车辆前后轴距
  this->declare_parameter("Vcmd", 1.0);  // 车辆命令速度
  this->declare_parameter("lfw", 0.1675); // 车辆前轮到车体中心的距离
  this->declare_parameter("lrv", 10.0);  // 车辆转弯半径
  this->declare_parameter("Lrv", 10.0);  // 转弯半径的某些参数

  // 控制器参数
  this->declare_parameter("controller_freq", 30); // 控制器的频率
  this->declare_parameter("Angle_gain_p", -1.0);  // 控制器的角度P增益
  this->declare_parameter("Angle_gain_d", -0.0);  // 控制器的角度D增益
  this->declare_parameter("baseSpeed", 0.0);  // 基础速度
  this->declare_parameter("baseAngle", 90.0);  // 基础角度
  this->declare_parameter("k_rou", 0.0);  // 可能与路径规划或控制相关的参数
  this->declare_parameter("vp_max_base", 0.0); // 最大基准速度
  this->declare_parameter("vp_min", 0.0); // 最小速度
  this->declare_parameter("goalRadius", 0.5); // 目标半径
  this->declare_parameter("Lfw", 0.3); // 与车辆前轮相关的参数
  this->declare_parameter("slow_final", 1.0);  // 最终减速比例
  this->declare_parameter("fast_final", 1.0);  // 最终加速比例

  // 获取参数值
  this->get_parameter("L", L);
  this->get_parameter("Vcmd", Vcmd);
  this->get_parameter("lfw", lfw);
  this->get_parameter("lrv", lrv);
  this->get_parameter("Lrv", Lrv);
  this->get_parameter("controller_freq", controller_freq);
  this->get_parameter("Angle_gain_p", Angle_gain_p);
  this->get_parameter("Angle_gain_d", Angle_gain_d);
  this->get_parameter("baseSpeed", baseSpeed);
  this->get_parameter("baseAngle", baseAngle);
  this->get_parameter("k_rou", k_rou);
  this->get_parameter("vp_max_base", vp_max_base);
  this->get_parameter("vp_min", vp_min);
  this->get_parameter("goalRadius", goalRadius);
  this->get_parameter("Lfw", Lfw);
  this->get_parameter("slow_final", slow_final);
  this->get_parameter("fast_final", fast_final);

  initbaseSpeed = baseSpeed;  // 初始化基础速度
  current_time = rclcpp::Clock().now();  // 当前时间
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_combined", 1, std::bind(&L1Controller::odomCB, this, std::placeholders::_1));  // 订阅车辆里程计
  path_sub = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", 1, std::bind(&L1Controller::pathCB, this, std::placeholders::_1));  // 订阅路径规划
  goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&L1Controller::goalCB, this, std::placeholders::_1));  // 订阅目标位置
  final_goal_sub = this->create_subscription<std_msgs::msg::Float64>(
      "/arrfinal", 1, std::bind(&L1Controller::stopCB, this, std::placeholders::_1));  // 订阅是否到达最终目标
  line_sub = this->create_subscription<std_msgs::msg::Float64>(
      "/line_wight", 1, std::bind(&L1Controller::lineCB, this, std::placeholders::_1));  // 订阅路径线宽
  traffic_light_sub = this->create_subscription<std_msgs::msg::String>(
      "/light", 1, std::bind(&L1Controller::lightCB, this, std::placeholders::_1));  // 订阅交通信号灯

  pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/car_cmd_vel", 1);  // 发布车辆速度命令
  marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("car_path", 10);  // 发布路径可视化信息

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());  // 创建TF缓存
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);  // 创建TF监听器

  // 定时器，控制回调
  timer1 = this->create_wall_timer(std::chrono::duration<double>(2.0 / controller_freq),
                                   std::bind(&L1Controller::controlLoopCB, this));  // 定时器，用于控制循环
  timer2 = this->create_wall_timer(std::chrono::duration<double>(1.0 / controller_freq),
                                   std::bind(&L1Controller::goalReachingCB, this));  // 定时器，用于目标达成检测
}

void L1Controller::initMarker()
{
  points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";  // 设置坐标系
  points.ns = line_strip.ns = goal_circle.ns = "Markers";  // 设置命名空间
  points.action = line_strip.action = goal_circle.action = visualization_msgs::msg::Marker::ADD;  // 设置标记类型
  points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;  // 设置旋转方向
  points.id = 0;  // 设置标记ID
  line_strip.id = 1;  // 设置标记ID
  goal_circle.id = 2;  // 设置目标圆标记ID

  points.type = visualization_msgs::msg::Marker::POINTS;  // 设置标记类型为点
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;  // 设置标记类型为线条
  goal_circle.type = visualization_msgs::msg::Marker::CYLINDER;  // 设置目标标记类型为圆柱

  points.scale.x = 0.2;  // 设置点的大小
  points.scale.y = 0.2;  // 设置点的大小

  line_strip.scale.x = 0.1;  // 设置线条宽度

  goal_circle.scale.x = goalRadius;  // 设置目标圆的半径
  goal_circle.scale.y = goalRadius;  // 设置目标圆的半径
  goal_circle.scale.z = 0.1;  // 设置目标圆的高度

  points.color.g = 1.0f;  // 设置点的颜色为绿色
  points.color.a = 1.0;  // 设置点的透明度为不透明

  line_strip.color.b = 1.0;  // 设置线条颜色为蓝色
  line_strip.color.a = 1.0;  // 设置线条透明度为不透明

  goal_circle.color.r = 1.0;  // 设置目标圆的红色分量
  goal_circle.color.g = 1.0;  // 设置目标圆的绿色分量
  goal_circle.color.b = 0.0;  // 设置目标圆的蓝色分量
  goal_circle.color.a = 0.5;  // 设置目标圆的透明度为半透明
}

void L1Controller::odomCB(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
  odom = *odomMsg;  // 更新里程计信息
}

void L1Controller::lightCB(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "red")  // 如果交通信号灯为红灯
  {
    traffic_flag = false;  // 设置交通标志为停止
  }
  else if (msg->data == "green")  // 如果交通信号灯为绿灯
  {
    traffic_flag = true;  // 设置交通标志为通行
  }
  else  // 其他情况，默认为通行
  {
    traffic_flag = true;
  }
}


// pathCB回调函数，处理接收到的路径消息
void L1Controller::pathCB(const nav_msgs::msg::Path::SharedPtr pathMsg)
{
  static int pathCBidx = 0;  // 用于记录路径回调的次数
  static nav_msgs::msg::Path last_map_path;  // 保存上一次接收到的路径

  // 检查并设置路径消息的 frame_id
  if (pathMsg->header.frame_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received path with empty frame_id, setting default frame_id 'map'");
    pathMsg->header.frame_id = "map";  // 如果 frame_id 为空，设置默认的 frame_id 为 'map'
  }

  // 检查并设置每个路径点的 frame_id
  for (auto& pose : pathMsg->poses) {
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = pathMsg->header.frame_id;  // 如果路径点的 frame_id 为空，设置为路径的 frame_id
    }
  }

  if (pathCBidx == 0)  // 如果是第一次回调
  {
    last_map_path.poses.clear();  // 清空上次的路径数据
  }

  map_path = *pathMsg;  // 保存当前接收到的路径
  mapPathNum = map_path.poses.size();  // 计算路径点的数量

  if (map_path.poses.size() <= 0)  // 如果接收到的路径为空
  {
    RCLCPP_WARN(this->get_logger(), "Received empty path, using last known path with %d poses", (int)last_map_path.poses.size());
    // 如果路径为空，使用上一次的路径
    for (int i = 0; i < last_map_path.poses.size(); i++)
    {
      map_path.poses.push_back(last_map_path.poses[i]);  // 把上一次的路径点加入当前路径
    }
  }
  else
  {
    last_map_path.poses.clear();  // 清空上一次的路径
    for (int i = 0; i < map_path.poses.size(); i++)
    {
      last_map_path.poses.push_back(map_path.poses[i]);  // 保存当前路径为上次路径
    }
  }
  pathCBidx++;  // 增加回调计数
}

// encoderCB回调函数，处理接收到的编码器数据
void L1Controller::encoderCB(const nav_msgs::msg::Odometry::SharedPtr encoderMsg)
{
  encoder = *encoderMsg;  // 保存编码器数据
}

// stopCB回调函数，处理停止命令
void L1Controller::stopCB(const std_msgs::msg::Float64::SharedPtr stopMsg)
{
  stop_flag = stopMsg->data;  // 更新停止标志
}

// lineCB回调函数，处理路径中的线性权重数据
void L1Controller::lineCB(const std_msgs::msg::Float64::SharedPtr lineMsg)
{
  line_wight = lineMsg->data;  // 更新线性权重
}

// goalCB回调函数，处理目标点信息
void L1Controller::goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalMsg)
{
  geometry_msgs::msg::PoseStamped odom_goal;  // 用于存储目标点的odom坐标
  goal_pos = *goalMsg;  // 保存目标点消息
  current_time = rclcpp::Clock().now();  // 获取当前时间
  RCLCPP_INFO(this->get_logger(), "Goal received and transformed to odom frame");

  try
  {
    // 转换目标点坐标到odom坐标系
    tf_buffer_->transform(*goalMsg, odom_goal, "odom", tf2::durationFromSec(1.0));

    odom_goal_pos = odom_goal.pose.position;  // 获取转换后的目标位置
    goal_received = true;  // 标记目标已接收
    goal_reached = false;  // 标记目标尚未到达
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());  // 错误日志输出
    RCLCPP_INFO(this->get_logger(),"goal error sleep");  // 日志输出转换错误
  }
}

// 获取当前姿态的偏航角（yaw）
double L1Controller::getYawFromPose(const geometry_msgs::msg::Pose &carPose)
{
  double roll, pitch, yaw;  // 定义滚转、俯仰和偏航角
  tf2::Quaternion q(carPose.orientation.x, carPose.orientation.y, carPose.orientation.z, carPose.orientation.w);  // 创建四元数
  tf2::Matrix3x3 m(q);  // 从四元数创建旋转矩阵
  m.getRPY(roll, pitch, yaw);  // 从旋转矩阵获取滚转、俯仰、偏航角
  return yaw;  // 返回偏航角
}

// 判断目标点是否在前方
bool L1Controller::isForwardWayPt(const geometry_msgs::msg::Point &wayPt,
                                  const geometry_msgs::msg::Pose &carPose)
{
  float car2wayPt_x = wayPt.x - carPose.position.x;  // 计算车辆与目标点的X轴距离
  float car2wayPt_y = wayPt.y - carPose.position.y;  // 计算车辆与目标点的Y轴距离
  double car_theta = getYawFromPose(carPose);  // 获取车辆的偏航角

  // 将目标点相对车辆坐标系的坐标计算出来
  float car_car2wayPt_x = cos(car_theta) * car2wayPt_x + sin(car_theta) * car2wayPt_y;
  float car_car2wayPt_y = -sin(car_theta) * car2wayPt_x + cos(car_theta) * car2wayPt_y;
  
  return car_car2wayPt_x > 0;  // 判断目标点是否在车辆前方
}

// 判断目标点是否离车辆一定距离
bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::msg::Point &wayPt, const geometry_msgs::msg::Point &car_pos)
{
  double dx = wayPt.x - car_pos.x;  // 计算目标点和车辆X轴距离
  double dy = wayPt.y - car_pos.y;  // 计算目标点和车辆Y轴距离
  double dist = sqrt(dx * dx + dy * dy);  // 计算目标点和车辆的欧几里得距离
  return dist >= Lfw;  // 判断距离是否大于预定的Lfw
}


geometry_msgs::msg::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::msg::Pose &carPose)
{
  geometry_msgs::msg::Point carPose_pos = carPose.position;  // 获取车辆位置
  double carPose_yaw = getYawFromPose(carPose);  // 获取车辆航向角（偏航角）
  geometry_msgs::msg::Point forwardPt;  // 用于存储目标点
  geometry_msgs::msg::Point odom_car2WayPtVec;  // 用于存储从车辆到目标点的相对坐标
  foundForwardPt = false;  // 初始化标志位，表示是否找到了前方目标点
  double car2goal_dist = getCar2GoalDist();  // 计算车辆到目标的距离
  bool start_flag = false;  // 标志位，表示是否开始寻找前方目标点

  if (!goal_reached)  // 如果目标尚未到达
  {
    for (int i = 0; i < map_path.poses.size(); i++)  // 遍历路径上的所有点
    {
      geometry_msgs::msg::PoseStamped map_path_pose = map_path.poses[i];  // 获取路径上的每个点
      geometry_msgs::msg::PoseStamped odom_path_pose;  // 用于存储从地图坐标系转换到里程计坐标系的坐标
      try
      {
        
        tf_buffer_->transform(map_path_pose, odom_path_pose, "odom", tf2::durationFromSec(1.0));  // 将地图路径点从地图坐标系转换到里程计坐标系
        geometry_msgs::msg::Point odom_path_wayPt = odom_path_pose.pose.position;  // 获取转换后的目标点位置
        bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt, carPose);  // 判断目标点是否在前方

        if (_isForwardWayPt && !start_flag)  // 如果目标点在前方且尚未开始寻找目标点
        {
          start_flag = true;  // 设置为已开始
        }
        if (!start_flag)  // 如果还没有开始，跳过此点
        {
          continue;
        }

        if (_isForwardWayPt)  // 如果目标点在前方
        {
          bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt, carPose_pos);  // 判断目标点是否超出车辆的前方距离
          if (_isWayPtAwayFromLfwDist)  // 如果超出前方距离
          {
            forwardPt = odom_path_wayPt;  // 更新前方目标点
            foundForwardPt = true;  // 设置已找到目标点
            break;  // 跳出循环，停止寻找
          }
        }

        if (car2goal_dist < Lfw)  // 如果车辆距离目标点小于某个阈值
        {
          forwardPt = odom_goal_pos;  // 设置目标点为目标位置
          foundForwardPt = true;  // 设置已找到目标点
        }
      }
      catch (tf2::TransformException &ex)  // 捕获坐标转换异常
      {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());  // 输出错误信息
        RCLCPP_INFO( this->get_logger(),"path error sleep");  // 输出路径错误信息
      }
    }
  }
  else if (goal_reached)  // 如果目标已经到达
  {
    forwardPt = odom_goal_pos;  // 设置目标点为目标位置
    foundForwardPt = false;  // 设置未找到前方目标点
  }

  points.points.clear();  // 清空显示用的点数据
  line_strip.points.clear();  // 清空线段数据

  if (foundForwardPt && !goal_reached)  // 如果找到了前方目标点且目标未到达
  {
    points.points.push_back(carPose_pos);  // 添加车辆当前位置
    points.points.push_back(forwardPt);  // 添加前方目标点
    line_strip.points.push_back(carPose_pos);  // 添加车辆当前位置
    line_strip.points.push_back(forwardPt);  // 添加前方目标点
  }

  marker_pub->publish(points);  // 发布点数据
  marker_pub->publish(line_strip);  // 发布线段数据

  odom_car2WayPtVec.x = cos(carPose_yaw) * (forwardPt.x - carPose_pos.x) + sin(carPose_yaw) * (forwardPt.y - carPose_pos.y);  // 计算车辆到目标点的x方向相对坐标
  odom_car2WayPtVec.y = -sin(carPose_yaw) * (forwardPt.x - carPose_pos.x) + cos(carPose_yaw) * (forwardPt.y - carPose_pos.y);  // 计算车辆到目标点的y方向相对坐标
  return odom_car2WayPtVec;  // 返回计算出的相对坐标
}

double L1Controller::getEta(const geometry_msgs::msg::Pose &carPose)
{
  geometry_msgs::msg::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);  // 获取车辆到目标点的相对坐标
  double eta = atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);  // 计算车辆与目标点的夹角（方位角）
  RCLCPP_INFO(this->get_logger(), "Calculated eta: %.2f degrees", eta*57.3);  // 输出计算得到的方位角（转换为角度）
  return eta;  // 返回方位角
}

double L1Controller::getCar2GoalDist()
{
  geometry_msgs::msg::Point car_pose = odom.pose.pose.position;  // 获取车辆当前位置
  double car2goal_x = odom_goal_pos.x - car_pose.x;  // 计算车辆与目标点的x方向距离
  double car2goal_y = odom_goal_pos.y - car_pose.y;  // 计算车辆与目标点的y方向距离
  double dist = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);  // 计算车辆与目标点的直线距离
  RCLCPP_INFO(this->get_logger(), "Calculated car to goal distance: %.2f meters", dist);  // 输出计算的车辆与目标的距离
  return dist;  // 返回距离
}

double L1Controller::getL1Distance(const double &_Vcmd)
{
  double L1 = 0;  // 初始化L1值
  double car2goal_dist = getCar2GoalDist();  // 获取车辆到目标的距离
  double v = _Vcmd;  // 获取车辆当前的速度
  
  L1 = 1.45;  // 示例固定值，通常基于速度Vcmd来计算L1
  
  return L1;  // 返回L1
}

double L1Controller::getSteeringAngle(double eta)
{
  double car2goal_dist = getCar2GoalDist();  // 获取车辆到目标的距离

  double steeringAngle = atan2((L * sin(eta)), ((car2goal_dist / 2) + lfw * cos(eta))) * (180.0 / PI);  // 计算转向角

  RCLCPP_INFO(this->get_logger(), "Calculated steering angle: %.2f degrees", steeringAngle);  // 输出计算的转向角
  return steeringAngle;  // 返回转向角
}

void L1Controller::goalReachingCB()
{
  if (1)  // 此条件始终为真
  {
    try
    {
      geometry_msgs::msg::PoseStamped odom_goal;  // 用于存储目标位姿
      current_time = rclcpp::Clock().now();  // 获取当前时间
      goal_pos.header.stamp = current_time;  // 设置目标位置的时间戳
      tf_buffer_->transform(goal_pos, odom_goal, "odom", tf2::durationFromSec(2.0));  // 将目标位置从地图坐标系转换到里程计坐标系
      RCLCPP_INFO( this->get_logger(),"yes sleep");  // 输出信息
      odom_goal_pos = odom_goal.pose.position;  // 更新目标位置
    }
    catch (tf2::TransformException &ex)  // 捕获坐标转换异常
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());  // 输出错误信息
      RCLCPP_INFO( this->get_logger(),"error sleep");  // 输出错误信息
    }
    double car2goal_dist = getCar2GoalDist();  // 获取车辆到目标的距离
    if (car2goal_dist < goalRadius)  // 如果距离目标很近
    {
      goal_reached = true;  // 设置目标已到达
      goal_received = false;  // 标记目标未接收到
      cmd_vel.linear.x = 1500.0;  // 停止速度
      cmd_vel.angular.z = 90.0;  // 设置角速度为零
      pub_->publish(cmd_vel);  // 发布停止命令
      pub_->publish(cmd_vel);  // 再次发布停止命令
      pub_->publish(cmd_vel);  // 再次发布停止命令
      RCLCPP_INFO(this->get_logger(), "Goal Reached! Stopping the vehicle.");  // 输出目标已到达的消息
    }
    else
    {
      goal_reached = false;  // 设置目标未到达
    }
  }
}

double L1Controller::isline(double line_wight)
{
  if (line_wight == 0.0)  // 如果线宽为零
  {
    return initbaseSpeed;  // 返回初始化的基础速度
  }
  
  double line_acc = 0.0;  // 初始化加速度
  line_acc = line_wight * 0.5;  // 根据线宽计算加速度
  baseSpeed = baseSpeed + line_acc;  // 更新基础速度
  
  return baseSpeed;  // 返回更新后的基础速度
}

void L1Controller::controlLoopCB()
{
  geometry_msgs::msg::Pose carPose = odom.pose.pose;  // 获取车辆位姿
  geometry_msgs::msg::Twist carVel = odom.twist.twist;  // 获取车辆速度
  cmd_vel.linear.x = 1500.0;  // 设置线速度
  cmd_vel.angular.z = baseAngle;  // 设置角速度
  static double speedlast;
  static double anglelast;

  double eta = getEta(carPose);  // 获取车辆与目标的方位角
  if (foundForwardPt)  // 如果找到了前方目标点
  {
    if (!goal_reached)  // 如果目标未到达
    {
      if (stop_flag == 1.0 && stopIdx <= 0)  // 判断是否需要减速
      {
        baseSpeed = slow_final * baseSpeed + 1500;  // 减速
        stopIdx++;  // 增加停止索引
        RCLCPP_WARN(this->get_logger(), "Final goal reached, slowing down. New base speed: %f", baseSpeed);  // 输出减速信息
      }
      if (stop_flag == 2.0 && stopIdx <= 0)  // 判断是否需要加速
      {
        baseSpeed = fast_final * baseSpeed + 1500;  // 加速
        stopIdx++;  // 增加停止索引
        RCLCPP_WARN(this->get_logger(), "Final goal reached, speeding up. New base speed: %f", baseSpeed);  // 输出加速信息
      }
      if (stop_flag == 3.0 && stopIdx > 0)  // 判断是否恢复速度
      {
        baseSpeed = initbaseSpeed + 1500;  // 恢复初始速度
        RCLCPP_WARN(this->get_logger(), "Recovering speed. New base speed: %f", baseSpeed);  // 输出恢复速度信息
      }
      
      cmd_vel.linear.x = baseSpeed;  // 更新线速度
      cmd_vel.angular.z = 90 - getSteeringAngle(eta) * Angle_gain_p - Angle_gain_d * (getSteeringAngle(eta) - last_steeringangle);  // 更新角速度
      last_steeringangle = getSteeringAngle(eta);  // 更新上一次的转向角

      if (cmd_vel.linear.x < vp_min + 1500)  // 如果速度小于最小值
      {
        cmd_vel.linear.x = vp_min + 1500;  // 设置为最小值
        RCLCPP_WARN(this->get_logger(), "Commanded speed below minimum, setting to minimum: %f", cmd_vel.linear.x);  // 输出警告信息
      }
      
      if (mapPathNum <= 0)  // 如果路径点为空
      {
        RCLCPP_WARN(this->get_logger(), "No path available, setting speed to minimal cruising speed.");  // 输出警告信息
        cmd_vel.linear.x = 115;  // 设置最小巡航速度
      }
      
      if (cmd_vel.linear.x > vp_max_base + 1500)  // 如果速度大于最大值
      {
        cmd_vel.linear.x = vp_max_base + 1500;  // 设置为最大值
        RCLCPP_WARN(this->get_logger(), "Commanded speed above maximum, setting to maximum: %f", cmd_vel.linear.x);  // 输出警告信息
      }

      if (cmd_vel.angular.z > 135)  // 如果角速度超过最大值
      {
        cmd_vel.angular.z = 135;  // 设置为最大角速度
        RCLCPP_WARN(this->get_logger(), "Commanded angle above maximum, setting to maximum: %f", cmd_vel.angular.z);  // 输出警告信息
      }
      else if (cmd_vel.angular.z < 45)  // 如果角速度低于最小值
      {
        cmd_vel.angular.z = 45;  // 设置为最小角速度
        RCLCPP_WARN(this->get_logger(), "Commanded angle below minimum, setting to minimum: %f", cmd_vel.angular.z);  // 输出警告信息
      }
    }
  }
  
  speedlast = cmd_vel.linear.x;  // 保存上一次的速度
  anglelast = cmd_vel.angular.z;  // 保存上一次的角速度
  
  if (traffic_flag)  // 如果有交通信号
  {
    pub_->publish(cmd_vel);  // 发布控制命令
    RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear = %.2f, angular = %.2f", cmd_vel.linear.x, cmd_vel.angular.z);  // 输出发布的命令
  }
  else  // 如果没有交通信号
  {
    cmd_vel.linear.x = 1500;  // 设置为默认速度
    cmd_vel.angular.z = 90;  // 设置为默认角速度
    pub_->publish(cmd_vel);  // 发布默认命令
    RCLCPP_WARN(this->get_logger(), "Traffic flag is false, setting default speed and angle.");  // 输出警告信息
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);  // 初始化ROS
  auto node = std::make_shared<L1Controller>();  // 创建L1控制器节点
  rclcpp::spin(node);  // 启动节点的处理循环
  RCLCPP_INFO(node->get_logger(), "Shutting down ROS node.");  // 输出关闭信息
  rclcpp::shutdown();  // 关闭ROS
  return 0;  // 返回0，表示程序正常结束
}

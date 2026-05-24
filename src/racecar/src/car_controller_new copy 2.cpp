
#include <iostream>

// Include geometry messages
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>


// Include navigation messages
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// Include standard messages
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>

// Include sensor messages
#include <sensor_msgs/msg/imu.hpp>

// Include tf2 messages
#include <tf2_msgs/msg/tf_message.hpp>

// Include tf2 libraries
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Include message filters
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>
#define PI 3.14159265358979
double last_steeringangle = 0;
double L, Lfw, Lrv, Lfw_, Vcmd, lfw, lrv, steering, u, v;
double Gas_gain, baseAngle, baseSpeed, Angle_gain_p, Angle_gain_d, goalRadius;
int controller_freq;
bool foundForwardPt, goal_received, goal_reached;
double k_rou;
double vp_max_base, vp_min;
double stop_flag = 0.0;
int mapPathNum;
double slow_final, fast_final;
int stopIdx = 0;
double line_wight = 0.0;
double initbaseSpeed;
double obs_flag = 0.0;
bool traffic_flag = true;

class L1Controller : public rclcpp::Node
{
public:
  L1Controller();
  void initMarker();
  bool isWayPtAwayFromLfwDist(const geometry_msgs::msg::Point &wayPt, const geometry_msgs::msg::Point &car_pos);
  bool isForwardWayPt(const geometry_msgs::msg::Point &wayPt, const geometry_msgs::msg::Pose &carPose);
  double getYawFromPose(const geometry_msgs::msg::Pose &carPose);
  double getEta(const geometry_msgs::msg::Pose &carPose);
  double getCar2GoalDist();
  double getL1Distance(const double &_Vcmd);
  double getSteeringAngle(double eta);
  double getGasInput(const float &current_v);
  double isline(double line_wight);

  geometry_msgs::msg::Point get_odom_car2WayPtVec(const geometry_msgs::msg::Pose &carPose);

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub, encoder_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr final_goal_sub, line_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traffic_light_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
  rclcpp::TimerBase::SharedPtr timer1, timer2;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  visualization_msgs::msg::Marker points, line_strip, goal_circle;
  geometry_msgs::msg::Twist cmd_vel;
  geometry_msgs::msg::Point odom_goal_pos;
  nav_msgs::msg::Odometry odom, encoder;
  nav_msgs::msg::Path map_path, odom_path;
  geometry_msgs::msg::PoseStamped goal_pos;
  rclcpp::Time current_time;
  void odomCB(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
  void pathCB(const nav_msgs::msg::Path::SharedPtr pathMsg);
  void encoderCB(const nav_msgs::msg::Odometry::SharedPtr encoderMsg);
  void goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalMsg);
  void goalReachingCB();
  void controlLoopCB();
  void stopCB(const std_msgs::msg::Float64::SharedPtr stopMsg);
  void lineCB(const std_msgs::msg::Float64::SharedPtr lineMsg);
  void lightCB(const std_msgs::msg::String::SharedPtr msg);
};

L1Controller::L1Controller() : Node("art_car_controller")
{
  // Car parameter
  this->declare_parameter("L", 0.305);
  this->declare_parameter("Vcmd", 1.0);
  this->declare_parameter("lfw", 0.1675);
  this->declare_parameter("lrv", 10.0);
  this->declare_parameter("Lrv", 10.0);

  // Controller parameter
  this->declare_parameter("controller_freq", 30);
  this->declare_parameter("Angle_gain_p", -1.0);
  this->declare_parameter("Angle_gain_d", -0.0);
  this->declare_parameter("baseSpeed", 0.0);
  this->declare_parameter("baseAngle", 90.0);
  this->declare_parameter("k_rou", 0.0);
  this->declare_parameter("vp_max_base", 0.0);
  this->declare_parameter("vp_min", 0.0);
  this->declare_parameter("goalRadius", 0.5);
  this->declare_parameter("Lfw", 0.3);
  this->declare_parameter("slow_final", 1.0);
  this->declare_parameter("fast_final", 1.0);

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

  initbaseSpeed = baseSpeed;
  current_time = rclcpp::Clock().now();
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_combined", 1, std::bind(&L1Controller::odomCB, this, std::placeholders::_1));
      // "/encoder_imu_odom", 1, std::bind(&L1Controller::odomCB, this, std::placeholders::_1));
  path_sub = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", 1, std::bind(&L1Controller::pathCB, this, std::placeholders::_1));
  goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&L1Controller::goalCB, this, std::placeholders::_1));
  // encoder_sub = this->create_subscription<nav_msgs::msg::Odometry>(
  //     "/encoder", 1, std::bind(&L1Controller::encoderCB, this, std::placeholders::_1));
  final_goal_sub = this->create_subscription<std_msgs::msg::Float64>(
      "/arrfinal", 1, std::bind(&L1Controller::stopCB, this, std::placeholders::_1));
  line_sub = this->create_subscription<std_msgs::msg::Float64>(
      "/line_wight", 1, std::bind(&L1Controller::lineCB, this, std::placeholders::_1));
  traffic_light_sub = this->create_subscription<std_msgs::msg::String>(
      "/light", 1, std::bind(&L1Controller::lightCB, this, std::placeholders::_1));

  pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/car_cmd_vel", 1);
  marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("car_path", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  timer1 = this->create_wall_timer(std::chrono::duration<double>(2.0 / controller_freq),
                                   std::bind(&L1Controller::controlLoopCB, this));
  timer2 = this->create_wall_timer(std::chrono::duration<double>(1.0 / controller_freq),
                                   std::bind(&L1Controller::goalReachingCB, this));

  //initMarker();
}

void L1Controller::initMarker()
{
  points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
  points.ns = line_strip.ns = goal_circle.ns = "Markers";
  points.action = line_strip.action = goal_circle.action = visualization_msgs::msg::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
  points.id = 0;
  line_strip.id = 1;
  goal_circle.id = 2;

  points.type = visualization_msgs::msg::Marker::POINTS;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  goal_circle.type = visualization_msgs::msg::Marker::CYLINDER;

  points.scale.x = 0.2;
  points.scale.y = 0.2;

  line_strip.scale.x = 0.1;

  goal_circle.scale.x = goalRadius;
  goal_circle.scale.y = goalRadius;
  goal_circle.scale.z = 0.1;

  points.color.g = 1.0f;
  points.color.a = 1.0;

  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  goal_circle.color.r = 1.0;
  goal_circle.color.g = 1.0;
  goal_circle.color.b = 0.0;
  goal_circle.color.a = 0.5;
}

void L1Controller::odomCB(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
  odom = *odomMsg;
  // RCLCPP_INFO(this->get_logger(), "Odometry received: position (%.2f, %.2f, %.2f)", 
  //             odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
}

void L1Controller::lightCB(const std_msgs::msg::String::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Traffic light message received: %s", msg->data.c_str());
  if (msg->data == "red")
  {
    traffic_flag = false;
  }
  else if (msg->data == "green")
  {
    traffic_flag = true;
  }
  else
  {
    traffic_flag = true;
  }
}

void L1Controller::pathCB(const nav_msgs::msg::Path::SharedPtr pathMsg)
{
  static int pathCBidx = 0;
  static nav_msgs::msg::Path last_map_path;

  // 检查并设置路径消息的 frame_id
  if (pathMsg->header.frame_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received path with empty frame_id, setting default frame_id 'map'");
    pathMsg->header.frame_id = "map"; // 或者其他适当的默认框架
  }

  // 检查并设置每个路径点的 frame_id
  for (auto& pose : pathMsg->poses) {
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = pathMsg->header.frame_id;
    }
  }

  if (pathCBidx == 0)
  {
    last_map_path.poses.clear();
  }

  map_path = *pathMsg;
  mapPathNum = map_path.poses.size();
  // RCLCPP_INFO(this->get_logger(), "Path received with %d poses", mapPathNum);

  if (map_path.poses.size() <= 0)
  {
    RCLCPP_WARN(this->get_logger(), "Received empty path, using last known path with %d poses", (int)last_map_path.poses.size());
    for (int i = 0; i < last_map_path.poses.size(); i++)
    {
      map_path.poses.push_back(last_map_path.poses[i]);
    }
  }
  else
  {
    last_map_path.poses.clear();
    for (int i = 0; i < map_path.poses.size(); i++)
    {
      last_map_path.poses.push_back(map_path.poses[i]);
    }
  }
  pathCBidx++;
}

void L1Controller::encoderCB(const nav_msgs::msg::Odometry::SharedPtr encoderMsg)
{
  encoder = *encoderMsg;
  // RCLCPP_INFO(this->get_logger(), "Encoder message received: speed = %.2f", encoder.twist.twist.linear.x);
}

void L1Controller::stopCB(const std_msgs::msg::Float64::SharedPtr stopMsg)
{
  stop_flag = stopMsg->data;
  // RCLCPP_INFO(this->get_logger(), "Stop flag received: stop_flag = %f", stop_flag);
}

void L1Controller::lineCB(const std_msgs::msg::Float64::SharedPtr lineMsg)
{
  line_wight = lineMsg->data;
  // RCLCPP_INFO(this->get_logger(), "Line width received: line_wight = %f", line_wight);
}

void L1Controller::goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalMsg)
{
  geometry_msgs::msg::PoseStamped odom_goal;
  goal_pos = *goalMsg;
  current_time = rclcpp::Clock().now();
  RCLCPP_INFO(this->get_logger(), "Goal received and transformed to odom frame");
  try
  {
    tf_buffer_->transform(*goalMsg, odom_goal, "odom", tf2::durationFromSec(1.0));

    odom_goal_pos = odom_goal.pose.position;
    // RCLCPP_INFO(this->get_logger(), "X pose: %.2f", odom_goal_pos.x);
    // RCLCPP_INFO(this->get_logger(), "Y pose: %.2f", odom_goal_pos.y);
    goal_received = true;
    goal_reached = false;

    //goal_circle.pose = odom_goal.pose;
    // marker_pub->publish(goal_circle);
    // RCLCPP_INFO(this->get_logger(), "Goal received and transformed to odom frame");
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    RCLCPP_INFO(this->get_logger(),"goal error sleep");
    // rclcpp::sleep_for(std::chrono::seconds(1));
  }
  // goal_received = true;
  // goal_reached = false;
  // RCLCPP_INFO(this->get_logger(), "X pose: %.2f", odom_goal_pos.x);
  // RCLCPP_INFO(this->get_logger(), "Y pose: %.2f", odom_goal_pos.y);
}

double L1Controller::getYawFromPose(const geometry_msgs::msg::Pose &carPose)
{
  double roll, pitch, yaw;
  tf2::Quaternion q(carPose.orientation.x, carPose.orientation.y, carPose.orientation.z, carPose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  //RCLCPP_INFO(this->get_logger(), "Yaw from pose: %.2f", yaw);
  return yaw;
}

bool L1Controller::isForwardWayPt(const geometry_msgs::msg::Point &wayPt,
                                  const geometry_msgs::msg::Pose &carPose)
{
  float car2wayPt_x = wayPt.x - carPose.position.x;
  float car2wayPt_y = wayPt.y - carPose.position.y;
  double car_theta = getYawFromPose(carPose);

  float car_car2wayPt_x = cos(car_theta) * car2wayPt_x + sin(car_theta) * car2wayPt_y;
  float car_car2wayPt_y = -sin(car_theta) * car2wayPt_x + cos(car_theta) * car2wayPt_y;

  // RCLCPP_INFO(this->get_logger(), "Checking forward waypoint: car2wayPt_x = %.2f, car2wayPt_y = %.2f, isForward = %d", 
              // car_car2wayPt_x, car_car2wayPt_y, (car_car2wayPt_x > 0));
  return car_car2wayPt_x > 0;
}

bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::msg::Point &wayPt, const geometry_msgs::msg::Point &car_pos)
{
  double dx = wayPt.x - car_pos.x;
  double dy = wayPt.y - car_pos.y;
  double dist = sqrt(dx * dx + dy * dy);
  // RCLCPP_INFO(this->get_logger(), "Checking if waypoint is away from Lfw distance: dist = %.2f, Lfw = %.2f", dist, Lfw);
  return dist >= Lfw;
}

geometry_msgs::msg::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::msg::Pose &carPose)
{
  geometry_msgs::msg::Point carPose_pos = carPose.position;
  double carPose_yaw = getYawFromPose(carPose);
  geometry_msgs::msg::Point forwardPt;
  geometry_msgs::msg::Point odom_car2WayPtVec;
  foundForwardPt = false;
  double car2goal_dist = getCar2GoalDist();
  bool start_flag = false;

  if (!goal_reached)
  {
    for (int i = 0; i < map_path.poses.size(); i++)
    {
      geometry_msgs::msg::PoseStamped map_path_pose = map_path.poses[i];
      geometry_msgs::msg::PoseStamped odom_path_pose;
      try
      {
        
        tf_buffer_->transform(map_path_pose, odom_path_pose, "odom", tf2::durationFromSec(1.0));
        geometry_msgs::msg::Point odom_path_wayPt = odom_path_pose.pose.position;
        bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt, carPose);

        if (_isForwardWayPt && !start_flag)
        {
          start_flag = true;
        }
        if (!start_flag)
        {
          continue;
        }

        if (_isForwardWayPt)
        {
          bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt, carPose_pos);
          if (_isWayPtAwayFromLfwDist)
          {
            forwardPt = odom_path_wayPt;
            foundForwardPt = true;
            // RCLCPP_INFO(this->get_logger(), "Found forward waypoint at (%.2f, %.2f)", forwardPt.x, forwardPt.y);
            break;
          }
        }

        if (car2goal_dist < Lfw)
        {
          forwardPt = odom_goal_pos;
          foundForwardPt = true;
          // RCLCPP_INFO(this->get_logger(), "Using goal position as forward waypoint at (%.2f, %.2f)", forwardPt.x, forwardPt.y);
        }
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        RCLCPP_INFO( this->get_logger(),"path error sleep");
        // rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }
  }
  else if (goal_reached)
  {
    forwardPt = odom_goal_pos;
    foundForwardPt = false;
    
  
  }











  points.points.clear();
  line_strip.points.clear();

  if (foundForwardPt && !goal_reached)
  {
    points.points.push_back(carPose_pos);
    points.points.push_back(forwardPt);
    line_strip.points.push_back(carPose_pos);
    line_strip.points.push_back(forwardPt);
  }

  marker_pub->publish(points);
  marker_pub->publish(line_strip);

  odom_car2WayPtVec.x = cos(carPose_yaw) * (forwardPt.x - carPose_pos.x) + sin(carPose_yaw) * (forwardPt.y - carPose_pos.y);
  odom_car2WayPtVec.y = -sin(carPose_yaw) * (forwardPt.x - carPose_pos.x) + cos(carPose_yaw) * (forwardPt.y - carPose_pos.y);
  //RCLCPP_INFO(this->get_logger(), "Calculated odom_car2WayPtVec: (%.2f, %.2f)", odom_car2WayPtVec.x, odom_car2WayPtVec.y);
  return odom_car2WayPtVec;
}

double L1Controller::getEta(const geometry_msgs::msg::Pose &carPose)
{
  geometry_msgs::msg::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
  double eta = atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);
  RCLCPP_INFO(this->get_logger(), "Calculated eta: %.2f degrees", eta*57.3);
  return eta;
}

double L1Controller::getCar2GoalDist()
{
  geometry_msgs::msg::Point car_pose = odom.pose.pose.position;
  double car2goal_x = odom_goal_pos.x - car_pose.x;
  double car2goal_y = odom_goal_pos.y - car_pose.y;
  double dist = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);
  RCLCPP_INFO(this->get_logger(), "Calculated car to goal distance: %.2f meters", dist);
  return dist;
}

double L1Controller::getL1Distance(const double &_Vcmd)
{

 
  double L1 = 0;
  double car2goal_dist = getCar2GoalDist();
  double v = _Vcmd;
  
  L1 = 1.45;  // Example fixed value, normally would be calculated based on Vcmd
  
  // RCLCPP_INFO(this->get_logger(), "Calculated L1 distance: %.2f meters", L1);
  return L1;
}

double L1Controller::getSteeringAngle(double eta)
{
  double car2goal_dist = getCar2GoalDist();

  double steeringAngle = atan2((L * sin(eta)), ((car2goal_dist / 2) + lfw * cos(eta))) * (180.0 / PI);
  // double steeringAngle = atan2((L * sin(eta)), ((Lfw / 2) + lfw * cos(eta))) * (180.0 / PI);
  
  RCLCPP_INFO(this->get_logger(), "Calculated steering angle: %.2f degrees", steeringAngle);
  return steeringAngle;
}

void L1Controller::goalReachingCB()
{
  if (1)
  {
    try
    {
      geometry_msgs::msg::PoseStamped odom_goal;
      current_time = rclcpp::Clock().now();
      goal_pos.header.stamp = current_time;
      tf_buffer_->transform(goal_pos, odom_goal, "odom", tf2::durationFromSec(2.0));
      RCLCPP_INFO( this->get_logger(),"yes sleep");
      odom_goal_pos = odom_goal.pose.position;
      
      // RCLCPP_INFO(this->get_logger(), "X pose: %.2f", odom_goal_pos.x);
      // RCLCPP_INFO(this->get_logger(), "Y pose: %.2f", odom_goal_pos.y);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      RCLCPP_INFO( this->get_logger(),"error sleep");
      // // 获取当前时间的秒数和纳秒数
      // auto sec = current_time.seconds();
      // auto nsec = current_time.nanoseconds() ;
      // // 减少2秒
      // sec = sec-1;
      // // 创建一个新的时间对象
      // current_time = rclcpp::Time(sec, nsec);
      //rclcpp::sleep_for(std::chrono::seconds(1));
    }
    double car2goal_dist = getCar2GoalDist();
    if (car2goal_dist < goalRadius)
    {
      goal_reached = true;
      goal_received = false;
      cmd_vel.linear.x = 1500.0;
      cmd_vel.angular.z = 90.0;
      pub_->publish(cmd_vel);
      pub_->publish(cmd_vel);
      pub_->publish(cmd_vel);
      RCLCPP_INFO(this->get_logger(), "Goal Reached! Stopping the vehicle.");
    }
    else
    {
      goal_reached = false;
      // RCLCPP_INFO(this->get_logger(), "Current distance to goal: %.2f meters", car2goal_dist);
    }
  }
}

double L1Controller::isline(double line_wight)
{
  // RCLCPP_INFO(this->get_logger(), "Line weight detected: %f", line_wight);
  
  if (line_wight == 0.0)
  {
    // RCLCPP_INFO(this->get_logger(), "Line weight is zero, returning initial base speed: %f", initbaseSpeed);
    return initbaseSpeed;
  }
  
  double line_acc = 0.0;
  line_acc = line_wight * 0.5;
  baseSpeed = baseSpeed + line_acc;
  // RCLCPP_WARN(this->get_logger(), "Line weight adjussted base speed: %f", baseSpeed);
  
  return baseSpeed;
}

void L1Controller::controlLoopCB()
{
  geometry_msgs::msg::Pose carPose = odom.pose.pose;
  geometry_msgs::msg::Twist carVel = odom.twist.twist;
  cmd_vel.linear.x = 1500.0;
  cmd_vel.angular.z = baseAngle;
  // double encoder_speed = encoder.twist.twist.linear.x;
  static double speedlast;
  static double anglelast;

  // if (goal_received)
  // {
    
    double eta = getEta(carPose);
    if (foundForwardPt)
    {
      if (!goal_reached)
      {
        if (stop_flag == 1.0 && stopIdx <= 0)
        {
          baseSpeed = slow_final * baseSpeed + 1500;
          stopIdx++;
          RCLCPP_WARN(this->get_logger(), "Final goal reached, slowing down. New base speed: %f", baseSpeed);
        }
        if (stop_flag == 2.0 && stopIdx <= 0)
        {
          baseSpeed = fast_final * baseSpeed + 1500;
          stopIdx++;
          RCLCPP_WARN(this->get_logger(), "Final goal reached, speeding up. New base speed: %f", baseSpeed);
        }
        if (stop_flag == 3.0 && stopIdx > 0)
        {
          baseSpeed = initbaseSpeed + 1500;
          RCLCPP_WARN(this->get_logger(), "Recovering speed. New base speed: %f", baseSpeed);
        }
        
        cmd_vel.linear.x = baseSpeed;
        cmd_vel.angular.z = 90 - getSteeringAngle(eta) * Angle_gain_p - Angle_gain_d * (getSteeringAngle(eta) - last_steeringangle);
        last_steeringangle = getSteeringAngle(eta);

        if (cmd_vel.linear.x < vp_min + 1500)
        {
          cmd_vel.linear.x = vp_min + 1500;
          RCLCPP_WARN(this->get_logger(), "Commanded speed below minimum, setting to minimum: %f", cmd_vel.linear.x);
        }
        
        if (mapPathNum <= 0)
        {
          RCLCPP_WARN(this->get_logger(), "No path available, setting speed to minimal cruising speed.");
          cmd_vel.linear.x = 115;
        }
        
        if (cmd_vel.linear.x > vp_max_base + 1500)
        {
          cmd_vel.linear.x = vp_max_base + 1500;
          RCLCPP_WARN(this->get_logger(), "Commanded speed above maximum, setting to maximum: %f", cmd_vel.linear.x);
        }

        if (cmd_vel.angular.z > 135)
        {
          cmd_vel.angular.z = 135;
          RCLCPP_WARN(this->get_logger(), "Commanded angle above maximum, setting to maximum: %f", cmd_vel.angular.z);
        }
        else if (cmd_vel.angular.z < 45)
        {
          cmd_vel.angular.z = 45;
          RCLCPP_WARN(this->get_logger(), "Commanded angle below minimum, setting to minimum: %f", cmd_vel.angular.z);
        }

        // RCLCPP_INFO(this->get_logger(), "Lfw = %.2f", Lfw);
        // RCLCPP_INFO(this->get_logger(), "eta = %.2f", eta * 180 / PI);
        // RCLCPP_INFO(this->get_logger(), "encoder_v = %.2f", encoder_speed);
        // RCLCPP_INFO(this->get_logger(), "out_speed = %.2f", cmd_vel.linear.x);
        // RCLCPP_INFO(this->get_logger(), "out_angle = %.2f", cmd_vel.angular.z);
        // RCLCPP_INFO(this->get_logger(), "------------------------");
      }
    }
  // }
  // else
  // {
  //   cmd_vel.linear.x = 1500;
  //   cmd_vel.angular.z = 90;
  //   RCLCPP_WARN(this->get_logger(), "No goal received, setting default speed and angle.");
  // }
  
  speedlast = cmd_vel.linear.x;
  anglelast = cmd_vel.angular.z;
  
  if (traffic_flag)
  {
    pub_->publish(cmd_vel);
    RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear = %.2f, angular = %.2f", cmd_vel.linear.x, cmd_vel.angular.z);
  }
  else
  {
    cmd_vel.linear.x = 1500;
    cmd_vel.angular.z = 90;
    pub_->publish(cmd_vel);
    RCLCPP_WARN(this->get_logger(), "Traffic flag is false, setting default speed and angle.");
  }
  //rclcpp::sleep_for(std::chrono::seconds(1));
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<L1Controller>();
  rclcpp::spin(node);
  RCLCPP_INFO(node->get_logger(), "Shutting down ROS node.");
  rclcpp::shutdown();
  return 0;
}
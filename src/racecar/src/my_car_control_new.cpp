#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <cmath>
#include <iostream>

#define _POSE(carpose_map) (sqrt(pow(carpose_map.pose.position.x, 2) + pow(carpose_map.pose.position.y, 2)))

using std::vector;

double angle_pd = 0.25;
double angle_protect = 1;
double line_protect = 1;
double brake = 4;
int baseSpeed = 30;

typedef struct
{
    float error_angle;
    float error_line;
    float P_angle;
    float D_angle;
    float P_line;
    float D_line;
    float last_error_angle;
    float last_error_line;
    int out_angle;
    int out_line;
    float line_D;
    float angle_D;
} pid_dir;

pid_dir PID_dir;

class MyCarControl : public rclcpp::Node
{
private:
    bool flag_go = false;
    bool flag_finish = false;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr point_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr carpose_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::PoseStamped point_baselink_pub;
    geometry_msgs::msg::PoseStamped point_map_pub;
    geometry_msgs::msg::PoseStamped carpose_odom;
    geometry_msgs::msg::PoseStamped carpose_map;
    std::vector<geometry_msgs::msg::PoseStamped> vec_pub;

    void GetPoint(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void CarPose(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

public:
    MyCarControl();
    ~MyCarControl() = default;
    float DisPoint(const geometry_msgs::msg::PoseStamped &x1, const geometry_msgs::msg::PoseStamped &x2);
};

MyCarControl::MyCarControl()
: Node("my_car_control"), buffer(get_clock()), listener(buffer)
{
    RCLCPP_INFO(this->get_logger(), "小车控制节点开启");

    PID_dir.P_line = 87;
    PID_dir.D_line = 40;
    PID_dir.P_angle = 20;
    PID_dir.D_angle = 20;

    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;

    point_map_pub.pose.position.z = 0;
    point_baselink_pub.header.frame_id = "base_link";
    carpose_odom.header.frame_id = "odom";

    cmd_vel.linear.x = 1500 + baseSpeed;

    point_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "/point_mid", 10, std::bind(&MyCarControl::GetPoint, this, std::placeholders::_1));

    carpose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10, std::bind(&MyCarControl::CarPose, this, std::placeholders::_1));

    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("car_cmd_vel", 100);
}

void MyCarControl::GetPoint(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    if (!flag_finish)
    {
        PID_dir.error_angle = msg->theta;
        PID_dir.error_line = msg->y;
        PID_dir.line_D = fabs(PID_dir.error_line - PID_dir.last_error_line);
        PID_dir.angle_D = fabs(PID_dir.error_angle - PID_dir.last_error_angle);

        PID_dir.out_line = PID_dir.P_line * PID_dir.error_line + PID_dir.D_line * (PID_dir.error_line - PID_dir.last_error_line);

        if (fabs(PID_dir.error_line) < angle_pd)
            PID_dir.out_angle = PID_dir.P_angle * PID_dir.error_angle + PID_dir.D_angle * (PID_dir.error_angle - PID_dir.last_error_angle);
        else
            PID_dir.out_angle = 0;
        RCLCPP_INFO(this->get_logger(),"输出PID_dir.error_angle为：%f\n",PID_dir.error_angle);
        RCLCPP_INFO(this->get_logger(),"输出PID_dir.error_line为：%f\n",PID_dir.error_line);

        cmd_vel.angular.z = 90 + PID_dir.out_line + PID_dir.out_angle;
        RCLCPP_INFO(this->get_logger(),"输出角度为：%d\n",cmd_vel.angular.z);

        if (PID_dir.error_angle > angle_protect)
        {
            cmd_vel.angular.z = 75;
            vel_pub->publish(cmd_vel);
        }
        else if (PID_dir.error_angle < -angle_protect)
        {
            cmd_vel.angular.z = 105;
            vel_pub->publish(cmd_vel);
        }

        if (fabs(PID_dir.error_line) > line_protect)
        {
            cmd_vel.angular.z = 90;
            vel_pub->publish(cmd_vel);
        }

        if (cmd_vel.angular.z > 150)
            cmd_vel.angular.z = 150;
        else if (cmd_vel.angular.z < 30)
            cmd_vel.angular.z = 30;

        // cmd_vel.linear.x = 1550 - 8 * fabs(90 - cmd_vel.angular.z);
        cmd_vel.linear.x = 1530;


        if (PID_dir.angle_D > brake)
        {
            cmd_vel.linear.x = 1500;
        }

        vel_pub->publish(cmd_vel);
        PID_dir.last_error_line = PID_dir.error_line;
        PID_dir.last_error_angle = PID_dir.error_angle;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "我已完成第一圈");
    }

    if (_POSE(carpose_map) > 5 && !flag_go)
    {
        flag_go = true;
        RCLCPP_INFO(this->get_logger(), "我已经离开原点");
    }

    if ((_POSE(carpose_map) <= 0.5) && flag_go && !flag_finish)
    {
        flag_finish = true;
        cmd_vel.linear.x = 1500;
        cmd_vel.angular.z = 90;
        vel_pub->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "我已完成第一圈");
    }
}

void MyCarControl::CarPose(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
    try
    {
        carpose_odom.pose.position.x = odomMsg->pose.pose.position.x;
        carpose_odom.pose.position.y = odomMsg->pose.pose.position.y;
        carpose_odom.pose.position.z = odomMsg->pose.pose.position.z;
        carpose_odom.pose.orientation.x = odomMsg->pose.pose.orientation.x;
        carpose_odom.pose.orientation.y = odomMsg->pose.pose.orientation.y;
        carpose_odom.pose.orientation.z = odomMsg->pose.pose.orientation.z;
        carpose_odom.pose.orientation.w = odomMsg->pose.pose.orientation.w;
        carpose_map = buffer.transform(carpose_odom, "map");
    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(this->get_logger(), "程序异常: %s", e.what());
    }
}

float MyCarControl::DisPoint(const geometry_msgs::msg::PoseStamped &x1, const geometry_msgs::msg::PoseStamped &x2)
{
    float dis;
    dis = sqrt(pow(x1.pose.position.x - x2.pose.position.x, 2) + pow(x1.pose.position.y - x2.pose.position.y, 2));
    return dis;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyCarControl>());
    rclcpp::shutdown();
    return 0;
}
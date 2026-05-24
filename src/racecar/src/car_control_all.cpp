#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "std_msgs/msg/header.hpp" 
#include <vector>
#include <cmath>
#include <iostream>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <termios.h> 
#include <unistd.h>  

class LidarProcessor : public rclcpp::Node
{
private:
    int biaozhi = 0;
    int dajiao;
    int ch = 0;
    int speed = 1525; // 基础速度
    int ztjs_L = 0;
    int oldjiaodu_L = 90; //角度中值
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

    rclcpp::Time start_time = rclcpp::Clock().now();
    rclcpp::Time time_now = rclcpp::Clock().now();

    geometry_msgs::msg::Pose2D point;
    geometry_msgs::msg::Twist cmd_vel;


    void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    

public:
    LidarProcessor();
    ~LidarProcessor();
};

LidarProcessor::LidarProcessor() : Node("lidar_processor")
{
    RCLCPP_INFO(this->get_logger(), "雷达数据分析节点开启");
    cmd_vel.angular.z = 90;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.linear.x = speed;
    sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 100, std::bind(&LidarProcessor::LidarCallback, this, std::placeholders::_1));
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("car_cmd_vel", 100);
}

LidarProcessor::~LidarProcessor()
{
    RCLCPP_INFO(this->get_logger(), "雷达数据分析节点关闭");
}

void LidarProcessor::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Lidar 回调函数处理逻辑
    float zuixiao_L = 90.0;
    float zuixiao_R = 90.0;
    int zhaodao_L = 0;
    int zhaodao_R = 0;
    int flag = 0;
    int jiaodu_L;
    int jiaodu_R;
    float Pi = 3.1415926;
    float sina;
    float dajiao;
    time_now = rclcpp::Clock().now();
    if (time_now - start_time < rclcpp::Duration::from_seconds(8))
    {
        cmd_vel.linear.x = 1525;
        cmd_vel.angular.z = 78;
        vel_pub->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "正在前进");
    }
    if (time_now - start_time >= rclcpp::Duration::from_seconds(8) && time_now - start_time < rclcpp::Duration::from_seconds(17))
    {
        cmd_vel.linear.x = 1525;
        cmd_vel.angular.z = 105;
        vel_pub->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "正在转弯");
    }
    if(time_now - start_time >= rclcpp::Duration::from_seconds(16) && time_now - start_time < rclcpp::Duration::from_seconds(38))
    {
        for (int i = 0; i < 1000; i++)
        {
            if (i > 0 && i < 208)  // 雷达检测角度范围限幅
            {
                float fMidDist = msg->ranges[i] * 100.0;
                if (fMidDist < 90 && fMidDist < zuixiao_L)
                {
                    zuixiao_L = fMidDist;
                    zhaodao_L = 1;
                    jiaodu_L = i;
                }
            }
            if (i > 791)
            {
                float fMidDist = msg->ranges[i] * 100.0;
                if (fMidDist < 90 && fMidDist < zuixiao_R)
                {
                    zuixiao_R = fMidDist;
                    zhaodao_R = 1;
                    jiaodu_R = i;
                    flag = 0;
                }
            }
        }
        if (zhaodao_L)
        {
            sina = sin(jiaodu_L * 0.36 * Pi / 180);
            dajiao = (zuixiao_L * sina);
            if (dajiao < -50) { dajiao = -50; } // 舵机打角限幅
            if (dajiao > 50) { dajiao = 50; }
            if ((oldjiaodu_L - jiaodu_L) > 25)
            {
                ztjs_L = ztjs_L + 1;
            }
            oldjiaodu_L = jiaodu_L;
            flag = 1;
            RCLCPP_INFO(this->get_logger(), "左角度：%.1f 左距离：%.1f 打角：%d 速度：%d ztjs:  %d", jiaodu_L * 0.36, zuixiao_L, static_cast<int>(dajiao), speed, ztjs_L);
        }
        if (ztjs_L > 60)
                ch = 3;
        if (ch != 3 && zhaodao_L)
        {
            cmd_vel.linear.x = speed;
            cmd_vel.angular.z = dajiao - 50 + 80;
            vel_pub->publish(cmd_vel);
            flag = 1;
            RCLCPP_INFO(this->get_logger(), "正在前进");
        }
        else
        {
            cmd_vel.linear.x = 1500;
            cmd_vel.angular.z =  80;
            vel_pub->publish(cmd_vel);
            RCLCPP_INFO(this->get_logger(), "正在停车");
        }
        if (zhaodao_R && flag == 0 )
        {
            cmd_vel.linear.x = speed;
            if(time_now - start_time >= rclcpp::Duration::from_seconds(24) )
            cmd_vel.angular.z =135 ;
            else cmd_vel.angular.z =110;
            vel_pub->publish(cmd_vel);
            RCLCPP_INFO(this->get_logger(), "右角度：%.1f  右距离：%.1f", jiaodu_R * 0.36, zuixiao_R);
        }
    }
    if(time_now -start_time > rclcpp::Duration::from_seconds(38))
    {
        cmd_vel.linear.x = 1500;
        cmd_vel.angular.z = 80;
        vel_pub->publish(cmd_vel);
    }
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}

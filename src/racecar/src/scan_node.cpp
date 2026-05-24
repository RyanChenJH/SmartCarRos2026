#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <termios.h> 
#include <unistd.h>  
#include <limits>

// 定义一个继承自 rclcpp::Node 的类，用于处理激光雷达数据
class LidarProcessor : public rclcpp::Node
{
private:
    int biaozhi = 0; // 标志变量
    int dajiao; // 当前打角值
    int ch = 0; // 状态变量
    int speed = 0.0; // 基础速度
    int ztjs_L = 0; // 左侧状态计数
    int oldjiaodu_L = 0.0; // 左侧角度中值

    // ROS2 订阅和发布器
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub; // 激光雷达数据订阅器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub; // 速度指令发布器

    geometry_msgs::msg::Pose2D point; // 位置点
    geometry_msgs::msg::Twist cmd_vel; // 速度指令消息

    // 激光雷达回调函数
    void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

public:
    LidarProcessor(); // 构造函数
    ~LidarProcessor(); // 析构函数
};

// 构造函数：初始化节点和订阅/发布器
LidarProcessor::LidarProcessor() : Node("lidar_processor")
{
    RCLCPP_INFO(this->get_logger(), "雷达数据分析节点开启");
    // 初始化速度指令
    cmd_vel.angular.z = 0.0;//初始角度
    cmd_vel.linear.x = 0.0; // 初始速度为0
    // 创建激光雷达数据订阅器
    sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 100, std::bind(&LidarProcessor::LidarCallback, this, std::placeholders::_1));

    // 创建速度指令发布器
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("car_cmd_vel", 100);
}

// 析构函数：节点关闭时打印日志
LidarProcessor::~LidarProcessor()
{
    RCLCPP_INFO(this->get_logger(), "雷达数据分析节点关闭");
}

// 激光雷达回调函数
void LidarProcessor::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 初始化变量
    float zuixiao_L = std::numeric_limits<float>::max(); // 左侧最小距离
    float zuixiao_R = std::numeric_limits<float>::max(); // 右侧最小距离
    int zhaodao_L = 0; // 是否找到左侧目标
    int zhaodao_R = 0; // 是否找到右侧目标
    int flag = 0; // 标志变量，表示是否找到目标
    int jiaodu_L; // 左侧目标角度
    int jiaodu_R; // 右侧目标角度
    float Pi = 3.1415926; // 圆周率
    float sina; // 正弦值
    float dajiao; // 打角值

    // 检查目标点数量
    int left_target_count = 0;
    int right_target_count = 0;

    for (int i = 0; i < 1000; i++)
    {
        if (i > 0 && i < 208 && msg->ranges[i] > 0.2 && msg->ranges[i] < 1.0)
            left_target_count++;
        if (i > 791 && i < 999 && msg->ranges[i] > 0.2 && msg->ranges[i] < 1.0)
            right_target_count++;
    }

    if (left_target_count < 10 && right_target_count < 10)
    {
        cmd_vel.linear.x = 0;           // 停止
        cmd_vel.angular.z = 0;          // 角度归零
        vel_pub->publish(cmd_vel);
        RCLCPP_WARN(this->get_logger(), "目标点过少，车辆可能跑出赛道，停车！");
        return;
    }

    // 遍历激光雷达数据
    for (int i = 0; i < 1000; i++)
    {
        // 检测左侧目标（角度范围：0-208）
        if (i > 0 && i < 208)
        {
            float fMidDist = msg->ranges[i] * 100.0f;
            if (std::isfinite(fMidDist) && fMidDist > 0.2f && fMidDist < zuixiao_L)
            {
                zuixiao_L = fMidDist;
                zhaodao_L = 1;
                jiaodu_L = i;
            }
        }

        // 检测右侧目标（角度范围：791-999）
        if (i > 791)
        {
            float fMidDist = msg->ranges[i] * 100.0f;
            if (std::isfinite(fMidDist) && fMidDist > 0.2f && fMidDist < zuixiao_R)
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
        // 计算打角值
        sina = sin(jiaodu_L * 0.36 * Pi / 180); // 角度转换为弧度并计算正弦值
        dajiao = (zuixiao_L * sina); // 根据距离和角度计算打角值
        if (dajiao < -50) { dajiao = -50; } // 限制打角值范围
        if (dajiao > 50) { dajiao = 50; }

        // 检测角度变化，更新状态计数
        if ((oldjiaodu_L - jiaodu_L) > 25)
        {
            ztjs_L = ztjs_L + 1;
        }
        oldjiaodu_L = jiaodu_L; // 更新角度
        flag = 1; // 标记找到目标

        // 打印左侧目标信息
        RCLCPP_INFO(this->get_logger(), "左角度：%.1f 左距离：%.1f 打角：%d 速度：%d ztjs:  %d", 
                    jiaodu_L * 0.36, zuixiao_L, static_cast<int>(dajiao), speed, ztjs_L);
    }

    // 如果状态计数超过阈值，改变状态
    if (ztjs_L > 60)
        ch = 3;

    // 如果未达到状态 3 且找到左侧目标
    if (ch != 3 && zhaodao_L)
    {
        cmd_vel.linear.x = 0.25; // 行走速度
        // 角度归一化到[-1,1]，左转为+1，右转为-1
        float norm_angle = (dajiao) / 50.0; // dajiao范围[-50,50]，归一化到[-1,1]
        if (norm_angle > 1) norm_angle = 1;
        if (norm_angle < -1) norm_angle = -1;
        cmd_vel.angular.z = norm_angle;
        vel_pub->publish(cmd_vel);
        flag = 1;
        // RCLCPP_INFO(this->get_logger(), "正在前进");
    }
    else
    {
        // 停车逻辑
        cmd_vel.linear.x = 0; // 停止
        cmd_vel.angular.z = 0;
        vel_pub->publish(cmd_vel); // 发布速度指令
        RCLCPP_INFO(this->get_logger(), "zhengzaitingche");
    }

    // 如果同时检测到左右两侧目标
    if (zhaodao_L && zhaodao_R)
    {
        // 计算两侧距离差
        float distance_diff = zuixiao_L - zuixiao_R;
        // 计算角速度，保持车辆居中
        float Kp = 0.5; // 比例系数，可根据实际调整
        float norm_angle = -Kp * distance_diff / 40.0;
        if (norm_angle > 1) norm_angle = 1;
        if (norm_angle < -1) norm_angle = -1;
        cmd_vel.angular.z = norm_angle;
        // 设置线速度
        cmd_vel.linear.x = 0.25; // 双边运行时用行走速度
        // 发布速度指令
        vel_pub->publish(cmd_vel);

        RCLCPP_INFO(this->get_logger(), "双边运行：左距离=%.1f 右距离=%.1f 差值=%.1f", 
                    zuixiao_L, zuixiao_R, distance_diff);
    }
    else if (zhaodao_L) // 仅检测到左侧目标
    {
        cmd_vel.linear.x = 0.25;
        float norm_angle = (dajiao) / 50.0;
        if (norm_angle > 1) norm_angle = 1;
        if (norm_angle < -1) norm_angle = -1;
        cmd_vel.angular.z = norm_angle;
        vel_pub->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "左侧运行：左距离=%.1f", zuixiao_L);
    }
    else if (zhaodao_R) // 仅检测到右侧目标
    {
        cmd_vel.linear.x = 0.25; // 转弯速度
        cmd_vel.angular.z = 1.0; // 最大左转
        vel_pub->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "右侧运行：右距离=%.1f", zuixiao_R);
    }
    else // 未检测到目标
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        vel_pub->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "未检测到目标，停车");
    }
}

// 主函数
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // 初始化 ROS2
    rclcpp::spin(std::make_shared<LidarProcessor>()); // 创建节点并运行
    rclcpp::shutdown(); // 关闭 ROS2
    return 0;
}
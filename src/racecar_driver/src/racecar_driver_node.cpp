//
// racecar_driver_node.cpp
//

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "../include/racecar_driver.h"
#include <cstring>

class RacecarDriver : public rclcpp::Node
{
public:
RacecarDriver()
: Node("racecar_driver")
{
this->declare_parameter<std::string>("serial_port", "/dev/car");
this->declare_parameter<int>("baud_rate", 38400);

std::string serial_port;
int baud_rate;

this->get_parameter("serial_port", serial_port);
this->get_parameter("baud_rate", baud_rate);

// Convert std::string to char*
char serial_port_char[serial_port.size() + 1];
std::strcpy(serial_port_char, serial_port.c_str());

art_racecar_init(baud_rate, serial_port_char);

subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
// "/cmd_vel", 1, std::bind(&RacecarDriver::TwistCallback, this, std::placeholders::_1));
"/car_cmd_vel", 1, std::bind(&RacecarDriver::TwistCallback, this, std::placeholders::_1));

teleop_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/teleop_cmd_vel", 1, std::bind(&RacecarDriver::teleop_TwistCallback, this, std::placeholders::_1));

}

private:
void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{

// double angle;
linear_x = twist->linear.x * 100 + 1500;
angle = 1500.0 - twist->angular.z * 1300;
// angle = 1500.0 - twist->angular.z * 1200;
RCLCPP_INFO(this->get_logger(), "线速度: %.2f    角速度: %.2f", linear_x, angle);

// angle = 2500.0 - twist->angular.z * 2000.0 / 180.0;
send_cmd(static_cast<uint16_t>(linear_x), static_cast<uint16_t>(angle));
// send_cmd(static_cast<uint16_t>(twist->linear.x), static_cast<uint16_t>(angle));
}


void teleop_TwistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
    angle = 2500.0 - twist->angular.z * 2000.0 / 180.0;
    linear_x = twist->linear.x;

    RCLCPP_INFO(this->get_logger(), "线速度: %.2f    角速度: %.2f", linear_x, angle);
    RCLCPP_INFO(this->get_logger(), "线速度: %.2f    角速度: %.2f", linear_x, angle);

    send_cmd(static_cast<uint16_t>(twist->linear.x), static_cast<uint16_t>(angle));
}





rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscription_;
    double linear_x;            // 全局变量
    double angle;               // 全局变量
};

int main(int argc, char** argv)
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<RacecarDriver>());
rclcpp::shutdown();
return 0;
}

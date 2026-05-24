#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class IMUFilterNode : public rclcpp::Node
{
public:
    IMUFilterNode() : Node("imu_filter_node")
    {
        RCLCPP_INFO(this->get_logger(), "IMU Filter Node started");

        // 订阅原始 IMU 数据
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/IMU_data",  // 输入话题
            rclcpp::QoS(rclcpp::SensorDataQoS()),
            std::bind(&IMUFilterNode::imu_callback, this, std::placeholders::_1)
        );

        // 创建发布器
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu_data",  // 输出话题
            rclcpp::QoS(rclcpp::SensorDataQoS())
        );

        // 初始化变量
        last_publish_time_ = std::chrono::steady_clock::now();
        publish_frequency_ =30.0;  // 输出频率 50Hz
        // publish_frequency_ = 50.0;  // 输出频率 50Hz
        publish_interval_ = 1.0 / publish_frequency_;
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration<double>(current_time - last_publish_time_).count();

        if (elapsed_time >= publish_interval_)
        {
            // 发布处理后的 IMU 数据
            publisher_->publish(*msg);
            last_publish_time_ = current_time;
            // RCLCPP_INFO(this->get_logger(), "Published filtered IMU data");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    std::chrono::steady_clock::time_point last_publish_time_;
    double publish_frequency_;
    double publish_interval_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

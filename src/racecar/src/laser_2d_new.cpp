#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <cmath>
#include <iostream>

#define PI 3.1415926

typedef struct
{
    float angle;
    float distance;
    float dk_x;
    float dk_y;
} dir_obstacle;

class ClassLaser2D : public rclcpp::Node
{
private:
    int ind[600] = {0};
    float new_laser[600] = {0};

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

    geometry_msgs::msg::Pose2D point;
    geometry_msgs::msg::Twist cmd_vel;
    dir_obstacle ob_1, ob_2;

    void GetGoal(const sensor_msgs::msg::LaserScan::SharedPtr msg_p);
    void BubbleSort(float *p, int length, int *ind_diff);
    void Print(float str[], int n);
    void PrintInt(int str[], int n);

public:
    ClassLaser2D();
    ~ClassLaser2D();
};

ClassLaser2D::ClassLaser2D() : Node("laser_point_pub")
{
    RCLCPP_INFO(this->get_logger(), "雷达数据分析节点开启");

    cmd_vel.angular.z = 90;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.linear.x = 1500;

    sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 100, std::bind(&ClassLaser2D::GetGoal, this, std::placeholders::_1));
    pub = this->create_publisher<geometry_msgs::msg::Pose2D>("/point_mid", 100);
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("car_cmd_vel", 100);
}

ClassLaser2D::~ClassLaser2D()
{
    RCLCPP_INFO(this->get_logger(), "雷达数据分析节点关闭");
}

void ClassLaser2D::GetGoal(const sensor_msgs::msg::LaserScan::SharedPtr msg_p)
{
    int num = 0;
    int num_division = 1;

    for (int i = 784; i <= 1000; i++)
    {
        new_laser[num] = msg_p->ranges[i];
        // RCLCPP_INFO(this->get_logger(),"%f\t",new_laser[num]);
        num++;
    }
    for (int i = 0; i <= 216; i++)
    {
        new_laser[num] = msg_p->ranges[i];
        // RCLCPP_INFO(this->get_logger(),"%f\t",new_laser[num]);
        num++;

    }
    

    BubbleSort(new_laser, 432, ind);

    while (true)
    {
        if (std::abs(ind[num_division + 1] - ind[num_division]) > 70)
        {
            break;
        }
        num_division++;
    }
    // if (ind[num_division]
    ob_1.angle = (ind[num_division] - 216) / 2.77;
    ob_1.distance = new_laser[num_division];

    ob_2.angle = (ind[num_division + 1] - 216) / 2.77;
    ob_2.distance = new_laser[num_division + 1];
    RCLCPP_INFO(this->get_logger(),"障碍物 1 的角度为：%f , 距离为： %f,索引为：%d\n", ob_1.angle, ob_1.distance,ind[num_division]);
    RCLCPP_INFO(this->get_logger(),"障碍物 2 的角度为：%f , 距离为： %f,索引为：%d\n", ob_2.angle, ob_2.distance,ind[num_division+1]);

    ob_1.dk_x = ob_1.distance * cos(ob_1.angle * PI / 180);
    ob_1.dk_y = ob_1.distance * sin(ob_1.angle * PI / 180);

    ob_2.dk_x = ob_2.distance * cos(ob_2.angle * PI / 180);
    ob_2.dk_y = ob_2.distance * sin(ob_2.angle * PI / 180);

    if (ob_1.dk_y != ob_2.dk_y)
    {
        point.x = (ob_1.dk_x + ob_2.dk_x) / 2;
        point.y = (ob_1.dk_y + ob_2.dk_y) / 2;
        point.theta = -(ob_1.dk_x - ob_2.dk_x) / (ob_1.dk_y - ob_2.dk_y);
        pub->publish(point);
    }
}

void ClassLaser2D::BubbleSort(float *p, int length, int *ind_diff)
{
    for (int m = 0; m < length; m++)
    {
        ind_diff[m] = m;
    }

    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < length - i - 1; j++)
        {
            if (p[j] > p[j + 1])
            {
                float temp = p[j];
                p[j] = p[j + 1];
                p[j + 1] = temp;

                int ind_temp = ind_diff[j];
                ind_diff[j] = ind_diff[j + 1];
                ind_diff[j + 1] = ind_temp;
            }
        }
    }
}

void ClassLaser2D::Print(float str[], int n)
{
    for (int i = 0; i < n; i++)
    {
        std::cout << str[i] << " ";
    }
    std::cout << std::endl;
}

void ClassLaser2D::PrintInt(int str[], int n)
{
    for (int i = 0; i < n; i++)
    {
        std::cout << str[i] << " ";
    }
    std::cout << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClassLaser2D>());
    rclcpp::shutdown();
    return 0;
}
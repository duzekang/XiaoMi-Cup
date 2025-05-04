#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <lcm/lcm-cpp.hpp>
#include "../lcm_defines/lcm_types/robot_control_cmd_lcmt.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Geometry>

static double current_yaw = 0.0;

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    Eigen::Quaterniond q(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z);
    q.normalize();
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    current_yaw = euler[2];
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("straight_walk");
    lcm::LCM lcm("udpm://239.255.76.67:7671?ttl=255");

    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu",
        rclcpp::SensorDataQoS(),
        imu_callback);

    // 初始化控制指令
    lcm_types::robot_control_cmd_lcmt cmd{};
    uint8_t life_count = 0;
    rclcpp::Rate rate(50); // 50Hz控制频率

    /////////////////////////
    // 1. 进入站立状态(3秒) //
    /////////////////////////
    cmd.mode = 12;          // 站立模式
    cmd.gait_id = 0;
    cmd.contact = 0x0F;     // 四足全触地
    cmd.ctrl_point[2] = 0.20;  // 站立高度
    cmd.step_height[0] = 0.0;
    cmd.step_height[1] = 0.0;

    auto start_time = node->now();
    while (rclcpp::ok() && 
          (node->now() - start_time).seconds() < 3.0) {
        cmd.life_count = life_count++;
        lcm.publish("robot_control_cmd", &cmd);
        RCLCPP_INFO(node->get_logger(), "进入站立状态");
        rate.sleep();
    }

    //////////////////////////
    // 2. 执行直行(10秒)    //
    //////////////////////////
    cmd.mode = 11;          // 运动模式
    cmd.gait_id = 27;       // TROT_24_16步态 26
    cmd.vel_des[0] = 0.4;   // 前向速度0.4m/s
    cmd.foot_pose[1]=-0.06;
    cmd.foot_pose[4]=-0.06;

    start_time = node->now();
    while (rclcpp::ok() && 
          (node->now() - start_time).seconds() < 20.0) {
        std::cout<<"yaw: "<<current_yaw<<std::endl;
        cmd.life_count = life_count++;
        lcm.publish("robot_control_cmd", &cmd);
        RCLCPP_INFO(node->get_logger(), "直行中... 剩余时间: %.1f秒", 
                  20.0 - (node->now() - start_time).seconds());
        rate.sleep();
    }

    /////////////////////////
    // 3. 返回站立状态     //
    /////////////////////////
    cmd.mode = 12;         // 重新进入站立模式
    cmd.gait_id = 0;
    start_time = node->now();
    while (rclcpp::ok() && 
          (node->now() - start_time).seconds() < 1.0) {
        cmd.life_count = life_count++;
        lcm.publish("robot_control_cmd", &cmd);
        RCLCPP_INFO(node->get_logger(), "返回站立状态");
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "程序正常退出");
    rclcpp::shutdown();
    return 0;
}
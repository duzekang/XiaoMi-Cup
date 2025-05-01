#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <lcm/lcm-cpp.hpp>
#include "../lcm_defines/lcm_types/robot_control_cmd_lcmt.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Geometry>

// 全局状态
static double current_pitch = 0.0;
static rclcpp::Time start_time;

// IMU回调
void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    Eigen::Quaterniond q(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z);
    q.normalize();
    current_pitch = q.toRotationMatrix().eulerAngles(2, 1, 0)[1];
}

// 发送控制指令
void send_command(lcm::LCM& lcm, bool climbing) {
    static uint8_t life_count = 0;
    lcm_types::robot_control_cmd_lcmt cmd{};

    if(!climbing) {  // 站立模式
        cmd.mode = 12;
        cmd.gait_id = 0;
        cmd.contact = 0x0F;
        cmd.pos_des[2] = 0.28;  // 站立高度
    } else {         // 爬坡模式
        std::cout<<"当前picth: "<<current_pitch * 180.0 / M_PI<<"（度）"<<std::endl;
        constexpr double slope_angle = 20.0 * M_PI/180.0;
        cmd.mode = 11;
        cmd.gait_id = 26;
        cmd.vel_des[0] = 0.6;
        cmd.vel_des[1] = 0.1;
        cmd.rpy_des[1] = -slope_angle;
        cmd.pos_des[2] = 0.28 * cos(slope_angle);
        // 步高调整（前腿抬高更多）
        cmd.step_height[0] = 0.06;  // 前腿步高
        cmd.step_height[1] = 0.04;  // 后腿步高
        // 足端位置补偿（防止前腿碰撞）
        constexpr float FOOT_OFFSET = 0.03;
        cmd.foot_pose[0] = FOOT_OFFSET;  // 前腿X方向偏移
        cmd.foot_pose[3] = FOOT_OFFSET;  // 后腿X方向偏移
    }

    cmd.life_count = life_count++;
    lcm.publish("robot_control_cmd", &cmd);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("slope_ctrl");
    lcm::LCM lcm("udpm://239.255.76.67:7671?ttl=255");
    
    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu",  // 确保主题名一致
        imu_qos,
        imu_callback);

    start_time = node->now();
    
    // 主控制循环
    while(rclcpp::ok()) {
        // 处理ROS回调
        rclcpp::spin_some(node);
        
        // 处理LCM消息
        lcm.handleTimeout(0);
        
        // 计算运行时间
        auto elapsed = node->now() - start_time;
        if(elapsed.seconds() > 20.0) break;  // 20秒后退出
        
        // 阶段判断（前3秒站立，后7秒爬坡）
        bool climbing = (elapsed.seconds() > 5.0);
        
        // 发送控制指令
        send_command(lcm, climbing);
        
        // 控制频率50Hz
        static rclcpp::Rate rate(50); // 50Hz
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "程序正常退出");
    rclcpp::shutdown();
    return 0;
}
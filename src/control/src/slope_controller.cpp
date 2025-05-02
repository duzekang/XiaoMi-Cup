#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <lcm/lcm-cpp.hpp>
#include "../lcm_defines/lcm_types/robot_control_cmd_lcmt.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Geometry>

// 全局状态
static double current_pitch = 0.0;
static rclcpp::Time start_time;


// IMU 回调
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
        cmd.mode = 12;       // RECOVERY_STAND
        cmd.gait_id = 0;
        cmd.contact = 0x0F;   // 四足全触地
        cmd.pos_des[2] = 0.28; // 标准站立高度
        cmd.step_height[0] = 0.0;
        cmd.step_height[1] = 0.0;
    } else {        // 爬坡模式（20度）
        std::cout<<"当前picth: "<<current_pitch * 180.0 / M_PI<<"（度）"<<std::endl;
        constexpr double slope_rad = 20.0 * M_PI/180.0;
        
        cmd.mode = 11;        // 运动模式
        cmd.gait_id = 26;      // TROT_24_16（变频步态）
        cmd.contact = 0x0F;    // 四足全触地
        
        // 速度控制（符合TROT_24_16参数范围）
        cmd.vel_des[0] = 0.4;  // X方向速度（0.4m/s，低于最大值1.6的1/3）
        cmd.vel_des[2] = 0.0;  // 禁止偏航旋转
        
        // 姿态补偿（最大允许0.52rad≈30度）
        cmd.rpy_des[1] = -slope_rad * 1.2; // 增加20%补偿量
        
        // 高度控制（基于斜坡几何计算）
        cmd.pos_des[2] = 0.28 * cos(slope_rad); // 实际高度≈0.26m
        
        // 质心前移补偿（不超过0.235m限制）
        cmd.pos_des[0] = 0.2 * tan(slope_rad); // 约0.05m
        
        // 足端轨迹优化
        cmd.foot_pose[0] = 0.04;  // 前腿X方向偏移（最大允许0.04m）
        //cmd.foot_pose[3] = -0.02; // 后腿X方向回缩
        
        // 步态参数（基于TROT_24_16特性）
        cmd.step_height[0] = 0.06; // 前腿最大步高（表格允许值）
        cmd.step_height[1] = 0.04; // 后腿较低步高
        // cmd.gait_param[0] = 0.3f;  // 缩短步态周期
        // cmd.gait_param[1] = 0.28f; // 摆动相占比
        
        // 运动保护
        cmd.value = 0x01;      // 启用MPC轨迹跟踪
        cmd.duration = 1000;   // 指令持续1秒（自动续期）
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
        "/imu",
        imu_qos,
        imu_callback);

    start_time = node->now();

    // 主控制循环
    while (rclcpp::ok()) {
        // 处理 ROS 回调
        rclcpp::spin_some(node);

        // 处理 LCM 消息
        lcm.handleTimeout(0);

        // 计算运行时间
        auto elapsed = node->now() - start_time;
        if (elapsed.seconds() > 60.0) break;  // 20 秒后退出

        // 阶段判断
        bool climbing = (elapsed.seconds() > 5.0);
        // 发送控制指令
        send_command(lcm, climbing);

        // 控制频率 50Hz
        static rclcpp::Rate rate(50); // 50Hz
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "程序正常退出");
    rclcpp::shutdown();
    return 0;
}
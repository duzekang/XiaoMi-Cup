#include <rclcpp/rclcpp.hpp>
#include <lcm/lcm-cpp.hpp>
#include "../lcm_defines/lcm_types/robot_control_cmd_lcmt.hpp"

class StandingController : public rclcpp::Node {
public:
    StandingController() : Node("standing_controller") {
        // LCM初始化
        if (!lcm_.good()) {
            RCLCPP_FATAL(this->get_logger(), "LCM初始化失败!");
            rclcpp::shutdown();
        }

        // 定时器驱动控制循环（50Hz）
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() { publishStandCommand(); }
        );

        // LCM处理线程
        lcm_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                lcm_.handleTimeout(50);  // 非阻塞式处理
            }
        });
    }

    ~StandingController() {
        if (lcm_thread_.joinable()) {
            lcm_thread_.join();
        }
    }

private:
    void publishStandCommand() {
        lcm_types::robot_control_cmd_lcmt cmd{};
        
        // 核心站立参数
        cmd.mode = 12;        // 站立模式
        cmd.gait_id = 0;      // 站立步态标识
        // 发送指令
        lcm_.publish("robot_control_cmd", &cmd);
        
        RCLCPP_DEBUG(this->get_logger(), "指令已发送");
    }

    lcm::LCM lcm_{"udpm://239.255.76.67:7671?ttl=255"};
    std::thread lcm_thread_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<StandingController>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
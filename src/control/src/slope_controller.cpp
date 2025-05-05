#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <lcm/lcm-cpp.hpp>
#include "../lcm_defines/lcm_types/robot_control_cmd_lcmt.hpp"
#include "../lcm_defines/lcm_types/simulator_lcmt.hpp"  // 添加simulator LCM类型
#include <Eigen/Geometry>

// 全局状态
enum State { STAND, ASCEND, DESCEND };
static State current_state = STAND;
static rclcpp::Time state_start_time;
static constexpr double SLOPE_ANGLE = 20.0; // 坡度角度

// 全局位置信息
static double current_x = 0.0;
static double current_y = 0.0;
static double current_vx = 0.0;
static double current_vy = 0.0;
static double lateral_vel = 0.0;
static double current_pitch = 0.0;
static constexpr double TARGET_X = 2.0;  // X轴目标位置
static constexpr double ASCEND_END_Y = 8.0;  // 上坡结束位置
static constexpr double STOP_Y = 10.0;    // 程序停止位置

// 横向位置PID控制
class LateralController {
    public:
        LateralController(double kp) : kp(kp) {}
        double calculate(double current) {
            return kp * (current - TARGET_X);
        }
    private:
        double kp;
};
static LateralController lateral_controller(0.5);  // 比例系数

class simulator_handler{
    public:
    ~simulator_handler() {}

    void handle_message(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& chan, 
                      const lcm_types::simulator_lcmt* msg) {
        (void)rbuf; // 开发阶段显式标记
        (void)chan;
        // 更新全局位置信息
        current_x = msg->p[0];
        current_y = msg->p[1];
        current_vx = msg->vb[0];
        current_vy = msg->vb[1];
        current_pitch = msg->rpy[1];
    }  
};

void state_transition(rclcpp::Node::SharedPtr node) {
    const auto now = node->now();
    
    switch(current_state) {
    case STAND:
        if ((now - state_start_time).seconds() > 5.0) { // 站立5秒后开始爬坡
            current_state = ASCEND;
            state_start_time = now;
            RCLCPP_INFO(node->get_logger(), "开始爬坡");
        }
        break;
        
    case ASCEND:
        if (current_y >= ASCEND_END_Y) { // 到达Y=8米转下坡
            current_state = DESCEND;
            state_start_time = now;
            RCLCPP_INFO(node->get_logger(), "到达Y=%.2fm，开始下坡", current_y);
        }
        break;
        
    case DESCEND:
        if (current_y >= STOP_Y) { // 到达Y=10米停止程序
            RCLCPP_INFO(node->get_logger(), "到达Y=%.2fm，程序终止", current_y);
            rclcpp::shutdown();
            exit(0);
        }
        break;
    }
}

void send_command(lcm::LCM& lcm) {
    static uint8_t life_count = 0;
    lcm_types::robot_control_cmd_lcmt cmd{};
    const double slope_rad = SLOPE_ANGLE * M_PI/180.0;
    
    // 横向位置控制
    lateral_vel = lateral_controller.calculate(current_x);

    switch(current_state) {
    case STAND: {
        cmd.mode = 12;
        cmd.gait_id = 0;
        break;
    }
    
    case ASCEND: {  // 上坡模式
        cmd.mode = 11;
        cmd.gait_id = 26;
        
        cmd.vel_des[0] = 0.4;  // X方向速度
        //cmd.vel_des[1] = lateral_vel;
        cmd.vel_des[2] = 0.0;  // 禁止偏航旋转
        
        // 姿态补偿（最大允许0.52rad≈30度）
        cmd.rpy_des[1] = -0.25; // 增加20%补偿量
        cmd.rpy_des[2] = 1.523;
        
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
        
        // 运动保护
        cmd.value = 0x01;      // 启用MPC轨迹跟踪
        break;
    }
    
    case DESCEND: {  // 下坡模式
        cmd.mode = 11;
        cmd.gait_id = 26;
        cmd.contact = 0x0F;
        
        cmd.vel_des[0] = 0.4;           // 降低前进速度
        cmd.vel_des[1] = lateral_vel * 1.2; // 加强位置修正
        
        cmd.rpy_des[1] = 0.30; // 增加20%补偿量
        cmd.rpy_des[2] = 1.523;
        
        // 高度控制（基于斜坡几何计算）
        cmd.pos_des[2] = 0.28 * cos(slope_rad);
        
        // 质心后移补偿
        cmd.pos_des[0] = -0.2 * tan(slope_rad);
        
        // 足端轨迹优化
        cmd.foot_pose[0] = -0.04;
        
        cmd.step_height[0] = 0.04; // 前腿最大步高（表格允许值）
        cmd.step_height[1] = 0.06; // 后腿较低步高
        
        // 运动保护
        cmd.value = 0x01;      // 启用MPC轨迹跟踪
        break;
    }
    }

    cmd.life_count = life_count++;
    lcm.publish("robot_control_cmd", &cmd);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("slope_ctrl");
    lcm::LCM lcm("udpm://239.255.76.67:7671?ttl=255");
    lcm::LCM lcm2("udpm://239.255.76.67:7667?ttl=255");
    
    // 订阅simulator LCM消息
    simulator_handler handle_object;
    lcm2.subscribe<lcm_types::simulator_lcmt>("simulator_state", &simulator_handler::handle_message, &handle_object);

    state_start_time = node->now();
    rclcpp::Rate rate(50);

    while(rclcpp::ok()) {
        lcm.handleTimeout(0);
        lcm2.handleTimeout(0);
        rclcpp::spin_some(node);
        
        // 状态转移判断
        state_transition(node);
        
        // 发送控制指令
        send_command(lcm);
        
        // 实时显示位置信息
        static int count = 0;
         if (++count % 50 == 0) { // 每秒输出1次
            std::cout<<"----------------------------------\n";
            std::cout<<"状态"<<current_state<<"\n";
            std::cout<<"X= "<<current_x<<"\n";
            std::cout<<"Y= "<<current_x<<"\n";
            std::cout<<"vx="<<current_vx<<"\n";
            std::cout<<"vy="<<current_vy<<"  期望vy="<<lateral_vel<<"\n";
            std::cout<<"pitch="<<current_pitch<<"\n";
        }
        
        // 最终停止条件双重检查
        if (current_y >= STOP_Y) {
            RCLCPP_INFO(node->get_logger(), "到达终点Y=%.2fm，程序终止", current_y);
            break;
        }
        
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}